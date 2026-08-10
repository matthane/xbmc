/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "guilib/guiinfo/GUIInfoLabels.h"
#include "utils/StringUtils.h"
#include "utils/TimeUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#if defined(HAVE_LIBDOVI) && defined(AML_FRAME_METADATA_PARSER)
#include <libdovi/rpu_parser.h>
#endif

// Live per-frame DV/HDR metadata read from the stream by the Amlogic codec and
// published for GUI info labels. The parser section below is compiled only into
// DVDVideoCodecAmlogic.cpp (AML_FRAME_METADATA_PARSER). Everything above it is
// visible to every consumer.

struct AMLFrameMetadata
{
  bool doviValid{false};
  int doviProfile{0};
  int doviCompatId{-1};
  std::string doviELType;

  // L1 dynamic brightness, sent with every RPU
  bool hasL1{false};
  uint16_t l1MinPq{0};
  uint16_t l1MaxPq{0};
  uint16_t l1AvgPq{0};

  bool hasL5{false};
  uint16_t l5Left{0};
  uint16_t l5Right{0};
  uint16_t l5Top{0};
  uint16_t l5Bottom{0};

  // L6 ST 2086 fallback. Units differ from the display-mastering hints below:
  // l6MinLum is in 0.0001 cd/m2 steps, l6MaxLum in cd/m2.
  bool hasL6{false};
  uint16_t l6MaxCll{0};
  uint16_t l6MaxFall{0};
  uint16_t l6MinLum{0};
  uint16_t l6MaxLum{0};

  // L254 CM v4.0 marker, absent means CM v2.9
  bool hasL254{false};
  uint8_t l254DmMode{0};
  uint8_t l254DmVersionIndex{0};

  // source PQ bounds, zeroed by the bitstream when dv_md_compression is active,
  // so only trusted from uncompressed frames. hasDmData marks that a frame
  // carried vdr_dm_data at all, so compressed can latch across frames without it.
  bool hasDmData{false};
  bool hasSourcePq{false};
  uint16_t sourceMinPq{0};
  uint16_t sourceMaxPq{0};
  bool compressed{false};

  // player-side transformations active on this stream
  bool converted{false};
  bool l5Zeroed{false};
  bool rpuRemoved{false};

  // HDR10 static metadata from stream hints (any HDR stream, not only DV)
  bool hasContentLight{false};
  unsigned int hdrMaxCll{0};
  unsigned int hdrMaxFall{0};
  bool hasMastering{false};
  double hdrMinLum{0.0};
  double hdrMaxLum{0.0};

  bool operator==(const AMLFrameMetadata&) const = default;

  // carry statics latched from earlier frames into a frame that omits them
  void InheritStatics(const AMLFrameMetadata& prev)
  {
    if (doviELType.empty())
      doviELType = prev.doviELType;
    if (!hasL6 && prev.hasL6)
    {
      hasL6 = true;
      l6MaxCll = prev.l6MaxCll;
      l6MaxFall = prev.l6MaxFall;
      l6MinLum = prev.l6MinLum;
      l6MaxLum = prev.l6MaxLum;
    }
    if (!hasL254 && prev.hasL254)
    {
      hasL254 = true;
      l254DmMode = prev.l254DmMode;
      l254DmVersionIndex = prev.l254DmVersionIndex;
    }
    if (!hasSourcePq && prev.hasSourcePq)
    {
      hasSourcePq = true;
      sourceMinPq = prev.sourceMinPq;
      sourceMaxPq = prev.sourceMaxPq;
    }
    if (!hasDmData && prev.hasDmData)
    {
      hasDmData = true;
      compressed = prev.compressed;
    }
  }
};

constexpr int AML_FRAME_METADATA_API_VERSION = 1;

// Singleton store: producers are codec instances, consumers are GUI threads.
// A stream switch opens the successor codec before the predecessor is closed,
// so clearing is gated on an ownership token instead of happening blindly.
class CAMLFrameMetadataStore
{
public:
  static CAMLFrameMetadataStore& GetInstance()
  {
    static CAMLFrameMetadataStore store;
    return store;
  }

  uint32_t Register()
  {
    std::lock_guard lock(m_lock);
    m_owner = ++m_nextToken;
    m_meta = {};
    return m_owner;
  }

  void Unregister(uint32_t token)
  {
    std::lock_guard lock(m_lock);
    if (m_owner == token)
    {
      m_owner = 0;
      m_meta = {};
    }
  }

  void Publish(uint32_t token, const AMLFrameMetadata& meta)
  {
    std::lock_guard lock(m_lock);
    if (token != 0 && m_owner == token && !(m_meta == meta))
      m_meta = meta;
  }

  AMLFrameMetadata Get() const
  {
    std::lock_guard lock(m_lock);
    return m_meta;
  }

private:
  CAMLFrameMetadataStore() = default;

  mutable std::mutex m_lock;
  AMLFrameMetadata m_meta;
  uint32_t m_owner{0};
  uint32_t m_nextToken{0};
};

// Orders metadata by presentation instead of feed time: values are committed
// per pts when the decoder accepts the packet and released when the drain
// target, the player clock plus the render latency, reaches their pts. All
// methods run on the VideoPlayerVideo thread.
class CAMLFrameMetadataSequencer
{
public:
  void Commit(double pts, const AMLFrameMetadata& meta)
  {
    // a pts far below the newest queued entry means the feed jumped backwards
    // without a flush, and the stranded entries would otherwise win eviction
    if (!m_queue.empty() && pts + BACKWARD_JUMP < m_queue.rbegin()->first)
      m_queue.clear();
    m_queue[pts] = meta;
    if (m_queue.size() > MAX_DEPTH)
      Compact();
    // the oldest entries are the next to be consumed, so overflow drops newest
    while (m_queue.size() > MAX_DEPTH)
      m_queue.erase(std::prev(m_queue.end()));
  }

  // newest entry at or before pts, consuming everything up to it. A miss keeps
  // the queue intact so the caller can hold the last published values.
  bool Consume(double pts, AMLFrameMetadata& meta)
  {
    auto it = m_queue.upper_bound(pts + PTS_TOLERANCE);
    if (it == m_queue.begin())
      return false;
    --it;
    meta = it->second;
    m_queue.erase(m_queue.begin(), std::next(it));
    return true;
  }

  void Reset() { m_queue.clear(); }

private:
  // the consumer holds values between entries, so an entry equal to its pts
  // predecessor can go. The newest entries stay untouched since a late
  // reordered commit could still land between them
  void Compact()
  {
    if (m_queue.size() <= REORDER_MARGIN)
      return;
    const auto stop = std::prev(m_queue.end(), REORDER_MARGIN);
    for (auto it = std::next(m_queue.begin()); it != stop;)
    {
      if (it->second == std::prev(it)->second)
        it = m_queue.erase(it);
      else
        ++it;
    }
  }

  static constexpr double PTS_TOLERANCE = 1000.0; // DVD_TIME_BASE is microseconds
  static constexpr double BACKWARD_JUMP = 5000000.0;
  // deep enough that after compaction only per frame churn can overflow it
  static constexpr size_t MAX_DEPTH = 512;
  static constexpr size_t REORDER_MARGIN = 64;

  std::map<double, AMLFrameMetadata> m_queue;
};

// ST 2084 PQ code (12 bit) to nits
inline double AMLPqToNits(uint16_t pq)
{
  constexpr double ST2084_Y_MAX = 10000.0;
  constexpr double ST2084_M1 = 2610.0 / 16384.0;
  constexpr double ST2084_M2 = (2523.0 / 4096.0) * 128.0;
  constexpr double ST2084_C1 = 3424.0 / 4096.0;
  constexpr double ST2084_C2 = (2413.0 / 4096.0) * 32.0;
  constexpr double ST2084_C3 = (2392.0 / 4096.0) * 32.0;

  // the well known codes are returned exactly, the 12 bit quantization rounds them off
  switch (pq)
  {
    case 0:
      return 0.0;
    case 7:
      return 0.0001;
    case 10:
      return 0.0002;
    case 17:
      return 0.0005;
    case 26:
      return 0.001;
    case 38:
      return 0.002;
    case 62:
      return 0.005;
    case 3079:
      return 1000.0;
    case 3388:
      return 2000.0;
    case 3696:
      return 4000.0;
    case 4095:
      return 10000.0;
    default:
      break;
  }

  const double pqPow = std::pow(pq / 4095.0, 1.0 / ST2084_M2);
  const double num = std::max(pqPow - ST2084_C1, 0.0);
  const double den = ST2084_C2 - ST2084_C3 * pqPow;

  if (std::abs(den) < std::numeric_limits<double>::epsilon())
    return 0.0;

  return ST2084_Y_MAX * std::pow(num / den, 1.0 / ST2084_M1);
}

// label ids and names, continuing the CE reserved Player.Process range after
// audiochannelssink

constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_API_VERSION = PLAYER_PROCESS_START + 34;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_PROFILE = PLAYER_PROCESS_START + 35;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_EL_TYPE = PLAYER_PROCESS_START + 36;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_PQ = PLAYER_PROCESS_START + 37;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_NITS = PLAYER_PROCESS_START + 38;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_PQ = PLAYER_PROCESS_START + 39;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_NITS = PLAYER_PROCESS_START + 40;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_PQ = PLAYER_PROCESS_START + 41;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_NITS = PLAYER_PROCESS_START + 42;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_PQ = PLAYER_PROCESS_START + 43;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_NITS = PLAYER_PROCESS_START + 44;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_PQ = PLAYER_PROCESS_START + 45;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_NITS = PLAYER_PROCESS_START + 46;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L5_LEFT_OFFSET = PLAYER_PROCESS_START + 47;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L5_RIGHT_OFFSET = PLAYER_PROCESS_START + 48;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L5_TOP_OFFSET = PLAYER_PROCESS_START + 49;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L5_BOTTOM_OFFSET = PLAYER_PROCESS_START + 50;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_CLL = PLAYER_PROCESS_START + 51;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_FALL = PLAYER_PROCESS_START + 52;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MIN_LUM = PLAYER_PROCESS_START + 53;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_LUM = PLAYER_PROCESS_START + 54;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_META_VERSION = PLAYER_PROCESS_START + 55;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_DOVI_FLAGS = PLAYER_PROCESS_START + 56;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_HDR_MAX_CLL = PLAYER_PROCESS_START + 57;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_HDR_MAX_FALL = PLAYER_PROCESS_START + 58;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_HDR_MIN_LUM = PLAYER_PROCESS_START + 59;
constexpr uint32_t CE_PLAYER_PROCESS_VIDEO_HDR_MAX_LUM = PLAYER_PROCESS_START + 60;

struct AMLFrameMetadataLabel
{
  const char* name;
  uint32_t id;
};

// full label expressions and their ids, registered with the CE label registry
// at GUI init
inline const std::array<AMLFrameMetadataLabel, 27>& AMLFrameMetadataLabels()
{
  static const std::array<AMLFrameMetadataLabel, 27> labels = {{
      {"player.process(video.dovi.apiversion)", CE_PLAYER_PROCESS_VIDEO_DOVI_API_VERSION},
      {"player.process(video.dovi.profile)", CE_PLAYER_PROCESS_VIDEO_DOVI_PROFILE},
      {"player.process(video.dovi.el.type)", CE_PLAYER_PROCESS_VIDEO_DOVI_EL_TYPE},
      {"player.process(video.dovi.source.min.pq)", CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_PQ},
      {"player.process(video.dovi.source.min.nits)", CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_NITS},
      {"player.process(video.dovi.source.max.pq)", CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_PQ},
      {"player.process(video.dovi.source.max.nits)", CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_NITS},
      {"player.process(video.dovi.l1.min.pq)", CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_PQ},
      {"player.process(video.dovi.l1.min.nits)", CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_NITS},
      {"player.process(video.dovi.l1.max.pq)", CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_PQ},
      {"player.process(video.dovi.l1.max.nits)", CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_NITS},
      {"player.process(video.dovi.l1.avg.pq)", CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_PQ},
      {"player.process(video.dovi.l1.avg.nits)", CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_NITS},
      {"player.process(video.dovi.l5.left.offset)", CE_PLAYER_PROCESS_VIDEO_DOVI_L5_LEFT_OFFSET},
      {"player.process(video.dovi.l5.right.offset)", CE_PLAYER_PROCESS_VIDEO_DOVI_L5_RIGHT_OFFSET},
      {"player.process(video.dovi.l5.top.offset)", CE_PLAYER_PROCESS_VIDEO_DOVI_L5_TOP_OFFSET},
      {"player.process(video.dovi.l5.bottom.offset)", CE_PLAYER_PROCESS_VIDEO_DOVI_L5_BOTTOM_OFFSET},
      {"player.process(video.dovi.l6.max.cll)", CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_CLL},
      {"player.process(video.dovi.l6.max.fall)", CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_FALL},
      {"player.process(video.dovi.l6.min.lum)", CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MIN_LUM},
      {"player.process(video.dovi.l6.max.lum)", CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_LUM},
      {"player.process(video.dovi.meta.version)", CE_PLAYER_PROCESS_VIDEO_DOVI_META_VERSION},
      {"player.process(video.dovi.flags)", CE_PLAYER_PROCESS_VIDEO_DOVI_FLAGS},
      {"player.process(video.hdr.max.cll)", CE_PLAYER_PROCESS_VIDEO_HDR_MAX_CLL},
      {"player.process(video.hdr.max.fall)", CE_PLAYER_PROCESS_VIDEO_HDR_MAX_FALL},
      {"player.process(video.hdr.min.lum)", CE_PLAYER_PROCESS_VIDEO_HDR_MIN_LUM},
      {"player.process(video.hdr.max.lum)", CE_PLAYER_PROCESS_VIDEO_HDR_MAX_LUM},
  }};
  return labels;
}

inline bool AMLFrameMetadataOwnsId(int info)
{
  const uint32_t id = static_cast<uint32_t>(info);
  return id >= CE_PLAYER_PROCESS_VIDEO_DOVI_API_VERSION &&
         id <= CE_PLAYER_PROCESS_VIDEO_HDR_MAX_LUM;
}

// one store snapshot per render pass so related labels cannot show values from
// different video frames, and thread_local avoids a shared cache lock
inline const AMLFrameMetadata& AMLGetCachedFrameMetadata()
{
  thread_local AMLFrameMetadata cached;
  thread_local unsigned int cachedFrameTime = 0;
  thread_local bool cachedOnce = false;

  const unsigned int frameTime = CTimeUtils::GetFrameTime();
  if (!cachedOnce || frameTime != cachedFrameTime)
  {
    cached = CAMLFrameMetadataStore::GetInstance().Get();
    cachedFrameTime = frameTime;
    cachedOnce = true;
  }
  return cached;
}

// one numeric format across the metadata labels: four decimals below 1, whole
// numbers above, no unit suffix baked into the value
inline std::string AMLFormatMetadataNumber(double value)
{
  if (value != 0.0 && std::abs(value) < 1.0)
    return StringUtils::Format("{:.4f}", value);
  return std::to_string(std::lround(value));
}

// false when the id is not ours. Ours answer true, empty when data is absent
inline bool AMLFrameMetadataGetLabel(std::string& value, int info)
{
  if (!AMLFrameMetadataOwnsId(info))
    return false;

  const AMLFrameMetadata& meta = AMLGetCachedFrameMetadata();
  switch (static_cast<uint32_t>(info))
  {
    case CE_PLAYER_PROCESS_VIDEO_DOVI_API_VERSION:
      value = std::to_string(AML_FRAME_METADATA_API_VERSION);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_PROFILE:
      if (meta.doviValid)
      {
        if (meta.doviCompatId >= 0)
          value = StringUtils::Format("{}.{}", meta.doviProfile, meta.doviCompatId);
        else
          value = std::to_string(meta.doviProfile);
      }
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_EL_TYPE:
      value = meta.doviELType;
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_PQ:
      if (meta.hasSourcePq)
        value = std::to_string(meta.sourceMinPq);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_NITS:
      if (meta.hasSourcePq)
        value = AMLFormatMetadataNumber(AMLPqToNits(meta.sourceMinPq));
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_PQ:
      if (meta.hasSourcePq)
        value = std::to_string(meta.sourceMaxPq);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_NITS:
      if (meta.hasSourcePq)
        value = AMLFormatMetadataNumber(AMLPqToNits(meta.sourceMaxPq));
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_PQ:
      if (meta.hasL1)
        value = std::to_string(meta.l1MinPq);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_NITS:
      if (meta.hasL1)
        value = AMLFormatMetadataNumber(AMLPqToNits(meta.l1MinPq));
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_PQ:
      if (meta.hasL1)
        value = std::to_string(meta.l1MaxPq);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_NITS:
      if (meta.hasL1)
        value = AMLFormatMetadataNumber(AMLPqToNits(meta.l1MaxPq));
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_PQ:
      if (meta.hasL1)
        value = std::to_string(meta.l1AvgPq);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_NITS:
      if (meta.hasL1)
        value = AMLFormatMetadataNumber(AMLPqToNits(meta.l1AvgPq));
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_LEFT_OFFSET:
      if (meta.hasL5)
        value = std::to_string(meta.l5Left);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_RIGHT_OFFSET:
      if (meta.hasL5)
        value = std::to_string(meta.l5Right);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_TOP_OFFSET:
      if (meta.hasL5)
        value = std::to_string(meta.l5Top);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_BOTTOM_OFFSET:
      if (meta.hasL5)
        value = std::to_string(meta.l5Bottom);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_CLL:
      if (meta.hasL6)
        value = std::to_string(meta.l6MaxCll);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_FALL:
      if (meta.hasL6)
        value = std::to_string(meta.l6MaxFall);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MIN_LUM:
      if (meta.hasL6)
        value = AMLFormatMetadataNumber(meta.l6MinLum * 0.0001);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_LUM:
      if (meta.hasL6)
        value = AMLFormatMetadataNumber(meta.l6MaxLum);
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_META_VERSION:
      if (meta.doviValid)
        value = meta.hasL254 ? "4.0" : "2.9";
      break;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_FLAGS:
      if (meta.doviValid)
      {
        const auto append = [&value](const char* flag)
        {
          if (!value.empty())
            value += ' ';
          value += flag;
        };
        if (meta.converted)
          append("converted");
        if (meta.l5Zeroed)
          append("l5zeroed");
        if (meta.rpuRemoved)
          append("rpuremoved");
        if (meta.compressed)
          append("compressed");
      }
      break;
    case CE_PLAYER_PROCESS_VIDEO_HDR_MAX_CLL:
      if (meta.hasContentLight)
        value = std::to_string(meta.hdrMaxCll);
      break;
    case CE_PLAYER_PROCESS_VIDEO_HDR_MAX_FALL:
      if (meta.hasContentLight)
        value = std::to_string(meta.hdrMaxFall);
      break;
    case CE_PLAYER_PROCESS_VIDEO_HDR_MIN_LUM:
      if (meta.hasMastering)
        value = AMLFormatMetadataNumber(meta.hdrMinLum);
      break;
    case CE_PLAYER_PROCESS_VIDEO_HDR_MAX_LUM:
      if (meta.hasMastering)
        value = AMLFormatMetadataNumber(meta.hdrMaxLum);
      break;
    default:
      return false;
  }
  return true;
}

// integer forms: pq labels return the raw 12 bit code, nits labels the value
// rounded to whole cd/m2
inline bool AMLFrameMetadataGetInt(int& value, int info)
{
  if (!AMLFrameMetadataOwnsId(info))
    return false;

  const AMLFrameMetadata& meta = AMLGetCachedFrameMetadata();
  switch (static_cast<uint32_t>(info))
  {
    case CE_PLAYER_PROCESS_VIDEO_DOVI_API_VERSION:
      value = AML_FRAME_METADATA_API_VERSION;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_PROFILE:
      if (!meta.doviValid)
        return false;
      value = meta.doviProfile;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_PQ:
      if (!meta.hasSourcePq)
        return false;
      value = meta.sourceMinPq;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MIN_NITS:
      if (!meta.hasSourcePq)
        return false;
      value = static_cast<int>(std::lround(AMLPqToNits(meta.sourceMinPq)));
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_PQ:
      if (!meta.hasSourcePq)
        return false;
      value = meta.sourceMaxPq;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_SOURCE_MAX_NITS:
      if (!meta.hasSourcePq)
        return false;
      value = static_cast<int>(std::lround(AMLPqToNits(meta.sourceMaxPq)));
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_PQ:
      if (!meta.hasL1)
        return false;
      value = meta.l1MinPq;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MIN_NITS:
      if (!meta.hasL1)
        return false;
      value = static_cast<int>(std::lround(AMLPqToNits(meta.l1MinPq)));
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_PQ:
      if (!meta.hasL1)
        return false;
      value = meta.l1MaxPq;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_MAX_NITS:
      if (!meta.hasL1)
        return false;
      value = static_cast<int>(std::lround(AMLPqToNits(meta.l1MaxPq)));
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_PQ:
      if (!meta.hasL1)
        return false;
      value = meta.l1AvgPq;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L1_AVG_NITS:
      if (!meta.hasL1)
        return false;
      value = static_cast<int>(std::lround(AMLPqToNits(meta.l1AvgPq)));
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_LEFT_OFFSET:
      if (!meta.hasL5)
        return false;
      value = meta.l5Left;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_RIGHT_OFFSET:
      if (!meta.hasL5)
        return false;
      value = meta.l5Right;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_TOP_OFFSET:
      if (!meta.hasL5)
        return false;
      value = meta.l5Top;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L5_BOTTOM_OFFSET:
      if (!meta.hasL5)
        return false;
      value = meta.l5Bottom;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_CLL:
      if (!meta.hasL6)
        return false;
      value = meta.l6MaxCll;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_FALL:
      if (!meta.hasL6)
        return false;
      value = meta.l6MaxFall;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_L6_MAX_LUM:
      if (!meta.hasL6)
        return false;
      value = meta.l6MaxLum;
      return true;
    case CE_PLAYER_PROCESS_VIDEO_DOVI_FLAGS:
      if (!meta.doviValid)
        return false;
      value = (meta.converted ? 1 : 0) | (meta.l5Zeroed ? 2 : 0) | (meta.rpuRemoved ? 4 : 0) |
              (meta.compressed ? 8 : 0);
      return true;
    case CE_PLAYER_PROCESS_VIDEO_HDR_MAX_CLL:
      if (!meta.hasContentLight)
        return false;
      value = static_cast<int>(meta.hdrMaxCll);
      return true;
    case CE_PLAYER_PROCESS_VIDEO_HDR_MAX_FALL:
      if (!meta.hasContentLight)
        return false;
      value = static_cast<int>(meta.hdrMaxFall);
      return true;
    case CE_PLAYER_PROCESS_VIDEO_HDR_MAX_LUM:
      if (!meta.hasMastering)
        return false;
      value = static_cast<int>(std::lround(meta.hdrMaxLum));
      return true;
    default:
      return false;
  }
}

#if defined(HAVE_LIBDOVI) && defined(AML_FRAME_METADATA_PARSER)

// takes ownership of rpu and frees it on every path
inline void AMLFillFromDoviRpu(DoviRpuOpaque* rpu, AMLFrameMetadata& meta)
{
  if (!rpu)
    return;

  const DoviRpuDataHeader* header = dovi_rpu_get_header(rpu);
  if (!header)
  {
    dovi_rpu_free(rpu);
    return;
  }

  meta.doviValid = true;
  meta.doviProfile = header->guessed_profile;
  if (header->el_type)
    meta.doviELType = header->el_type;

  const DoviVdrDmData* vdr = dovi_rpu_get_vdr_dm_data(rpu);
  if (vdr)
  {
    meta.hasDmData = true;
    meta.compressed = vdr->compressed;
    if (!vdr->compressed)
    {
      meta.hasSourcePq = true;
      meta.sourceMinPq = vdr->source_min_pq;
      meta.sourceMaxPq = vdr->source_max_pq;
    }
    if (vdr->dm_data.level1)
    {
      meta.hasL1 = true;
      meta.l1MinPq = vdr->dm_data.level1->min_pq;
      meta.l1MaxPq = vdr->dm_data.level1->max_pq;
      meta.l1AvgPq = vdr->dm_data.level1->avg_pq;
    }
    if (vdr->dm_data.level5)
    {
      meta.hasL5 = true;
      meta.l5Left = vdr->dm_data.level5->active_area_left_offset;
      meta.l5Right = vdr->dm_data.level5->active_area_right_offset;
      meta.l5Top = vdr->dm_data.level5->active_area_top_offset;
      meta.l5Bottom = vdr->dm_data.level5->active_area_bottom_offset;
    }
    if (vdr->dm_data.level6)
    {
      meta.hasL6 = true;
      meta.l6MaxCll = vdr->dm_data.level6->max_content_light_level;
      meta.l6MaxFall = vdr->dm_data.level6->max_frame_average_light_level;
      meta.l6MinLum = vdr->dm_data.level6->min_display_mastering_luminance;
      meta.l6MaxLum = vdr->dm_data.level6->max_display_mastering_luminance;
    }
    if (vdr->dm_data.level254)
    {
      meta.hasL254 = true;
      meta.l254DmMode = vdr->dm_data.level254->dm_mode;
      meta.l254DmVersionIndex = vdr->dm_data.level254->dm_version_index;
    }
    dovi_rpu_free_vdr_dm_data(vdr);
  }

  dovi_rpu_free_header(header);
  dovi_rpu_free(rpu);
}

// Walks one demux packet for the DV RPU (HEVC NAL UNSPEC62) and parses it.
// nalLengthSize is the length-prefix size from hvcC extradata, 0 for Annex-B.
// dovi_parse_unspec62_nalu wants the escaped NAL including its nal_unit_header
// (7C 01), so nothing is stripped or unescaped here.
inline void AMLParseHevcDoviRpu(const uint8_t* data,
                                size_t size,
                                int nalLengthSize,
                                AMLFrameMetadata& meta)
{
  if (!data || size < 4)
    return;

  if (nalLengthSize >= 1 && nalLengthSize <= 4)
  {
    size_t pos = 0;
    while (pos + nalLengthSize <= size)
    {
      uint32_t len = 0;
      for (int i = 0; i < nalLengthSize; ++i)
        len = (len << 8) | data[pos + i];
      pos += nalLengthSize;
      if (len == 0 || len > size - pos)
        return;
      if (((data[pos] >> 1) & 0x3f) == 62)
      {
        AMLFillFromDoviRpu(dovi_parse_unspec62_nalu(data + pos, len), meta);
        return;
      }
      pos += len;
    }
    return;
  }

  // Annex-B with mixed 3- and 4-byte start codes
  size_t nal = SIZE_MAX;
  for (size_t i = 0; i + 2 < size; ++i)
  {
    if (data[i] != 0 || data[i + 1] != 0 || data[i + 2] != 1)
      continue;
    if (nal != SIZE_MAX && ((data[nal] >> 1) & 0x3f) == 62)
    {
      // a 4-byte start code owns the zero before this prefix, and an RPU never
      // ends in 0x00 (rbsp_trailing_bits), so trailing zeros are not payload
      size_t end = i;
      while (end > nal && data[end - 1] == 0)
        end--;
      AMLFillFromDoviRpu(dovi_parse_unspec62_nalu(data + nal, end - nal), meta);
      return;
    }
    nal = i + 3;
    i += 2;
  }
  if (nal != SIZE_MAX && nal < size && ((data[nal] >> 1) & 0x3f) == 62)
  {
    size_t end = size;
    while (end > nal && data[end - 1] == 0)
      end--;
    AMLFillFromDoviRpu(dovi_parse_unspec62_nalu(data + nal, end - nal), meta);
  }
}

// One shot fill of the HDR10 static fields from the mastering display and
// content light SEI, for streams whose container carries no side data (HEVC
// mkv and ts keep these in the SEI only). Returns true once both are present.
inline bool AMLParseHevcStaticSei(const uint8_t* data,
                                  size_t size,
                                  int nalLengthSize,
                                  AMLFrameMetadata& meta)
{
  if (!data || size < 4)
    return meta.hasContentLight && meta.hasMastering;

  const auto parseSeiNal = [&meta](const uint8_t* nal, size_t len)
  {
    if (len < 3 || ((nal[0] >> 1) & 0x3f) != 39) // HEVC_NAL_SEI_PREFIX
      return;

    std::vector<uint8_t> rbsp;
    rbsp.reserve(len);
    for (size_t i = 2; i < len; ++i)
    {
      if (i + 2 < len && nal[i] == 0 && nal[i + 1] == 0 && nal[i + 2] == 3)
      {
        rbsp.push_back(0);
        rbsp.push_back(0);
        i += 2;
      }
      else
        rbsp.push_back(nal[i]);
    }

    size_t p = 0;
    const size_t end = rbsp.size();
    while (p + 2 < end)
    {
      uint32_t type = 0;
      while (p < end && rbsp[p] == 0xFF)
      {
        type += 255;
        ++p;
      }
      if (p >= end)
        break;
      type += rbsp[p++];

      uint32_t payload = 0;
      while (p < end && rbsp[p] == 0xFF)
      {
        payload += 255;
        ++p;
      }
      if (p >= end)
        break;
      payload += rbsp[p++];
      if (payload > end - p)
        break;

      if (type == 137 && payload >= 24 && !meta.hasMastering)
      {
        const uint32_t maxLum = (rbsp[p + 16] << 24) | (rbsp[p + 17] << 16) |
                                (rbsp[p + 18] << 8) | rbsp[p + 19];
        const uint32_t minLum = (rbsp[p + 20] << 24) | (rbsp[p + 21] << 16) |
                                (rbsp[p + 22] << 8) | rbsp[p + 23];
        meta.hasMastering = true;
        meta.hdrMaxLum = maxLum * 0.0001;
        meta.hdrMinLum = minLum * 0.0001;
      }
      else if (type == 144 && payload >= 4 && !meta.hasContentLight)
      {
        meta.hasContentLight = true;
        meta.hdrMaxCll = (rbsp[p] << 8) | rbsp[p + 1];
        meta.hdrMaxFall = (rbsp[p + 2] << 8) | rbsp[p + 3];
      }
      p += payload;
    }
  };

  if (nalLengthSize >= 1 && nalLengthSize <= 4)
  {
    size_t pos = 0;
    while (pos + nalLengthSize <= size)
    {
      uint32_t len = 0;
      for (int i = 0; i < nalLengthSize; ++i)
        len = (len << 8) | data[pos + i];
      pos += nalLengthSize;
      if (len == 0 || len > size - pos)
        break;
      parseSeiNal(data + pos, len);
      pos += len;
    }
  }
  else
  {
    size_t nal = SIZE_MAX;
    for (size_t i = 0; i + 2 < size; ++i)
    {
      if (data[i] != 0 || data[i + 1] != 0 || data[i + 2] != 1)
        continue;
      if (nal != SIZE_MAX)
      {
        size_t end = i;
        while (end > nal && data[end - 1] == 0)
          end--;
        parseSeiNal(data + nal, end - nal);
      }
      nal = i + 3;
      i += 2;
    }
    if (nal != SIZE_MAX && nal < size)
      parseSeiNal(data + nal, size - nal);
  }

  return meta.hasContentLight && meta.hasMastering;
}

inline bool AMLReadLeb128(const uint8_t* data, size_t end, size_t& pos, uint64_t& value)
{
  value = 0;
  for (int i = 0; i < 8; ++i)
  {
    if (pos >= end)
      return false;
    const uint8_t b = data[pos++];
    value |= static_cast<uint64_t>(b & 0x7f) << (7 * i);
    if (!(b & 0x80))
      return true;
  }
  return false;
}

// Scans AV1 OBUs for the Dolby Vision ITU-T T.35 metadata OBU (country 0xB5,
// provider 0x003B, provider oriented code 0x00000800) and hands the payload,
// starting at the country code, to libdovi.
inline void AMLParseAv1DoviRpu(const uint8_t* data, size_t size, AMLFrameMetadata& meta)
{
  if (!data)
    return;

  size_t pos = 0;
  while (pos < size)
  {
    const uint8_t hdr = data[pos];
    if (hdr & 0x80)
      return;
    const int type = (hdr >> 3) & 0x0f;
    const bool extension = hdr & 0x04;
    const bool hasSize = hdr & 0x02;
    pos++;
    if (extension)
      pos++;
    if (pos >= size)
      return;
    uint64_t obuSize = 0;
    if (hasSize)
    {
      if (!AMLReadLeb128(data, size, pos, obuSize))
        return;
    }
    else
      obuSize = size - pos;
    if (obuSize > size - pos)
      return;

    if (type == 5) // OBU_METADATA
    {
      const size_t obuEnd = pos + obuSize;
      size_t p = pos;
      uint64_t metadataType = 0;
      if (AMLReadLeb128(data, obuEnd, p, metadataType) && metadataType == 4) // ITUT_T35
      {
        const size_t len = obuEnd - p;
        if (len >= 34 && data[p] == 0xb5 && data[p + 1] == 0x00 && data[p + 2] == 0x3b &&
            data[p + 3] == 0x00 && data[p + 4] == 0x00 && data[p + 5] == 0x08 &&
            data[p + 6] == 0x00)
        {
          AMLFillFromDoviRpu(dovi_parse_itu_t35_dovi_metadata_obu(data + p, len), meta);
          return;
        }
      }
    }
    pos += obuSize;
  }
}

#endif // HAVE_LIBDOVI && AML_FRAME_METADATA_PARSER
