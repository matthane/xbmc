/*
 *  Copyright (C) 2026 Team CoreELEC
 *  This file is part of CoreELEC - https://coreelec.org
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "OverlayRendererDRM.h"
#include "SubtitleComposition.h"

#include "cores/VideoPlayer/DVDCodecs/Overlay/DVDOverlayImage.h"
#include "threads/CriticalSection.h"
#include "utils/Geometry.h"
#include "utils/log.h"
#include "ServiceBroker.h"
#include "windowing/GraphicContext.h"
#include "windowing/WinSystem.h"
#include "windowing/amlogic/WinSystemAmlogic.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

namespace OVERLAY
{

namespace
{
std::atomic<uint64_t> s_nextInstanceId{1};

struct SubtitleCanvas
{
  // PGS is normally authored at 1920x1080; the plane scales this canvas
  // to the display mode through its per-commit source and CRTC rectangles.
  unsigned int m_fbWidth{0};
  unsigned int m_fbHeight{0};
  CCriticalSection m_presMutex;
  bool m_kmsOpen{false};
  bool m_collecting{false};
  CSubtitleComposition m_composition;

  void WriteBoxLocked(void* dstMap, unsigned int dstStride, const int* box)
  {
    const int x0 = std::max(box[0], 0);
    const int y0 = std::max(box[1], 0);
    const int x1 = std::min(box[2], static_cast<int>(m_fbWidth));
    const int y1 = std::min(box[3], static_cast<int>(m_fbHeight));
    if (x0 >= x1 || y0 >= y1)
      return;
    const size_t spanBytes = static_cast<size_t>(x1 - x0) * 4;
    for (int y = y0; y < y1; ++y)
    {
      auto* row = static_cast<uint8_t*>(dstMap) + static_cast<size_t>(y) * dstStride;
      const uint32_t* src = m_composition.Pixels().data() +
                            static_cast<size_t>(y) * m_fbWidth + x0;
      memcpy(row + static_cast<size_t>(x0) * 4, src, spanBytes);
    }
  }

  bool EnsureOpen()
  {
    if (m_kmsOpen)
      return true;

    auto* ws = static_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem());
    if (!ws->ArmOSD2Plane())
      return false;

    void* map{nullptr};
    uint32_t stride{0}, w{0}, h{0}, bufferIndex{0};
    if (!ws->GetOSD2BackBuffer(&map, &stride, &w, &h, &bufferIndex))
      return false;

    m_fbWidth = w;
    m_fbHeight = h;
    m_composition.Reset(w, h);
    m_kmsOpen = true;

    CLog::LogF(LOGINFO, "osd2 canvas opened {}x{}", m_fbWidth, m_fbHeight);
    return true;
  }

  void PresentKmsLocked()
  {
    if (!m_collecting)
      return;
    m_collecting = false;

    if (m_composition.Compose() == CSubtitleComposition::Result::IDLE)
      return;

    auto* ws = static_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem());
    void* map{nullptr};
    uint32_t stride{0}, w{0}, h{0}, bufferIndex{0};
    if (!ws->GetOSD2BackBuffer(&map, &stride, &w, &h, &bufferIndex))
      return;

    const auto dirty = m_composition.CopyRect(bufferIndex);
    const int box[4] = {dirty.x0, dirty.y0, dirty.x1, dirty.y1};
    WriteBoxLocked(map, stride, box);

    if (ws->PresentOSD2Frame())
      m_composition.Commit(bufferIndex);
  }

  void ShutdownKmsLocked()
  {
    auto* ws = static_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem());
    ws->DisableOSD2();

    m_composition.Reset(m_fbWidth, m_fbHeight);
    m_collecting = false;
  }
};

SubtitleCanvas s_canvas;

} // namespace

void SubtitlePlaneFrameBegin(bool hasHdrImageOverlay)
{
  if (!CServiceBroker::GetWinSystem()->IsHdrSubtitlePlaneActive())
  {
    std::unique_lock lock(s_canvas.m_presMutex);
    if (s_canvas.m_kmsOpen &&
        (s_canvas.m_composition.HasPresentedContent() ||
         s_canvas.m_composition.DisablePending() || s_canvas.m_collecting))
      s_canvas.ShutdownKmsLocked();
    return;
  }

  if (hasHdrImageOverlay && !s_canvas.EnsureOpen())
    return;
  if (!s_canvas.m_kmsOpen)
    return;

  std::unique_lock lock(s_canvas.m_presMutex);
  if (s_canvas.m_composition.DisablePending() && !hasHdrImageOverlay)
  {
    s_canvas.ShutdownKmsLocked();
    return;
  }
  s_canvas.m_composition.BeginFrame(hasHdrImageOverlay);
  s_canvas.m_collecting = true;
}

void SubtitlePlaneFramePresent()
{
  if (!s_canvas.m_kmsOpen)
    return;
  std::unique_lock lock(s_canvas.m_presMutex);
  s_canvas.PresentKmsLocked();
}

void SubtitlePlaneFlush()
{
  if (!s_canvas.m_kmsOpen)
    return;
  std::unique_lock lock(s_canvas.m_presMutex);
  s_canvas.ShutdownKmsLocked();
}

void SubtitleCanvasReset()
{
  std::unique_lock lock(s_canvas.m_presMutex);
  if (!s_canvas.m_kmsOpen)
    return;

  s_canvas.ShutdownKmsLocked();
  s_canvas.m_kmsOpen = false;
  s_canvas.m_fbWidth = 0;
  s_canvas.m_fbHeight = 0;
}

COverlayDRM::COverlayDRM(const CDVDOverlayImage& o, CRect& rs)
  : m_instanceId(s_nextInstanceId.fetch_add(1, std::memory_order_relaxed))
{
  m_isHDROverlay = o.m_isHDROverlay;

  m_pixels = o.pixels;
  m_palette = o.palette;
  m_linesize = o.linesize;
  m_bmpW = o.width;
  m_bmpH = o.height;

  if (o.source_width > 0 && o.source_height > 0)
  {
    m_pos = POSITION_RELATIVE;
    m_x = (0.5f * o.width + o.x) / o.source_width;
    m_y = (0.5f * o.height + o.y) / o.source_height;

    const float subRatio{static_cast<float>(o.source_width) / o.source_height};
    const float vidRatio{rs.Width() / rs.Height()};

    // We always consider aligning 4/3 subtitles to the video,
    // for example SD DVB subtitles (4:3) must be stretched on fullhd video
    if (std::fabs(subRatio - vidRatio) < 0.001f || IsSquareResolution(subRatio))
    {
      m_align = ALIGN_VIDEO;
      m_width = static_cast<float>(o.width) / o.source_width;
      m_height = static_cast<float>(o.height) / o.source_height;
    }
    else
    {
      // We should have a re-encoded/cropped (removed black bars) video source.
      // Then we cannot align to video otherwise the subtitles will be deformed
      // better align to screen by keeping the aspect-ratio.
      m_align = ALIGN_SCREEN_AR;
      m_width = static_cast<float>(o.width);
      m_height = static_cast<float>(o.height);
      m_source_width = static_cast<float>(o.source_width);
      m_source_height = static_cast<float>(o.source_height);
    }
  }
  else
  {
    m_align = ALIGN_VIDEO;
    m_pos = POSITION_ABSOLUTE;
    m_x = static_cast<float>(o.x);
    m_y = static_cast<float>(o.y);
    m_width = static_cast<float>(o.width);
    m_height = static_cast<float>(o.height);
  }

  CLog::LogF(LOGDEBUG,
             "osd2 overlay: bmp {}x{} at {},{} source {}x{} -> align {} pos {} x/y {}/{} w/h {}/{}",
             o.width, o.height, o.x, o.y, o.source_width, o.source_height,
             m_align == ALIGN_VIDEO ? 0 : (m_align == ALIGN_SCREEN_AR ? 1 : 2),
             m_pos == POSITION_RELATIVE ? 0 : 1, m_x, m_y, m_width, m_height);
}

void COverlayDRM::Render(SRenderState& state)
{
  if (!s_canvas.EnsureOpen())
    return;

  float left, top, width, height;
  if (m_pos == POSITION_RELATIVE)
  {
    top = state.y - state.height * 0.5f;
    left = state.x - state.width * 0.5f;
    width = state.width;
    height = state.height;
  }
  else
  {
    left = state.x;
    top = state.y;
    width = state.width;
    height = state.height;
  }

  auto& gfx = CServiceBroker::GetWinSystem()->GetGfxContext();
  const float fx = static_cast<float>(s_canvas.m_fbWidth) / gfx.GetWidth();
  const float fy = static_cast<float>(s_canvas.m_fbHeight) / gfx.GetHeight();

  const int dstX = static_cast<int>(left * fx);
  const int dstY = static_cast<int>(top * fy);
  const int dstW = static_cast<int>(width * fx);
  const int dstH = static_cast<int>(height * fy);

  if (dstW <= 0 || dstH <= 0)
    return;

  const bool sizeChanged = dstW != m_stageW || dstH != m_stageH;
  if (sizeChanged)
  {
    BuildStage(dstW, dstH);
    ++m_stageRevision;
  }

  if (sizeChanged || dstX != m_stageX || dstY != m_stageY)
  {
    CLog::LogF(LOGDEBUG, "osd2 overlay dst {}x{} at {},{} (gui {}x{} -> plane {}x{})", dstW, dstH,
               dstX, dstY, gfx.GetWidth(), gfx.GetHeight(), s_canvas.m_fbWidth,
               s_canvas.m_fbHeight);
  }
  m_stageX = dstX;
  m_stageY = dstY;

  std::unique_lock lock(s_canvas.m_presMutex);
  s_canvas.m_composition.Submit(
      {m_instanceId, m_stageRevision, dstX, dstY, dstW, dstH,
       {dstX + m_bboxX0, dstY + m_bboxY0, dstX + m_bboxX0 + m_bboxW,
        dstY + m_bboxY0 + m_bboxH},
       m_stage.data(), m_stageW});
}

void COverlayDRM::BuildStage(int dstW, int dstH)
{
  // PGS uses an indexed palette; premultiply each palette entry once.
  uint32_t pmaPal[256] = {};
  const uint32_t* palette = m_palette.data();
  const size_t paletteSize = m_palette.size();
  const size_t n = std::min(paletteSize, static_cast<size_t>(256));
  for (size_t i = 0; i < n; ++i)
  {
    const uint32_t argb = palette[i];
    const uint32_t a = (argb >> 24) & 0xFF;
    if (a == 0)
      continue;
    pmaPal[i] = (a << 24) | ((((argb >> 16) & 0xFF) * a / 255) << 16) |
                ((((argb >> 8) & 0xFF) * a / 255) << 8) | (((argb & 0xFF) * a / 255));
  }
  m_stageW = dstW;
  m_stageH = dstH;
  m_stage.resize(static_cast<size_t>(dstW) * dstH);
  m_bboxW = 0;
  m_bboxH = 0;

  if (m_bmpW <= 0 || m_bmpH <= 0 || paletteSize == 0)
    return;

  const bool identity = (dstW == m_bmpW) && (dstH == m_bmpH);
  std::vector<int> colLUT;
  std::vector<int> rowLUT;
  if (!identity)
  {
    colLUT.resize(dstW);
    for (int x = 0; x < dstW; ++x)
      colLUT[x] = (x * m_bmpW) / dstW;
    rowLUT.resize(dstH);
    for (int y = 0; y < dstH; ++y)
      rowLUT[y] = (y * m_bmpH) / dstH;
  }

  int bx0 = dstW, by0 = dstH, bx1 = -1, by1 = -1;
  for (int y = 0; y < dstH; ++y)
  {
    const int srcY = identity ? y : rowLUT[y];
    const uint8_t* srcRow = m_pixels.data() + static_cast<size_t>(srcY) * m_linesize;
    uint32_t* dstRow = m_stage.data() + static_cast<size_t>(y) * dstW;
    int fx = -1, lx = -1;
    for (int x = 0; x < dstW; ++x)
    {
      const uint32_t v = pmaPal[srcRow[identity ? x : colLUT[x]]];
      dstRow[x] = v;
      if (v)
      {
        if (fx < 0)
          fx = x;
        lx = x;
      }
    }
    if (fx >= 0)
    {
      bx0 = std::min(bx0, fx);
      bx1 = std::max(bx1, lx);
      by0 = std::min(by0, y);
      by1 = std::max(by1, y);
    }
  }
  if (bx1 >= 0)
  {
    m_bboxX0 = bx0;
    m_bboxY0 = by0;
    m_bboxW = bx1 - bx0 + 1;
    m_bboxH = by1 - by0 + 1;
  }
}

} // namespace OVERLAY
