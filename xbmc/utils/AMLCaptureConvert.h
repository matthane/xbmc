/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

extern "C"
{
#include <libavutil/pixfmt.h>
}

struct SwsContext;

//! How vdin1 codes the video layer
enum class AMLCaptureSignal
{
  SDR, //!< BT.709 limited Y'CbCr
  PQ, //!< BT.2020 limited Y'CbCr, PQ
  HLG, //!< BT.2020 limited Y'CbCr, HLG
  DV_TV_LED, //!< the Dolby Vision core3 input of a TV-led signal
  DV_LL, //!< the Dolby Vision core3 input in player-led output
  DV_HDR10, //!< the Dolby Vision core3 input in HDR10 output
  DV_SDR, //!< the Dolby Vision core3 input in SDR output
};

struct AMLCaptureColor
{
  AMLCaptureSignal signal{AMLCaptureSignal::SDR};
  //! the source luminance of the SDR tonemap in cd/m2
  float luminance{0.0f};
};

//! A view of pixels the caller owns; NV12 uses both planes, packed formats only the first.
struct AMLCaptureImage
{
  AVPixelFormat format{AV_PIX_FMT_NONE};
  int width{0};
  int height{0};
  uint8_t* data[2]{};
  int linesize[2]{};
};

//! A buffer kept between calls whose growth leaves the new elements uninitialized
template<typename T>
class CAMLCaptureScratch
{
public:
  void Resize(std::size_t count)
  {
    if (count <= m_size)
      return;
    m_data = std::make_unique_for_overwrite<T[]>(count);
    m_size = count;
  }
  T* Data() const { return m_data.get(); }

private:
  std::unique_ptr<T[]> m_data;
  std::size_t m_size{0};
};

//! \brief Converts vdin1 captures. Not thread safe; the buffers and contexts persist between
//! calls, the transfer tables for the process.
class CAMLCaptureConvert
{
public:
  CAMLCaptureConvert() = default;
  ~CAMLCaptureConvert();
  CAMLCaptureConvert(const CAMLCaptureConvert&) = delete;
  CAMLCaptureConvert& operator=(const CAMLCaptureConvert&) = delete;

  //! Copy the NV12 image src out of its mapping, scale it to the size of dst and convert it to
  //! SDR AV_PIX_FMT_BGRA, tonemapping HDR.
  bool Convert(const AMLCaptureImage& src,
               const AMLCaptureColor& color,
               const AMLCaptureImage& dst);

private:
  bool Stage(const AMLCaptureImage& src, int width, int height);
  const std::vector<float>& GetToneMap(float luminance);

  SwsContext* m_scaleContext{nullptr};
  CAMLCaptureScratch<uint8_t> m_source;
  CAMLCaptureScratch<uint16_t> m_yuv;
  std::vector<float> m_toneMap;
  float m_toneMapLuminance{0.0f};
};
