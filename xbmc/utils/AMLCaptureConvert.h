/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include <cstdint>
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

//! The coding of an R'G'B' image: SDR BT.709, or BT.2020 with SDR white at whiteNits
struct AMLCaptureCoding
{
  enum class Transfer
  {
    SDR,
    PQ,
    HLG,
  };
  Transfer transfer{Transfer::SDR};
  float whiteNits{0.0f};
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
  //! SDR, tonemapping HDR, then to coding as the GUI places SDR. dst is AV_PIX_FMT_BGRA or
  //! AV_PIX_FMT_RGBA64LE.
  bool Convert(const AMLCaptureImage& src,
               const AMLCaptureColor& color,
               const AMLCaptureCoding& coding,
               const AMLCaptureImage& dst);

private:
  const std::vector<float>& GetToneMap(float luminance);

  SwsContext* m_scaleContext{nullptr};
  std::vector<uint8_t> m_source;
  std::vector<uint16_t> m_yuv;
  std::vector<float> m_toneMap;
  float m_toneMapLuminance{0.0f};
};
