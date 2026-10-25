/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
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
  //! an output mode display-mapped the signal, so it is not the content
  bool displayMapped{false};
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

constexpr int AML_FULL_PACK_PAIR_BYTES = 5;

//! A view of pixels the caller owns; NV12 uses both planes, packed formats only the first.
struct AMLCaptureImage
{
  AVPixelFormat format{AV_PIX_FMT_NONE};
  int width{0};
  int height{0};
  uint8_t* data[2]{};
  int linesize[2]{};
  //! vdin1's 10-bit 4:2:2 full pack, which has no AVPixelFormat: each pixel pair is one
  //! little-endian 40-bit group of Cb, Y, Cr and Y, 10 bits each
  bool fullPack{false};
  //! full pack chroma is the even pixel's, else the average of the pair
  bool cositedChroma{false};

  static AMLCaptureImage Packed(
      AVPixelFormat format, int width, int height, uint8_t* data, int linesize)
  {
    AMLCaptureImage image;
    image.format = format;
    image.width = width;
    image.height = height;
    image.data[0] = data;
    image.linesize[0] = linesize;
    return image;
  }
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

  //! Copy the NV12 image src out of its mapping, or unpack a full pack src, scale it to the size
  //! of dst and convert it to SDR, tonemapping HDR, then to coding as the GUI places SDR. dst is
  //! AV_PIX_FMT_BGRA, AV_PIX_FMT_RGB24 or AV_PIX_FMT_RGBA64LE. Where gui, a premultiplied 8-bit
  //! overlay of the size of dst, is clear, the pixel goes to sdr, AV_PIX_FMT_RGBA of that size,
  //! in SDR instead.
  bool Convert(const AMLCaptureImage& src,
               const AMLCaptureColor& color,
               const AMLCaptureCoding& coding,
               const AMLCaptureImage& dst,
               const AMLCaptureImage* gui = nullptr,
               const AMLCaptureImage* sdr = nullptr);

  //! Unpack the full pack src to dst, an AV_PIX_FMT_RGB48BE image of the same size, in the
  //! signal's own coding without the tonemap: SDR, PQ and HLG as vdin1 holds them, Dolby Vision
  //! decoded to PQ BT.2020.
  static bool ConvertNative(const AMLCaptureImage& src,
                            const AMLCaptureColor& color,
                            const AMLCaptureImage& dst);
  static AMLCaptureCoding::Transfer GetNativeTransfer(AMLCaptureSignal signal);
  //! Recode a full-range R'G'B' pixel between two codings through linear light
  static void Recode(const float in[3],
                     const AMLCaptureCoding& from,
                     const AMLCaptureCoding& to,
                     float out[3]);
  //! Scale src, a four-channel 8-bit image whose rows may run bottom-up, to the size of dst.
  static bool Scale(const AMLCaptureImage& src, const AMLCaptureImage& dst);

  //! Map src, an AV_PIX_FMT_RGBA64LE image in an HDR coding, to SDR AV_PIX_FMT_RGBA of the same
  //! size by the inverse of the coding, as the GUI places SDR. Pixels where gui is clear are left.
  void MapToSdr(const AMLCaptureImage& src,
                const AMLCaptureCoding& coding,
                const AMLCaptureImage& dst,
                const AMLCaptureImage* gui);

  //! A premultiplied 8-bit pixel that leaves what is under it unchanged
  static bool IsClear(const uint8_t* pixel)
  {
    uint32_t value = 0;
    std::memcpy(&value, pixel, sizeof(value));
    return value == 0;
  }

  //! An AV_PIX_FMT_RGBA64LE work image of width x height, reused between calls.
  AMLCaptureImage GetCompositeImage(int width, int height);

private:
  bool Stage(const AMLCaptureImage& src, int width, int height);
  const std::vector<float>& GetToneMap(float luminance);

  SwsContext* m_scaleContext{nullptr};
  CAMLCaptureScratch<uint8_t> m_source;
  CAMLCaptureScratch<uint16_t> m_yuv;
  CAMLCaptureScratch<uint16_t> m_unpacked;
  CAMLCaptureScratch<uint16_t> m_composite;
  std::vector<float> m_toneMap;
  float m_toneMapLuminance{0.0f};
};
