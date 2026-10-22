/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "rendering/capture/CapturePixels.h"
#include "rendering/capture/CaptureTypes.h"

#include <cstdint>
#include <memory>

extern "C"
{
#include <libavutil/pixfmt.h>
}

class CWinSystemBase;

//! \brief Subsystem dependencies a capture readback needs, injected so the
//! surface does not reach for global state.
struct ScreenshotContext
{
  CWinSystemBase& winSystem;
};

class IScreenshotSurface
{
public:
  //! frees the buffer only when the caller did not take ownership via TakeBuffer
  virtual ~IScreenshotSurface() { delete[] m_buffer; }

  //! \brief Read back the current framebuffer only; the caller guarantees a
  //! fully rendered frame and render-thread context.
  virtual bool Read(const ScreenshotContext& ctx) { return false; }

  //! \brief Render thread: serve a capture without the framebuffer readback, e.g. from a
  //! display writeback. False leaves it to Read; true with no pixels fails the request.
  virtual bool CaptureDirect(const KODI::RENDERING::CAPTURE::CaptureSpec& spec,
                             KODI::RENDERING::CAPTURE::CaptureResult& result)
  {
    return false;
  }

  int GetWidth() const { return m_width; }
  int GetHeight() const { return m_height; }
  //! signed linesize; negative when the delivered rows are bottom-up
  int GetStride() const { return m_stride; }
  //! source coding of the delivered bytes, handed to the consumer's swscale
  AVPixelFormat GetFormat() const { return m_format; }
  unsigned char* GetBuffer() const { return m_buffer; }

  //! \brief Transfer buffer ownership to the caller.
  unsigned char* TakeBuffer()
  {
    unsigned char* buffer = m_buffer;
    m_buffer = nullptr;
    return buffer;
  }

  //! \brief Hand the pixels for spec to result with their size and format; a surface may defer
  //! the read to Lock(). result arrives tagged as the output; a surface whose pixels are coded
  //! otherwise retags it.
  virtual void TakePixels(const KODI::RENDERING::CAPTURE::CaptureSpec& spec,
                          KODI::RENDERING::CAPTURE::CaptureResult& result)
  {
    result.pixels = std::make_shared<KODI::RENDERING::CAPTURE::CHeapCapturePixels>(
        std::unique_ptr<uint8_t[]>(TakeBuffer()));
    result.width = static_cast<unsigned int>(m_width);
    result.height = static_cast<unsigned int>(m_height);
    result.stride = m_stride;
    result.format = m_format;
  }

protected:
  int m_width{0};
  int m_height{0};
  int m_stride{0};
  AVPixelFormat m_format{AV_PIX_FMT_NONE};
  unsigned char* m_buffer{nullptr};
};
