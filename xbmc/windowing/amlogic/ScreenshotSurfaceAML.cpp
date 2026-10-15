/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "ScreenshotSurfaceAML.h"

#include "ServiceBroker.h"
#include "rendering/capture/CapturePixels.h"
#include "utils/AMLCapture.h"
#include "utils/Screenshot.h"
#include "windowing/amlogic/WinSystemAmlogicGLESContext.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <mutex>

using namespace KODI::RENDERING::CAPTURE;

namespace
{

// 8-bit limited range: black at 16, 219 codes to white; 16-bit codes are 257 per 8-bit code
constexpr int LIMITED_BLACK_8 = 16;
constexpr int LIMITED_SPAN_8 = 219;
constexpr int FULL_SPAN_8 = 255;
constexpr int CODE_SCALE_8_TO_16 = 257;
constexpr int LIMITED_BLACK_16 = LIMITED_BLACK_8 * CODE_SCALE_8_TO_16;
// the GUI composite peak is PQ-normalized luminance
constexpr float PQ_PEAK_NITS = 10000.0f;

uint16_t ToLimited(uint16_t v)
{
  return static_cast<uint16_t>(LIMITED_BLACK_16 + v * LIMITED_SPAN_8 / FULL_SPAN_8);
}

uint16_t ToFull(uint16_t v)
{
  return static_cast<uint16_t>(
      std::clamp((v - LIMITED_BLACK_16) * FULL_SPAN_8 / LIMITED_SPAN_8, 0, 65535));
}

struct GuiReadback
{
  std::unique_ptr<uint8_t[]> pixels;
  int width{0};
  int height{0};
  int stride{0};
  bool bgra{false};
  //! the transfer the GUI is coded in, 0 for SDR
  int transfer{0};
  //! SDR white of an HDR GUI in cd/m2
  float whiteNits{0.0f};
  bool limited{false};
  //! the visible video, normalized to the output
  CRect video;
  int streamWidth{0};
  int streamHeight{0};

  const uint8_t* Row(int y) const
  {
    return CaptureSrcRow0(pixels.get(), stride, static_cast<unsigned int>(height)) +
           static_cast<std::ptrdiff_t>(y) * stride;
  }
};

class CAMLCompositePixels : public ICapturePixels
{
public:
  CAMLCompositePixels(std::weak_ptr<CAMLCapture> session,
                      std::shared_ptr<void> lease,
                      GuiReadback gui)
    : m_session(std::move(session)),
      m_lease(std::move(lease)),
      m_gui(std::move(gui))
  {
  }

  const uint8_t* Lock() override
  {
    std::lock_guard lock(m_lock);

    if (m_read)
      return m_buffer.get();
    m_read = true;

    m_buffer = std::make_unique<uint8_t[]>(static_cast<size_t>(m_gui.width) * m_gui.height * 4);
    const auto session = m_session.lock();
    if (!session || !session->Read(m_gui.streamWidth, m_gui.streamHeight,
                                   [this](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
                                   { return Blend(frame, convert); }))
      CopyGui();
    return m_buffer.get();
  }

private:
  // place the visible video, mapped to SDR, in the GUI's coding at its place on black, blend the
  // premultiplied GUI over it as the OSD blender does, then invert the coding to SDR in the GUI's
  // range, so the GUI keeps its look and the video matches a video capture
  bool Blend(const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
  {
    const int width = m_gui.width;
    const int height = m_gui.height;
    const size_t size = static_cast<size_t>(width) * height * 4;

    AMLCaptureCoding coding;
    if (m_gui.transfer == AVCOL_TRC_SMPTE2084)
      coding.transfer = AMLCaptureCoding::Transfer::PQ;
    else if (m_gui.transfer == AVCOL_TRC_ARIB_STD_B67)
      coding.transfer = AMLCaptureCoding::Transfer::HLG;
    else if (m_gui.transfer != 0)
      return false;
    coding.whiteNits = m_gui.whiteNits;
    if (coding.transfer != AMLCaptureCoding::Transfer::SDR && coding.whiteNits <= 0)
      return false;

    const AMLCaptureImage image = convert.GetCompositeImage(width, height);
    uint16_t* composite = reinterpret_cast<uint16_t*>(image.data[0]);
    std::fill(composite, composite + size, 0);

    // vdin1 holds the visible video at the frame origin, at its size on screen
    const auto edge = [](float v, int pixels)
    { return std::clamp(static_cast<int>(std::lround(v * pixels)), 0, pixels); };
    const CRect& video = m_gui.video;
    AMLCaptureImage place = image;
    const int x1 = edge(video.x1, width);
    const int y1 = edge(video.y1, height);
    place.width = edge(video.x2, width) - x1;
    place.height = edge(video.y2, height) - y1;
    place.data[0] += static_cast<std::ptrdiff_t>(y1) * image.linesize[0] + x1 * 8;
    if (place.width > 0 && place.height > 0 &&
        !convert.Convert(
            CAMLCapture::Crop(frame.image, CRect(0.0f, 0.0f, video.Width(), video.Height())),
            frame.color, coding, place))
      return false;

    const int r = m_gui.bgra ? 2 : 0;
    const int b = m_gui.bgra ? 0 : 2;
    for (int y = 0; y < height; y++)
    {
      const uint8_t* gui = m_gui.Row(y);
      uint16_t* out = composite + static_cast<size_t>(y) * width * 4;
      for (int x = 0; x < width; x++, gui += 4, out += 4)
      {
        const int alpha = gui[3];
        const int channel[3] = {gui[r], gui[1], gui[b]};
        for (int c = 0; c < 3; c++)
        {
          const int video = m_gui.limited ? ToLimited(out[c]) : out[c];
          const int value =
              channel[c] * CODE_SCALE_8_TO_16 + (FULL_SPAN_8 - alpha) * video / FULL_SPAN_8;
          out[c] = static_cast<uint16_t>(std::min(value, 65535));
        }
        out[3] = 65535;
      }
    }

    if (coding.transfer == AMLCaptureCoding::Transfer::SDR)
    {
      for (size_t i = 0; i < size; i++)
        m_buffer[i] =
            static_cast<uint8_t>((composite[i] + CODE_SCALE_8_TO_16 / 2) / CODE_SCALE_8_TO_16);
      return true;
    }

    if (m_gui.limited)
      std::transform(composite, composite + size, composite, ToFull);

    AMLCaptureImage dst;
    dst.format = AV_PIX_FMT_RGBA;
    dst.width = width;
    dst.height = height;
    dst.data[0] = m_buffer.get();
    dst.linesize[0] = width * 4;
    convert.MapToSdr(image, coding, dst);

    if (m_gui.limited)
    {
      for (size_t i = 0; i < size; i++)
      {
        if (i % 4 != 3)
          m_buffer[i] =
              static_cast<uint8_t>(LIMITED_BLACK_8 + m_buffer[i] * LIMITED_SPAN_8 / FULL_SPAN_8);
      }
    }
    return true;
  }

  void CopyGui()
  {
    for (int y = 0; y < m_gui.height; y++)
    {
      const uint8_t* gui = m_gui.Row(y);
      uint8_t* out = m_buffer.get() + static_cast<size_t>(y) * m_gui.width * 4;
      std::memcpy(out, gui, static_cast<size_t>(m_gui.width) * 4);
      if (m_gui.bgra)
      {
        for (int x = 0; x < m_gui.width; x++)
          std::swap(out[x * 4], out[x * 4 + 2]);
      }
    }
  }

  std::weak_ptr<CAMLCapture> m_session;
  std::shared_ptr<void> m_lease;
  GuiReadback m_gui;
  std::mutex m_lock;
  bool m_read{false};
  std::unique_ptr<uint8_t[]> m_buffer;
};

} // namespace

void CScreenshotSurfaceAML::Register()
{
  CScreenShot::Register(CScreenshotSurfaceAML::CreateSurface);
}

std::unique_ptr<IScreenshotSurface> CScreenshotSurfaceAML::CreateSurface()
{
  return std::make_unique<CScreenshotSurfaceAML>();
}

std::shared_ptr<ICapturePixels> CScreenshotSurfaceAML::TakePixels()
{
  using KODI::WINDOWING::AML::CWinSystemAmlogicGLESContext;

  const auto session = CAMLCapture::GetActive();
  auto* winSystem = dynamic_cast<CWinSystemAmlogicGLESContext*>(CServiceBroker::GetWinSystem());
  if (!session || !session->IsPresented() || !winSystem ||
      (m_format != AV_PIX_FMT_RGBA && m_format != AV_PIX_FMT_BGRA))
    return CScreenshotSurfaceGLES::TakePixels();

  GuiReadback gui;
  gui.pixels.reset(TakeBuffer());
  gui.width = m_width;
  gui.height = m_height;
  gui.stride = m_stride;
  gui.bgra = m_format == AV_PIX_FMT_BGRA;
  gui.transfer = winSystem->GetGuiCompositeTransfer();
  gui.whiteNits = winSystem->GetGuiCompositePeak() * PQ_PEAK_NITS;
  gui.limited = winSystem->UseLimitedColor();
  gui.video = session->GetVisibleRect();
  CAMLCapture::GetCaptureSize(gui.streamWidth, gui.streamHeight);

  m_format = AV_PIX_FMT_RGBA;
  m_stride = m_width * 4;
  return std::make_shared<CAMLCompositePixels>(session, session->Lease(), std::move(gui));
}
