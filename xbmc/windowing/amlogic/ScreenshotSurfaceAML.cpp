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

constexpr int FULL_SPAN_8 = 255;
constexpr int CODE_SPAN_8_TO_16 = 257;
// the GUI composite peak is PQ-normalized luminance
constexpr float PQ_PEAK_NITS = 10000.0f;

// a normalized coordinate in pixels
int Edge(float v, int pixels)
{
  return std::clamp(static_cast<int>(std::lround(v * pixels)), 0, pixels);
}

AMLCaptureImage Offset(AMLCaptureImage image, int x, int y, int pixelBytes)
{
  image.data[0] += static_cast<std::ptrdiff_t>(y) * image.linesize[0] + x * pixelBytes;
  return image;
}

// fill(y, from, to) on the row spans of width x height around a w x h place at (x1, y1)
template<typename Fill>
void FillAround(int width, int height, int x1, int y1, int w, int h, const Fill& fill)
{
  for (int y = 0; y < height; y++)
  {
    if (y < y1 || y >= y1 + h)
      fill(y, 0, width);
    else
    {
      fill(y, 0, x1);
      fill(y, x1 + w, width);
    }
  }
}

struct GuiReadback
{
  std::unique_ptr<uint8_t[]> pixels;
  int width{0};
  int height{0};
  int stride{0};
  bool bgra{false};
  AMLCaptureCoding coding;
  //! the visible video, normalized to the output
  CRect video;
  int streamWidth{0};
  int streamHeight{0};

  const uint8_t* Row(int y) const
  {
    return CaptureSrcRow0(pixels.get(), stride, static_cast<unsigned int>(height)) +
           static_cast<std::ptrdiff_t>(y) * stride;
  }
  AMLCaptureImage Image() const
  {
    return AMLCaptureImage::Packed(bgra ? AV_PIX_FMT_BGRA : AV_PIX_FMT_RGBA, width, height,
                                   const_cast<uint8_t*>(Row(0)), stride);
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

    m_buffer = std::make_unique_for_overwrite<uint8_t[]>(static_cast<size_t>(m_gui.width) *
                                                         m_gui.height * 4);
    const auto session = m_session.lock();
    if (!session || !session->Read(m_gui.streamWidth, m_gui.streamHeight,
                                   [this](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
                                   { return Blend(frame, convert); }))
    {
      // a GUI in an HDR coding would pass for SDR
      if (m_gui.coding.transfer != AMLCaptureCoding::Transfer::SDR)
        m_buffer.reset();
      else
        CopyGui();
    }
    return m_buffer.get();
  }

private:
  // place the visible video, mapped to SDR, in the GUI's coding at its place on black, blend the
  // premultiplied GUI over it as the OSD blender does, then invert the coding to SDR, so the GUI
  // keeps its look and the video matches a video capture
  bool Blend(const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
  {
    const int width = m_gui.width;
    const int height = m_gui.height;
    const size_t size = static_cast<size_t>(width) * height * 4;

    const AMLCaptureCoding& coding = m_gui.coding;
    const AMLCaptureImage image = convert.GetCompositeImage(width, height);
    uint16_t* composite = reinterpret_cast<uint16_t*>(image.data[0]);

    // in an HDR coding the video under clear GUI goes straight to SDR, the value the inverse of
    // the coding would give
    const AMLCaptureImage overlay = m_gui.Image();
    const AMLCaptureImage dst =
        AMLCaptureImage::Packed(AV_PIX_FMT_RGBA, width, height, m_buffer.get(), width * 4);
    const bool hdr = coding.transfer != AMLCaptureCoding::Transfer::SDR;

    // vdin1 holds the visible video at the frame origin, at its size on screen
    const CRect& video = m_gui.video;
    const int x1 = Edge(video.x1, width);
    const int y1 = Edge(video.y1, height);
    AMLCaptureImage place = Offset(image, x1, y1, 8);
    place.width = Edge(video.x2, width) - x1;
    place.height = Edge(video.y2, height) - y1;
    const AMLCaptureImage overlayPlace = Offset(overlay, x1, y1, 4);
    const AMLCaptureImage dstPlace = Offset(dst, x1, y1, 4);
    if (place.width > 0 && place.height > 0 &&
        !convert.Convert(
            CAMLCapture::Crop(frame.image, CRect(0.0f, 0.0f, video.Width(), video.Height())),
            frame.color, coding, place, hdr ? &overlayPlace : nullptr, &dstPlace))
      return false;

    // the video place is converted whole; around it the GUI is over black
    FillAround(width, height, x1, y1, place.width, place.height,
               [&](int y, int from, int to)
               {
                 const size_t first = static_cast<size_t>(y) * width + from;
                 std::fill(composite + first * 4, composite + (first + to - from) * 4, 0);
                 uint8_t* out = m_buffer.get() + first * 4;
                 for (int x = from; hdr && x < to; x++, out += 4)
                 {
                   out[0] = 0;
                   out[1] = 0;
                   out[2] = 0;
                   out[3] = 255;
                 }
               });

    const int r = m_gui.bgra ? 2 : 0;
    const int b = m_gui.bgra ? 0 : 2;
    for (int y = 0; y < height; y++)
    {
      const uint8_t* gui = m_gui.Row(y);
      uint16_t* out = composite + static_cast<size_t>(y) * width * 4;
      for (int x = 0; x < width; x++, gui += 4, out += 4)
      {
        if (hdr && CAMLCaptureConvert::IsClear(gui))
          continue;
        const int alpha = gui[3];
        const int channel[3] = {gui[r], gui[1], gui[b]};
        for (int c = 0; c < 3; c++)
        {
          const int value =
              channel[c] * CODE_SPAN_8_TO_16 + (FULL_SPAN_8 - alpha) * out[c] / FULL_SPAN_8;
          out[c] = static_cast<uint16_t>(std::min(value, 65535));
        }
        out[3] = 65535;
      }
    }

    if (!hdr)
    {
      for (size_t i = 0; i < size; i++)
        m_buffer[i] =
            static_cast<uint8_t>((composite[i] + CODE_SPAN_8_TO_16 / 2) / CODE_SPAN_8_TO_16);
      return true;
    }

    convert.MapToSdr(image, coding, dst, &overlay);
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

void CScreenshotSurfaceAML::TakePixels(const CaptureSpec& spec, CaptureResult& result)
{
  using KODI::WINDOWING::AML::CWinSystemAmlogicGLESContext;

  const auto session = CAMLCapture::GetActive();
  auto* winSystem = dynamic_cast<CWinSystemAmlogicGLESContext*>(CServiceBroker::GetWinSystem());
  if (!session || !session->IsPresented() || !winSystem ||
      (m_format != AV_PIX_FMT_RGBA && m_format != AV_PIX_FMT_BGRA))
    return CScreenshotSurfaceGLES::TakePixels(spec, result);

  GuiReadback gui;
  gui.pixels.reset(TakeBuffer());
  gui.width = m_width;
  gui.height = m_height;
  gui.stride = m_stride;
  gui.bgra = m_format == AV_PIX_FMT_BGRA;
  const int transfer = winSystem->GetGuiCompositeTransfer();
  if (transfer == AVCOL_TRC_SMPTE2084)
    gui.coding.transfer = AMLCaptureCoding::Transfer::PQ;
  else if (transfer == AVCOL_TRC_ARIB_STD_B67)
    gui.coding.transfer = AMLCaptureCoding::Transfer::HLG;
  gui.coding.whiteNits = winSystem->GetGuiCompositePeak() * PQ_PEAK_NITS;
  gui.video = session->GetVisibleRect();
  CAMLCapture::GetCaptureSize(gui.streamWidth, gui.streamHeight);

  result.pixels = std::make_shared<CAMLCompositePixels>(session, session->Lease(), std::move(gui));
  result.width = static_cast<unsigned int>(m_width);
  result.height = static_cast<unsigned int>(m_height);
  result.stride = m_width * 4;
  result.format = AV_PIX_FMT_RGBA;
  result.color.primaries = AVCOL_PRI_BT709;
  result.color.transfer = AVCOL_TRC_IEC61966_2_1;
  result.color.range = AVCOL_RANGE_JPEG;
}
