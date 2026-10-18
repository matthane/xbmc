/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "guilib/DispResource.h"
#include "utils/AMLCaptureConvert.h"
#include "utils/Geometry.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <tuple>
#include <utility>
#include <vector>

extern "C"
{
#include <libavutil/mastering_display_metadata.h>
}

namespace KODI
{
namespace RENDERING
{
namespace CAPTURE
{
class ICapturePixels;
}
} // namespace RENDERING
} // namespace KODI

//! What the renderer knows about the playing video
struct AMLCaptureSource
{
  bool dolbyVision{false};
  AVColorTransferCharacteristic transfer{AVCOL_TRC_UNSPECIFIED};
  bool hasDisplayMetadata{false};
  AVMasteringDisplayMetadata displayMetadata{};
  bool hasLightMetadata{false};
  AVContentLightMetadata lightMetadata{};
};

//! The vdin1 loopback inputs: the video layer, and the GUI layer, which is black over video
enum class AMLCaptureInput
{
  VIDEO,
  GUI,
};

//! A vdin1 frame, valid only inside CAMLCapture::Read
struct AMLCaptureFrame
{
  AMLCaptureImage image;
  AMLCaptureColor color;
};

struct AMLCaptureVideoSpec
{
  int width{0};
  int height{0};
  //! the visible video, normalized to the output; vdin1 holds it at the frame origin
  CRect rect;
  bool continuous{false};
};

//! \brief The process-wide capture session on the vdin1 V4L2 node: video after the VPP and the
//! DV and HDR cores, without the GUI, or the GUI alone while no video is presented.
//!
//! The renderer or a GUI capture owns the session. The stream starts on the first read and stops
//! when the last lease of its input ends, on OnLostDisplay and when the session is dropped. Only
//! OnLostDisplay and the destructor make ioctls on the calling thread; everything else runs on
//! the consumer thread.
class CAMLCapture : public IDispResource, public std::enable_shared_from_this<CAMLCapture>
{
public:
  using ReadFunc = std::function<bool(const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)>;

  //! the session, created when no renderer holds one
  static std::shared_ptr<CAMLCapture> Get();
  //! the session a renderer holds, if any
  static std::shared_ptr<CAMLCapture> GetActive();
  //! the largest capture size for the current output mode
  static void GetCaptureSize(int& width, int& height);

  CAMLCapture();
  ~CAMLCapture() override;

  void OnLostDisplay() override;
  void OnResetDisplay() override;

  void Configure(const AMLCaptureSource& source);
  void SetPresented();
  //! the renderer releases the session: no video is presented
  void ClearPresented() { m_presented = false; }
  bool IsPresented() const { return m_presented; }
  //! Render thread: the video destination in GUI coordinates
  void SetVideoRect(const CRect& rect);
  //! Render thread: the video destination on screen, normalized to the output
  CRect GetVisibleRect();

  //! Keeps the stream running until the last lease is released. A continuous consumer passes
  //! the stream size it reads at, which the stream keeps while the lease lasts.
  std::shared_ptr<void> Lease(AMLCaptureInput input = AMLCaptureInput::VIDEO,
                              int width = 0,
                              int height = 0);

  //! Consumer thread: take the newest frame of a stream of input at least width x height, or of
  //! the input's continuous size when larger, else wait for the next one, and run `use` on it. An
  //! input or size change restarts the stream. A wait ends after a bound of output frame periods.
  bool Read(AMLCaptureInput input, int width, int height, const ReadFunc& use);

  //! The crop of an NV12 image to a normalized rect, on even pixels for the chroma plane
  static AMLCaptureImage Crop(const AMLCaptureImage& image, const CRect& rect);

  //! Deferred BGRA pixels of the visible video, SDR BT.709 full range.
  std::shared_ptr<KODI::RENDERING::CAPTURE::ICapturePixels> CreateVideoPixels(
      const AMLCaptureVideoSpec& spec);

  //! Render thread: continuous BGRA pixels of the GUI, SDR BT.709 full range, read at each Lock.
  //! Empty while video is presented or when vdin1 gave no GUI in this output mode.
  static std::shared_ptr<KODI::RENDERING::CAPTURE::ICapturePixels> CreateGuiPixels(int width,
                                                                                   int height);

private:
  class CStream;

  void ReleaseLease(AMLCaptureInput input, int width, int height);
  //! m_lock must be held
  bool CanRead(AMLCaptureInput input) const;
  //! takes m_stream for Close; m_lock must be held
  std::shared_ptr<CStream> TakeStream();
  void Close(std::shared_ptr<CStream> stream);
  AMLCaptureColor GetSignal(const AMLCaptureSource& source) const;

  std::mutex m_lock;
  std::shared_ptr<CStream> m_stream;
  //! taken streams not closed yet; the node refuses a new input while one is open
  int m_closing{0};
  std::condition_variable m_closed;
  //! per AMLCaptureInput
  std::array<int, 2> m_leases{};
  std::vector<std::tuple<AMLCaptureInput, int, int>> m_continuousSizes;
  bool m_displayLost{false};
  //! another vdin1 user holds the node: no reopen before the next Configure
  bool m_busy{false};
  //! the last GUI open found the node busy; every GUI read retries it
  bool m_guiBusy{false};
  AMLCaptureSource m_source;
  CRect m_videoRect;
  std::atomic<bool> m_presented{false};

  //! serializes consumers; held across the frame wait, never taken by Stop
  std::mutex m_readLock;
  CAMLCaptureConvert m_convert;
};
