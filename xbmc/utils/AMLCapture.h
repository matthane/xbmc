/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "guilib/DispResource.h"
#include "rendering/capture/CaptureTypes.h"
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

struct AMLFrameMetadata;

//! What the renderer knows about the playing video, and the title peak of the frame on screen
struct AMLCaptureSource
{
  bool dolbyVision{false};
  AVColorTransferCharacteristic transfer{AVCOL_TRC_UNSPECIFIED};
  //! in cd/m2 from the HDR statics, 0 without them
  float titlePeak{0.0f};
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

//! A 10-bit vdin1 frame copied out of its stream
struct AMLCaptureShotFrame
{
  //! frame.image points into data
  std::unique_ptr<uint8_t[]> data;
  AMLCaptureFrame frame;
};

//! What only the read of a native screenshot knows about its pixels
struct AMLCaptureNative
{
  bool resolved{false};
  AMLCaptureColor color;
  int width{0};
  int height{0};

  //! once resolved, width x height RGB48BE pixels in the signal's own coding
  bool Describe(KODI::RENDERING::CAPTURE::CaptureResult& result) const;
};

struct AMLCaptureVideoSpec
{
  int width{0};
  int height{0};
  //! the visible video, normalized to the output; vdin1 holds it at the frame origin
  CRect rect;
  bool continuous{false};
  KODI::RENDERING::CAPTURE::CaptureScreenshot shot{
      KODI::RENDERING::CAPTURE::CaptureScreenshot::NONE};
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

  //! the session, created when no renderer or GUI capture holds one
  static std::shared_ptr<CAMLCapture> Get();
  //! the session a renderer or GUI capture holds, if any
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
  //! Consumer thread: copy out a frame of a one-shot 10-bit video stream. NATIVE reads the
  //! output size up to 2160p, else the largest capture size when vdin1 refuses that; STANDARD
  //! reads the largest capture size. The stream closes before this returns; a continuous reader
  //! reopens its own.
  bool ReadShot(KODI::RENDERING::CAPTURE::CaptureScreenshot shot, AMLCaptureShotFrame& frame);
  //! Consumer thread: run `use` on the frame of a shot, else, or when that fails, on a Read of
  //! the video at width x height.
  bool ReadVideo(KODI::RENDERING::CAPTURE::CaptureScreenshot shot,
                 int width,
                 int height,
                 const ReadFunc& use);

  //! The crop of an NV12 or full pack image to a normalized rect, on even pixels for the chroma
  static AMLCaptureImage Crop(const AMLCaptureImage& image, const CRect& rect);

  //! Deferred sRGB pixels of the visible video in result: BGRA, RGB24 for a STANDARD shot; a
  //! NATIVE shot describes its own.
  void CreateVideoPixels(const AMLCaptureVideoSpec& spec,
                         KODI::RENDERING::CAPTURE::CaptureResult& result);

  //! Render thread: continuous sRGB BGRA pixels of the GUI in result, read at each Lock. The
  //! caller checks that no video is presented. No pixels when vdin1 gave no GUI in this output
  //! mode.
  static void CreateGuiPixels(int width,
                              int height,
                              KODI::RENDERING::CAPTURE::CaptureResult& result);

private:
  class CStream;
  using StreamFunc =
      std::function<bool(CStream& stream, unsigned int index, const AMLCaptureFrame& frame)>;

  void ReleaseLease(AMLCaptureInput input, int width, int height);
  //! m_readLock must be held; `use` runs on buffer `index`, which is queued again after it.
  //! `refused` tells that vdin1 cannot give this input at this size. With `logCause`, it logs why
  //! it gets no frame; a native shot does not fall back
  bool ReadStream(AMLCaptureInput input,
                  uint32_t pixelFormat,
                  unsigned int buffers,
                  int width,
                  int height,
                  const StreamFunc& use,
                  bool& refused,
                  bool logCause = false);
  //! m_lock must be held
  bool CanRead(AMLCaptureInput input) const;
  //! m_lock must be held; false, after logging why the video cannot be read when `log`
  bool Unreadable(bool log) const;
  //! takes m_stream for Close; m_lock must be held
  std::shared_ptr<CStream> TakeStream();
  void Close(std::shared_ptr<CStream> stream);
  AMLCaptureColor GetSignal(const AMLCaptureSource& source) const;
  //! m_lock must be held
  void UpdateStatics(std::shared_ptr<const AMLFrameMetadata> meta);

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
  //! the frame metadata the title peak in m_source was decoded from
  std::shared_ptr<const AMLFrameMetadata> m_staticsMeta;
  CRect m_videoRect;
  std::atomic<bool> m_presented{false};

  //! serializes consumers; held across the frame wait, never taken by Stop
  std::mutex m_readLock;
  //! the last open failed on a node that was not busy: open failures log at debug until one
  //! succeeds
  bool m_openFailed{false};
  CAMLCaptureConvert m_convert;
};
