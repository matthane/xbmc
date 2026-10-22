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

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
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
//! DV and HDR cores, without the GUI.
//!
//! The renderer owns the session. The stream starts on the first read and stops when the last
//! lease ends, on OnLostDisplay and when the renderer drops the session. Only OnLostDisplay and
//! the destructor make ioctls on the calling thread; everything else runs on the consumer thread.
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
  std::shared_ptr<void> Lease(int width = 0, int height = 0);

  //! Consumer thread: take the newest frame of a stream of at least width x height, or of the
  //! continuous size when larger, else wait for the next one, and run `use` on it. A size change
  //! restarts the stream. A wait ends after a bound of output frame periods.
  bool Read(int width, int height, const ReadFunc& use);

  //! The crop of an NV12 image to a normalized rect, on even pixels for the chroma plane
  static AMLCaptureImage Crop(const AMLCaptureImage& image, const CRect& rect);

  //! Deferred sRGB BGRA pixels of the visible video in result.
  void CreateVideoPixels(const AMLCaptureVideoSpec& spec,
                         KODI::RENDERING::CAPTURE::CaptureResult& result);

private:
  class CStream;

  void ReleaseLease(int width, int height);
  //! m_lock must be held
  bool CanRead() const;
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
  int m_leases{0};
  std::vector<std::pair<int, int>> m_continuousSizes;
  bool m_displayLost{false};
  //! another vdin1 user holds the node: no reopen before the next Configure
  bool m_busy{false};
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
