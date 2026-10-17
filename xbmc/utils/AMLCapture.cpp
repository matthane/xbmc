/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "utils/AMLCapture.h"

#include "ServiceBroker.h"
#include "cores/VideoPlayer/VideoRenderers/VideoShaders/ToneMappers.h"
#include "jobs/JobManager.h"
#include "utils/log.h"
#include "windowing/GraphicContext.h"
#include "windowing/amlogic/WinSystemAmlogic.h"

#include "platform/linux/SysfsPath.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace
{

// the first frame after STREAMON came 3.0 to 4.97 output periods later on every tested SoC
constexpr int FRAME_WAIT_PERIODS = 10;
// vdin1 has 8 canvas ids and an NV12 buffer takes two
constexpr unsigned int CAPTURE_BUFFERS = 4;
// the vdin1 line buffer is 1920 pixels wide; a 2160p output is downscaled 2:1
constexpr int MAX_CAPTURE_WIDTH = 1920;
constexpr int MAX_CAPTURE_HEIGHT = 1080;
// vdin strides each line to 64 pixels but sizes the buffer from the width
constexpr int CAPTURE_WIDTH_ALIGN = 64;
// DV player-led, dv2hdr and sdr2hdr frames are already display-mapped
constexpr float MAPPED_PEAK_NITS = 1000.0f;
// the grade of a Dolby Vision TV-led signal when the stream carries no mastering metadata
constexpr float TVLED_PEAK_NITS = 4000.0f;
// the nominal display of ITU-R BT.2100 HLG
constexpr float HLG_PEAK_NITS = 1000.0f;
constexpr uint32_t BUFFER_TYPE = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

// amdv dv_mode and dv_ll_policy values (dolby_vision.h, amdv.h)
enum class DvMode
{
  IPT = 0,
  IPT_TUNNEL = 1,
  HDR10 = 2,
  SDR10 = 3,
  SDR8 = 4,
};
constexpr int DV_LL_YUV422 = 1;

// amvecm hdr_conversion_ctrl values (HDR_OUTPUT_MODE_* in amvecm.h)
enum class HdrOutput
{
  HDR10 = 1,
  HLG = 2,
  HDR10PLUS = 3,
  CUVA_HDR = 4,
  CUVA_HLG = 5,
};

struct AMLCaptureDevice
{
  std::string path;
  int input{-1};
};

AMLCaptureDevice Discover()
{
  AMLCaptureDevice device;

  // only vdin nodes are opened: other V4L2 drivers allocate on open
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator("/sys/class/video4linux", ec))
  {
    CSysfsPath name{entry.path().string() + "/name"};
    if (name.Get<std::string>().value_or("") != "vdin_video")
      continue;

    const std::string path = "/dev/" + entry.path().filename().string();
    const int fd = open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
      continue;

    v4l2_capability cap{};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
        std::string(reinterpret_cast<const char*>(cap.bus_info)) == "vdin1 v4l2")
    {
      v4l2_input input{};
      for (input.index = 0; ioctl(fd, VIDIOC_ENUMINPUT, &input) == 0; input.index++)
      {
        if (std::string(reinterpret_cast<const char*>(input.name)) == "VIU1_WB0_VD1")
        {
          device.path = path;
          device.input = static_cast<int>(input.index);
          break;
        }
      }
    }
    close(fd);
    if (device.input >= 0)
      break;
  }

  if (device.input >= 0)
    CLog::Log(LOGINFO, "CAMLCapture::{}: video capture on {} input {}", __FUNCTION__, device.path,
              device.input);
  else
    CLog::Log(LOGINFO, "CAMLCapture::{}: no vdin1 video capture node", __FUNCTION__);
  return device;
}

const AMLCaptureDevice& FindDevice()
{
  static const AMLCaptureDevice device = Discover();
  return device;
}

bool HasPeak(const AMLCaptureSource& source)
{
  return (source.hasDisplayMetadata && source.displayMetadata.has_luminance) ||
         (source.hasLightMetadata && source.lightMetadata.MaxCLL);
}

float GetLuminance(const AMLCaptureSource& source)
{
  return CToneMappers::GetLuminanceValue(source.hasDisplayMetadata, source.displayMetadata,
                                         source.hasLightMetadata, source.lightMetadata);
}

std::mutex s_sessionLock;
std::weak_ptr<CAMLCapture> s_session;

} // namespace

class CAMLCapture::CStream
{
public:
  enum class Wake
  {
    FRAME,
    STOPPED,
    TIMEOUT,
  };

  //! `busy` tells whether another vdin1 user holds the node
  static std::shared_ptr<CStream> Open(int width, int height, bool& busy);
  ~CStream();

  void Stop();
  bool Matches(int width, int height) const { return width == m_width && height == m_height; }
  //! Takes the newest frame vdin1 finished since the last dequeue, false when there is none or
  //! when that dequeue is older than `maxAge`
  bool TakeNewest(std::chrono::milliseconds maxAge, unsigned int& index);
  Wake Wait(int timeoutMs, unsigned int& index);
  bool Queue(unsigned int index);
  AMLCaptureImage Image(unsigned int index) const;

private:
  bool Dequeue(unsigned int& index);
  bool HasFrame(int timeoutMs, bool& stopped);

  int m_fd{-1};
  int m_eventFd{-1};
  int m_width{0};
  int m_height{0};
  bool m_streaming{false};
  std::atomic<bool> m_stopped{false};
  std::vector<std::pair<void*, size_t>> m_maps;
  std::chrono::steady_clock::time_point m_dequeued;
};

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::CStream::Open(int width, int height, bool& busy)
{
  const AMLCaptureDevice& device = FindDevice();
  if (device.input < 0)
    return {};

  auto stream = std::make_shared<CStream>();
  stream->m_width = width;
  stream->m_height = height;
  stream->m_fd = open(device.path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  stream->m_eventFd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (stream->m_fd < 0 || stream->m_eventFd < 0)
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: cannot open {}: {}", __FUNCTION__, device.path,
              strerror(errno));
    return {};
  }

  int input = device.input;
  if (ioctl(stream->m_fd, VIDIOC_S_INPUT, &input) < 0)
  {
    busy = errno == EBUSY;
    CLog::Log(LOGWARNING, "CAMLCapture::{}: input {} unavailable: {}", __FUNCTION__, input,
              strerror(errno));
    return {};
  }

  v4l2_format format{};
  format.type = BUFFER_TYPE;
  format.fmt.pix_mp.width = width;
  format.fmt.pix_mp.height = height;
  format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
  format.fmt.pix_mp.field = V4L2_FIELD_NONE;
  format.fmt.pix_mp.num_planes = 1;
  const size_t frameSize = static_cast<size_t>(width) * height * 3 / 2;
  if (ioctl(stream->m_fd, VIDIOC_S_FMT, &format) < 0 ||
      format.fmt.pix_mp.width != static_cast<uint32_t>(width) ||
      format.fmt.pix_mp.height != static_cast<uint32_t>(height) ||
      format.fmt.pix_mp.plane_fmt[0].sizeimage < frameSize)
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: NV12 {}x{} refused", __FUNCTION__, width, height);
    return {};
  }

  v4l2_requestbuffers request{};
  request.count = CAPTURE_BUFFERS;
  request.type = BUFFER_TYPE;
  request.memory = V4L2_MEMORY_MMAP;
  if (ioctl(stream->m_fd, VIDIOC_REQBUFS, &request) < 0 || request.count == 0)
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no capture buffers: {}", __FUNCTION__, strerror(errno));
    return {};
  }

  for (unsigned int i = 0; i < request.count; i++)
  {
    v4l2_plane plane{};
    v4l2_buffer buffer{};
    buffer.index = i;
    buffer.type = BUFFER_TYPE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.m.planes = &plane;
    buffer.length = 1;
    if (ioctl(stream->m_fd, VIDIOC_QUERYBUF, &buffer) < 0 || plane.length < frameSize)
      return {};

    void* map =
        mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, stream->m_fd, plane.m.mem_offset);
    if (map == MAP_FAILED)
      return {};
    stream->m_maps.emplace_back(map, plane.length);
  }

  for (unsigned int i = 0; i < stream->m_maps.size(); i++)
  {
    if (!stream->Queue(i))
      return {};
  }

  uint32_t type = BUFFER_TYPE;
  if (ioctl(stream->m_fd, VIDIOC_STREAMON, &type) < 0)
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: stream start failed: {}", __FUNCTION__,
              strerror(errno));
    return {};
  }
  stream->m_streaming = true;

  CLog::Log(LOGINFO, "CAMLCapture::{}: started {}x{}", __FUNCTION__, width, height);
  return stream;
}

CAMLCapture::CStream::~CStream()
{
  Stop();
  for (const auto& [map, length] : m_maps)
    munmap(map, length);
  if (m_fd >= 0)
    close(m_fd);
  if (m_eventFd >= 0)
    close(m_eventFd);
}

void CAMLCapture::CStream::Stop()
{
  if (m_stopped.exchange(true))
    return;

  if (m_eventFd >= 0)
  {
    const uint64_t wake = 1;
    if (write(m_eventFd, &wake, sizeof(wake)) < 0)
      CLog::Log(LOGDEBUG, "CAMLCapture::{}: wake failed: {}", __FUNCTION__, strerror(errno));
  }

  if (m_streaming)
  {
    // vdin serializes ioctls on the device lock, so this is safe beside a waiting reader
    uint32_t type = BUFFER_TYPE;
    ioctl(m_fd, VIDIOC_STREAMOFF, &type);
    CLog::Log(LOGINFO, "CAMLCapture::{}: stopped", __FUNCTION__);
  }
}

bool CAMLCapture::CStream::Queue(unsigned int index)
{
  v4l2_plane plane{};
  v4l2_buffer buffer{};
  buffer.index = index;
  buffer.type = BUFFER_TYPE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.m.planes = &plane;
  buffer.length = 1;
  return ioctl(m_fd, VIDIOC_QBUF, &buffer) == 0;
}

bool CAMLCapture::CStream::Dequeue(unsigned int& index)
{
  v4l2_plane plane{};
  v4l2_buffer buffer{};
  buffer.type = BUFFER_TYPE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.m.planes = &plane;
  buffer.length = 1;
  if (ioctl(m_fd, VIDIOC_DQBUF, &buffer) < 0)
    return false;

  m_dequeued = std::chrono::steady_clock::now();
  index = buffer.index;
  return true;
}

bool CAMLCapture::CStream::HasFrame(int timeoutMs, bool& stopped)
{
  pollfd fds[] = {{m_fd, POLLIN, 0}, {m_eventFd, POLLIN, 0}};
  int ret = -1;
  do
    ret = poll(fds, 2, timeoutMs);
  while (ret < 0 && errno == EINTR);

  stopped = m_stopped || (fds[1].revents & POLLIN);
  return ret > 0 && !stopped && (fds[0].revents & POLLIN) && !(fds[0].revents & POLLERR);
}

bool CAMLCapture::CStream::TakeNewest(std::chrono::milliseconds maxAge, unsigned int& index)
{
  const auto age = std::chrono::steady_clock::now() - m_dequeued;

  // a DQBUF on an empty queue logs in the driver, so every one waits for poll
  bool stopped = false;
  size_t done = 0;
  unsigned int next = 0;
  while (HasFrame(0, stopped) && Dequeue(next))
  {
    if (done++ > 0)
      Queue(index);
    index = next;
  }
  if (done == 0)
    return false;

  // vdin1 hands each frame over a vsync late and drops frames once the buffers the reader does
  // not hold are done, so after that many periods since the last dequeue the newest may be old
  if (stopped || age >= maxAge)
  {
    Queue(index);
    return false;
  }
  return true;
}

CAMLCapture::CStream::Wake CAMLCapture::CStream::Wait(int timeoutMs, unsigned int& index)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  for (;;)
  {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end - std::chrono::steady_clock::now())
                          .count();
    bool stopped = false;
    if (!HasFrame(static_cast<int>(std::max<int64_t>(left, 0)), stopped))
      return stopped ? Wake::STOPPED : Wake::TIMEOUT;

    if (Dequeue(index))
      return Wake::FRAME;
    if (left <= 0)
      return Wake::TIMEOUT;
  }
}

AMLCaptureImage CAMLCapture::CStream::Image(unsigned int index) const
{
  // single-plane NV12: the chroma plane follows the luma plane at the same stride
  AMLCaptureImage image;
  image.format = AV_PIX_FMT_NV12;
  image.width = m_width;
  image.height = m_height;
  image.data[0] = static_cast<uint8_t*>(m_maps[index].first);
  image.data[1] = image.data[0] + static_cast<size_t>(m_width) * m_height;
  image.linesize[0] = m_width;
  image.linesize[1] = m_width;
  return image;
}

std::shared_ptr<CAMLCapture> CAMLCapture::Get()
{
  std::lock_guard lock(s_sessionLock);

  auto session = s_session.lock();
  if (!session)
  {
    session = std::make_shared<CAMLCapture>();
    s_session = session;
  }
  return session;
}

std::shared_ptr<CAMLCapture> CAMLCapture::GetActive()
{
  std::lock_guard lock(s_sessionLock);

  return s_session.lock();
}

void CAMLCapture::GetCaptureSize(int& width, int& height)
{
  const RESOLUTION_INFO screen = CServiceBroker::GetWinSystem()->GetGfxContext().GetResInfo();
  const int screenWidth = std::max(screen.iScreenWidth, 1);
  const int screenHeight = std::max(screen.iScreenHeight, 1);
  const float scale = std::min({1.0f, static_cast<float>(MAX_CAPTURE_WIDTH) / screenWidth,
                                static_cast<float>(MAX_CAPTURE_HEIGHT) / screenHeight});
  width = static_cast<int>(screenWidth * scale) & ~(CAPTURE_WIDTH_ALIGN - 1);
  height = static_cast<int>(screenHeight * scale) & ~1;
}

CAMLCapture::CAMLCapture()
{
  if (auto* winSystem = dynamic_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem()))
    winSystem->Register(this);
}

CAMLCapture::~CAMLCapture()
{
  if (auto* winSystem = dynamic_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem()))
    winSystem->Unregister(this);

  if (m_stream)
    m_stream->Stop();
}

void CAMLCapture::OnLostDisplay()
{
  std::shared_ptr<CStream> stream;
  {
    std::lock_guard lock(m_lock);
    m_displayLost = true;
    stream = TakeStream();
  }
  Close(std::move(stream));
}

void CAMLCapture::OnResetDisplay()
{
  std::lock_guard lock(m_lock);

  m_displayLost = false;
}

void CAMLCapture::Configure(const AMLCaptureSource& source)
{
  std::lock_guard lock(m_lock);

  m_source = source;
  m_presented = false;
  m_busy = false;
}

void CAMLCapture::SetPresented()
{
  m_presented = true;
}

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::TakeStream()
{
  if (m_stream)
    m_closing++;
  return std::move(m_stream);
}

void CAMLCapture::Close(std::shared_ptr<CStream> stream)
{
  if (!stream)
    return;

  stream->Stop();
  stream.reset();

  std::lock_guard lock(m_lock);
  m_closing--;
  m_closed.notify_all();
}

std::shared_ptr<void> CAMLCapture::Lease(int width, int height)
{
  std::lock_guard lock(m_lock);

  m_leases++;
  if (width > 0 && height > 0)
    m_continuousSizes.emplace_back(width, height);
  return std::shared_ptr<void>(nullptr,
                               [weak = weak_from_this(), width, height](void*)
                               {
                                 if (const auto session = weak.lock())
                                   session->ReleaseLease(width, height);
                               });
}

void CAMLCapture::ReleaseLease(int width, int height)
{
  {
    std::lock_guard lock(m_lock);
    const auto it = std::find(m_continuousSizes.begin(), m_continuousSizes.end(),
                              std::make_pair(width, height));
    if (it != m_continuousSizes.end())
      m_continuousSizes.erase(it);
    if (--m_leases > 0 || !m_stream)
      return;
  }

  // a lease can end on the render thread, which makes no stream ioctls
  CServiceBroker::GetJobManager()->Submit(
      [weak = weak_from_this()]
      {
        const auto session = weak.lock();
        if (!session)
          return;

        std::shared_ptr<CStream> stream;
        {
          std::lock_guard lock(session->m_lock);
          if (session->m_leases > 0)
            return;
          stream = session->TakeStream();
        }
        session->Close(std::move(stream));
      });
}

bool CAMLCapture::Read(int width, int height, const ReadFunc& use)
{
  std::lock_guard read(m_readLock);

  const float fps = CServiceBroker::GetWinSystem()->GetGfxContext().GetFPS();
  const int waitMs =
      static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * 1000.0f / std::max(fps, 1.0f)));
  const std::chrono::milliseconds maxAge(
      static_cast<int>((CAPTURE_BUFFERS - 2) * 1000.0f / std::max(fps, 1.0f)));
  bool restarted = false;
  for (;;)
  {
    std::shared_ptr<CStream> stream;
    std::shared_ptr<CStream> stale;
    AMLCaptureSource source;
    {
      std::lock_guard lock(m_lock);
      if (m_displayLost || !m_presented || m_busy)
        return false;
      for (const auto& [continuousWidth, continuousHeight] : m_continuousSizes)
      {
        width = std::max(width, continuousWidth);
        height = std::max(height, continuousHeight);
      }
      if (m_stream && !m_stream->Matches(width, height))
        stale = TakeStream();
      stream = m_stream;
      source = m_source;
    }
    Close(std::move(stale));

    if (!stream)
    {
      {
        std::unique_lock lock(m_lock);
        m_closed.wait(lock, [this] { return m_closing == 0; });
        if (m_displayLost || !m_presented)
          return false;
      }

      bool busy = false;
      stream = CStream::Open(width, height, busy);

      std::lock_guard lock(m_lock);
      m_busy = busy;
      if (!stream)
        return false;
      if (m_displayLost || !m_presented || m_stream)
        return false;
      m_stream = stream;
    }

    unsigned int index = 0;
    const CStream::Wake wake =
        stream->TakeNewest(maxAge, index) ? CStream::Wake::FRAME : stream->Wait(waitMs, index);
    if (wake == CStream::Wake::FRAME)
    {
      AMLCaptureFrame frame;
      frame.color = GetSignal(source);
      frame.image = stream->Image(index);
      const bool ok = use(frame, m_convert);
      stream->Queue(index);
      return ok;
    }
    if (wake == CStream::Wake::STOPPED)
      return false;

    // another vdin user can stop the stream underneath us: restart it once, then drop it
    {
      std::lock_guard lock(m_lock);
      if (m_stream != stream)
        return false;
      stale = TakeStream();
    }
    stream.reset();
    Close(std::move(stale));
    if (restarted)
    {
      CLog::Log(LOGWARNING, "CAMLCapture::{}: no frame in {} ms, stream dropped", __FUNCTION__,
                waitMs);
      return false;
    }
    restarted = true;
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no frame in {} ms, restarting the stream", __FUNCTION__,
              waitMs);
  }
}

AMLCaptureColor CAMLCapture::GetSignal(const AMLCaptureSource& source) const
{
  AMLCaptureColor color;

  auto* winSystem = dynamic_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem());
  CAMLDisplay* display = winSystem ? winSystem->GetAmlDisplay() : nullptr;
  if (!display)
    return color;

  const bool sourcePQ = source.transfer == AVCOL_TRC_SMPTE2084;
  bool mapped = !sourcePQ;

  // hdr_conversion_ctrl does not follow the DV core, so it is read only without DV; under DV,
  // vdin1 sees the core3 input in every mode
  if (display->aml_get_drmProperty("dv_enable", DRM_MODE_OBJECT_CRTC) == 1 &&
      display->aml_get_drmProperty("dv_video_on", DRM_MODE_OBJECT_CRTC) == 1)
  {
    switch (static_cast<DvMode>(display->aml_get_drmProperty("dv_mode", DRM_MODE_OBJECT_CRTC)))
    {
      case DvMode::IPT:
      case DvMode::IPT_TUNNEL:
        if (display->aml_get_drmProperty("dv_ll_policy", DRM_MODE_OBJECT_CRTC) >= DV_LL_YUV422)
        {
          color.signal = AMLCaptureSignal::DV_LL;
          mapped = true;
        }
        else
          color.signal = AMLCaptureSignal::DV_TV_LED;
        break;
      case DvMode::HDR10:
        color.signal = AMLCaptureSignal::DV_HDR10;
        mapped = true;
        break;
      case DvMode::SDR10:
      case DvMode::SDR8:
        color.signal = AMLCaptureSignal::DV_SDR;
        break;
      default:
        if (sourcePQ)
          color.signal = AMLCaptureSignal::PQ;
        else if (source.transfer == AVCOL_TRC_ARIB_STD_B67)
          color.signal = AMLCaptureSignal::HLG;
        break;
    }
  }
  else
  {
    switch (static_cast<HdrOutput>(
        display->aml_get_drmProperty("hdr_conversion_ctrl", DRM_MODE_OBJECT_CRTC)))
    {
      case HdrOutput::HDR10:
      case HdrOutput::HDR10PLUS:
      case HdrOutput::CUVA_HDR:
        color.signal = AMLCaptureSignal::PQ;
        break;
      case HdrOutput::HLG:
      case HdrOutput::CUVA_HLG:
        color.signal = AMLCaptureSignal::HLG;
        break;
      default:
        break;
    }
  }

  switch (color.signal)
  {
    case AMLCaptureSignal::PQ:
    case AMLCaptureSignal::DV_LL:
    case AMLCaptureSignal::DV_HDR10:
      color.luminance = mapped ? MAPPED_PEAK_NITS : GetLuminance(source);
      break;
    case AMLCaptureSignal::HLG:
      color.luminance = HLG_PEAK_NITS;
      break;
    case AMLCaptureSignal::DV_TV_LED:
      if (HasPeak(source))
        color.luminance = GetLuminance(source);
      else
        color.luminance = source.dolbyVision ? TVLED_PEAK_NITS : MAPPED_PEAK_NITS;
      break;
    default:
      break;
  }
  return color;
}
