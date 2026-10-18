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
#include "rendering/capture/CapturePixels.h"
#include "utils/log.h"
#include "windowing/GraphicContext.h"
#include "windowing/amlogic/WinSystemAmlogic.h"

#include "platform/linux/SysfsPath.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace KODI::RENDERING::CAPTURE;

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
// the vdin scalers cannot shrink by 32 or more
constexpr int MAX_CAPTURE_SHRINK = 32;
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
  int guiInput{-1};
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
        const std::string inputName(reinterpret_cast<const char*>(input.name));
        if (inputName == "VIU1_WB0_VD1")
        {
          device.path = path;
          device.input = static_cast<int>(input.index);
        }
        else if (inputName == "VIU1_WB0_OSD1")
          device.guiInput = static_cast<int>(input.index);
      }
    }
    close(fd);
    if (device.input >= 0)
      break;
  }

  if (device.input >= 0)
    CLog::Log(LOGINFO, "CAMLCapture::{}: video capture on {} input {}, GUI input {}", __FUNCTION__,
              device.path, device.input, device.guiInput);
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

// the stream size that holds width x height pixels in the normalized crop
void GetStreamSize(const CRect& crop, int width, int height, int& streamWidth, int& streamHeight)
{
  CAMLCapture::GetCaptureSize(streamWidth, streamHeight);
  if (crop.IsEmpty())
    return;

  const RESOLUTION_INFO screen = CServiceBroker::GetWinSystem()->GetGfxContext().GetResInfo();

  // a size within one alignment step of the capture is the capture, so full-size requests that
  // differ by rounding share the stream
  const auto size = [](int pixels, float part, int align, int output, int limit)
  {
    const int needed =
        std::max(static_cast<int>(std::ceil(pixels / part)), output / MAX_CAPTURE_SHRINK + 1);
    const int aligned = (needed + align - 1) / align * align;
    return aligned >= limit - align ? limit : aligned;
  };
  streamWidth = size(width, crop.Width(), CAPTURE_WIDTH_ALIGN, screen.iScreenWidth, streamWidth);
  streamHeight = size(height, crop.Height(), 2, screen.iScreenHeight, streamHeight);
}

class CAMLVideoPixels : public ICapturePixels
{
public:
  CAMLVideoPixels(std::weak_ptr<CAMLCapture> session,
                  std::shared_ptr<void> lease,
                  const AMLCaptureVideoSpec& spec,
                  int streamWidth,
                  int streamHeight)
    : m_session(std::move(session)),
      m_lease(std::move(lease)),
      m_spec(spec),
      m_streamWidth(streamWidth),
      m_streamHeight(streamHeight)
  {
  }

  const uint8_t* Lock() override
  {
    std::lock_guard lock(m_lock);

    // per-frame buffers stay immutable: every Lock returns the first read
    if (m_read)
      return m_buffer.get();
    m_read = true;

    const auto session = m_session.lock();
    if (!session)
      return nullptr;

    auto buffer =
        std::make_unique<uint8_t[]>(static_cast<size_t>(m_spec.width) * m_spec.height * 4);
    AMLCaptureImage dst;
    dst.format = AV_PIX_FMT_BGRA;
    dst.width = m_spec.width;
    dst.height = m_spec.height;
    dst.data[0] = buffer.get();
    dst.linesize[0] = m_spec.width * 4;

    const bool ok =
        session->Read(AMLCaptureInput::VIDEO, m_streamWidth, m_streamHeight,
                      [this, &dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
                      {
                        return convert.Convert(CAMLCapture::Crop(frame.image, m_spec.rect),
                                               frame.color, AMLCaptureCoding(), dst);
                      });

    if (ok)
      m_buffer = std::move(buffer);
    return m_buffer.get();
  }

private:
  std::weak_ptr<CAMLCapture> m_session;
  std::shared_ptr<void> m_lease;
  AMLCaptureVideoSpec m_spec;
  int m_streamWidth{0};
  int m_streamHeight{0};
  std::mutex m_lock;
  bool m_read{false};
  std::unique_ptr<uint8_t[]> m_buffer;
};

class CAMLGuiPixels : public ICapturePixels
{
public:
  CAMLGuiPixels(std::shared_ptr<CAMLCapture> session,
                std::shared_ptr<void> lease,
                int width,
                int height,
                int streamWidth,
                int streamHeight)
    : m_session(std::move(session)),
      m_lease(std::move(lease)),
      m_width(width),
      m_height(height),
      m_streamWidth(streamWidth),
      m_streamHeight(streamHeight)
  {
  }

  // every Lock reads the next frame, so a consumer is served while the GUI draws nothing; the
  // buffer holds it until Unlock
  const uint8_t* Lock() override
  {
    m_lock.lock();

    if (!m_buffer)
      m_buffer = std::make_unique<uint8_t[]>(static_cast<size_t>(m_width) * m_height * 4);
    AMLCaptureImage dst;
    dst.format = AV_PIX_FMT_BGRA;
    dst.width = m_width;
    dst.height = m_height;
    dst.data[0] = m_buffer.get();
    dst.linesize[0] = m_width * 4;

    const bool ok =
        m_session->Read(AMLCaptureInput::GUI, m_streamWidth, m_streamHeight,
                        [&dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
                        {
                          return convert.Convert(frame.image, frame.color, AMLCaptureCoding(), dst);
                        });
    return ok ? m_buffer.get() : nullptr;
  }

  void Unlock() override { m_lock.unlock(); }

private:
  // the lease ends first, so the stream closes before the session can go
  std::shared_ptr<CAMLCapture> m_session;
  std::shared_ptr<void> m_lease;
  int m_width{0};
  int m_height{0};
  int m_streamWidth{0};
  int m_streamHeight{0};
  std::mutex m_lock;
  std::unique_ptr<uint8_t[]> m_buffer;
};

using OutputMode = std::tuple<int, int, float, uint32_t>;

OutputMode GetOutputMode()
{
  const RESOLUTION_INFO screen = CServiceBroker::GetWinSystem()->GetGfxContext().GetResInfo();
  return {screen.iScreenWidth, screen.iScreenHeight, screen.fRefreshRate, screen.dwFlags};
}

std::mutex s_sessionLock;
std::weak_ptr<CAMLCapture> s_session;
//! the output mode in which vdin1 gave no GUI, until the next display reset; it outlives sessions
std::mutex s_guiLock;
std::optional<OutputMode> s_guiRefusedMode;

bool IsGuiRefused()
{
  const OutputMode mode = GetOutputMode();

  std::lock_guard lock(s_guiLock);

  return s_guiRefusedMode == mode;
}

void RefuseGui()
{
  const OutputMode mode = GetOutputMode();

  std::lock_guard lock(s_guiLock);

  if (s_guiRefusedMode == mode)
    return;
  s_guiRefusedMode = mode;
  CLog::Log(LOGINFO, "CAMLCapture::{}: no GUI from vdin1 at {}x{}, scaled GUI captures fail",
            __FUNCTION__, std::get<0>(mode), std::get<1>(mode));
}

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

  //! `busy` tells whether another vdin1 user holds the node, `refused` whether vdin1 cannot give
  //! this input at this size
  static std::shared_ptr<CStream> Open(
      AMLCaptureInput input, int width, int height, bool& busy, bool& refused);
  ~CStream();

  void Stop();
  AMLCaptureInput GetInput() const { return m_input; }
  bool Matches(AMLCaptureInput input, int width, int height) const
  {
    return input == m_input && width == m_width && height == m_height;
  }
  void Drain();
  Wake Wait(int timeoutMs, unsigned int& index);
  bool Queue(unsigned int index);
  AMLCaptureImage Image(unsigned int index) const;

private:
  bool Dequeue(unsigned int& index);
  bool HasFrame(int timeoutMs, bool& stopped);

  int m_fd{-1};
  int m_eventFd{-1};
  AMLCaptureInput m_input{AMLCaptureInput::VIDEO};
  int m_width{0};
  int m_height{0};
  bool m_streaming{false};
  std::atomic<bool> m_stopped{false};
  std::vector<std::pair<void*, size_t>> m_maps;
};

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::CStream::Open(AMLCaptureInput input,
                                                                 int width,
                                                                 int height,
                                                                 bool& busy,
                                                                 bool& refused)
{
  const AMLCaptureDevice& device = FindDevice();
  int index = input == AMLCaptureInput::GUI ? device.guiInput : device.input;
  if (device.input < 0 || index < 0)
  {
    refused = true;
    return {};
  }

  auto stream = std::make_shared<CStream>();
  stream->m_input = input;
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

  if (ioctl(stream->m_fd, VIDIOC_S_INPUT, &index) < 0)
  {
    busy = errno == EBUSY;
    refused = errno == EINVAL;
    if (!busy)
      CLog::Log(LOGWARNING, "CAMLCapture::{}: input {} unavailable: {}", __FUNCTION__, index,
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
    refused = true;
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

  CLog::Log(LOGINFO, "CAMLCapture::{}: started input {} {}x{}", __FUNCTION__, index, width, height);
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

void CAMLCapture::CStream::Drain()
{
  // a DQBUF on an empty queue logs in the driver, so every one waits for poll
  bool stopped = false;
  unsigned int index = 0;
  while (HasFrame(0, stopped) && Dequeue(index))
    Queue(index);
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
  std::lock_guard guiLock(s_guiLock);
  s_guiRefusedMode.reset();
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

void CAMLCapture::SetVideoRect(const CRect& rect)
{
  std::lock_guard lock(m_lock);

  m_videoRect = rect;
}

CRect CAMLCapture::GetVisibleRect()
{
  // the codec places the video at the GUI rect scaled to the output, so the GUI size normalizes it
  const CGraphicContext& context = CServiceBroker::GetWinSystem()->GetGfxContext();
  const RESOLUTION_INFO gui = context.GetResInfo(context.GetVideoResolution());
  if (gui.iWidth <= 0 || gui.iHeight <= 0)
    return {};

  std::lock_guard lock(m_lock);

  CRect rect(m_videoRect.x1 / gui.iWidth, m_videoRect.y1 / gui.iHeight, m_videoRect.x2 / gui.iWidth,
             m_videoRect.y2 / gui.iHeight);
  return rect.Intersect(CRect(0.0f, 0.0f, 1.0f, 1.0f));
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

std::shared_ptr<void> CAMLCapture::Lease(AMLCaptureInput input, int width, int height)
{
  std::lock_guard lock(m_lock);

  m_leases[static_cast<size_t>(input)]++;
  if (width > 0 && height > 0)
    m_continuousSizes.emplace_back(input, width, height);
  return std::shared_ptr<void>(nullptr,
                               [weak = weak_from_this(), input, width, height](void*)
                               {
                                 if (const auto session = weak.lock())
                                   session->ReleaseLease(input, width, height);
                               });
}

void CAMLCapture::ReleaseLease(AMLCaptureInput input, int width, int height)
{
  {
    std::lock_guard lock(m_lock);
    const auto it = std::find(m_continuousSizes.begin(), m_continuousSizes.end(),
                              std::make_tuple(input, width, height));
    if (it != m_continuousSizes.end())
      m_continuousSizes.erase(it);
    m_leases[static_cast<size_t>(input)]--;
    if (!m_stream || m_leases[static_cast<size_t>(m_stream->GetInput())] > 0)
      return;
  }

  // a lease can end on the render thread, which makes no stream ioctls; the job holds the
  // session, so its destructor does not stop the stream there either
  CServiceBroker::GetJobManager()->Submit(
      [session = shared_from_this()]
      {
        std::shared_ptr<CStream> stream;
        {
          std::lock_guard lock(session->m_lock);
          if (!session->m_stream ||
              session->m_leases[static_cast<size_t>(session->m_stream->GetInput())] > 0)
            return;
          stream = session->TakeStream();
        }
        session->Close(std::move(stream));
      });
}

bool CAMLCapture::CanRead(AMLCaptureInput input) const
{
  if (m_displayLost)
    return false;
  if (input == AMLCaptureInput::GUI)
    return !m_presented;
  return m_presented && !m_busy;
}

bool CAMLCapture::Read(AMLCaptureInput input, int width, int height, const ReadFunc& use)
{
  if (input == AMLCaptureInput::GUI && IsGuiRefused())
    return false;

  std::lock_guard read(m_readLock);

  const float fps = CServiceBroker::GetWinSystem()->GetGfxContext().GetFPS();
  const int waitMs =
      static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * 1000.0f / std::max(fps, 1.0f)));
  bool restarted = false;
  for (;;)
  {
    std::shared_ptr<CStream> stream;
    std::shared_ptr<CStream> stale;
    AMLCaptureSource source;
    {
      std::lock_guard lock(m_lock);
      if (!CanRead(input))
        return false;
      for (const auto& [continuousInput, continuousWidth, continuousHeight] : m_continuousSizes)
      {
        if (continuousInput != input)
          continue;
        width = std::max(width, continuousWidth);
        height = std::max(height, continuousHeight);
      }
      if (m_stream && !m_stream->Matches(input, width, height))
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
        if (!CanRead(input))
          return false;
      }

      bool busy = false;
      bool refused = false;
      stream = CStream::Open(input, width, height, busy, refused);
      if (refused && input == AMLCaptureInput::GUI)
        RefuseGui();

      std::lock_guard lock(m_lock);
      bool& wasBusy = input == AMLCaptureInput::GUI ? m_guiBusy : m_busy;
      if (busy && !wasBusy)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: another vdin1 user holds the node", __FUNCTION__);
      wasBusy = busy;
      if (!stream)
        return false;
      if (!CanRead(input) || m_stream)
        return false;
      m_stream = stream;
    }

    stream->Drain();
    unsigned int index = 0;
    const CStream::Wake wake = stream->Wait(waitMs, index);
    if (wake == CStream::Wake::FRAME)
    {
      AMLCaptureFrame frame;
      // with no video presented the GUI is SDR
      if (input == AMLCaptureInput::VIDEO)
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

AMLCaptureImage CAMLCapture::Crop(const AMLCaptureImage& image, const CRect& rect)
{
  if (rect.IsEmpty())
    return image;

  const auto edge = [](float v, int size)
  { return std::clamp(static_cast<int>(std::lround(v * size / 2)) * 2, 0, size); };
  const int x1 = edge(rect.x1, image.width);
  const int x2 = edge(rect.x2, image.width);
  const int y1 = edge(rect.y1, image.height);
  const int y2 = edge(rect.y2, image.height);

  AMLCaptureImage crop = image;
  crop.width = x2 - x1;
  crop.height = y2 - y1;
  crop.data[0] += y1 * image.linesize[0] + x1;
  crop.data[1] += y1 / 2 * image.linesize[1] + x1;
  return crop;
}

std::shared_ptr<ICapturePixels> CAMLCapture::CreateVideoPixels(const AMLCaptureVideoSpec& spec)
{
  int width = 0;
  int height = 0;
  GetStreamSize(spec.rect, spec.width, spec.height, width, height);
  auto lease = spec.continuous ? Lease(AMLCaptureInput::VIDEO, width, height) : Lease();
  return std::make_shared<CAMLVideoPixels>(weak_from_this(), std::move(lease), spec, width, height);
}

std::shared_ptr<ICapturePixels> CAMLCapture::CreateGuiPixels(int width, int height)
{
  if (IsGuiRefused())
    return {};

  const auto session = Get();
  if (session->IsPresented())
    return {};

  int streamWidth = 0;
  int streamHeight = 0;
  GetStreamSize(CRect(0.0f, 0.0f, 1.0f, 1.0f), width, height, streamWidth, streamHeight);
  auto lease = session->Lease(AMLCaptureInput::GUI, streamWidth, streamHeight);
  return std::make_shared<CAMLGuiPixels>(session, std::move(lease), width, height, streamWidth,
                                         streamHeight);
}
