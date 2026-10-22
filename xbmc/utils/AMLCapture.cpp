/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "utils/AMLCapture.h"

#include "ServiceBroker.h"
#include "application/ApplicationComponents.h"
#include "application/ApplicationPlayer.h"
#include "cores/VideoPlayer/DVDCodecs/Video/AMLFrameMetadata.h"
#include "cores/VideoPlayer/Interface/StreamInfo.h"
#include "jobs/JobManager.h"
#include "rendering/capture/CapturePixels.h"
#include "settings/Settings.h"
#include "settings/SettingsComponent.h"
#include "utils/AMLUtils.h"
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
// V4L2_PIX_FMT_AML_YUV422_10P in the vendor tvin.h, which the sysroot does not ship
constexpr uint32_t PIX_FMT_YUV422_10P = v4l2_fourcc('A', 'Y', '2', 'P');
// vdin1 refuses a single buffer
constexpr unsigned int SHOT_BUFFERS = 3;
// the vdin1 line buffer is 1920 pixels wide; a 2160p output is downscaled 2:1
constexpr int MAX_CAPTURE_WIDTH = 1920;
constexpr int MAX_CAPTURE_HEIGHT = 1080;
// vdin strides each line to 64 pixels but sizes the buffer from the width
constexpr int CAPTURE_WIDTH_ALIGN = 64;
// the vdin scalers cannot shrink by 32 or more
constexpr int MAX_CAPTURE_SHRINK = 32;
// DV player-led, dv2hdr and sdr2hdr frames are already display-mapped
constexpr float MAPPED_PEAK_NITS = 1000.0f;
// the grade of a Dolby Vision TV-led signal when the stream carries no HDR statics
constexpr float TVLED_PEAK_NITS = 4000.0f;
// the nominal display of ITU-R BT.2100 HLG
constexpr float HLG_PEAK_NITS = 1000.0f;
// the range of ITU-R BT.2100 PQ, the peak of a stream that states none
constexpr float PQ_PEAK_NITS = 10000.0f;
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

// G12A, G12B and SM1 vdin keeps the even chroma sample of a full pack pair
bool HasCositedChroma()
{
  const int family = aml_get_cpufamily_id();
  return family == AML_G12A || family == AML_G12B || family == AML_SM1;
}

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

// the title peak, else fallback
float GetPeak(const AMLCaptureSource& source, float fallback)
{
  return source.titlePeak > 0 ? source.titlePeak : fallback;
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

// width x height sRGB pixels
void DescribeSdr(int width, int height, bool rgb, CaptureResult& result)
{
  result.width = static_cast<unsigned int>(width);
  result.height = static_cast<unsigned int>(height);
  result.stride = width * (rgb ? 3 : 4);
  result.format = rgb ? AV_PIX_FMT_RGB24 : AV_PIX_FMT_BGRA;
  result.color.primaries = AVCOL_PRI_BT709;
  result.color.transfer = AVCOL_TRC_IEC61966_2_1;
  result.color.range = AVCOL_RANGE_JPEG;
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

    // a standard screenshot is the rows of an 8-bit PNG
    const bool rgb = m_spec.shot == CaptureScreenshot::STANDARD;
    const int pixelBytes = rgb ? 3 : 4;
    auto buffer = std::make_unique_for_overwrite<uint8_t[]>(static_cast<size_t>(m_spec.width) *
                                                            m_spec.height * pixelBytes);
    const AMLCaptureImage dst =
        AMLCaptureImage::Packed(rgb ? AV_PIX_FMT_RGB24 : AV_PIX_FMT_BGRA, m_spec.width,
                                m_spec.height, buffer.get(), m_spec.width * pixelBytes);
    const bool ok =
        session->ReadVideo(m_spec.shot, m_streamWidth, m_streamHeight,
                           [this, &dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
                           {
                             return convert.Convert(CAMLCapture::Crop(frame.image, m_spec.rect),
                                                    frame.color, AMLCaptureCoding(), dst);
                           });
    // a screenshot holds no stream while it is encoded
    if (m_spec.shot != CaptureScreenshot::NONE)
      m_lease.reset();

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

  // every Lock reads a new frame, so a consumer is served while the GUI draws nothing; the
  // buffer holds it until Unlock
  const uint8_t* Lock() override
  {
    m_lock.lock();

    if (!m_buffer)
      m_buffer = std::make_unique<uint8_t[]>(static_cast<size_t>(m_width) * m_height * 4);
    const AMLCaptureImage dst =
        AMLCaptureImage::Packed(AV_PIX_FMT_BGRA, m_width, m_height, m_buffer.get(), m_width * 4);

    const bool ok =
        m_session->Read(AMLCaptureInput::GUI, m_streamWidth, m_streamHeight,
                        [&dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert) {
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

  //! failures log at `logLevel`; `busy` tells whether another vdin1 user holds the node, `refused`
  //! whether vdin1 cannot give this input at this size
  static std::shared_ptr<CStream> Open(AMLCaptureInput input,
                                       uint32_t pixelFormat,
                                       unsigned int buffers,
                                       int width,
                                       int height,
                                       int logLevel,
                                       bool& busy,
                                       bool& refused);
  ~CStream();

  void Stop();
  AMLCaptureInput GetInput() const { return m_input; }
  uint32_t GetPixelFormat() const { return m_pixelFormat; }
  int GetBuffers() const { return static_cast<int>(m_maps.size()); }
  bool Matches(AMLCaptureInput input, uint32_t pixelFormat, int width, int height) const
  {
    return input == m_input && pixelFormat == m_pixelFormat && width == m_width &&
           height == m_height;
  }
  //! Takes the newest frame vdin1 finished since the last dequeue, false when there is none or
  //! when that dequeue is older than `maxAge`
  bool TakeNewest(std::chrono::milliseconds maxAge, unsigned int& index);
  Wake Wait(int timeoutMs, unsigned int& index);
  bool Queue(unsigned int index);
  AMLCaptureImage Image(unsigned int index) const;
  size_t GetLength(unsigned int index) const { return m_maps[index].second; }
  //! a read-only dmabuf of buffer `index`, which keeps the buffer after the stream closes; -1 on
  //! failure
  int Export(unsigned int index) const;

private:
  bool Dequeue(unsigned int& index);
  bool HasFrame(int timeoutMs, bool& stopped);

  int m_fd{-1};
  int m_eventFd{-1};
  AMLCaptureInput m_input{AMLCaptureInput::VIDEO};
  uint32_t m_pixelFormat{V4L2_PIX_FMT_NV12};
  int m_width{0};
  int m_height{0};
  int m_stride{0};
  bool m_streaming{false};
  std::atomic<bool> m_stopped{false};
  std::vector<std::pair<void*, size_t>> m_maps;
  std::chrono::steady_clock::time_point m_dequeued;
};

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::CStream::Open(AMLCaptureInput input,
                                                                 uint32_t pixelFormat,
                                                                 unsigned int buffers,
                                                                 int width,
                                                                 int height,
                                                                 int logLevel,
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
  stream->m_pixelFormat = pixelFormat;
  stream->m_width = width;
  stream->m_height = height;
  stream->m_fd = open(device.path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  stream->m_eventFd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (stream->m_fd < 0 || stream->m_eventFd < 0)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: cannot open {}: {}", __FUNCTION__, device.path,
              strerror(errno));
    return {};
  }

  if (ioctl(stream->m_fd, VIDIOC_S_INPUT, &index) < 0)
  {
    busy = errno == EBUSY;
    refused = errno == EINVAL;
    if (!busy)
      CLog::Log(logLevel, "CAMLCapture::{}: input {} unavailable: {}", __FUNCTION__, index,
                strerror(errno));
    return {};
  }

  v4l2_format format{};
  format.type = BUFFER_TYPE;
  format.fmt.pix_mp.width = width;
  format.fmt.pix_mp.height = height;
  format.fmt.pix_mp.pixelformat = pixelFormat;
  format.fmt.pix_mp.field = V4L2_FIELD_NONE;
  format.fmt.pix_mp.num_planes = 1;
  const bool fullPack = pixelFormat == PIX_FMT_YUV422_10P;
  const int ioctlResult = ioctl(stream->m_fd, VIDIOC_S_FMT, &format);
  stream->m_stride =
      fullPack ? static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline) : width;
  const size_t frameSize = fullPack ? static_cast<size_t>(stream->m_stride) * height
                                    : static_cast<size_t>(width) * height * 3 / 2;
  if (ioctlResult < 0 || format.fmt.pix_mp.pixelformat != pixelFormat ||
      format.fmt.pix_mp.width != static_cast<uint32_t>(width) ||
      format.fmt.pix_mp.height != static_cast<uint32_t>(height) ||
      (fullPack && stream->m_stride < width / 2 * AML_FULL_PACK_PAIR_BYTES) ||
      format.fmt.pix_mp.plane_fmt[0].sizeimage < frameSize)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: {} {}x{} refused", __FUNCTION__,
              fullPack ? "10-bit" : "NV12", width, height);
    refused = true;
    return {};
  }

  v4l2_requestbuffers request{};
  request.count = buffers;
  request.type = BUFFER_TYPE;
  request.memory = V4L2_MEMORY_MMAP;
  if (ioctl(stream->m_fd, VIDIOC_REQBUFS, &request) < 0 || request.count == 0)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: no capture buffers: {}", __FUNCTION__, strerror(errno));
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
    // the line buffer limits a 10-bit stream that scales vertically
    refused = fullPack && errno == EINVAL;
    CLog::Log(logLevel, "CAMLCapture::{}: stream start failed: {}", __FUNCTION__, strerror(errno));
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

int CAMLCapture::CStream::Export(unsigned int index) const
{
  v4l2_exportbuffer exportBuffer{};
  exportBuffer.type = BUFFER_TYPE;
  exportBuffer.index = index;
  exportBuffer.flags = O_RDONLY | O_CLOEXEC;
  if (ioctl(m_fd, VIDIOC_EXPBUF, &exportBuffer) < 0)
    return -1;
  return exportBuffer.fd;
}

AMLCaptureImage CAMLCapture::CStream::Image(unsigned int index) const
{
  AMLCaptureImage image;
  image.width = m_width;
  image.height = m_height;
  image.data[0] = static_cast<uint8_t*>(m_maps[index].first);
  if (m_pixelFormat == PIX_FMT_YUV422_10P)
  {
    image.fullPack = true;
    image.cositedChroma = HasCositedChroma();
    image.linesize[0] = m_stride;
    return image;
  }

  // single-plane NV12: the chroma plane follows the luma plane at the same stride
  image.format = AV_PIX_FMT_NV12;
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
  m_staticsMeta.reset();
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

  return ReadStream(input, V4L2_PIX_FMT_NV12, CAPTURE_BUFFERS, width, height,
                    [this, &use](CStream&, unsigned int, const AMLCaptureFrame& frame)
                    { return use(frame, m_convert); });
}

bool CAMLCapture::ReadVideo(CaptureScreenshot shot, int width, int height, const ReadFunc& use)
{
  if (shot != CaptureScreenshot::NONE)
  {
    AMLCaptureShotFrame frame;
    CAMLCaptureConvert convert;
    if (ReadShot(frame) && use(frame.frame, convert))
      return true;
  }
  return Read(AMLCaptureInput::VIDEO, width, height, use);
}

bool CAMLCapture::ReadShot(AMLCaptureShotFrame& frame)
{
  std::unique_lock read(m_readLock);

  // a full frame copy takes longer than a continuous reader can wait, so it runs from a dmabuf of
  // the frame after the stream closes
  int dmabuf = -1;
  void* map = MAP_FAILED;
  size_t length = 0;
  size_t size = 0;
  const auto copy = [&](CStream& stream, unsigned int index, const AMLCaptureFrame& source)
  {
    size = static_cast<size_t>(source.image.linesize[0]) * source.image.height;
    frame.data = std::make_unique_for_overwrite<uint8_t[]>(size);
    frame.frame = source;
    dmabuf = stream.Export(index);
    if (dmabuf >= 0)
    {
      length = stream.GetLength(index);
      map = mmap(nullptr, length, PROT_READ, MAP_SHARED, dmabuf, 0);
    }
    if (map == MAP_FAILED)
    {
      CLog::Log(LOGDEBUG, "CAMLCapture::{}: no dmabuf of the frame, copying in place: {}",
                __FUNCTION__, strerror(errno));
      if (dmabuf >= 0)
        close(dmabuf);
      dmabuf = -1;
      std::memcpy(frame.data.get(), source.image.data[0], size);
      frame.frame.image.data[0] = frame.data.get();
      return true;
    }
    // vdin1 must not refill the exported buffer once it is queued again
    stream.Stop();
    return true;
  };

  int width = 0;
  int height = 0;
  GetCaptureSize(width, height);
  const bool ok =
      ReadStream(AMLCaptureInput::VIDEO, PIX_FMT_YUV422_10P, SHOT_BUFFERS, width, height, copy);

  // close the shot stream whatever leases remain
  std::shared_ptr<CStream> stream;
  {
    std::lock_guard lock(m_lock);
    if (m_stream && m_stream->GetPixelFormat() == PIX_FMT_YUV422_10P)
      stream = TakeStream();
  }
  Close(std::move(stream));
  read.unlock();

  if (map != MAP_FAILED)
  {
    std::memcpy(frame.data.get(), map, size);
    frame.frame.image.data[0] = frame.data.get();
    munmap(map, length);
    close(dmabuf);
  }

  if (!ok)
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no 10-bit frame at {}x{}", __FUNCTION__, width, height);
  return ok;
}

bool CAMLCapture::ReadStream(AMLCaptureInput input,
                             uint32_t pixelFormat,
                             unsigned int buffers,
                             int width,
                             int height,
                             const StreamFunc& use)
{
  const float fps = CServiceBroker::GetWinSystem()->GetGfxContext().GetFPS();
  const int waitMs =
      static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * 1000.0f / std::max(fps, 1.0f)));
  bool restarted = false;
  for (;;)
  {
    std::shared_ptr<CStream> stream;
    std::shared_ptr<CStream> stale;
    AMLCaptureSource source;
    // the store lock is never taken under m_lock
    AMLFrameMetadataPtr meta;
    if (input == AMLCaptureInput::VIDEO)
      meta = CAMLFrameMetadataStore::GetInstance().Get();
    {
      std::lock_guard lock(m_lock);
      if (!CanRead(input))
        return false;
      for (const auto& [continuousInput, continuousWidth, continuousHeight] : m_continuousSizes)
      {
        if (continuousInput != input || pixelFormat != V4L2_PIX_FMT_NV12)
          continue;
        width = std::max(width, continuousWidth);
        height = std::max(height, continuousHeight);
      }
      if (m_stream && !m_stream->Matches(input, pixelFormat, width, height))
        stale = TakeStream();
      stream = m_stream;
      if (input == AMLCaptureInput::VIDEO)
        UpdateStatics(std::move(meta));
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
      const int logLevel = m_openFailed ? LOGDEBUG : LOGWARNING;
      stream = CStream::Open(input, pixelFormat, buffers, width, height, logLevel, busy, refused);
      if (refused && input == AMLCaptureInput::GUI)
        RefuseGui();
      else if (!busy)
        m_openFailed = !stream;

      std::lock_guard lock(m_lock);
      bool& wasBusy = input == AMLCaptureInput::GUI ? m_guiBusy : m_busy;
      if (busy && !wasBusy)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: another vdin1 user holds the node", __FUNCTION__);
      wasBusy = busy;
      if (!stream)
        return false;
      if (!CanRead(input))
        return false;
      m_stream = stream;
    }

    const std::chrono::milliseconds maxAge(
        static_cast<int>((stream->GetBuffers() - 2) * 1000.0f / std::max(fps, 1.0f)));
    unsigned int index = 0;
    const CStream::Wake wake =
        stream->TakeNewest(maxAge, index) ? CStream::Wake::FRAME : stream->Wait(waitMs, index);
    if (wake == CStream::Wake::FRAME)
    {
      AMLCaptureFrame frame;
      // with no video presented the GUI is SDR
      if (input == AMLCaptureInput::VIDEO)
        frame.color = GetSignal(source);
      frame.image = stream->Image(index);
      const bool ok = use(*stream, index, frame);
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

void CAMLCapture::UpdateStatics(AMLFrameMetadataPtr meta)
{
  if (meta == m_staticsMeta)
    return;

  // the metadata object changes with the dynamic payloads, the statics rarely
  if (!meta || !m_staticsMeta || meta->hdrMdcv != m_staticsMeta->hdrMdcv ||
      meta->hdrCll != m_staticsMeta->hdrCll)
    m_source.titlePeak = meta ? AMLGetTitlePeak(*meta) : 0.0f;
  m_staticsMeta = std::move(meta);
}

AMLCaptureColor CAMLCapture::GetSignal(const AMLCaptureSource& source) const
{
  AMLCaptureColor color;

  auto* winSystem = dynamic_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem());
  CAMLDisplay* display = winSystem ? winSystem->GetAmlDisplay() : nullptr;
  if (!display)
    return color;

  bool sourcePQ = source.transfer == AVCOL_TRC_SMPTE2084;
  bool sourceHLG = source.transfer == AVCOL_TRC_ARIB_STD_B67;
  // hdr2sdr and sdr2hdr rewrite the stream colour to the output colour; the demuxer keeps the type
  const auto settings = CServiceBroker::GetSettingsComponent()->GetSettings();
  if (!source.dolbyVision && (settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_HDR2SDR) ||
                              settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_SDR2HDR)))
  {
    const auto& components = CServiceBroker::GetAppComponents();
    const auto appPlayer = components.GetComponent<CApplicationPlayer>();
    VideoStreamInfo stream;
    appPlayer->GetVideoStreamInfo(CURRENT_STREAM, stream);
    if (stream.valid && (aml_convert_to_sdr(stream.hdrType) || aml_convert_to_hdr(stream.hdrType)))
    {
      sourceHLG = stream.hdrType == StreamHdrType::HDR_TYPE_HLG;
      sourcePQ = stream.hdrType != StreamHdrType::HDR_TYPE_NONE && !sourceHLG;
    }
  }
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
        else if (sourceHLG)
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
      color.luminance = mapped ? MAPPED_PEAK_NITS : GetPeak(source, PQ_PEAK_NITS);
      break;
    case AMLCaptureSignal::HLG:
      color.luminance = HLG_PEAK_NITS;
      break;
    case AMLCaptureSignal::DV_TV_LED:
      color.luminance = GetPeak(source, source.dolbyVision ? TVLED_PEAK_NITS : MAPPED_PEAK_NITS);
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
  if (image.fullPack)
  {
    crop.data[0] += y1 * image.linesize[0] + x1 / 2 * AML_FULL_PACK_PAIR_BYTES;
    return crop;
  }
  crop.data[0] += y1 * image.linesize[0] + x1;
  crop.data[1] += y1 / 2 * image.linesize[1] + x1;
  return crop;
}

void CAMLCapture::CreateVideoPixels(const AMLCaptureVideoSpec& spec, CaptureResult& result)
{
  int width = 0;
  int height = 0;
  GetStreamSize(spec.rect, spec.width, spec.height, width, height);
  auto lease = spec.continuous ? Lease(AMLCaptureInput::VIDEO, width, height) : Lease();
  result.pixels =
      std::make_shared<CAMLVideoPixels>(weak_from_this(), std::move(lease), spec, width, height);
  DescribeSdr(spec.width, spec.height, spec.shot == CaptureScreenshot::STANDARD, result);
  result.content = CaptureContent::VIDEO;
}

void CAMLCapture::CreateGuiPixels(int width, int height, CaptureResult& result)
{
  if (IsGuiRefused())
    return;

  const auto session = Get();
  int streamWidth = 0;
  int streamHeight = 0;
  GetStreamSize(CRect(0.0f, 0.0f, 1.0f, 1.0f), width, height, streamWidth, streamHeight);
  auto lease = session->Lease(AMLCaptureInput::GUI, streamWidth, streamHeight);
  result.pixels = std::make_shared<CAMLGuiPixels>(session, std::move(lease), width, height,
                                                  streamWidth, streamHeight);
  DescribeSdr(width, height, false, result);
  result.content = CaptureContent::COMPOSITE;
  result.live = true;
}
