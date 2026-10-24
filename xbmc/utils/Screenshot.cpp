/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "Screenshot.h"

#include "ServiceBroker.h"
#include "URL.h"
#include "Util.h"
#include "guilib/TextureFormats.h"
#include "pictures/Picture.h"
#include "rendering/capture/CaptureHandle.h"
#include "rendering/capture/CapturePixels.h"
#include "rendering/capture/CaptureService.h"
#include "rendering/capture/CaptureTypes.h"
#include "resources/LocalizeStrings.h"
#include "resources/ResourcesComponent.h"
#include "settings/SettingPath.h"
#include "settings/Settings.h"
#include "settings/SettingsComponent.h"
#include "settings/windows/GUIControlSettings.h"
#include "utils/StringUtils.h"
#include "utils/URIUtils.h"
#include "utils/log.h"

#include <memory>
#include <string_view>

std::vector<std::function<std::unique_ptr<IScreenshotSurface>()>> CScreenShot::m_screenShotSurfaces;

void CScreenShot::Register(const std::function<std::unique_ptr<IScreenshotSurface>()>& createFunc)
{
  m_screenShotSurfaces.emplace_back(createFunc);
}

std::unique_ptr<IScreenshotSurface> CScreenShot::CreateSurface()
{
  if (m_screenShotSurfaces.empty())
    return {};
  return m_screenShotSurfaces.back()();
}

namespace
{
#if defined(HAS_LIBAMCODEC)
constexpr int SCREENSHOT_TYPE_NATIVE = 1;
#endif

// Map a capture's source coding to the encoder's XB_FMT input format.
unsigned int CaptureXbFormat(AVPixelFormat format)
{
  switch (format)
  {
    case AV_PIX_FMT_BGRA:
      return XB_FMT_A8R8G8B8;
    case AV_PIX_FMT_RGBA:
      return XB_FMT_RGBA8;
    case AV_PIX_FMT_RGB24:
      return XB_FMT_RGB8;
    case AV_PIX_FMT_RGBA64LE:
      return XB_FMT_RGBA16;
    case AV_PIX_FMT_RGB48BE:
      return XB_FMT_RGB16BE;
    case AV_PIX_FMT_X2BGR10LE:
      return XB_FMT_X2BGR10;
    default:
      return XB_FMT_UNKNOWN;
  }
}

// the next free auto name in dir; the suffix is part of the numbering pattern, so each suffix
// numbers on its own and no name is reused
std::string NextScreenshotName(const std::string& dir, std::string_view suffix)
{
  const std::string file = CUtil::GetNextFilename(
      URIUtils::AddFileToFolder(dir, StringUtils::Format("screenshot{{:05}}{}.png", suffix)),
      65535);
  if (file.empty())
    CLog::Log(LOGWARNING, "Too many screen shots or invalid folder");
  else
    CLog::Log(LOGDEBUG, "Saving screenshot {}", CURL::GetRedacted(file));
  return file;
}

// Encode a delivered capture to the destination file; runs on the capture
// service's callback worker. With autoName, path is the folder, and the file in it is named once
// the pixels of a native screenshot tell whether they are display-mapped.
void WriteCapture(KODI::RENDERING::CAPTURE::CaptureResult result,
                  const std::string& path,
                  bool autoName = false)
{
  using namespace KODI::RENDERING::CAPTURE;
  if (!result.pixels)
  {
    CLog::Log(LOGERROR, "Unable to write screenshot {}: no pixels", CURL::GetRedacted(path));
    return;
  }

  CScopedCapturePixels lock(*result.pixels);
  if (!lock.data())
  {
    CLog::Log(LOGERROR, "Unable to write screenshot {}: pixel lock failed",
              CURL::GetRedacted(path));
    return;
  }

  std::string_view suffix;
  if (result.pixels->Describe(result))
    suffix = result.displayMapped ? "-native-mapped" : "-native-raw";
  const std::string file = autoName ? NextScreenshotName(path, suffix) : path;
  if (file.empty())
    return;

  // OPAQUE: the encoder drops the framebuffer alpha through an X-variant source
  const unsigned int format = CaptureXbFormat(result.format) | XB_FMT_OPAQUE;
  const uint8_t* src0 = CaptureSrcRow0(lock.data(), result.stride, result.height);

  if (!CPicture::CreateThumbnailFromSurface(src0, result.width, result.height,
                                            static_cast<unsigned int>(result.stride), file, format,
                                            result.color))
    CLog::Log(LOGERROR, "Unable to write screenshot {}", CURL::GetRedacted(file));
}
} // namespace

void CScreenShot::TakeScreenshot(const std::string& filename,
                                 KODI::RENDERING::CAPTURE::CaptureContent content,
                                 KODI::RENDERING::CAPTURE::CaptureScreenshot type)
{
  using namespace KODI::RENDERING::CAPTURE;

  const auto captureService = CServiceBroker::GetCaptureService();
  if (!captureService)
  {
    CLog::Log(LOGERROR, "Screenshot {} failed: no capture service", CURL::GetRedacted(filename));
    return;
  }

  CLog::Log(LOGDEBUG, "Saving screenshot {}", CURL::GetRedacted(filename));

  // screenshots are tagged captures whatever their content: native depth
  // kept, the display's coding carried as cICP by the writer
  CaptureSpec spec;
  spec.content = content;
  spec.format = CaptureFormat::NATIVE;
  spec.screenshot = type;

  // failure dispatches an empty result because some consumers must act on
  // it (a bookmark still writes its DB row); here WriteCapture declines it
  auto handle = captureService->Submit(spec, [filename](const CaptureResult& result)
                                       { WriteCapture(result, filename); });
  handle->Detach();
}

namespace
{
// Resolve the configured screenshot folder, prompting for it once if unset.
std::string ResolveScreenshotDir()
{
  std::shared_ptr<CSettingPath> setting = std::static_pointer_cast<CSettingPath>(
      CServiceBroker::GetSettingsComponent()->GetSettings()->GetSetting(
          CSettings::SETTING_DEBUG_SCREENSHOTPATH));
  if (!setting)
    return {};

  std::string dir = setting->GetValue();
  if (dir.empty())
  {
    if (!CGUIControlButtonSetting::GetPath(
            setting, &CServiceBroker::GetResourcesComponent().GetLocalizeStrings()))
      return {};
    dir = setting->GetValue();
  }

  URIUtils::RemoveSlashAtEnd(dir);
  return dir;
}
} // namespace

void CScreenShot::TakeScreenshot()
{
  using namespace KODI::RENDERING::CAPTURE;

  CaptureScreenshot type = CaptureScreenshot::STANDARD;
#if defined(HAS_LIBAMCODEC)
  if (CServiceBroker::GetSettingsComponent()->GetSettings()->GetInt(
          CSettings::SETTING_COREELEC_AMLOGIC_SCREENSHOTTYPE) == SCREENSHOT_TYPE_NATIVE)
    type = CaptureScreenshot::NATIVE;
#endif
  TakeScreenshot(CaptureContent::COMPOSITE, type);
}

void CScreenShot::TakeScreenshot(KODI::RENDERING::CAPTURE::CaptureContent content,
                                 KODI::RENDERING::CAPTURE::CaptureScreenshot type)
{
  using namespace KODI::RENDERING::CAPTURE;

  const auto captureService = CServiceBroker::GetCaptureService();
  if (!captureService)
    return;

  CaptureSpec spec;
  spec.content = content;
  spec.format = CaptureFormat::NATIVE;
  spec.screenshot = type;

  // Submitted before any filesystem work so the readback grabs the current frame;
  // the folder dialog (when the path is unset), naming, and write all run in the
  // callback afterward, so none of them delay the capture.
  auto handle =
      captureService->Submit(spec,
                             [](const CaptureResult& result)
                             {
                               if (!result.pixels)
                                 return;
                               const std::string dir = ResolveScreenshotDir();
                               if (dir.empty())
                               {
                                 CLog::Log(LOGWARNING, "No screenshot path configured");
                                 return;
                               }
                               WriteCapture(result, dir, true);
                             });
  handle->Detach();
}

void CScreenShot::TakeScreenshotBoth()
{
  using namespace KODI::RENDERING::CAPTURE;

  const auto captureService = CServiceBroker::GetCaptureService();
  if (!captureService)
    return;

  auto composite = std::make_shared<std::string>();
  CaptureSpec spec;
  spec.content = CaptureContent::BOTH;
  spec.format = CaptureFormat::NATIVE;
  auto handle = captureService->Submit(
      spec,
      [composite](const CaptureResult& result)
      {
        if (!result.pixels)
          return;
        if (composite->empty())
        {
          const std::string dir = ResolveScreenshotDir();
          if (dir.empty())
          {
            CLog::Log(LOGWARNING, "No screenshot path configured");
            return;
          }
          *composite =
              CUtil::GetNextFilename(URIUtils::AddFileToFolder(dir, "screenshot{:05}.png"), 65535);
        }
        if (composite->empty())
        {
          CLog::Log(LOGWARNING, "Too many screen shots or invalid folder");
          return;
        }
        // derive the video name from the same NNNNN so the pair is obvious
        const std::string file = result.content == CaptureContent::VIDEO
                                     ? composite->substr(0, composite->length() - 4) + "-video.png"
                                     : *composite;
        CLog::Log(LOGDEBUG, "Saving screenshot {}", CURL::GetRedacted(file));
        WriteCapture(result, file);
      });
  handle->Detach();
}
