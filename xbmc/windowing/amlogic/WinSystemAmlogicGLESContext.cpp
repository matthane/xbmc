/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "VideoSyncAML.h"
#include "WinSystemAmlogicGLESContext.h"
#include "cores/VideoPlayer/DVDCodecs/Video/AMLCodec.h"
#include "platform/linux/SysfsPath.h"
#include "ServiceBroker.h"
#include "guilib/GUIComponent.h"
#include "guilib/GUIWindowManager.h"
#include "guilib/IDirtyRegionSolver.h"
#include "jobs/JobManager.h"
#include "settings/AdvancedSettings.h"
#include "settings/Settings.h"
#include "settings/SettingsComponent.h"
#include "utils/AMLUtils.h"
#include "utils/MathUtils.h"
#include "utils/XTimeUtils.h"
#include "utils/log.h"
#include "threads/SingleLock.h"
#include "windowing/GraphicContext.h"
#include "windowing/WindowSystemFactory.h"

#include <optional>

extern "C"
{
#include <libavutil/pixfmt.h>
}

using namespace KODI;
using namespace KODI::WINDOWING::AML;
using namespace std::chrono_literals;

namespace
{
bool CoversSurface(const CRect& rect, int width, int height)
{
  return rect.x1 <= 0 && rect.y1 <= 0 && rect.x2 >= width && rect.y2 >= height;
}
} // namespace

// shared with the job, which may outlive the window system; nodes nobody wants are dropped
struct CWinSystemAmlogicGLESContext::CLut3DBuild
{
  void Run()
  {
    std::unique_lock lock(mutex);
    while (wanted && built != wanted)
    {
      const Lut3DKey key = *wanted;
      lock.unlock();
      std::vector<uint32_t> lut =
          CGuiCompositeShaderGLES::GenerateLUT3DNodes(key.colorTransfer, key.peak, key.limited);
      lock.lock();
      if (wanted == key)
      {
        built = key;
        nodes = std::move(lut);
      }
    }
    building = false;
  }

  std::mutex mutex;
  std::optional<Lut3DKey> wanted;
  std::optional<Lut3DKey> built;
  std::vector<uint32_t> nodes;
  bool building{false};
};

CWinSystemAmlogicGLESContext::CWinSystemAmlogicGLESContext()
: m_pGLContext(new CEGLContextUtils(EGL_PLATFORM_GBM_MESA, "EGL_EXT_platform_base")),
  m_lut3DBuild(std::make_shared<CLut3DBuild>())
{
}

void CWinSystemAmlogicGLESContext::Register()
{
  KODI::WINDOWING::CWindowSystemFactory::RegisterWindowSystem(CreateWinSystem, "aml");
}

std::unique_ptr<CWinSystemBase> CWinSystemAmlogicGLESContext::CreateWinSystem()
{
  return std::make_unique<CWinSystemAmlogicGLESContext>();
}

bool CWinSystemAmlogicGLESContext::InitWindowSystem()
{
  if (!CWinSystemAmlogic::InitWindowSystem())
  {
    return false;
  }

  if (!m_pGLContext->CreatePlatformDisplay(m_amlGBMUtils->GetDevice(), m_amlGBMUtils->GetDevice()))
  {
    m_pGLContext->Destroy();
    return false;
  }

  if (!m_pGLContext->InitializeDisplay(EGL_OPENGL_ES_API))
  {
    m_pGLContext->Destroy();
    return false;
  }

  EGLint renderableType{EGL_OPENGL_ES3_BIT};
  if (!m_pGLContext->ChooseConfig(renderableType))
  {
    renderableType = EGL_OPENGL_ES2_BIT;
    if (!m_pGLContext->ChooseConfig(renderableType))
    {
      m_pGLContext->Destroy();
      return false;
    }
  }

  CEGLAttributesVec contextAttribs;
  contextAttribs.Add({{EGL_CONTEXT_CLIENT_VERSION, (renderableType == EGL_OPENGL_ES3_BIT) ? 3 : 2}});

  if (!m_pGLContext->CreateContext(contextAttribs))
  {
    m_pGLContext->Destroy();
    return false;
  }

  m_eglBufferAge = m_pGLContext->HasBufferAge();

  if (CEGLUtils::HasExtension(GetEGLDisplay(), "EGL_ANDROID_native_fence_sync") &&
      CEGLUtils::HasExtension(GetEGLDisplay(), "EGL_KHR_fence_sync"))
  {
    m_eglFence = std::make_unique<KODI::UTILS::EGL::CEGLFence>(GetEGLDisplay());
  }

  return true;
}

bool CWinSystemAmlogicGLESContext::DestroyWindowSystem()
{
  ResetHdrGuiSession();
  m_compositeShader.reset();
  m_lut3DShader.reset();

  if (IsPresentationReady())
  {
    SetPresentationReady(false);
    m_amlDisplay->aml_set_drmDevice_active(false);
  }

  m_pGLContext->DestroyContext();
  m_pGLContext->Destroy();
  return CWinSystemAmlogic::DestroyWindowSystem();
}

bool CWinSystemAmlogicGLESContext::CreateNewWindow(const std::string& name,
                                               bool fullScreen,
                                               RESOLUTION_INFO& res)
{
  RESOLUTION_INFO current_resolution;
  current_resolution.iWidth = current_resolution.iHeight = 0;
  const RenderStereoMode stereo_mode = CServiceBroker::GetWinSystem()->GetGfxContext().GetStereoMode();

  // check for frac_rate_policy change
  int fractional_rate = (res.fRefreshRate == floor(res.fRefreshRate)) ? 0 : 1;
  int cur_fractional_rate = m_amlDisplay->aml_get_drmProperty("FRAC_RATE_POLICY", DRM_MODE_OBJECT_CONNECTOR);

  bool nativeGUI = CServiceBroker::GetSettingsComponent()->GetSettings()->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_DISABLEGUISCALING);

  StreamHdrType hdrType = CServiceBroker::GetWinSystem()->GetGfxContext().GetHDRType();
  bool force_mode_switch_by_hdr = (m_hdrType != hdrType);
  bool force_mode_switch_by_stereo_mode = (m_stereo_mode != stereo_mode);
  bool force_mode_switch_by_fractional_rate = (cur_fractional_rate != fractional_rate);
  bool force_mode_switch_by_hotplug = m_amlDisplay->GetHotPlug();

  // get current used resolution
  if (!m_amlDisplay->aml_get_native_resolution(&current_resolution))
  {
    CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext::{}: failed to receive current resolution", __FUNCTION__);
    SetPresentationReady(false);
    return false;
  }

  const std::string new_hdrStr = CStreamDetails::HdrTypeToString(hdrType);
  const std::string old_hdrStr = CStreamDetails::HdrTypeToString(m_hdrType);
  CLog::Log(LOGDEBUG,
            "CWinSystemAmlogicGLESContext::{}: "
            "m_bWindowCreated: {}, "
            "hdrType: {}({}), "
            "force mode switch by - hdr: {}, frac rate: {}, stereo mode: {}, hotplug: {}",
            __FUNCTION__, m_bWindowCreated, new_hdrStr.empty() ? "none" : new_hdrStr,
            old_hdrStr.empty() ? "none" : old_hdrStr, force_mode_switch_by_hdr,
            force_mode_switch_by_fractional_rate, force_mode_switch_by_stereo_mode,
            force_mode_switch_by_hotplug);
  CLog::Log(LOGDEBUG, "CWinSystemAmlogicGLESContext::{}: "
    "cur: iWidth: {:04d}, iHeight: {:04d}, iScreenWidth: {:04d}, iScreenHeight: {:04d}, fRefreshRate: {:02.2f}, dwFlags: {:02x}, nativeGUI: {}",
    __FUNCTION__,
    current_resolution.iWidth, current_resolution.iHeight, current_resolution.iScreenWidth, current_resolution.iScreenHeight,
    current_resolution.fRefreshRate, current_resolution.dwFlags, m_nativeGUI);
  CLog::Log(LOGDEBUG, "CWinSystemAmlogicGLESContext::{}: "
    "res: iWidth: {:04d}, iHeight: {:04d}, iScreenWidth: {:04d}, iScreenHeight: {:04d}, fRefreshRate: {:02.2f}, dwFlags: {:02x}, nativeGUI: {}",
    __FUNCTION__,
    res.iWidth, res.iHeight, res.iScreenWidth, res.iScreenHeight, res.fRefreshRate, res.dwFlags, nativeGUI);

  // check if mode switch is needed
  if (current_resolution.iWidth == res.iWidth && current_resolution.iHeight == res.iHeight &&
      current_resolution.iScreenWidth == res.iScreenWidth &&
      current_resolution.iScreenHeight == res.iScreenHeight && m_bFullScreen == fullScreen &&
      current_resolution.fRefreshRate == res.fRefreshRate &&
      (current_resolution.dwFlags & D3DPRESENTFLAG_MODEMASK) ==
          (res.dwFlags & D3DPRESENTFLAG_MODEMASK) &&
      m_bWindowCreated && nativeGUI == m_nativeGUI && !force_mode_switch_by_hdr &&
      !force_mode_switch_by_hotplug && !force_mode_switch_by_fractional_rate &&
      !force_mode_switch_by_stereo_mode)
  {
    CLog::Log(LOGDEBUG, "CWinSystemAmlogicGLESContext::{}: No need to create a new window", __FUNCTION__);
    return true;
  }

  // destroy old window, then create a new one
  DestroyWindow();

  // check if a forced mode switch is required
  if (force_mode_switch_by_hotplug)
  {
    m_force_mode_switch = true;
    m_hotplug_mode_switch = true;
  }
  else if (force_mode_switch_by_stereo_mode)
  {
    m_force_mode_switch = true;
  }
  else
  if (current_resolution.iWidth == res.iWidth && current_resolution.iHeight == res.iHeight &&
      current_resolution.iScreenWidth == res.iScreenWidth && current_resolution.iScreenHeight == res.iScreenHeight &&
      MathUtils::FloatEquals(current_resolution.fRefreshRate, res.fRefreshRate, 0.06f))
  {
    // same resolution, check frac rate and other parameter
    if (force_mode_switch_by_fractional_rate || force_mode_switch_by_hdr)
      m_force_mode_switch = true;
  }

  if (m_force_mode_switch)
    CLog::Log(LOGDEBUG, "CWinSystemAmlogicGLESContext::{}: force mode switch", __FUNCTION__);

  // refresh backup data
  m_hdrType = hdrType;
  m_stereo_mode = stereo_mode;
  m_bFullScreen = fullScreen;
  m_nativeGUI = nativeGUI;

  if (!CWinSystemAmlogic::CreateNewWindow(name, fullScreen, res))
  {
    return false;
  }

  uint32_t format = m_pGLContext->GetConfigAttrib(EGL_NATIVE_VISUAL_ID);
  if (!m_amlGBMUtils->CreateSurface(res.iWidth, res.iHeight, format))
  {
    CLog::Log(LOGDEBUG, "CWinSystemAmlogicGLESContext::{} - failed to create GBM surface", __FUNCTION__);
    DestroyWindow();
    return false;
  }

  if (!m_pGLContext->CreatePlatformSurface(
          m_amlGBMUtils->GetSurface(),
          reinterpret_cast<EGLNativeWindowType>(m_amlGBMUtils->GetSurface())))
  {
    CLog::Log(LOGDEBUG, "CWinSystemAmlogicGLESContext::{} - failed to create CreatePlatformSurface", __FUNCTION__);
    DestroyWindow();
    return false;
  }

  if (!m_pGLContext->BindContext())
  {
    DestroyWindow();
    return false;
  }

  // a display that is plugged in later gets the composite before its first HDR start
  if (force_mode_switch_by_hotplug && m_bRenderCreated)
    PrecompileGuiComposite();

  if (!m_delayDispReset)
  {
    std::unique_lock<CCriticalSection> lock(m_resourceSection);
    // tell any shared resources
    for (std::vector<IDispResource *>::iterator i = m_resources.begin(); i != m_resources.end(); ++i)
      (*i)->OnResetDisplay();
  }

  if (m_amlDisplay->aml_get_display_connected())
    SetPresentationReady(true);

  return true;
}

bool CWinSystemAmlogicGLESContext::DestroyWindow()
{
  m_pGLContext->DestroySurface();
  return CWinSystemAmlogic::DestroyWindow();
}

bool CWinSystemAmlogicGLESContext::ResizeWindow(int newWidth, int newHeight, int newLeft, int newTop)
{
  CRenderSystemGLES::ResetRenderSystem(newWidth, newHeight);
  return true;
}

bool CWinSystemAmlogicGLESContext::SetFullScreen(bool fullScreen, RESOLUTION_INFO& res, bool blankOtherDisplays)
{
  if (!CreateNewWindow("", fullScreen, res))
    return false;

  CRenderSystemGLES::ResetRenderSystem(res.iWidth, res.iHeight);
  return true;
}

void CWinSystemAmlogicGLESContext::SetVSyncImpl(bool enable)
{
  if (!m_pGLContext->SetVSync(enable))
  {
    CLog::Log(LOGERROR, "{},Could not set egl vsync", __FUNCTION__);
  }
}

void CWinSystemAmlogicGLESContext::PresentRender(bool rendered, bool videoLayer)
{
  if (IsHotplugPending() || !IsPresentationReady())
  {
    // the next render cannot build on what this one drew
    if (rendered)
      m_unswapped = true;
    KODI::TIME::Sleep(10ms);
    return;
  }

  SetVSync(true);
  if (rendered)
  {
#if defined(EGL_ANDROID_native_fence_sync) && defined(EGL_KHR_fence_sync)
    if (m_eglFence)
    {
      int fd = m_amlDisplay->TakeOutFenceFd();
      if (fd != -1)
      {
        m_eglFence->CreateKMSFence(fd);
        m_eglFence->WaitSyncGPU();
      }

      m_eglFence->CreateGPUFence();
    }
#endif

    // eglSwapBuffers() sometimes fails during modeswaps on AML, there is probably
    // nothing we can do about it but redraw in full next time
    if (m_pGLContext->TrySwapBuffers())
    {
      m_swapCount++;
      if (m_frameForcedFull)
      {
        m_fullRedrawSwap = m_swapCount;
        m_fullRedrawDone = m_frameFullRedraw;
      }
      m_unswapped = false;
    }
    else
    {
      m_unswapped = true;
    }

#if defined(EGL_ANDROID_native_fence_sync) && defined(EGL_KHR_fence_sync)
    if (m_eglFence)
    {
      int fd = m_eglFence->FlushFence();
      m_amlDisplay->SetInFenceFd(fd);

      m_eglFence->WaitSyncCPU();
    }
#endif

    if (m_amlGBMUtils && m_amlGBMUtils->LockFrontBuffer(m_amlDisplay->aml_get_Device_handle()))
    {
      // this flip latches no earlier than the vsync after the last step: pace on the next one
      m_presentStepSeen = CAMLCodec::PresentSteps();
      m_amlDisplay->FlipPage(m_amlGBMUtils->GetFBId());
    }
  }
  else if (!videoLayer)
  {
    m_amlDisplay->aml_drmDevice_vsync();
  }

  // video frames reach the plane from the vsync thread, so the loop paces on its steps; while
  // it has no vsync, a loop that did not flip waits for the vblank as it does without video
  if (!CAMLCodec::WaitPresentStep(m_presentStepSeen,
                                  rendered ? m_amlDisplay->GetOutFenceFd() : -1) &&
      !rendered && videoLayer)
    m_amlDisplay->aml_drmDevice_vsync();

  if (m_delayDispReset && m_dispResetTimer.IsTimePast())
  {
    m_delayDispReset = false;
    std::unique_lock<CCriticalSection> lock(m_resourceSection);
    // tell any shared resources
    for (std::vector<IDispResource *>::iterator i = m_resources.begin(); i != m_resources.end(); ++i)
      (*i)->OnResetDisplay();
    // the player clock runs again: pick now, not a vsync later
    CAMLCodec::RequestVsyncStep();
  }
}

void CWinSystemAmlogicGLESContext::DecideBufferAge(bool guiWillRender)
{
  // the stock value, under the other algorithms and while partial redraw cannot work
  m_frameBufferAge = 2;
  m_frameForcedFull = false;
  m_frameDamage = CRect();
  // regions age once per Render call, so they match the buffers only with one Render
  // call per swap, which stereo breaks
  m_canRedrawPartially =
      m_eglBufferAge && GetGfxContext().GetStereoMode() == RenderStereoMode::OFF;
  m_partialFrame =
      guiWillRender &&
      CServiceBroker::GetSettingsComponent()->GetAdvancedSettings()->m_guiAlgorithmDirtyRegions ==
          DIRTYREGION_SOLVER_UNION;
  // no region tracks what the buffers presented meanwhile hold
  if (m_partialFrame && !m_canRedrawPartially)
  {
    m_partialFrame = false;
    m_fullRedrawRequest++;
  }
  m_frameFullRedraw = m_fullRedrawRequest;
  if (!m_partialFrame)
    return;

  // an unswapped frame aged the regions for nothing, and the window manager draws the
  // dirty region overlay in a full pass
  m_frameForcedFull = m_fullRedrawRequest != m_fullRedrawDone || m_unswapped;
  if (m_frameForcedFull ||
      CServiceBroker::GetSettingsComponent()->GetAdvancedSettings()->m_guiVisualizeDirtyRegions)
  {
    m_frameBufferAge = 0;
    return;
  }

  // a buffer last presented before the latest forced full redraw predates its cause
  const int age = m_pGLContext->GetBufferAge();
  m_frameBufferAge = static_cast<uint64_t>(age) > m_swapCount - m_fullRedrawSwap + 1 ? 0 : age;
}

int CWinSystemAmlogicGLESContext::GetBufferAge()
{
  return m_frameBufferAge;
}

bool CWinSystemAmlogicGLESContext::CanRedrawPartially() const
{
  return m_canRedrawPartially;
}

std::optional<CRect> CWinSystemAmlogicGLESContext::GetFullRedrawArea() const
{
  // Mali reloads every damaged tile of a partial redraw; a full one shades the whole
  // surface, or in the HDR composite only the bounds the GUI draws
  if (m_guiCompositing)
    return m_guiDrawnBounds;
  return CRect(0, 0, m_nWidth, m_nHeight);
}

bool CWinSystemAmlogicGLESContext::RedrawsFullScreen() const
{
  return !m_partialFrame || m_frameBufferAge == 0 ||
         CoversSurface(m_frameDamage, m_nWidth, m_nHeight);
}

void CWinSystemAmlogicGLESContext::SetDirtyRegions(const CDirtyRegionList& dirtyRegions)
{
  if (CServiceBroker::GetSettingsComponent()->GetAdvancedSettings()->m_guiAlgorithmDirtyRegions !=
      DIRTYREGION_SOLVER_UNION)
    return;

  m_pGLContext->SetDamagedRegions(dirtyRegions);

  for (const auto& region : dirtyRegions)
    m_frameDamage.Union(region);

  if (!m_guiPassInFbo)
    return;

  m_guiDamaged = true;
  // a clean FBO needs no clear; clearing all of it for a smaller damage would write it
  // all back on every frame
  const bool whole = CoversSurface(m_frameDamage, m_nWidth, m_nHeight);
  if (!m_guiFboClean)
  {
    if (whole)
      ResetScissors();
    else
      SetScissors(m_frameDamage);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
  }
  m_guiFboClean = m_guiFboClean || whole;
}

bool CWinSystemAmlogicGLESContext::SetGuiCompositing(int colorTransfer)
{
  // the composite output changes everywhere, and no dirty region says so
  m_fullRedrawRequest++;
  m_guiCompositing = (colorTransfer != 0);

  if (m_guiCompositing)
  {
    // GUI reference white follows videoscreen.guipeakluminance instead of the
    // composite's hardcoded 203 nits, so the setting is a working brightness
    // control here too and means the same thing as on the per-primitive path.
    // At the shipped default this is ~199 nits, i.e. a <2% change from before.
    const float peak(CGuiCompositeShaderGLES::PeakFromPQCode(GetGuiSdrPeakLuminance()));
    const bool limited = UseLimitedColor();

    // chosen on every call: a session keeps its owner across a renderer
    // reconfigure, so the transfer can change within one session; the 3D LUT
    // folds the sRGB decode and the transfer into its one fetch, so both prefer it
    // once its nodes are built
    if (!BuildGuiComposite(colorTransfer, peak, limited))
    {
      m_compositeShader.reset();
      m_lut3DShader.reset();
      CancelLut3D();
      m_guiCompositing = false;
      return false;
    }
    m_guiCompositeTransfer = colorTransfer;
    m_guiCompositePeak = peak;
    // only a live step may keep drawing the 3D LUT of an earlier peak
    if (m_lut3DKey.peak != peak)
      m_lut3DKey = {};
    m_lut3D = m_lut3DShader &&
              (colorTransfer == AVCOL_TRC_SMPTE2084 || colorTransfer == AVCOL_TRC_ARIB_STD_B67);
    RequestLut3D();
  }
  else
  {
    m_guiFbo.Cleanup();
    m_guiFboWidth = 0;
    m_guiFboHeight = 0;
    CancelLut3D();
  }

  return m_guiCompositing;
}

bool CWinSystemAmlogicGLESContext::InitRenderSystem()
{
  if (!CRenderSystemGLES::InitRenderSystem())
    return false;

  PrecompileGuiComposite();
  return true;
}

void CWinSystemAmlogicGLESContext::PrecompileGuiComposite()
{
  // a compile takes a frame or more, which no HDR playback start should pay; a display
  // without HDR compiles at its first use, and a running composite keeps its programs
  if (IsHDRDisplay() && !m_guiCompositing)
    CompileGuiComposite(UseLimitedColor());
}

bool CWinSystemAmlogicGLESContext::CompileGuiComposite(bool limited)
{
  if (m_RenderVersionMajor >= 3 && IsExtSupported("GL_OES_texture_3D") && !m_lut3DShader)
  {
    auto shader =
        std::make_unique<CGuiCompositeShaderGLES>("", CGuiCompositeShaderGLES::Input::LUT3D);
    if (shader->CompileAndLink())
    {
      m_lut3DShader = std::move(shader);
      m_lut3DKey = {};
    }
    else
      CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext: failed to compile GUI composite shader");
  }

  if (!m_compositeShader || m_guiCompositeLimited != limited)
  {
    std::string defines;
    if (limited)
      defines += "#define KODI_LIMITED_RANGE 1\n";
    auto shader = std::make_unique<CGuiCompositeShaderGLES>(defines);
    if (!shader->CompileAndLink())
    {
      CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext: failed to compile GUI composite shader");
      return false;
    }
    m_compositeShader = std::move(shader);
    m_guiCompositeLimited = limited;
  }
  return true;
}

bool CWinSystemAmlogicGLESContext::BuildGuiComposite(int colorTransfer, float peak, bool limited)
{
  if (!CompileGuiComposite(limited))
    return false;

  m_compositeShader->SetSdrPeak(peak);
  if (!m_compositeShader->CreateLUTs(colorTransfer))
  {
    CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext: failed to create LUTs");
    return false;
  }
  return true;
}

CGuiCompositeShaderGLES& CWinSystemAmlogicGLESContext::GetCompositeShader() const
{
  // a 3D LUT for an earlier peak draws until the one for the new peak is loaded
  if (m_lut3DShader && m_lut3DKey.colorTransfer == m_guiCompositeTransfer &&
      m_lut3DKey.limited == m_guiCompositeLimited)
    return *m_lut3DShader;
  return *m_compositeShader;
}

void CWinSystemAmlogicGLESContext::RequestLut3D()
{
  const Lut3DKey key{m_guiCompositeTransfer, m_guiCompositePeak, m_guiCompositeLimited};
  if (!m_lut3D || (m_lut3DShader && m_lut3DKey == key))
  {
    CancelLut3D();
    return;
  }

  m_lut3DPending = true;
  std::unique_lock lock(m_lut3DBuild->mutex);
  if (m_lut3DBuild->wanted == key && (m_lut3DBuild->building || m_lut3DBuild->built))
    return;
  m_lut3DBuild->wanted = key;
  m_lut3DBuild->built.reset();
  if (m_lut3DBuild->building)
    return;
  m_lut3DBuild->building = true;
  lock.unlock();

  // the nodes take a frame or more of CPU time, which the GUI thread must not stall on
  if (!CServiceBroker::GetJobManager()->AddJob(
          new CLambdaJob([build = m_lut3DBuild]() { build->Run(); }), nullptr,
          CJob::PRIORITY_HIGH))
  {
    lock.lock();
    m_lut3DBuild->building = false;
    m_lut3DBuild->wanted.reset();
    m_lut3DPending = false;
  }
}

void CWinSystemAmlogicGLESContext::LoadLut3D()
{
  Lut3DKey key;
  std::vector<uint32_t> nodes;
  {
    std::unique_lock lock(m_lut3DBuild->mutex);
    if (!m_lut3DBuild->built)
      return;
    key = *m_lut3DBuild->built;
    nodes = std::move(m_lut3DBuild->nodes);
    m_lut3DBuild->built.reset();
  }
  m_lut3DPending = false;

  m_lut3DShader->SetSdrPeak(key.peak);
  if (!m_lut3DShader->CreateLUTs(key.colorTransfer, nodes))
  {
    // the analytic composite is built for the current key
    CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext: failed to create LUTs");
    key = {};
  }
  m_lut3DKey = key;
  m_fullRedrawRequest++;

  // a skipped frame shows the composite of the last drawn one
  if (!m_guiWillRender)
    CServiceBroker::GetGUI()->GetWindowManager().MarkDirty();
}

void CWinSystemAmlogicGLESContext::CancelLut3D()
{
  m_lut3DPending = false;
  std::unique_lock lock(m_lut3DBuild->mutex);
  m_lut3DBuild->wanted.reset();
  m_lut3DBuild->built.reset();
  m_lut3DBuild->nodes = std::vector<uint32_t>();
}

bool CWinSystemAmlogicGLESContext::SetDvGraphicFormat(unsigned int format)
{
  CSysfsPath graphicFormat{"/sys/class/amdolby_vision/graphic_fmt"};
  if (!graphicFormat.Exists())
    return false;

  try
  {
    graphicFormat.Set(format);
    return true;
  }
  catch (const std::exception& e)
  {
    CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext: failed to set DV graphic format: {}",
              e.what());
    return false;
  }
}

bool CWinSystemAmlogicGLESContext::SetDvGraphicsState(bool enabled)
{
  return SetDvGraphicFormat(enabled ? 1 /* FORMAT_HDR10 */ : 2 /* FORMAT_SDR */);
}

uint64_t CWinSystemAmlogicGLESContext::ConfigureHdrGuiSession(uint64_t owner,
                                                              int colorTransfer,
                                                              bool dvGraphics)
{
  std::lock_guard lock(m_hdrGuiMutex);

  if (owner == 0 || owner != m_hdrGuiOwner)
  {
    owner = ++m_hdrGuiNextOwner;
    if (owner == 0)
      owner = ++m_hdrGuiNextOwner;
  }

  // the FBO composite performs the GUI transfer; the direct late PGS pass
  // must therefore use the texture shader without its SDR-peak multiplier
  GetGfxContext().SetTransferPQ(false);

  if (!SetGuiCompositing(colorTransfer))
  {
    if (m_hdrGuiDvGraphics)
      SetDvGraphicsState(false);
    m_hdrGuiOwner = 0;
    m_hdrGuiDvGraphics = false;
    return 0;
  }

  if (dvGraphics != m_hdrGuiDvGraphics)
  {
    if (!SetDvGraphicsState(dvGraphics))
    {
      SetGuiCompositing(0);
      if (m_hdrGuiDvGraphics)
        SetDvGraphicsState(false);
      m_hdrGuiOwner = 0;
      m_hdrGuiDvGraphics = false;
      return 0;
    }
  }

  m_hdrGuiOwner = owner;
  m_hdrGuiDvGraphics = dvGraphics;
  return owner;
}

void CWinSystemAmlogicGLESContext::ReleaseHdrGuiSession(uint64_t owner)
{
  std::lock_guard lock(m_hdrGuiMutex);
  if (owner == 0 || owner != m_hdrGuiOwner)
    return;

  SetGuiCompositing(0);
  if (m_hdrGuiDvGraphics)
    SetDvGraphicsState(false);
  m_hdrGuiOwner = 0;
  m_hdrGuiDvGraphics = false;
}

void CWinSystemAmlogicGLESContext::ResetHdrGuiSession()
{
  std::lock_guard lock(m_hdrGuiMutex);
  SetGuiCompositing(0);
  if (m_hdrGuiDvGraphics)
    SetDvGraphicsState(false);
  m_hdrGuiOwner = 0;
  m_hdrGuiDvGraphics = false;
}

bool CWinSystemAmlogicGLESContext::BeginGuiComposite(bool guiWillRender)
{
  DecideBufferAge(guiWillRender);
  m_guiPassInFbo = false;
  m_guiDamaged = false;

  if (!m_guiCompositing)
    return false;

  m_guiWillRender = guiWillRender;

  int width = m_nWidth;
  int height = m_nHeight;

  // create or recreate FBO if size changed
  if (!m_guiFbo.IsValid() || m_guiFboWidth != width || m_guiFboHeight != height)
  {
    m_guiFbo.Cleanup();

    if (!m_guiFbo.Initialize())
    {
      CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext: failed to initialize GUI FBO");
      return false;
    }

    if (!m_guiFbo.CreateAndBindToTexture(GL_TEXTURE_2D, width, height, GL_RGBA))
    {
      CLog::Log(LOGERROR, "CWinSystemAmlogicGLESContext: failed to create GUI FBO texture {}x{}", width,
                height);
      m_guiFbo.Cleanup();
      return false;
    }

    if (GetEnabledFrontToBackRendering() && !m_guiFbo.AttachDepthBuffer(width, height))
    {
      CLog::Log(LOGERROR,
                "CWinSystemAmlogicGLESContext: failed to attach depth buffer to GUI FBO {}x{}", width,
                height);
      m_guiFbo.Cleanup();
      return false;
    }

    m_guiFboWidth = width;
    m_guiFboHeight = height;
    m_guiFboClean = false; // fresh FBO is undefined, force a clear
    m_fullRedrawRequest++;
    CLog::Log(LOGDEBUG, "CWinSystemAmlogicGLESContext: created GUI FBO {}x{}", width, height);
  }

  // When GUI render is being skipped, leave the FBO bind/clear out: nothing
  // will draw into it this frame. The FBO's prior sRGB GUI content is
  // implicitly preserved across the skipped frame as a side effect.
  //! @todo The preserved sRGB FBO is currently not leveraged: D2P reuses the
  //! post-PQ GUI plane back buffer directly via display HW, and single-plane
  //! never reaches !guiWillRender (the dirty-driven skip is gated on
  //! IsRenderingVideoLayer).
  if (m_lut3DPending)
    LoadLut3D();

  if (!guiWillRender)
    return true;

  // Pick up a live guipeakluminance change so GUI brightness can be dialled in
  // while a disc is playing, rather than only at the next stream start. One
  // settings read per composited frame - cheaper than the per-primitive path,
  // which already reads it on every shader enable (CGLESShader::OnEnabled) - and
  // the LUT is rebuilt only when the value actually moves.
  if (m_compositeShader)
  {
    const float peak(CGuiCompositeShaderGLES::PeakFromPQCode(GetGuiSdrPeakLuminance()));
    if (peak != m_guiCompositePeak)
    {
      m_compositeShader->SetSdrPeak(peak);
      const bool rebuilt = m_compositeShader->CreateLUTs(m_guiCompositeTransfer);
      if (!rebuilt)
      {
        // CreateLUTs commits only on success, so the previous LUTs are still
        // live and the GUI keeps rendering correctly at the old reference white.
        // Put the shader's peak back in step with them, but still record the
        // requested value so a failed rebuild is not retried on every frame.
        CLog::Log(LOGWARNING, "CWinSystemAmlogicGLESContext: GUI peak luminance change "
                              "rejected, keeping the previous reference white");
        m_compositeShader->SetSdrPeak(m_guiCompositePeak);
      }
      m_guiCompositePeak = peak;
      // a 3D LUT for the old peak draws until the new one is loaded
      if (&GetCompositeShader() == m_compositeShader.get())
        m_fullRedrawRequest++;
      if (rebuilt)
        RequestLut3D();
    }
  }

  // the FBO or the LUTs changed after this frame's age was decided
  if (m_fullRedrawRequest != m_frameFullRedraw)
    DecideBufferAge(guiWillRender);

  if (!m_guiFbo.BeginRender())
    return false;

  // Clear only when the FBO holds stale content; idle frames are already clean.
  // In partial redraw only a full redraw clears it all; SetDirtyRegions clears
  // the damage of a partial one.
  if (!m_guiFboClean && (!m_partialFrame || m_frameBufferAge == 0))
  {
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    m_guiFboClean = true;
  }

  m_guiPassInFbo = true;
  return true;
}

void CWinSystemAmlogicGLESContext::EndGuiComposite()
{
  // an unpresented frame gets no draws
  if (!m_guiWillRender)
    return;

  // a partial redraw with no damage drew nothing and presents nothing
  if (m_partialFrame && m_frameBufferAge != 0 && !m_guiDamaged)
  {
    m_guiFbo.EndRender();
    m_guiWillRender = false;
    return;
  }

  // taken before the raw HDR PGS pass, which draws on the surface, not into the FBO
  m_guiCompositeBounds = GetGUIDrawBounds();
  m_guiDrawnBounds = m_guiCompositeBounds;

  // every GUI pass clears the depth it tests against, so the FBO need not keep it; kept,
  // a partial pass has to load it back
  if (m_RenderVersionMajor >= 3)
  {
    const GLenum depth = GL_DEPTH_ATTACHMENT;
    glInvalidateFramebuffer(GL_FRAMEBUFFER, 1, &depth);
  }
  m_guiFbo.EndRender();

  // outside the damage the buffer still holds an older frame
  if (m_guiDamaged && !CoversSurface(m_frameDamage, m_nWidth, m_nHeight))
  {
    SetScissors(m_frameDamage);
    m_guiCompositeBounds.Intersect(m_frameDamage);
  }

  // Clear the backbuffer before video renders. In the FBO compositing path,
  // video renders in the RenderEx pass with clear=false, so DrawBlackBars is
  // never called. Without this clear, letterbox areas retain stale content
  // from the swap chain when the display resolution doesn't change between
  // GUI and video playback.
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT);
}

// CompositeGui is the last GL operation in the frame (called just before EndRender).
// GL state (blend mode, active texture, vertex arrays) is not restored afterward;
// the next frame's rendering sets its own state.
void CWinSystemAmlogicGLESContext::CompositeGui()
{
  // the next frame's clears must not be clipped to the damage
  if (m_guiDamaged)
    ResetScissors();

  if (!m_guiFbo.IsValid() || !m_guiFbo.IsBound() || !m_compositeShader)
    return;
  CGuiCompositeShaderGLES& shader = GetCompositeShader();

  // Only update m_guiFboClean when GUI render fired this frame; otherwise the
  // FBO is in the same state as the previous frame and the flag stays as-is.
  // m_guiFboClean means "FBO is empty/clean" (no composite work needed).
  if (m_guiWillRender)
  {
    // bounds, not the element count: add-on renders and opaque clears are not counted;
    // outside the damage of a partial redraw the FBO is clean only if it was before
    const bool guiEmpty = m_guiCompositeBounds.IsEmpty();
    m_guiFboClean = guiEmpty && (m_guiFboClean || !m_guiDamaged);
    if (guiEmpty)
      return;
  }
  else
  {
    return;
  }

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, m_guiFbo.Texture());

  glEnable(GL_BLEND);
  // source-over the converted GUI while preserving destination alpha from
  // the raw PGS pass underneath it
  glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE,
                      GL_ONE_MINUS_SRC_ALPHA);

  // set up orthographic projection (screen coords, Y-down)
  float w = static_cast<float>(m_guiFboWidth);
  float h = static_cast<float>(m_guiFboHeight);

  GLfloat proj[16] = {2.0f / w, 0, 0, 0, 0, -2.0f / h, 0, 0, 0, 0, -1, 0, -1.0f, 1.0f, 0, 1};

  shader.SetProjection(proj);
  shader.Enable();

  GLint posLoc = shader.GetPosLoc();
  GLint texLoc = shader.GetTexLoc();

  // outside the bounds the FBO is transparent, which the shader would discard
  const CRect& b = m_guiCompositeBounds;
  GLfloat vert[4][2] = {{b.x1, b.y1}, {b.x2, b.y1}, {b.x2, b.y2}, {b.x1, b.y2}};
  GLfloat tex[4][2] = {{b.x1 / w, 1 - b.y1 / h},
                       {b.x2 / w, 1 - b.y1 / h},
                       {b.x2 / w, 1 - b.y2 / h},
                       {b.x1 / w, 1 - b.y2 / h}};
  GLubyte idx[4] = {0, 1, 3, 2};

  glVertexAttribPointer(posLoc, 2, GL_FLOAT, GL_FALSE, 0, vert);
  glVertexAttribPointer(texLoc, 2, GL_FLOAT, GL_FALSE, 0, tex);
  glEnableVertexAttribArray(posLoc);
  glEnableVertexAttribArray(texLoc);

  glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_BYTE, idx);

  glDisableVertexAttribArray(posLoc);
  glDisableVertexAttribArray(texLoc);

  shader.Disable();
}

EGLDisplay CWinSystemAmlogicGLESContext::GetEGLDisplay() const
{
  return m_pGLContext->GetEGLDisplay();
}

EGLSurface CWinSystemAmlogicGLESContext::GetEGLSurface() const
{
  return m_pGLContext->GetEGLSurface();
}

EGLContext CWinSystemAmlogicGLESContext::GetEGLContext() const
{
  return m_pGLContext->GetEGLContext();
}

EGLConfig  CWinSystemAmlogicGLESContext::GetEGLConfig() const
{
  return m_pGLContext->GetEGLConfig();
}

std::unique_ptr<CVideoSync> CWinSystemAmlogicGLESContext::GetVideoSync(CVideoReferenceClock *clock)
{
  std::unique_ptr<CVideoSync> pVSync(new CVideoSyncAML(clock));
  return pVSync;
}

bool CWinSystemAmlogicGLESContext::BindTextureUploadContext()
{
  // without a usable shared context the texture is uploaded at its first draw, as before
  if (!m_pGLContext->HasUploadContext() || m_uploadContextFailed)
    return false;

  if (m_pGLContext->BindTextureUploadContext())
    return true;

  m_uploadContextFailed = true;
  return false;
}

bool CWinSystemAmlogicGLESContext::UnbindTextureUploadContext()
{
  return m_pGLContext->UnbindTextureUploadContext();
}

bool CWinSystemAmlogicGLESContext::HasContext()
{
  return m_pGLContext->HasContext();
}

bool CWinSystemAmlogicGLESContext::SupportsStereo(const RenderStereoMode mode) const
{
  if (m_amlDisplay->aml_display_support_3d() &&
      mode == RenderStereoMode::HARDWAREBASED) {
    // yes, we support hardware based MVC decoding
    return true;
  }

  return CRenderSystemGLES::SupportsStereo(mode);
}
