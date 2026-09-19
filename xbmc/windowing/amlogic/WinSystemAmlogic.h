/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "AMLDisplay.h"
#include "DynamicDll.h"

#include "platform/linux/input/LibInputHandler.h"
#include "rendering/gles/RenderSystemGLES.h"
#include "threads/CriticalSection.h"
#include "windowing/WinSystem.h"
#include "threads/SystemClock.h"
#include "system_egl.h"
#include "utils/EGLFence.h"
#include "utils/EGLUtils.h"

#include <atomic>
#include <chrono>
#include <gbm.h>

class IDispResource;

class CWinSystemAmlogic : public CWinSystemBase
{
public:
  CWinSystemAmlogic();
  virtual ~CWinSystemAmlogic();

  bool InitWindowSystem() override;
  bool DestroyWindowSystem() override;

  bool CreateNewWindow(const std::string& name,
                       bool fullScreen,
                       RESOLUTION_INFO& res) override;

  bool DestroyWindow() override;
  bool MessagePump() override;
  void UpdateResolutions() override;
  bool IsHDRDisplay() override;
  CHDRCapabilities GetDisplayHDRCapabilities() const override;
  float GetGuiSdrPeakLuminance() const override;
  bool IsHdrSubtitlePlaneActive() const override;
  bool EnsureHdrSubtitlePlane() override;

  void EngageOSDBackend(bool engage, bool duringDv = false);
  void SetOSDBackendDVEngagePending();
  void TickOSDBackendPending();

  // OSD2 subtitle plane, render thread only. GetOSD2BackBuffer returns
  // the off-scanout buffer and its stable slot. PresentOSD2Frame stages
  // that slot for the next GUI atomic; it does not report scanout completion.
  bool ArmOSD2Plane();
  bool GetOSD2BackBuffer(void** map,
                         uint32_t* stride,
                         uint32_t* width,
                         uint32_t* height,
                         uint32_t* bufferIndex);
  bool PresentOSD2Frame();
  void DisableOSD2();

  HDR_STATUS GetOSHDRStatus() override;

  virtual void Register(IDispResource *resource);
  virtual void Unregister(IDispResource *resource);

  static void SettingOptionsComponentsFiller(const std::shared_ptr<const CSetting>& setting,
                                             std::vector<IntegerSettingOption>& list,
                                             int& current);

  void MonitorStart();
  void MonitorStop();

  CAMLDisplay* GetAmlDisplay() const { return m_amlDisplay.get(); }
protected:
  std::string m_framebuffer_name;
  bool IsHotplugPending() const { return m_hotplugPending.load(); }
  bool IsPresentationReady() const { return m_presentationReady; }
  void SetPresentationReady(bool ready) { m_presentationReady = ready; }

  EGLDisplay m_nativeDisplay;

  RenderStereoMode m_stereo_mode;

  bool m_delayDispReset;
  XbmcThreads::EndTime<> m_dispResetTimer;

  CCriticalSection m_resourceSection;
  std::vector<IDispResource*> m_resources;
  std::unique_ptr<CLibInputHandler> m_libinput;
  bool m_force_mode_switch;
  bool m_hotplug_mode_switch{false};
  bool m_presentationReady{false};
  bool m_nativeGUI;
  bool m_osdHdrSubtitleActive{false};
  bool m_osd2Armed{false};
  int m_osd2Front{-1}; // scanout buffer index, -1 = none shown
  bool m_dvEngagePending{false};
  bool m_osdHdrEngagedViaDv{false};
  std::chrono::steady_clock::time_point m_dvPendingSince{};
  static std::unique_ptr<CAMLDisplay> m_amlDisplay;
  std::unique_ptr<CAMLGBMUtils> m_amlGBMUtils{nullptr};
  std::unique_ptr<KODI::UTILS::EGL::CEGLFence> m_eglFence{nullptr};
private:
  struct callback_data
  {
    struct udev_monitor* udevMonitor;
    CWinSystemAmlogic* object;
  };

  void RefreshResolutions();
  void HotplugEvent();
  void RefreshDisplayCapabilities();
  static void FDEventCallback(int id, int fd, short revents, void *data);

  int m_fdMonitorId;

  struct udev *m_udev;
  struct callback_data m_callback_data;
  std::atomic<bool> m_hotplugPending{false};
};
