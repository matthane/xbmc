/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "WinSystemAmlogic.h"
#include "cores/VideoPlayer/VideoRenderers/FrameBufferObject.h"
#include "rendering/gles/GuiCompositeShaderGLES.h"
#include "rendering/gles/RenderSystemGLES.h"
#include "utils/EGLUtils.h"
#include "utils/GlobalsHandling.h"
#include "utils/StreamDetails.h"

#include <atomic>
#include <memory>
#include <mutex>

namespace KODI
{
namespace WINDOWING
{
namespace AML
{

class CWinSystemAmlogicGLESContext : public CWinSystemAmlogic, public CRenderSystemGLES
{
public:
  CWinSystemAmlogicGLESContext();
  virtual ~CWinSystemAmlogicGLESContext() = default;

  using CWinSystemAmlogic::Register;
  static void Register();
  static std::unique_ptr<CWinSystemBase> CreateWinSystem();

  // Implementation of CWinSystemBase via CWinSystemAmlogic
  CRenderSystemBase *GetRenderSystem() override { return this; }
  bool InitRenderSystem() override;
  bool InitWindowSystem() override;
  bool DestroyWindowSystem() override;
  bool CreateNewWindow(const std::string& name,
                       bool fullScreen,
                       RESOLUTION_INFO& res) override;
  bool DestroyWindow() override;

  bool ResizeWindow(int newWidth, int newHeight, int newLeft, int newTop) override;
  bool SetFullScreen(bool fullScreen, RESOLUTION_INFO& res, bool blankOtherDisplays) override;

  virtual std::unique_ptr<CVideoSync> GetVideoSync(CVideoReferenceClock *clock) override;

  bool SupportsStereo(const RenderStereoMode mode) const override;
  void PresentRender(bool rendered, bool videoLayer) override;
  void SetDirtyRegions(const CDirtyRegionList& dirtyRegions) override;
  int GetBufferAge() override;
  bool CanRedrawPartially() const override;

  bool BindTextureUploadContext() override;
  bool UnbindTextureUploadContext() override;
  bool HasContext() override;

  // GUI compositing for HDR
  bool SetGuiCompositing(int colorTransfer) override;
  uint64_t ConfigureHdrGuiSession(uint64_t owner, int colorTransfer, bool dvGraphics) override;
  void ReleaseHdrGuiSession(uint64_t owner) override;
  bool BeginGuiComposite(bool guiWillRender) override;
  void EndGuiComposite() override;
  void CompositeGui() override;
  bool IsHdrComposite() const override { return m_guiCompositing; }
  bool GuiWillRender() const override { return m_guiWillRender; }

  EGLDisplay GetEGLDisplay() const;
  EGLSurface GetEGLSurface() const;
  EGLContext GetEGLContext() const;
  EGLConfig  GetEGLConfig() const;
protected:
  void SetVSyncImpl(bool enable) override;
  void PresentRenderImpl(bool rendered) override {};

private:
  std::unique_ptr<CEGLContextUtils> m_pGLContext;
  StreamHdrType m_hdrType = StreamHdrType::HDR_TYPE_NONE;
  uint64_t m_presentStepSeen{0};
  // set by the first job thread that could not bind the upload context
  std::atomic<bool> m_uploadContextFailed{false};

  // partial redraw: the buffer age is decided once per frame and only reported by
  // GetBufferAge, which runs once per Render call
  void DecideBufferAge(bool guiWillRender);
  bool m_eglBufferAge{false};
  bool m_canRedrawPartially{false};
  int m_frameBufferAge{2};
  bool m_frameForcedFull{false};
  // changes no dirty region covers (compositing, LUTs, FBO) request a full redraw; done
  // is the request the latest presented forced full redraw covered
  unsigned int m_fullRedrawRequest{0};
  unsigned int m_fullRedrawDone{0};
  unsigned int m_frameFullRedraw{0};
  bool m_unswapped{false};
  uint64_t m_swapCount{0};
  uint64_t m_fullRedrawSwap{0};

  bool m_guiCompositing{false};
  CFrameBufferObject m_guiFbo;
  int m_guiFboWidth{0};
  int m_guiFboHeight{0};
  // True when the GUI FBO is empty (no draws this frame); CompositeGui skips composite when true.
  bool m_guiFboClean{false};
  // window-space bounds of what the GUI pass drew into the FBO this frame
  CRect m_guiCompositeBounds;
  // Whether the GUI render pass will run this frame; set by BeginGuiComposite.
  bool m_guiWillRender{true};
  // Transfer function the LUTs were built for, and the GUI reference white
  // (PQ-normalized) baked into them - kept so a live guipeakluminance change can
  // rebuild the PQ LUT without waiting for the next stream start.
  int m_guiCompositeTransfer{0};
  float m_guiCompositePeak{-1.0f};
  // range the shader was compiled for
  bool m_guiCompositeLimited{false};

  // draws until a 3D LUT for the transfer and range is loaded
  std::unique_ptr<CGuiCompositeShaderGLES> m_compositeShader;

  // the 3D LUT composite; a job builds its nodes, and the GUI thread loads them
  struct Lut3DKey
  {
    int colorTransfer{0};
    float peak{0.0f};
    bool limited{false};
    bool operator==(const Lut3DKey&) const = default;
  };
  struct CLut3DBuild;
  std::unique_ptr<CGuiCompositeShaderGLES> m_lut3DShader;
  Lut3DKey m_lut3DKey;
  bool m_lut3D{false};
  bool m_lut3DPending{false};
  std::shared_ptr<CLut3DBuild> m_lut3DBuild;

  void PrecompileGuiComposite();
  bool CompileGuiComposite(bool limited);
  bool BuildGuiComposite(int colorTransfer, float peak, bool limited);
  CGuiCompositeShaderGLES& GetCompositeShader() const;
  void RequestLut3D();
  void LoadLut3D();
  void CancelLut3D();

  void ResetHdrGuiSession();
  bool SetDvGraphicFormat(unsigned int format);
  bool SetDvGraphicsState(bool enabled);
  std::mutex m_hdrGuiMutex;
  uint64_t m_hdrGuiOwner{0};
  uint64_t m_hdrGuiNextOwner{0};
  bool m_hdrGuiDvGraphics{false};
};

}
}
}
