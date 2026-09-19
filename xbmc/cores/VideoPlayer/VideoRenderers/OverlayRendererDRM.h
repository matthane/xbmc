/*
 *  Copyright (C) 2026 Team CoreELEC
 *  This file is part of CoreELEC - https://coreelec.org
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */
#pragma once

#include "cores/VideoPlayer/VideoRenderers/OverlayRenderer.h"

class CDVDOverlayImage;

#include <cstddef>
#include <cstdint>
#include <vector>

namespace OVERLAY
{

// Session OSD2 subtitle canvas (see OverlayRendererDRM.cpp): immutable
// premultiplied stages fold into a CPU canvas, presents write a
// CPU-mapped scanout buffer and flip its FB_ID with the window system's
// per-frame GUI atomic.
void SubtitlePlaneFrameBegin(bool hasHdrImageOverlay);
void SubtitlePlaneFramePresent();

// Flush hook (CRenderer::Flush tail): clear the plane immediately, a
// flush means seek or stream change
void SubtitlePlaneFlush();

// Teardown/reset (CRenderer::UnInit)
void SubtitleCanvasReset();

class COverlayDRM : public COverlay
{
public:
  COverlayDRM(const CDVDOverlayImage& o, CRect& rs);

  void Render(SRenderState& state) override;

private:
  void BuildStage(int dstW, int dstH);

  // Overlay bitmap, immutable per instance (content change is a new
  // instance, never a mutation)
  std::vector<uint8_t> m_pixels;
  std::vector<uint32_t> m_palette;
  int m_linesize{0};
  int m_bmpW{0};
  int m_bmpH{0};

  // Premultiplied copy of the bitmap at the destination size, rebuilt
  // when the rect changes
  std::vector<uint32_t> m_stage;
  int m_stageW{0};
  int m_stageH{0};

  // Non-transparent bounding box within the stage. Dense PGS tracks
  // re-rasterize full-frame bitmaps; the bbox limits folds and
  // dirty-marking to the content region
  int m_bboxX0{0};
  int m_bboxY0{0};
  int m_bboxW{0};
  int m_bboxH{0};

  const uint64_t m_instanceId;
  uint64_t m_stageRevision{0};
  int m_stageX{-1};
  int m_stageY{-1};
};
}
