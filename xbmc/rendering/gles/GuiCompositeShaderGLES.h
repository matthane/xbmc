/*
 *  Copyright (C) 2026 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "guilib/Shader.h"

#include <cstdint>
#include <string>
#include <vector>

class CGuiCompositeShaderGLES : public Shaders::CGLSLShaderProgram
{
public:
  // how the composite converts the GUI FBO
  enum class Input
  {
    // degamma and transfer through 1D LUTs
    LUT,
    // one 3D LUT maps the sRGB value to the output (GLES 3 and
    // GL_OES_texture_3D)
    LUT3D,
  };

  explicit CGuiCompositeShaderGLES(const std::string& prefix, Input input = Input::LUT);
  ~CGuiCompositeShaderGLES() override;

  Input GetInput() const { return m_input; }

  void SetProjection(const GLfloat* proj) { m_proj = proj; }

  // GUI reference white, in PQ-normalized units (nits / 10000). Takes effect on
  // the next CreateLUTs, which bakes it into the PQ LUT or the HLG white scale.
  void SetSdrPeak(float peak) { m_sdrPeak = peak; }

  // Convert a legacy PQ-signal-domain GUI peak (as CWinSystemAmlogic::
  // GetGuiSdrPeakLuminance returns) into the PQ-normalized luminance SetSdrPeak
  // expects. Lives here so the ST2084 constants are never duplicated.
  static float PeakFromPQCode(float code);

  // The nodes of the 3D LUT for a transfer, GUI reference white and output range: pure
  // CPU work of a frame or more, safe on any thread.
  static std::vector<uint32_t> GenerateLUT3DNodes(int colorTransfer, float sdrPeak, bool limited);

  // Input::LUT3D takes the nodes GenerateLUT3DNodes built for the transfer
  bool CreateLUTs(int colorTransfer, const std::vector<uint32_t>& lut3D = {});

  GLint GetPosLoc() { return m_hPos; }
  GLint GetTexLoc() { return m_hTex; }

protected:
  void OnCompiledAndLinked() override;
  bool OnEnabled() override;

private:
  // One entry per RGBA8 input value; increase to match GUI bit depth.
  static constexpr int LUT_SIZE = 256;
  // Entries of the PQ LUT, which is read unfiltered: enough that the nearest one stays
  // within a code of the transfer at 10 bit.
  static constexpr int PQ_LUT_SIZE = 4096;
  // Nodes per axis of the 3D LUTs, sqrt-spaced. 33 keep every PQ output within a code
  // of the exact transfer at 10 bit. The HLG OOTF bends the output across the channels
  // and needs 49 to stay within a code at 8 bit, the depth of the GUI plane.
  static constexpr int PQ_LUT3D_SIZE = 33;
  static constexpr int HLG_LUT3D_SIZE = 49;

  GLuint CreateLUTTexture(const std::vector<float>& data, GLint filter);
  GLuint CreateLUT3DTexture(const std::vector<uint32_t>& data, int size);
  static std::vector<float> GenerateDegammaLUT();
  static std::vector<float> GeneratePQLUT(float sdrPeak);
  static std::vector<uint32_t> GeneratePQLUT3D(float sdrPeak, bool limited);
  static std::vector<uint32_t> GenerateHLGLUT3D(float sdrPeak, bool limited);

  const Input m_input;
  const GLfloat* m_proj{nullptr};
  float m_sdrPeak{203.0f / 10000.0f};

  GLuint m_lutDegammaTexId{0};
  GLuint m_lutTFTexId{0};
  GLuint m_lut3DTexId{0};
  int m_lut3DSize{0};
  float m_ootfGamma{0.0f};
  float m_hlgWhite{0.0f};

  GLint m_hPos{-1};
  GLint m_hTex{-1};
  GLint m_hSamp{-1};
  GLint m_hLutDegamma{-1};
  GLint m_hLutTF{-1};
  GLint m_hLut3D{-1};
  GLint m_hLut3DMap{-1};
  GLint m_hProj{-1};
  GLint m_hOotfGamma{-1};
  GLint m_hHlgWhite{-1};
};
