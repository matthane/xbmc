/*
 *  Copyright (C) 2026 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "GuiCompositeShaderGLES.h"

#include "utils/log.h"

extern "C"
{
#include <libavutil/pixfmt.h>
}

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>

namespace
{
// ST2084 (PQ) constants
constexpr float ST2084_m1 = 0.1593017578125f; // 2610/16384
constexpr float ST2084_m2 = 78.84375f; // 2523/4096 * 128
constexpr float ST2084_c1 = 0.8359375f; // 3424/4096
constexpr float ST2084_c2 = 18.8515625f; // 2413/4096 * 32
constexpr float ST2084_c3 = 18.6875f; // 2392/4096 * 32

float ForwardPQ(float L)
{
  float Lm1 = std::pow(L, ST2084_m1);
  return std::pow((ST2084_c1 + ST2084_c2 * Lm1) / (1.0f + ST2084_c3 * Lm1), ST2084_m2);
}

// ST2084 EOTF: PQ code -> PQ-normalized luminance (nits / 10000). Exact inverse
// of ForwardPQ, kept beside it so the constants are never duplicated elsewhere.
float InversePQ(float E)
{
  if (E <= 0.0f)
    return 0.0f;
  const float Em2 = std::pow(std::min(E, 1.0f), 1.0f / ST2084_m2);
  const float num = std::max(Em2 - ST2084_c1, 0.0f);
  const float den = ST2084_c2 - ST2084_c3 * Em2;
  if (den <= 0.0f)
    return 1.0f;
  return std::pow(num / den, 1.0f / ST2084_m1);
}

// BT.709 -> BT.2020 in linear light, rows of the matrix in gles_gui_composite.frag
constexpr std::array<std::array<double, 3>, 3> BT709_TO_BT2020 = {{
    {0.6274, 0.3293, 0.0433},
    {0.0691, 0.9195, 0.0114},
    {0.0164, 0.0880, 0.8956},
}};

// IEC 61966-2-1 sRGB EOTF.
float SRGBToLinear(float v)
{
  return v <= 0.04045f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f);
}

// IEEE 754 binary16, round to nearest even, for finite input
uint16_t FloatToHalf(float value)
{
  const uint32_t bits = std::bit_cast<uint32_t>(value);
  const uint32_t sign = (bits >> 16) & 0x8000;
  const int exponent = static_cast<int>((bits >> 23) & 0xff) - 127 + 15;
  uint32_t mantissa = bits & 0x7fffff;

  if (exponent >= 31)
    return static_cast<uint16_t>(sign | 0x7c00);

  if (exponent <= 0)
  {
    if (exponent < -10)
      return static_cast<uint16_t>(sign);
    mantissa |= 0x800000;
    const int shift = 14 - exponent;
    uint32_t half = mantissa >> shift;
    const uint32_t rest = mantissa & ((1u << shift) - 1);
    const uint32_t tie = 1u << (shift - 1);
    if (rest > tie || (rest == tie && (half & 1)))
      half++;
    return static_cast<uint16_t>(sign | half);
  }

  // a carry out of the mantissa rounds up into the exponent, as it should
  uint32_t half = (static_cast<uint32_t>(exponent) << 10) | (mantissa >> 13);
  const uint32_t rest = mantissa & 0x1fff;
  if (rest > 0x1000 || (rest == 0x1000 && (half & 1)))
    half++;
  return static_cast<uint16_t>(sign | half);
}

} // namespace

CGuiCompositeShaderGLES::CGuiCompositeShaderGLES(const std::string& prefix, Input input)
  : m_input(input)
{
  std::string defines = prefix + "#define KODI_LUT_SIZE " + std::to_string(LUT_SIZE) +
                        ".0\n#define KODI_PQ_LUT_SIZE " + std::to_string(PQ_LUT_SIZE) + ".0\n";
  if (m_input == Input::LUT3D)
    defines += "#define KODI_GUI_LUT3D 1\n#define KODI_LUT3D_SIZE " +
               std::to_string(LUT3D_SIZE) + ".0\n";
  VertexShader()->LoadSource("gles_gui_composite.vert", defines);
  PixelShader()->LoadSource("gles_gui_composite.frag", defines);
}

CGuiCompositeShaderGLES::~CGuiCompositeShaderGLES()
{
  if (m_lutDegammaTexId)
    glDeleteTextures(1, &m_lutDegammaTexId);
  if (m_lutTFTexId)
    glDeleteTextures(1, &m_lutTFTexId);
  if (m_lut3DTexId)
    glDeleteTextures(1, &m_lut3DTexId);
}

void CGuiCompositeShaderGLES::OnCompiledAndLinked()
{
  m_hPos = glGetAttribLocation(ProgramHandle(), "a_pos");
  m_hTex = glGetAttribLocation(ProgramHandle(), "a_tex");
  m_hSamp = glGetUniformLocation(ProgramHandle(), "u_samp");
  m_hLutDegamma = glGetUniformLocation(ProgramHandle(), "u_lutDegamma");
  m_hLutTF = glGetUniformLocation(ProgramHandle(), "u_lutTF");
  m_hLut3D = glGetUniformLocation(ProgramHandle(), "u_lut3d");
  m_hProj = glGetUniformLocation(ProgramHandle(), "u_proj");
  m_hOotfGamma = glGetUniformLocation(ProgramHandle(), "u_ootfGamma");
  m_hHlgWhite = glGetUniformLocation(ProgramHandle(), "u_hlgWhite");
  glUseProgram(ProgramHandle());
  glUniform1i(m_hSamp, 0);
  glUniform1i(m_hLutDegamma, 1);
  glUniform1i(m_hLutTF, 2);
  glUniform1i(m_hLut3D, 3);
  glUseProgram(0);
}

bool CGuiCompositeShaderGLES::OnEnabled()
{
  if (m_proj)
    glUniformMatrix4fv(m_hProj, 1, GL_FALSE, m_proj);

  glUniform1f(m_hOotfGamma, m_ootfGamma);
  glUniform1f(m_hHlgWhite, m_hlgWhite);

  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, m_lutDegammaTexId);
  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, m_lutTFTexId);
  if (m_input == Input::LUT3D)
  {
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_3D, m_lut3DTexId);
  }
  glActiveTexture(GL_TEXTURE0);

  return true;
}

GLuint CGuiCompositeShaderGLES::CreateLUTTexture(const std::vector<float>& data, GLint filter)
{
  while (glGetError() != GL_NO_ERROR)
  {
  }

  GLuint texId;
  glGenTextures(1, &texId);
  glBindTexture(GL_TEXTURE_2D, texId);

  // Prefer GL_R16F (GLES 3.0 core) over GL_LUMINANCE + GL_FLOAT (GLES 2.0).
  // The GLES 3.0 spec tightens format validation for unsized internal formats,
  // and some drivers (e.g. V3D on RPi5) silently reject GL_LUMINANCE + GL_FLOAT
  // despite advertising OES_texture_float. GL_R16F avoids this by using a sized
  // format with well-defined behavior. The halves are rounded here, not by the
  // driver, so every GPU stores the same table.
  std::vector<uint16_t> halves(data.size());
  std::transform(data.begin(), data.end(), halves.begin(), FloatToHalf);

  bool uploaded = false;
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, data.size(), 1, 0, GL_RED, GL_HALF_FLOAT,
               halves.data());
  if (glGetError() == GL_NO_ERROR)
  {
    uploaded = true;
  }
  else
  {
    while (glGetError() != GL_NO_ERROR)
    {
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, data.size(), 1, 0, GL_LUMINANCE, GL_FLOAT,
                 data.data());
    if (glGetError() == GL_NO_ERROR)
      uploaded = true;
    else
      CLog::Log(LOGERROR,
                "CGuiCompositeShaderGLES::CreateLUTTexture - failed to create {} entry "
                "LUT texture (GL_R16F and GL_LUMINANCE+GL_FLOAT both failed)",
                data.size());
  }

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);

  if (!uploaded)
  {
    glDeleteTextures(1, &texId);
    return 0;
  }
  return texId;
}

GLuint CGuiCompositeShaderGLES::CreateLUT3DTexture(const std::vector<uint32_t>& data)
{
  while (glGetError() != GL_NO_ERROR)
  {
  }

  GLuint texId;
  glGenTextures(1, &texId);
  glBindTexture(GL_TEXTURE_3D, texId);
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB10_A2, LUT3D_SIZE, LUT3D_SIZE, LUT3D_SIZE, 0, GL_RGBA,
               GL_UNSIGNED_INT_2_10_10_10_REV, data.data());
  const bool uploaded = glGetError() == GL_NO_ERROR;
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_3D, 0);

  if (!uploaded)
  {
    CLog::Log(LOGERROR, "CGuiCompositeShaderGLES::CreateLUT3DTexture - failed to create {}^3 LUT",
              LUT3D_SIZE);
    glDeleteTextures(1, &texId);
    return 0;
  }
  return texId;
}

std::vector<float> CGuiCompositeShaderGLES::GenerateDegammaLUT()
{
  std::vector<float> lut(LUT_SIZE);
  for (int i = 0; i < LUT_SIZE; i++)
  {
    float x = static_cast<float>(i) / (LUT_SIZE - 1);
    lut[i] = SRGBToLinear(x);
  }
  return lut;
}

float CGuiCompositeShaderGLES::PeakFromPQCode(float code)
{
  // The legacy Amlogic GUI peak is a PQ CODE, not nits. On the per-primitive
  // path the scalar-encoded OSD plane is declared FORMAT_HDR8, so the DV core
  // reads it as PQ - which is exactly why the default (0.7*40+30)/100 = 0.58
  // lands GUI white on ~199 nits, within 2% of the 203-nit BT.2408 reference
  // white this composite used to hardcode. Decoding the code therefore makes
  // the same setting mean the same luminance on both paths.
  //
  // Clamped to 1000 nits. The raw curve reaches 10000 nits at the top of the
  // slider, which no panel can show.
  // The clamp engages around slider 64 (code 0.748), so the top third of the
  // range is deliberately flat; CreateLUTs logs the resolved nits so a log shows
  // when it is in effect. Above that point this intentionally stops tracking the
  // per-primitive path, which applies no clamp because it needs no LUT.
  return std::min(InversePQ(code), 0.1f);
}

std::vector<float> CGuiCompositeShaderGLES::GeneratePQLUT(float sdrPeak)
{
  // PQ is display-referred (absolute luminance). sdrPeak is in PQ-normalized
  // units (nits / 10000), e.g. 203 nits = 0.0203. The shader indexes the LUT by
  // sqrt(linear), so entry i holds ForwardPQ(sdrPeak * (i / (PQ_LUT_SIZE - 1))^2):
  // a uniform spacing in linear light leaves the darkest step wider than a
  // dozen output codes.
  std::vector<float> lut(PQ_LUT_SIZE);
  for (int i = 0; i < PQ_LUT_SIZE; i++)
  {
    const float x = static_cast<float>(i) / (PQ_LUT_SIZE - 1);
    lut[i] = ForwardPQ(x * x * sdrPeak);
  }
  return lut;
}

std::vector<uint32_t> CGuiCompositeShaderGLES::GeneratePQLUT3D(float sdrPeak)
{
  // RGB10_A2 nodes: 8-bit nodes lose up to three codes at 10 bit, half floats filter
  // at half rate. A live guipeakluminance change rebuilds this on the GUI thread, so
  // the ~36k nodes read the PQ curve from the 1D LUT instead of calling pow.
  const std::vector<float> table = GeneratePQLUT(sdrPeak);
  const std::vector<double> pq(table.begin(), table.end());

  std::array<double, LUT3D_SIZE> linear;
  for (int i = 0; i < LUT3D_SIZE; i++)
  {
    const double x = static_cast<double>(i) / (LUT3D_SIZE - 1);
    linear[i] = SRGBToLinear(static_cast<float>(x * x));
  }

  const auto encode = [&pq](double value)
  {
    const double t = std::sqrt(std::clamp(value, 0.0, 1.0)) * (PQ_LUT_SIZE - 1);
    const int i = std::min(static_cast<int>(t), PQ_LUT_SIZE - 2);
    const double v = pq[i] + (pq[i + 1] - pq[i]) * (t - i);
    return static_cast<uint32_t>(std::lround(std::clamp(v, 0.0, 1.0) * 1023.0));
  };

  std::vector<uint32_t> lut;
  lut.reserve(LUT3D_SIZE * LUT3D_SIZE * LUT3D_SIZE);
  for (int b = 0; b < LUT3D_SIZE; b++)
  {
    for (int g = 0; g < LUT3D_SIZE; g++)
    {
      for (int r = 0; r < LUT3D_SIZE; r++)
      {
        const std::array<double, 3> rgb = {linear[r], linear[g], linear[b]};
        uint32_t texel = 3u << 30;
        for (int c = 0; c < 3; c++)
        {
          const auto& row = BT709_TO_BT2020[c];
          texel |= encode(row[0] * rgb[0] + row[1] * rgb[1] + row[2] * rgb[2]) << (10 * c);
        }
        lut.push_back(texel);
      }
    }
  }
  return lut;
}

bool CGuiCompositeShaderGLES::CreateLUTs(int colorTransfer)
{
  // Build into locals and only commit on success. Deleting the live textures up
  // front would leave the shader sampling destroyed/zero texture names on any
  // failure - the GUI composites to solid black, and a caller that retries (a
  // live SetSdrPeak change) would thrash glDeleteTextures/glTexImage2D every
  // frame. Failure must be a no-op so the previous LUTs keep working.
  GLuint degamma = 0;
  if (m_input == Input::LUT)
  {
    degamma = CreateLUTTexture(GenerateDegammaLUT(), GL_LINEAR);
    if (!degamma)
    {
      CLog::Log(LOGERROR, "CGuiCompositeShaderGLES::CreateLUTs - failed to create degamma LUT");
      return false;
    }
  }

  GLuint tf = 0;
  float ootfGamma = 0.0f;
  float hlgWhite = 0.0f;

  GLuint tf3D = 0;

  if (colorTransfer == AVCOL_TRC_SMPTE2084 && m_input == Input::LUT3D)
  {
    tf3D = CreateLUT3DTexture(GeneratePQLUT3D(m_sdrPeak));
    if (!tf3D)
      return false;
    CLog::Log(LOGDEBUG, "CGuiCompositeShaderGLES::CreateLUTs - created PQ 3D LUT ({}^3, {:.0f} nits)",
              LUT3D_SIZE, m_sdrPeak * 10000.0f);
  }
  else if (colorTransfer == AVCOL_TRC_SMPTE2084)
  {
    tf = CreateLUTTexture(GeneratePQLUT(m_sdrPeak), GL_NEAREST);
    if (!tf)
    {
      CLog::Log(LOGERROR, "CGuiCompositeShaderGLES::CreateLUTs - failed to create PQ LUT");
      glDeleteTextures(1, &degamma);
      return false;
    }
    CLog::Log(LOGDEBUG,
              "CGuiCompositeShaderGLES::CreateLUTs - created PQ LUT ({} entries, {:.0f} nits)",
              PQ_LUT_SIZE, m_sdrPeak * 10000.0f);
  }
  else if (m_input == Input::LUT3D)
  {
    CLog::Log(LOGERROR, "CGuiCompositeShaderGLES::CreateLUTs - the 3D LUT is PQ only");
    return false;
  }
  else if (colorTransfer == AVCOL_TRC_ARIB_STD_B67)
  {
    // HLG: no TF LUT needed, shader computes OETF + inverse OOTF directly.
    // BT.2100: gamma = 1.2 + 0.42 * log10(Lw/1000). For 1000-nit ref: 1.2.
    ootfGamma = 1.2f;
    hlgWhite = m_sdrPeak * 10000.0f / 1000.0f;
    CLog::Log(LOGDEBUG, "CGuiCompositeShaderGLES::CreateLUTs - HLG mode (gamma {}, {:.0f} nits)",
              ootfGamma, m_sdrPeak * 10000.0f);
  }
  else
  {
    CLog::Log(LOGERROR, "CGuiCompositeShaderGLES::CreateLUTs - unsupported transfer function {}",
              colorTransfer);
    glDeleteTextures(1, &degamma);
    return false;
  }

  if (m_lutDegammaTexId)
    glDeleteTextures(1, &m_lutDegammaTexId);
  if (m_lutTFTexId)
    glDeleteTextures(1, &m_lutTFTexId);
  if (m_lut3DTexId)
    glDeleteTextures(1, &m_lut3DTexId);

  m_lutDegammaTexId = degamma;
  m_lutTFTexId = tf;
  m_lut3DTexId = tf3D;
  m_ootfGamma = ootfGamma;
  m_hlgWhite = hlgWhite;
  return true;
}
