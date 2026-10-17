/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "AMLCaptureConvert.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>

extern "C"
{
#include <libswscale/swscale.h>
}

namespace
{

using Matrix = std::array<std::array<double, 3>, 3>;

// clang-format off
// ITU-R BT.2100 table 7
constexpr Matrix BT2100_RGB_TO_LMS{{
  {1688.0 / 4096, 2146.0 / 4096, 262.0 / 4096},
  {683.0 / 4096, 2951.0 / 4096, 462.0 / 4096},
  {99.0 / 4096, 309.0 / 4096, 3688.0 / 4096},
}};
constexpr Matrix BT2100_LMS_TO_ICTCP{{
  {2048.0 / 4096, 2048.0 / 4096, 0.0},
  {6610.0 / 4096, -13613.0 / 4096, 7003.0 / 4096},
  {17933.0 / 4096, -17390.0 / 4096, -543.0 / 4096},
}};

// ITU-R BT.2087, linear light
constexpr Matrix BT709_TO_BT2020{{
  {0.6274039, 0.3292830, 0.0433131},
  {0.0690973, 0.9195404, 0.0113623},
  {0.0163914, 0.0880133, 0.8955953},
}};
// clang-format on

// under Dolby Vision, vdin1 VD1 holds full-range ICtCp in every output mode, Ct at a quarter and
// Cp at half the I scale; fitted from VD1, tunnel and wire grabs of two titles on three SoCs
constexpr float VD1_I_SPAN = 256.0f;
constexpr float VD1_CT_SPAN = 64.0f;
constexpr float VD1_CP_SPAN = 128.0f;
constexpr float VD1_CHROMA_ZERO = 128.0f;

// the Dolby Vision SDR output codes BT.709 by the ITU-R BT.1886 EOTF, for the SDR target display
// amdolby_vision sets (amdv_target_max, amdv_target_min)
constexpr double BT1886_GAMMA = 2.4;
constexpr double DV_SDR_WHITE_NITS = 100.0;
constexpr double DV_SDR_BLACK_NITS = 0.005;

// swscale widens 8-bit codes to YUV444P16 by a shift of 8
constexpr float CODE_SCALE_8_TO_16 = 256.0f;

constexpr float LIMITED_BLACK = 16.0f;
constexpr float LIMITED_LUMA_SPAN = 219.0f;
constexpr float LIMITED_CHROMA_ZERO = 128.0f;
constexpr float LIMITED_CHROMA_SPAN = 224.0f;

constexpr float BT709_KR = 0.2126f;
constexpr float BT709_KB = 0.0722f;
constexpr float BT2020_KR = 0.2627f;
constexpr float BT2020_KB = 0.0593f;

constexpr double PQ_M1 = 2610.0 / 16384;
constexpr double PQ_M2 = 2523.0 / 4096 * 128;
constexpr double PQ_C1 = 3424.0 / 4096;
constexpr double PQ_C2 = 2413.0 / 4096 * 32;
constexpr double PQ_C3 = 2392.0 / 4096 * 32;
constexpr double PQ_PEAK_NITS = 10000.0;

// ITU-R BT.2100 HLG at the nominal 1000 cd/m2 display, as the GUI composite codes it
constexpr double HLG_A = 0.17883277;
constexpr double HLG_B = 0.28466892;
constexpr double HLG_C = 0.55991073;
constexpr double HLG_GAMMA = 1.2;
constexpr double HLG_PEAK_NITS = 1000.0;
constexpr double HLG_SCENE_SCALE = 12.0;
constexpr double HLG_MIN_LUMA = 1e-6;

// IEC 61966-2-1, the coding of the GUI
constexpr double SRGB_THRESHOLD = 0.04045;
constexpr double SRGB_LINEAR_THRESHOLD = 0.0031308;
constexpr double SRGB_SLOPE = 12.92;
constexpr double SRGB_OFFSET = 0.055;
constexpr double SRGB_GAMMA = 2.4;

// Kodi's default video tonemap: Hable at parameter 1.0 (CMediaSettings), as the GLES video
// shader applies it (YUV2RGBShaderGLES.cpp, gles_tonemap.frag, gles_yuv2rgb_basic.frag)
constexpr std::array<double, 6> HABLE{0.15, 0.5, 0.1, 0.2, 0.02, 0.3};
constexpr double TONEMAP_PARAM = 1.0;
constexpr double HABLE_EXPOSURE = 2.0;
constexpr double HABLE_WHITE_NITS = 100.0;
// that curve clips at half the peak; captures double its exposure and put white at the peak,
// which lifts the midtones and rolls the highlights off up to the peak
constexpr double CAPTURE_EXPOSURE = 2.0;
// the shader scales linear light by (10000 / peak) * (2 / param), then by the white point
// peak / 100, so the scale is fixed and the peak only sets where white sits
constexpr double TONEMAP_SCALE =
    PQ_PEAK_NITS / HABLE_WHITE_NITS * HABLE_EXPOSURE / TONEMAP_PARAM * CAPTURE_EXPOSURE;
constexpr double SDR_OUTPUT_GAMMA = 2.2;

// interpolated tables stay within 0.1 8-bit code of the exact transfer functions; tables of
// linear light are indexed by its fourth root, which spaces the steps much like PQ does
constexpr int PQ_EOTF_STEPS = 16384;
constexpr int TABLE_STEPS = 4096;

constexpr Matrix Invert(const Matrix& m)
{
  const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                     m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  Matrix out{};
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
    {
      const int r0 = (c + 1) % 3;
      const int r1 = (c + 2) % 3;
      const int c0 = (r + 1) % 3;
      const int c1 = (r + 2) % 3;
      out[r][c] = (m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0]) / det;
    }
  return out;
}

constexpr Matrix BT2100_ICTCP_TO_LMS = Invert(BT2100_LMS_TO_ICTCP);
constexpr Matrix BT2100_LMS_TO_RGB = Invert(BT2100_RGB_TO_LMS);
constexpr Matrix BT2020_TO_BT709 = Invert(BT709_TO_BT2020);

void Apply(const Matrix& m, const float in[3], float out[3])
{
  for (int r = 0; r < 3; r++)
    out[r] = static_cast<float>(m[r][0] * static_cast<double>(in[0]) +
                                m[r][1] * static_cast<double>(in[1]) +
                                m[r][2] * static_cast<double>(in[2]));
}

double PQToLinear(double e)
{
  const double p = std::pow(std::clamp(e, 0.0, 1.0), 1 / PQ_M2);
  return std::pow(std::max(p - PQ_C1, 0.0) / (PQ_C2 - PQ_C3 * p), 1 / PQ_M1);
}

double LinearToPQ(double l)
{
  const double y = std::pow(std::max(l, 0.0), PQ_M1);
  return std::pow((PQ_C1 + PQ_C2 * y) / (1 + PQ_C3 * y), PQ_M2);
}

double SrgbToLinear(double v)
{
  return v <= SRGB_THRESHOLD ? v / SRGB_SLOPE
                             : std::pow((v + SRGB_OFFSET) / (1 + SRGB_OFFSET), SRGB_GAMMA);
}

double LinearToSrgb(double l)
{
  return l <= SRGB_LINEAR_THRESHOLD ? l * SRGB_SLOPE
                                    : (1 + SRGB_OFFSET) * std::pow(l, 1 / SRGB_GAMMA) - SRGB_OFFSET;
}

double HlgToScene(double e)
{
  return e <= 0.5 ? e * e / 3 : (std::exp((e - HLG_C) / HLG_A) + HLG_B) / HLG_SCENE_SCALE;
}

double LinearToBt1886(double l)
{
  const double white = std::pow(DV_SDR_WHITE_NITS, 1 / BT1886_GAMMA);
  const double black = std::pow(DV_SDR_BLACK_NITS, 1 / BT1886_GAMMA);
  const double v = std::pow(l * PQ_PEAK_NITS, 1 / BT1886_GAMMA);
  return std::clamp((v - black) / (white - black), 0.0, 1.0);
}

double Hable(double x)
{
  const auto& [a, b, c, d, e, f] = HABLE;
  return ((x * (a * x + c * b) + d * e) / (x * (a * x + b) + d * f)) - e / f;
}

// one entry past the end, so a lookup at 1.0 interpolates without a bounds check
template<typename F>
std::vector<float> MakeTable(int steps, F f)
{
  std::vector<float> table;
  table.reserve(steps + 2);
  for (int i = 0; i <= steps + 1; i++)
    table.push_back(static_cast<float>(f(std::min(static_cast<double>(i) / steps, 1.0))));
  return table;
}

std::vector<float> MakeRootTable(int steps, double (*f)(double))
{
  return MakeTable(steps, [f](double root) { return f(root * root * root * root); });
}

float Lookup(const std::vector<float>& table, float v)
{
  const int steps = static_cast<int>(table.size()) - 2;
  const float pos = std::clamp(v, 0.0f, 1.0f) * steps;
  const int i = static_cast<int>(pos);
  return table[i] + (table[i + 1] - table[i]) * (pos - i);
}

float LookupRoot(const std::vector<float>& table, float linear)
{
  return Lookup(table, std::sqrt(std::sqrt(std::max(linear, 0.0f))));
}

struct Tables
{
  std::vector<float> pqToLinear = MakeTable(PQ_EOTF_STEPS, PQToLinear);
  std::vector<float> linearToPQ = MakeRootTable(TABLE_STEPS, LinearToPQ);
  std::vector<float> srgbToLinear = MakeTable(TABLE_STEPS, SrgbToLinear);
  std::vector<float> linearToSrgb = MakeRootTable(TABLE_STEPS, LinearToSrgb);
  std::vector<float> hlgToScene = MakeTable(TABLE_STEPS, HlgToScene);
  std::vector<float> linearToBt1886 = MakeRootTable(TABLE_STEPS, LinearToBt1886);
};

const Tables& GetTables()
{
  static const Tables tables;
  return tables;
}

bool IsSdr(AMLCaptureSignal signal)
{
  return signal == AMLCaptureSignal::SDR || signal == AMLCaptureSignal::DV_SDR;
}

// what a decoded pixel holds: R'G'B' in a transfer, or linear BT.2020 with 1.0 at the PQ peak
enum class Light
{
  SDR,
  PQ,
  HLG,
  LINEAR,
};

void YccToRgb(const float ycc[3], float kr, float kb, float out[3])
{
  const float y = (ycc[0] - LIMITED_BLACK) / LIMITED_LUMA_SPAN;
  const float cb = (ycc[1] - LIMITED_CHROMA_ZERO) / LIMITED_CHROMA_SPAN;
  const float cr = (ycc[2] - LIMITED_CHROMA_ZERO) / LIMITED_CHROMA_SPAN;
  out[0] = y + 2 * (1 - kr) * cr;
  out[2] = y + 2 * (1 - kb) * cb;
  out[1] = (y - kr * out[0] - kb * out[2]) / (1 - kr - kb);
}

void DecodeIctcp(const float ycc[3], const Tables& tables, float out[3])
{
  const float ictcp[3] = {ycc[0] / VD1_I_SPAN, (ycc[1] - VD1_CHROMA_ZERO) / VD1_CT_SPAN,
                          (ycc[2] - VD1_CHROMA_ZERO) / VD1_CP_SPAN};
  float lms[3];
  Apply(BT2100_ICTCP_TO_LMS, ictcp, lms);
  for (int c = 0; c < 3; c++)
    lms[c] = Lookup(tables.pqToLinear, lms[c]);
  Apply(BT2100_LMS_TO_RGB, lms, out);
}

Light Decode(const float ycc[3], AMLCaptureSignal signal, const Tables& tables, float out[3])
{
  switch (signal)
  {
    case AMLCaptureSignal::PQ:
      YccToRgb(ycc, BT2020_KR, BT2020_KB, out);
      return Light::PQ;
    case AMLCaptureSignal::HLG:
      YccToRgb(ycc, BT2020_KR, BT2020_KB, out);
      return Light::HLG;
    case AMLCaptureSignal::DV_TV_LED:
    case AMLCaptureSignal::DV_LL:
    case AMLCaptureSignal::DV_HDR10:
      DecodeIctcp(ycc, tables, out);
      return Light::LINEAR;
    case AMLCaptureSignal::DV_SDR:
    {
      float linear[3];
      DecodeIctcp(ycc, tables, linear);
      Apply(BT2020_TO_BT709, linear, out);
      for (int c = 0; c < 3; c++)
        out[c] = LookupRoot(tables.linearToBt1886, out[c]);
      return Light::SDR;
    }
    case AMLCaptureSignal::SDR:
    default:
      YccToRgb(ycc, BT709_KR, BT709_KB, out);
      return Light::SDR;
  }
}

void HlgToLinear(const float hlg[3], const Tables& tables, float out[3])
{
  float scene[3];
  for (int c = 0; c < 3; c++)
    scene[c] = Lookup(tables.hlgToScene, hlg[c]);
  const float luma =
      BT2020_KR * scene[0] + (1 - BT2020_KR - BT2020_KB) * scene[1] + BT2020_KB * scene[2];
  const double gain = luma > 0 ? std::pow(luma, HLG_GAMMA - 1) * HLG_PEAK_NITS / PQ_PEAK_NITS : 0.0;
  for (int c = 0; c < 3; c++)
    out[c] = static_cast<float>(static_cast<double>(scene[c]) * gain);
}

// the GUI composite's HLG coding (gles_gui_composite.frag)
void LinearToHlg(const float linear[3], float out[3])
{
  double display[3];
  for (int c = 0; c < 3; c++)
    display[c] = static_cast<double>(std::max(linear[c], 0.0f)) * PQ_PEAK_NITS / HLG_PEAK_NITS;
  const double luma = static_cast<double>(BT2020_KR) * display[0] +
                      static_cast<double>(1 - BT2020_KR - BT2020_KB) * display[1] +
                      static_cast<double>(BT2020_KB) * display[2];
  const double gain =
      HLG_SCENE_SCALE * std::pow(std::max(HLG_MIN_LUMA, luma), (1 - HLG_GAMMA) / HLG_GAMMA);
  for (int c = 0; c < 3; c++)
  {
    const double scene = display[c] * gain;
    out[c] = static_cast<float>(scene < 1 ? 0.5 * std::sqrt(scene)
                                          : HLG_A * std::log(scene - HLG_B) + HLG_C);
  }
}

void SdrToLinear(float whiteNits, const Tables& tables, float v[3])
{
  float linear[3];
  for (int c = 0; c < 3; c++)
    linear[c] = static_cast<float>(static_cast<double>(Lookup(tables.srgbToLinear, v[c])) *
                                   static_cast<double>(whiteNits) / PQ_PEAK_NITS);
  Apply(BT709_TO_BT2020, linear, v);
}

void HdrToLinear(Light light, const Tables& tables, float v[3])
{
  switch (light)
  {
    case Light::PQ:
      for (int c = 0; c < 3; c++)
        v[c] = Lookup(tables.pqToLinear, v[c]);
      break;
    case Light::HLG:
    {
      const float hlg[3] = {v[0], v[1], v[2]};
      HlgToLinear(hlg, tables, v);
      break;
    }
    default:
      break;
  }
}

void ToneMap(const float linear[3], const std::vector<float>& toneMap, float out[3])
{
  Apply(BT2020_TO_BT709, linear, out);
  for (int c = 0; c < 3; c++)
    out[c] = LookupRoot(toneMap, out[c]);
}

void ConvertPixel(const float ycc[3],
                  const AMLCaptureColor& color,
                  const AMLCaptureCoding& coding,
                  const std::vector<float>* toneMap,
                  const Tables& tables,
                  float out[3])
{
  float v[3];
  const Light light = Decode(ycc, color.signal, tables, v);
  if (light != Light::SDR)
  {
    HdrToLinear(light, tables, v);
    const float linear[3] = {v[0], v[1], v[2]};
    ToneMap(linear, *toneMap, v);
  }

  // an HDR coding holds the SDR video as the GUI shader places SDR
  if (coding.transfer != AMLCaptureCoding::Transfer::SDR)
  {
    SdrToLinear(coding.whiteNits, tables, v);
    const float linear[3] = {v[0], v[1], v[2]};
    if (coding.transfer == AMLCaptureCoding::Transfer::PQ)
    {
      for (int c = 0; c < 3; c++)
        v[c] = LookupRoot(tables.linearToPQ, linear[c]);
    }
    else
      LinearToHlg(linear, v);
  }

  for (int c = 0; c < 3; c++)
    out[c] = std::clamp(v[c], 0.0f, 1.0f);
}

uint8_t To8(float v)
{
  return static_cast<uint8_t>(std::lround(v * 255));
}

uint16_t To16(float v)
{
  return static_cast<uint16_t>(std::lround(v * 65535));
}

} // namespace

CAMLCaptureConvert::~CAMLCaptureConvert()
{
  sws_freeContext(m_scaleContext);
}

const std::vector<float>& CAMLCaptureConvert::GetToneMap(float luminance)
{
  if (luminance == m_toneMapLuminance && !m_toneMap.empty())
    return m_toneMap;

  const double white = Hable(TONEMAP_SCALE * static_cast<double>(luminance) / PQ_PEAK_NITS);
  m_toneMap = MakeTable(TABLE_STEPS,
                        [white](double root)
                        {
                          const double linear = root * root * root * root;
                          const double sdr =
                              std::clamp(Hable(linear * TONEMAP_SCALE) / white, 0.0, 1.0);
                          return std::pow(sdr, 1 / SDR_OUTPUT_GAMMA);
                        });
  m_toneMapLuminance = luminance;
  return m_toneMap;
}

bool CAMLCaptureConvert::Convert(const AMLCaptureImage& src,
                                 const AMLCaptureColor& color,
                                 const AMLCaptureCoding& coding,
                                 const AMLCaptureImage& dst)
{
  const bool bgra = dst.format == AV_PIX_FMT_BGRA;
  if (src.format != AV_PIX_FMT_NV12 || src.width <= 0 || src.height <= 0 || dst.width <= 0 ||
      dst.height <= 0 || (!bgra && dst.format != AV_PIX_FMT_RGBA64LE))
    return false;

  // the capture mapping is uncached: one sequential read, and every pass after it on the copy
  const std::size_t luma = static_cast<std::size_t>(src.width) * src.height;
  m_source.resize(luma + luma / 2);
  for (int y = 0; y < src.height; y++)
    std::memcpy(m_source.data() + static_cast<std::size_t>(y) * src.width,
                src.data[0] + static_cast<std::ptrdiff_t>(y) * src.linesize[0], src.width);
  for (int y = 0; y < src.height / 2; y++)
    std::memcpy(m_source.data() + luma + static_cast<std::size_t>(y) * src.width,
                src.data[1] + static_cast<std::ptrdiff_t>(y) * src.linesize[1], src.width);

  m_scaleContext = sws_getCachedContext(m_scaleContext, src.width, src.height, AV_PIX_FMT_NV12,
                                        dst.width, dst.height, AV_PIX_FMT_YUV444P16LE, SWS_BILINEAR,
                                        nullptr, nullptr, nullptr);
  if (!m_scaleContext)
    return false;

  const std::size_t plane = static_cast<std::size_t>(dst.width) * dst.height;
  m_yuv.resize(plane * 3);
  uint8_t* yuv[] = {reinterpret_cast<uint8_t*>(m_yuv.data()),
                    reinterpret_cast<uint8_t*>(m_yuv.data() + plane),
                    reinterpret_cast<uint8_t*>(m_yuv.data() + 2 * plane), nullptr};
  const int yuvStride[] = {dst.width * 2, dst.width * 2, dst.width * 2, 0};
  const uint8_t* srcData[] = {m_source.data(), m_source.data() + luma, nullptr, nullptr};
  const int srcStride[] = {src.width, src.width, 0, 0};
  sws_scale(m_scaleContext, srcData, srcStride, 0, src.height, yuv, yuvStride);

  const std::vector<float>* toneMap = IsSdr(color.signal) ? nullptr : &GetToneMap(color.luminance);
  const Tables& tables = GetTables();
  for (int y = 0; y < dst.height; y++)
  {
    uint8_t* row = dst.data[0] + static_cast<std::ptrdiff_t>(y) * dst.linesize[0];
    for (int x = 0; x < dst.width; x++)
    {
      const std::size_t i = static_cast<std::size_t>(y) * dst.width + x;
      const float ycc[3] = {m_yuv[i] / CODE_SCALE_8_TO_16, m_yuv[plane + i] / CODE_SCALE_8_TO_16,
                            m_yuv[2 * plane + i] / CODE_SCALE_8_TO_16};
      float rgb[3];
      ConvertPixel(ycc, color, coding, toneMap, tables, rgb);

      if (bgra)
      {
        uint8_t* out = row + x * 4;
        out[0] = To8(rgb[2]);
        out[1] = To8(rgb[1]);
        out[2] = To8(rgb[0]);
        out[3] = 255;
      }
      else
      {
        uint16_t* out = reinterpret_cast<uint16_t*>(row) + x * 4;
        for (int c = 0; c < 3; c++)
          out[c] = To16(rgb[c]);
        out[3] = 65535;
      }
    }
  }
  return true;
}
