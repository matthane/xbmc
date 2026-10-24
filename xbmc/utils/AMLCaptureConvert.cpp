/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "utils/AMLCaptureConvert.h"

#include <algorithm>
#include <array>
#include <bit>
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
constexpr int CODE_SHIFT_10_TO_16 = 6;

constexpr int FULL_PACK_CODE_BITS = 10;
constexpr uint64_t FULL_PACK_CODE_MASK = (1 << FULL_PACK_CODE_BITS) - 1;

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

// Kodi's default video tonemap curve: Hable at parameter 1.0 (CMediaSettings), as the GLES video
// shader applies it (YUV2RGBShaderGLES.cpp, gles_tonemap.frag, gles_yuv2rgb_basic.frag)
constexpr std::array<double, 6> HABLE{0.15, 0.5, 0.1, 0.2, 0.02, 0.3};
constexpr double TONEMAP_PARAM = 1.0;
constexpr double HABLE_EXPOSURE = 2.0;
constexpr double HABLE_WHITE_NITS = 100.0;
// the shader scales linear light by (10000 / peak) * (2 / param), then by the white point
// peak / 100, so the scale is fixed and the peak only sets where white sits
constexpr double TONEMAP_SCALE = PQ_PEAK_NITS / HABLE_WHITE_NITS * HABLE_EXPOSURE / TONEMAP_PARAM;
constexpr double SDR_OUTPUT_GAMMA = 2.2;

// interpolated tables stay within 0.1 8-bit code of the exact transfer functions; tables of
// linear light are indexed by its fourth root, which spaces the steps much like PQ does
constexpr int PQ_EOTF_STEPS = 16384;
constexpr int TABLE_STEPS = 4096;
// a log table indexes linear light by the float's exponent and top mantissa bits, uniform in
// linear within each of its octaves; it needs no root
constexpr int LOG_TABLE_STEP_BITS = 8;
constexpr int LOG_TABLE_OCTAVES = 48;
constexpr int FLOAT_MANTISSA_BITS = 23;
constexpr int LOG_TABLE_SHIFT = FLOAT_MANTISSA_BITS - LOG_TABLE_STEP_BITS;
constexpr float LOG_TABLE_MIN = 1.0f / (1ULL << LOG_TABLE_OCTAVES);

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

double LinearToSdr(double l)
{
  return std::pow(l, 1 / SDR_OUTPUT_GAMMA);
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

// the HLG OOTF gain on scene light by its luma, to linear light at the PQ peak
double HlgSystemGain(double luma)
{
  return std::pow(luma, HLG_GAMMA - 1) * HLG_PEAK_NITS / PQ_PEAK_NITS;
}

// the inverse OOTF gain by the luma of linear light at the PQ peak, to scene light at 1.0 for
// the HLG maximum
double HlgInverseGain(double luma)
{
  const double display = PQ_PEAK_NITS / HLG_PEAK_NITS;
  return display * std::pow(std::max(HLG_MIN_LUMA, luma * display), (1 - HLG_GAMMA) / HLG_GAMMA);
}

double SceneToHlg(double v)
{
  const double scene = v * HLG_SCENE_SCALE;
  return scene < 1 ? 0.5 * std::sqrt(scene) : HLG_A * std::log(scene - HLG_B) + HLG_C;
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

template<typename F>
std::vector<float> MakeLogTable(F f)
{
  const int octaveSteps = 1 << LOG_TABLE_STEP_BITS;
  const int steps = LOG_TABLE_OCTAVES * octaveSteps;
  std::vector<float> table;
  table.reserve(steps + 2);
  for (int i = 0; i <= steps; i++)
  {
    const double linear = std::ldexp(1.0 + static_cast<double>(i % octaveSteps) / octaveSteps,
                                     i / octaveSteps - LOG_TABLE_OCTAVES);
    table.push_back(static_cast<float>(f(linear)));
  }
  table.push_back(table.back());
  return table;
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

float LookupLog(const float* table, float linear)
{
  const uint32_t bits = std::bit_cast<uint32_t>(std::clamp(linear, LOG_TABLE_MIN, 1.0f)) -
                        std::bit_cast<uint32_t>(LOG_TABLE_MIN);
  const uint32_t i = bits >> LOG_TABLE_SHIFT;
  const float fraction =
      static_cast<float>(bits & ((1u << LOG_TABLE_SHIFT) - 1)) * (1.0f / (1u << LOG_TABLE_SHIFT));
  return table[i] + (table[i + 1] - table[i]) * fraction;
}

struct Tables
{
  std::vector<float> pqToLinear = MakeTable(PQ_EOTF_STEPS, PQToLinear);
  std::vector<float> linearToPQ = MakeRootTable(TABLE_STEPS, LinearToPQ);
  std::vector<float> linearToPQLog = MakeLogTable(LinearToPQ);
  std::vector<float> srgbToLinear = MakeTable(TABLE_STEPS, SrgbToLinear);
  std::vector<float> linearToSrgb = MakeRootTable(TABLE_STEPS, LinearToSrgb);
  std::vector<float> linearToSrgbLog = MakeLogTable(LinearToSrgb);
  std::vector<float> hlgToScene = MakeTable(TABLE_STEPS, HlgToScene);
  std::vector<float> hlgSystemGain = MakeLogTable(HlgSystemGain);
  std::vector<float> hlgInverseGain = MakeLogTable(HlgInverseGain);
  std::vector<float> sceneToHlg = MakeLogTable(SceneToHlg);
  std::vector<float> linearToBt1886 = MakeLogTable(LinearToBt1886);
  std::vector<float> linearToSdr = MakeLogTable(LinearToSdr);
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

void YccToRgb(const float ycc[3], float kr, float kb, float out[3])
{
  const float y = (ycc[0] - LIMITED_BLACK) / LIMITED_LUMA_SPAN;
  const float cb = (ycc[1] - LIMITED_CHROMA_ZERO) / LIMITED_CHROMA_SPAN;
  const float cr = (ycc[2] - LIMITED_CHROMA_ZERO) / LIMITED_CHROMA_SPAN;
  out[0] = y + 2 * (1 - kr) * cr;
  out[2] = y + 2 * (1 - kb) * cb;
  out[1] = (y - kr * out[0] - kb * out[2]) / (1 - kr - kb);
}

// ICtCp codes to linear BT.2020; both matrices stay in double, as in saturated colours the terms
// nearly cancel; inlined, as an out-of-line call per pixel keeps the row from vectorizing
[[gnu::always_inline]] inline void IctcpToLinear(
    uint16_t y, uint16_t cb, uint16_t cr, const float* eotf, float eotfSteps, float linear[3])
{
  const float ictcp[3] = {y / CODE_SCALE_8_TO_16 / VD1_I_SPAN,
                          (cb / CODE_SCALE_8_TO_16 - VD1_CHROMA_ZERO) / VD1_CT_SPAN,
                          (cr / CODE_SCALE_8_TO_16 - VD1_CHROMA_ZERO) / VD1_CP_SPAN};
  float lms[3];
  Apply(BT2100_ICTCP_TO_LMS, ictcp, lms);
  for (int c = 0; c < 3; c++)
  {
    const float pos = std::clamp(lms[c], 0.0f, 1.0f) * eotfSteps;
    const int i = static_cast<int>(pos);
    lms[c] = eotf[i] + (eotf[i + 1] - eotf[i]) * (pos - i);
  }
  Apply(BT2100_LMS_TO_RGB, lms, linear);
}

void HlgToLinear(const float hlg[3], const Tables& tables, float out[3])
{
  float scene[3];
  for (int c = 0; c < 3; c++)
    scene[c] = Lookup(tables.hlgToScene, hlg[c]);
  const float luma =
      BT2020_KR * scene[0] + (1 - BT2020_KR - BT2020_KB) * scene[1] + BT2020_KB * scene[2];
  const double gain = luma > 0 ? HlgSystemGain(luma) : 0.0;
  for (int c = 0; c < 3; c++)
    out[c] = static_cast<float>(static_cast<double>(scene[c]) * gain);
}

void LinearToHlg(const float linear[3], float out[3])
{
  double v[3];
  for (int c = 0; c < 3; c++)
    v[c] = static_cast<double>(std::max(linear[c], 0.0f));
  const double gain = HlgInverseGain(static_cast<double>(BT2020_KR) * v[0] +
                                     static_cast<double>(1 - BT2020_KR - BT2020_KB) * v[1] +
                                     static_cast<double>(BT2020_KB) * v[2]);
  for (int c = 0; c < 3; c++)
    out[c] = static_cast<float>(SceneToHlg(v[c] * gain));
}

void SdrToLinear(float whiteNits, const Tables& tables, float v[3])
{
  float linear[3];
  for (int c = 0; c < 3; c++)
    linear[c] = static_cast<float>(static_cast<double>(Lookup(tables.srgbToLinear, v[c])) *
                                   static_cast<double>(whiteNits) / PQ_PEAK_NITS);
  Apply(BT709_TO_BT2020, linear, v);
}

void HdrToLinear(AMLCaptureCoding::Transfer transfer, const Tables& tables, float v[3])
{
  switch (transfer)
  {
    case AMLCaptureCoding::Transfer::PQ:
      for (int c = 0; c < 3; c++)
        v[c] = Lookup(tables.pqToLinear, v[c]);
      break;
    case AMLCaptureCoding::Transfer::HLG:
    {
      const float hlg[3] = {v[0], v[1], v[2]};
      HlgToLinear(hlg, tables, v);
      break;
    }
    default:
      break;
  }
}

// Y'CbCr to R'G'B' in 16-bit codes, the spans folded into the coefficients
struct YccCoefficients
{
  float y{0.0f};
  float rCr{0.0f};
  float gCb{0.0f};
  float gCr{0.0f};
  float bCb{0.0f};
};

YccCoefficients MakeYccCoefficients(float kr, float kb)
{
  const double luma = 65535 / static_cast<double>(LIMITED_LUMA_SPAN * CODE_SCALE_8_TO_16);
  const double chroma = 65535 / static_cast<double>(LIMITED_CHROMA_SPAN * CODE_SCALE_8_TO_16);
  const double r = static_cast<double>(kr);
  const double b = static_cast<double>(kb);
  const double g = 1 - r - b;
  YccCoefficients out;
  out.y = static_cast<float>(luma);
  out.rCr = static_cast<float>(chroma * 2 * (1 - r));
  out.gCb = static_cast<float>(-chroma * 2 * b * (1 - b) / g);
  out.gCr = static_cast<float>(-chroma * 2 * r * (1 - r) / g);
  out.bCb = static_cast<float>(chroma * 2 * (1 - b));
  return out;
}

uint16_t ClampTo16(float code)
{
  return static_cast<uint16_t>(std::clamp(code, 0.0f, 65535.0f) + 0.5f);
}

uint16_t ToBigEndian(uint16_t code)
{
  if constexpr (std::endian::native == std::endian::little)
    return static_cast<uint16_t>(code << 8 | code >> 8);
  return code;
}

void YccRowToRgb(const uint16_t* y,
                 const uint16_t* cb,
                 const uint16_t* cr,
                 int width,
                 const YccCoefficients& k,
                 uint16_t* __restrict out)
{
  const float black = LIMITED_BLACK * CODE_SCALE_8_TO_16;
  const float zero = LIMITED_CHROMA_ZERO * CODE_SCALE_8_TO_16;
  for (int x = 0; x < width; x++, out += 3)
  {
    const float luma = k.y * (y[x] - black);
    const float b = cb[x] - zero;
    const float r = cr[x] - zero;
    out[0] = ToBigEndian(ClampTo16(luma + k.rCr * r));
    out[1] = ToBigEndian(ClampTo16(luma + k.gCb * b + k.gCr * r));
    out[2] = ToBigEndian(ClampTo16(luma + k.bCb * b));
  }
}

// a row of pixels as three planes of floats, which each step converts in place
using Planes = std::array<float*, 3>;

void LookupRow(const std::vector<float>& table, float* v, int width)
{
  const float* t = table.data();
  const float steps = static_cast<float>(table.size() - 2);
  for (int x = 0; x < width; x++)
  {
    const float pos = std::clamp(v[x], 0.0f, 1.0f) * steps;
    const int i = static_cast<int>(pos);
    v[x] = t[i] + (t[i + 1] - t[i]) * (pos - i);
  }
}

void LookupLogRow(const std::vector<float>& table, float* v, int width)
{
  const float* t = table.data();
  for (int x = 0; x < width; x++)
    v[x] = LookupLog(t, v[x]);
}

// scales a pixel by the curve at its brightest channel, which keeps its hue (libplacebo
// PL_TONE_MAP_MAX, ffmpeg vf_tonemap without desaturation); a channel past the PQ range scales
// to white with the others
void ToneMapRow(const std::vector<float>& encode,
                const std::vector<float>& gain,
                const Planes& p,
                int width)
{
  const float* e = encode.data();
  const float* g = gain.data();
  for (int x = 0; x < width; x++)
  {
    float brightest = p[0][x] > p[1][x] ? p[0][x] : p[1][x];
    brightest = brightest > p[2][x] ? brightest : p[2][x];
    const float k = LookupLog(g, brightest);
    const float scale = brightest > 1.0f ? 1.0f / brightest : 1.0f;
    for (float* v : p)
      v[x] = LookupLog(e, v[x] * scale) * k;
  }
}

// in float, which stays within a code of double for these gamut matrices; the Dolby Vision
// decode keeps double
void ApplyRow(const Matrix& m, const Planes& p, int width)
{
  float k[3][3];
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      k[r][c] = static_cast<float>(m[r][c]);
  for (int x = 0; x < width; x++)
  {
    const float in[3] = {p[0][x], p[1][x], p[2][x]};
    for (int r = 0; r < 3; r++)
      p[r][x] = k[r][0] * in[0] + k[r][1] * in[1] + k[r][2] * in[2];
  }
}

Matrix ScaleMatrix(const Matrix& m, double scale)
{
  Matrix out = m;
  for (auto& row : out)
    for (double& v : row)
      v *= scale;
  return out;
}

void YccRow(const uint16_t* y,
            const uint16_t* cb,
            const uint16_t* cr,
            int width,
            float kr,
            float kb,
            const Planes& p)
{
  for (int x = 0; x < width; x++)
  {
    const float ycc[3] = {y[x] / CODE_SCALE_8_TO_16, cb[x] / CODE_SCALE_8_TO_16,
                          cr[x] / CODE_SCALE_8_TO_16};
    float rgb[3];
    YccToRgb(ycc, kr, kb, rgb);
    for (int c = 0; c < 3; c++)
      p[c][x] = rgb[c];
  }
}

// restrict, as plane stores that may alias the table reads keep the row from vectorizing
void IctcpRow(const uint16_t* y,
              const uint16_t* cb,
              const uint16_t* cr,
              int width,
              const Tables& tables,
              float* __restrict r,
              float* __restrict g,
              float* __restrict b)
{
  const float* eotf = tables.pqToLinear.data();
  const float eotfSteps = static_cast<float>(tables.pqToLinear.size() - 2);
  for (int x = 0; x < width; x++)
  {
    float linear[3];
    IctcpToLinear(y[x], cb[x], cr[x], eotf, eotfSteps, linear);
    r[x] = linear[0];
    g[x] = linear[1];
    b[x] = linear[2];
  }
}

// Dolby Vision decoded to PQ BT.2020
void IctcpRowToPQ(const uint16_t* y,
                  const uint16_t* cb,
                  const uint16_t* cr,
                  int width,
                  const Tables& tables,
                  uint16_t* __restrict out)
{
  const float* eotf = tables.pqToLinear.data();
  const float eotfSteps = static_cast<float>(tables.pqToLinear.size() - 2);
  const float* oetf = tables.linearToPQLog.data();
  for (int x = 0; x < width; x++, out += 3)
  {
    float linear[3];
    IctcpToLinear(y[x], cb[x], cr[x], eotf, eotfSteps, linear);
    for (int c = 0; c < 3; c++)
      out[c] = ToBigEndian(ClampTo16(LookupLog(oetf, linear[c]) * 65535));
  }
}

void HlgRowToLinear(const Tables& tables, const Planes& p, int width)
{
  for (float* v : p)
    LookupRow(tables.hlgToScene, v, width);
  const float* gain = tables.hlgSystemGain.data();
  for (int x = 0; x < width; x++)
  {
    const float luma =
        BT2020_KR * p[0][x] + (1 - BT2020_KR - BT2020_KB) * p[1][x] + BT2020_KB * p[2][x];
    const float g = LookupLog(gain, luma);
    for (float* v : p)
      v[x] *= g;
  }
}

// the GUI composite's HLG coding (gles_gui_composite.frag)
void LinearRowToHlg(const Tables& tables, const Planes& p, int width)
{
  const float* gain = tables.hlgInverseGain.data();
  const float* oetf = tables.sceneToHlg.data();
  for (int x = 0; x < width; x++)
  {
    float linear[3];
    for (int c = 0; c < 3; c++)
      linear[c] = std::max(p[c][x], 0.0f);
    const float luma =
        BT2020_KR * linear[0] + (1 - BT2020_KR - BT2020_KB) * linear[1] + BT2020_KB * linear[2];
    const float g = LookupLog(gain, luma);
    for (int c = 0; c < 3; c++)
      p[c][x] = LookupLog(oetf, linear[c] * g);
  }
}

// R'G'B' of an SDR signal, else linear BT.2020; true for linear
bool DecodeRow(AMLCaptureSignal signal,
               const uint16_t* y,
               const uint16_t* cb,
               const uint16_t* cr,
               int width,
               const Tables& tables,
               const Planes& p)
{
  switch (signal)
  {
    case AMLCaptureSignal::PQ:
      YccRow(y, cb, cr, width, BT2020_KR, BT2020_KB, p);
      for (float* v : p)
        LookupRow(tables.pqToLinear, v, width);
      return true;
    case AMLCaptureSignal::HLG:
      YccRow(y, cb, cr, width, BT2020_KR, BT2020_KB, p);
      HlgRowToLinear(tables, p, width);
      return true;
    case AMLCaptureSignal::DV_TV_LED:
    case AMLCaptureSignal::DV_LL:
    case AMLCaptureSignal::DV_HDR10:
      IctcpRow(y, cb, cr, width, tables, p[0], p[1], p[2]);
      return true;
    case AMLCaptureSignal::DV_SDR:
      IctcpRow(y, cb, cr, width, tables, p[0], p[1], p[2]);
      ApplyRow(BT2020_TO_BT709, p, width);
      for (float* v : p)
        LookupLogRow(tables.linearToBt1886, v, width);
      return false;
    case AMLCaptureSignal::SDR:
    default:
      YccRow(y, cb, cr, width, BT709_KR, BT709_KB, p);
      return false;
  }
}

// fill the odd samples of a chroma row that holds the pair's sample at the even ones
void UpsampleChroma(uint16_t* row, int pairs, bool cosited)
{
  if (pairs <= 0)
    return;

  const int last = pairs - 1;
  if (cosited)
  {
    for (int k = 0; k < last; k++)
      row[2 * k + 1] = static_cast<uint16_t>((row[2 * k] + row[2 * k + 2] + 1) / 2);
    row[2 * last + 1] = row[2 * last];
    return;
  }

  // a pair average sits between its pixels, a quarter of a sample from each
  int previous = row[0];
  for (int k = 0; k < last; k++)
  {
    const int current = row[2 * k];
    const int next = row[2 * k + 2];
    row[2 * k] = static_cast<uint16_t>((previous + 3 * current + 2) / 4);
    row[2 * k + 1] = static_cast<uint16_t>((3 * current + next + 2) / 4);
    previous = current;
  }
  const int current = row[2 * last];
  row[2 * last] = static_cast<uint16_t>((previous + 3 * current + 2) / 4);
  row[2 * last + 1] = static_cast<uint16_t>(current);
}

// a full pack row to YUV444P16 rows, codes shifted as swscale widens 8-bit ones
void UnpackFullPackRow(const AMLCaptureImage& src, int row, uint16_t* y, uint16_t* cb, uint16_t* cr)
{
  const auto code = [](uint64_t group, int index)
  {
    return static_cast<uint16_t>(((group >> (index * FULL_PACK_CODE_BITS)) & FULL_PACK_CODE_MASK)
                                 << CODE_SHIFT_10_TO_16);
  };
  const int pairs = src.width / 2;
  // a 64-bit load reads past its pair, so the last pair of a row is read bytewise
  const int loads = std::endian::native == std::endian::little ? pairs - 1 : 0;
  const uint8_t* in = src.data[0] + static_cast<std::ptrdiff_t>(row) * src.linesize[0];
  const auto store = [&](int k, uint64_t group)
  {
    cb[2 * k] = code(group, 0);
    y[2 * k] = code(group, 1);
    cr[2 * k] = code(group, 2);
    y[2 * k + 1] = code(group, 3);
  };
  int k = 0;
  for (; k < loads; k++, in += AML_FULL_PACK_PAIR_BYTES)
  {
    uint64_t group = 0;
    std::memcpy(&group, in, sizeof(group));
    store(k, group);
  }
  for (; k < pairs; k++, in += AML_FULL_PACK_PAIR_BYTES)
  {
    uint64_t group = 0;
    for (int b = AML_FULL_PACK_PAIR_BYTES - 1; b >= 0; b--)
      group = group << 8 | in[b];
    store(k, group);
  }
  UpsampleChroma(cb, pairs, src.cositedChroma);
  UpsampleChroma(cr, pairs, src.cositedChroma);
}

// full pack to YUV444P16 planes of the source size
void UnpackFullPack(const AMLCaptureImage& src, uint16_t* y, uint16_t* cb, uint16_t* cr)
{
  for (int row = 0; row < src.height; row++)
  {
    const std::size_t offset = static_cast<std::size_t>(row) * src.width;
    UnpackFullPackRow(src, row, y + offset, cb + offset, cr + offset);
  }
}

uint8_t To8(float v)
{
  return static_cast<uint8_t>(v * 255 + 0.5f);
}

uint16_t To16(float v)
{
  return static_cast<uint16_t>(v * 65535 + 0.5f);
}

// a row of planes to 8-bit pixels of STEP bytes, red at R and blue at B, opaque at 4 bytes
template<int R, int B, int STEP>
[[gnu::always_inline]] inline void StoreRow8(const Planes& p, int width, uint8_t* __restrict out)
{
  const float* __restrict red = p[0];
  const float* __restrict green = p[1];
  const float* __restrict blue = p[2];
  for (int x = 0; x < width; x++, out += STEP)
  {
    out[R] = To8(std::clamp(red[x], 0.0f, 1.0f));
    out[1] = To8(std::clamp(green[x], 0.0f, 1.0f));
    out[B] = To8(std::clamp(blue[x], 0.0f, 1.0f));
    if constexpr (STEP == 4)
      out[3] = 255;
  }
}

// the end of the run from x of pixels where the overlay gui is clear, or is not; without an overlay
// the row is one run
int RunEnd(const uint8_t* gui, int x, int width)
{
  if (!gui)
    return width;

  const bool clear = CAMLCaptureConvert::IsClear(gui + 4 * x);
  int end = x + 1;
  while (end < width && CAMLCaptureConvert::IsClear(gui + 4 * end) == clear)
    end++;
  return end;
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

  // the shader clips at half the peak; white at the peak rolls the highlights off instead
  const double white = Hable(TONEMAP_SCALE * static_cast<double>(luminance) / PQ_PEAK_NITS);
  m_toneMap = MakeLogTable(
      [white](double linear)
      {
        const double sdr = std::clamp(Hable(linear * TONEMAP_SCALE) / white, 0.0, 1.0);
        return LinearToSdr(sdr / linear);
      });
  m_toneMapLuminance = luminance;
  return m_toneMap;
}

bool CAMLCaptureConvert::Stage(const AMLCaptureImage& src, int width, int height)
{
  const std::size_t plane = static_cast<std::size_t>(width) * height;
  m_yuv.Resize(plane * 3);
  uint8_t* yuv[] = {reinterpret_cast<uint8_t*>(m_yuv.Data()),
                    reinterpret_cast<uint8_t*>(m_yuv.Data() + plane),
                    reinterpret_cast<uint8_t*>(m_yuv.Data() + 2 * plane), nullptr};
  const int yuvStride[] = {width * 2, width * 2, width * 2, 0};

  // a full pack is the caller's copy, so it is unpacked straight to planes
  if (src.fullPack)
  {
    const std::size_t srcPlane = static_cast<std::size_t>(src.width) * src.height;
    m_unpacked.Resize(srcPlane * 3);
    UnpackFullPack(src, m_unpacked.Data(), m_unpacked.Data() + srcPlane,
                   m_unpacked.Data() + 2 * srcPlane);
    m_scaleContext = sws_getCachedContext(
        m_scaleContext, src.width, src.height, AV_PIX_FMT_YUV444P16LE, width, height,
        AV_PIX_FMT_YUV444P16LE, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!m_scaleContext)
      return false;

    const uint8_t* srcData[] = {reinterpret_cast<const uint8_t*>(m_unpacked.Data()),
                                reinterpret_cast<const uint8_t*>(m_unpacked.Data() + srcPlane),
                                reinterpret_cast<const uint8_t*>(m_unpacked.Data() + 2 * srcPlane),
                                nullptr};
    const int srcStride[] = {src.width * 2, src.width * 2, src.width * 2, 0};
    sws_scale(m_scaleContext, srcData, srcStride, 0, src.height, yuv, yuvStride);
    return true;
  }

  // the capture mapping is uncached: one sequential read, and every pass after it on the copy
  const std::size_t luma = static_cast<std::size_t>(src.width) * src.height;
  m_source.Resize(luma + luma / 2);
  for (int y = 0; y < src.height; y++)
    std::memcpy(m_source.Data() + static_cast<std::size_t>(y) * src.width,
                src.data[0] + static_cast<std::ptrdiff_t>(y) * src.linesize[0], src.width);
  for (int y = 0; y < src.height / 2; y++)
    std::memcpy(m_source.Data() + luma + static_cast<std::size_t>(y) * src.width,
                src.data[1] + static_cast<std::ptrdiff_t>(y) * src.linesize[1], src.width);

  m_scaleContext =
      sws_getCachedContext(m_scaleContext, src.width, src.height, AV_PIX_FMT_NV12, width, height,
                           AV_PIX_FMT_YUV444P16LE, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (!m_scaleContext)
    return false;

  const uint8_t* srcData[] = {m_source.Data(), m_source.Data() + luma, nullptr, nullptr};
  const int srcStride[] = {src.width, src.width, 0, 0};
  sws_scale(m_scaleContext, srcData, srcStride, 0, src.height, yuv, yuvStride);
  return true;
}

bool CAMLCaptureConvert::Convert(const AMLCaptureImage& src,
                                 const AMLCaptureColor& color,
                                 const AMLCaptureCoding& coding,
                                 const AMLCaptureImage& dst,
                                 const AMLCaptureImage* gui,
                                 const AMLCaptureImage* sdr)
{
  const bool bgra = dst.format == AV_PIX_FMT_BGRA;
  const bool rgb = dst.format == AV_PIX_FMT_RGB24;
  if ((src.format != AV_PIX_FMT_NV12 && !src.fullPack) || src.width <= 0 || src.height <= 0 ||
      dst.width <= 0 || dst.height <= 0 || (!bgra && !rgb && dst.format != AV_PIX_FMT_RGBA64LE))
    return false;

  // a full pack at the output size is unpacked a row at a time, which spares the planes
  const bool packRows = src.fullPack && src.width == dst.width && src.height == dst.height;
  if (packRows)
    m_yuv.Resize(static_cast<std::size_t>(dst.width) * 3);
  else if (!Stage(src, dst.width, dst.height))
    return false;

  const std::size_t plane = static_cast<std::size_t>(dst.width) * (packRows ? 1 : dst.height);
  const std::vector<float>* toneMap = IsSdr(color.signal) ? nullptr : &GetToneMap(color.luminance);
  const Tables& tables = GetTables();
  // an HDR coding holds the SDR video as the GUI shader places SDR
  const Matrix sdrToLinear =
      ScaleMatrix(BT709_TO_BT2020, static_cast<double>(coding.whiteNits) / PQ_PEAK_NITS);
  std::vector<float> rows(static_cast<std::size_t>(dst.width) * 3);
  const Planes p = {rows.data(), rows.data() + dst.width, rows.data() + 2 * dst.width};
  for (int y = 0; y < dst.height; y++)
  {
    uint16_t* luma = m_yuv.Data() + (packRows ? 0 : static_cast<std::size_t>(y) * dst.width);
    if (packRows)
      UnpackFullPackRow(src, y, luma, luma + plane, luma + 2 * plane);
    if (DecodeRow(color.signal, luma, luma + plane, luma + 2 * plane, dst.width, tables, p))
    {
      ApplyRow(BT2020_TO_BT709, p, dst.width);
      ToneMapRow(tables.linearToSdr, *toneMap, p, dst.width);
    }

    const uint8_t* overlay =
        gui ? gui->data[0] + static_cast<std::ptrdiff_t>(y) * gui->linesize[0] : nullptr;
    int x2 = 0;
    for (int x1 = 0; x1 < dst.width; x1 = x2)
    {
      x2 = RunEnd(overlay, x1, dst.width);
      const int width = x2 - x1;
      const Planes span = {p[0] + x1, p[1] + x1, p[2] + x1};
      if (overlay && IsClear(overlay + 4 * x1))
      {
        StoreRow8<0, 2, 4>(
            span, width, sdr->data[0] + static_cast<std::ptrdiff_t>(y) * sdr->linesize[0] + x1 * 4);
        continue;
      }

      if (coding.transfer != AMLCaptureCoding::Transfer::SDR)
      {
        for (float* v : span)
          LookupRow(tables.srgbToLinear, v, width);
        ApplyRow(sdrToLinear, span, width);
        if (coding.transfer == AMLCaptureCoding::Transfer::PQ)
        {
          for (float* v : span)
            LookupLogRow(tables.linearToPQLog, v, width);
        }
        else
          LinearRowToHlg(tables, span, width);
      }

      uint8_t* row = dst.data[0] + static_cast<std::ptrdiff_t>(y) * dst.linesize[0];
      if (bgra)
        StoreRow8<2, 0, 4>(span, width, row + x1 * 4);
      else if (rgb)
        StoreRow8<0, 2, 3>(span, width, row + x1 * 3);
      else
      {
        uint16_t* out = reinterpret_cast<uint16_t*>(row) + x1 * 4;
        for (int x = 0; x < width; x++, out += 4)
        {
          for (int c = 0; c < 3; c++)
            out[c] = To16(std::clamp(span[c][x], 0.0f, 1.0f));
          out[3] = 65535;
        }
      }
    }
  }
  return true;
}

bool CAMLCaptureConvert::ConvertNative(const AMLCaptureImage& src,
                                       const AMLCaptureColor& color,
                                       const AMLCaptureImage& dst)
{
  if (!src.fullPack || src.width <= 0 || src.height <= 0 || src.width != dst.width ||
      src.height != dst.height || dst.format != AV_PIX_FMT_RGB48BE)
    return false;

  // the signal's own R'G'B'; Dolby Vision has none, so it is coded PQ
  bool ictcp = false;
  YccCoefficients coefficients;
  switch (color.signal)
  {
    case AMLCaptureSignal::PQ:
    case AMLCaptureSignal::HLG:
      coefficients = MakeYccCoefficients(BT2020_KR, BT2020_KB);
      break;
    case AMLCaptureSignal::DV_TV_LED:
    case AMLCaptureSignal::DV_LL:
    case AMLCaptureSignal::DV_HDR10:
    case AMLCaptureSignal::DV_SDR:
      ictcp = true;
      break;
    case AMLCaptureSignal::SDR:
    default:
      coefficients = MakeYccCoefficients(BT709_KR, BT709_KB);
      break;
  }

  const int width = dst.width;
  const auto yuv = std::make_unique_for_overwrite<uint16_t[]>(static_cast<std::size_t>(width) * 3);
  uint16_t* luma = yuv.get();
  const Tables& tables = GetTables();
  for (int y = 0; y < dst.height; y++)
  {
    UnpackFullPackRow(src, y, luma, luma + width, luma + 2 * width);
    uint16_t* out =
        reinterpret_cast<uint16_t*>(dst.data[0] + static_cast<std::ptrdiff_t>(y) * dst.linesize[0]);
    if (ictcp)
      IctcpRowToPQ(luma, luma + width, luma + 2 * width, width, tables, out);
    else
      YccRowToRgb(luma, luma + width, luma + 2 * width, width, coefficients, out);
  }
  return true;
}

AMLCaptureCoding::Transfer CAMLCaptureConvert::GetNativeTransfer(AMLCaptureSignal signal)
{
  switch (signal)
  {
    case AMLCaptureSignal::PQ:
    case AMLCaptureSignal::DV_TV_LED:
    case AMLCaptureSignal::DV_LL:
    case AMLCaptureSignal::DV_HDR10:
    case AMLCaptureSignal::DV_SDR:
      return AMLCaptureCoding::Transfer::PQ;
    case AMLCaptureSignal::HLG:
      return AMLCaptureCoding::Transfer::HLG;
    case AMLCaptureSignal::SDR:
    default:
      return AMLCaptureCoding::Transfer::SDR;
  }
}

void CAMLCaptureConvert::Recode(const float in[3],
                                const AMLCaptureCoding& from,
                                const AMLCaptureCoding& to,
                                float out[3])
{
  const Tables& tables = GetTables();
  float v[3] = {in[0], in[1], in[2]};
  if (from.transfer == AMLCaptureCoding::Transfer::SDR)
    SdrToLinear(from.whiteNits, tables, v);
  else
    HdrToLinear(from.transfer, tables, v);

  switch (to.transfer)
  {
    case AMLCaptureCoding::Transfer::PQ:
      for (int c = 0; c < 3; c++)
        out[c] = LookupRoot(tables.linearToPQ, v[c]);
      break;
    case AMLCaptureCoding::Transfer::HLG:
      LinearToHlg(v, out);
      break;
    case AMLCaptureCoding::Transfer::SDR:
    default:
    {
      for (int c = 0; c < 3; c++)
        v[c] = static_cast<float>(static_cast<double>(v[c]) * PQ_PEAK_NITS /
                                  static_cast<double>(to.whiteNits));
      Apply(BT2020_TO_BT709, v, out);
      for (int c = 0; c < 3; c++)
        out[c] = LookupRoot(tables.linearToSrgb, out[c]);
      break;
    }
  }

  for (int c = 0; c < 3; c++)
    out[c] = std::clamp(out[c], 0.0f, 1.0f);
}

bool CAMLCaptureConvert::Scale(const AMLCaptureImage& src, const AMLCaptureImage& dst)
{
  SwsContext* context = sws_getContext(
      src.width, src.height, AV_PIX_FMT_RGBA, dst.width, dst.height, AV_PIX_FMT_RGBA,
      SWS_BILINEAR | SWS_FULL_CHR_H_INP | SWS_ACCURATE_RND, nullptr, nullptr, nullptr);
  if (!context)
    return false;

  const uint8_t* srcData[] = {src.data[0], nullptr, nullptr, nullptr};
  const int srcStride[] = {src.linesize[0], 0, 0, 0};
  uint8_t* dstData[] = {dst.data[0], nullptr, nullptr, nullptr};
  const int dstStride[] = {dst.linesize[0], 0, 0, 0};
  sws_scale(context, srcData, srcStride, 0, src.height, dstData, dstStride);
  sws_freeContext(context);
  return true;
}

void CAMLCaptureConvert::MapToSdr(const AMLCaptureImage& src,
                                  const AMLCaptureCoding& coding,
                                  const AMLCaptureImage& dst,
                                  const AMLCaptureImage* gui)
{
  const Tables& tables = GetTables();
  const Matrix toSdr =
      ScaleMatrix(BT2020_TO_BT709, PQ_PEAK_NITS / static_cast<double>(coding.whiteNits));
  std::vector<float> rows(static_cast<std::size_t>(src.width) * 3);
  const Planes p = {rows.data(), rows.data() + src.width, rows.data() + 2 * src.width};
  for (int y = 0; y < src.height; y++)
  {
    const uint8_t* overlay =
        gui ? gui->data[0] + static_cast<std::ptrdiff_t>(y) * gui->linesize[0] : nullptr;
    int x2 = 0;
    for (int x1 = 0; x1 < src.width; x1 = x2)
    {
      x2 = RunEnd(overlay, x1, src.width);
      if (overlay && IsClear(overlay + 4 * x1))
        continue;

      const int width = x2 - x1;
      const Planes span = {p[0] + x1, p[1] + x1, p[2] + x1};
      const uint16_t* in = reinterpret_cast<const uint16_t*>(
                               src.data[0] + static_cast<std::ptrdiff_t>(y) * src.linesize[0]) +
                           x1 * 4;
      for (int x = 0; x < width; x++)
      {
        for (int c = 0; c < 3; c++)
          span[c][x] = in[4 * x + c] / 65535.0f;
      }
      if (coding.transfer == AMLCaptureCoding::Transfer::HLG)
        HlgRowToLinear(tables, span, width);
      else
      {
        for (float* v : span)
          LookupRow(tables.pqToLinear, v, width);
      }
      ApplyRow(toSdr, span, width);
      for (float* v : span)
        LookupLogRow(tables.linearToSrgbLog, v, width);

      StoreRow8<0, 2, 4>(span, width,
                         dst.data[0] + static_cast<std::ptrdiff_t>(y) * dst.linesize[0] + x1 * 4);
    }
  }
}

AMLCaptureImage CAMLCaptureConvert::GetCompositeImage(int width, int height)
{
  m_composite.Resize(static_cast<std::size_t>(width) * height * 4);
  return AMLCaptureImage::Packed(AV_PIX_FMT_RGBA64LE, width, height,
                                 reinterpret_cast<uint8_t*>(m_composite.Data()), width * 8);
}
