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

// interpolated tables stay within 0.1 8-bit code of the exact transfer functions
constexpr int PQ_EOTF_STEPS = 16384;
constexpr int TABLE_STEPS = 4096;
// a log table indexes linear light by the float's exponent and top mantissa bits, uniform in
// linear within each of its octaves
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
  std::vector<float> hlgToScene = MakeTable(TABLE_STEPS, HlgToScene);
  std::vector<float> hlgSystemGain = MakeLogTable(HlgSystemGain);
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

uint8_t To8(float v)
{
  return static_cast<uint8_t>(v * 255 + 0.5f);
}

// a row of planes to opaque 8-bit pixels, red at R and blue at B
template<int R, int B>
[[gnu::always_inline]] inline void StoreRow8(const Planes& p, int width, uint8_t* __restrict out)
{
  const float* __restrict red = p[0];
  const float* __restrict green = p[1];
  const float* __restrict blue = p[2];
  for (int x = 0; x < width; x++, out += 4)
  {
    out[R] = To8(std::clamp(red[x], 0.0f, 1.0f));
    out[1] = To8(std::clamp(green[x], 0.0f, 1.0f));
    out[B] = To8(std::clamp(blue[x], 0.0f, 1.0f));
    out[3] = 255;
  }
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
                                 const AMLCaptureImage& dst)
{
  if (src.format != AV_PIX_FMT_NV12 || src.width <= 0 || src.height <= 0 || dst.width <= 0 ||
      dst.height <= 0 || dst.format != AV_PIX_FMT_BGRA)
    return false;

  if (!Stage(src, dst.width, dst.height))
    return false;

  const std::size_t plane = static_cast<std::size_t>(dst.width) * dst.height;
  const std::vector<float>* toneMap = IsSdr(color.signal) ? nullptr : &GetToneMap(color.luminance);
  const Tables& tables = GetTables();
  std::vector<float> rows(static_cast<std::size_t>(dst.width) * 3);
  const Planes p = {rows.data(), rows.data() + dst.width, rows.data() + 2 * dst.width};
  for (int y = 0; y < dst.height; y++)
  {
    const uint16_t* luma = m_yuv.Data() + static_cast<std::size_t>(y) * dst.width;
    if (DecodeRow(color.signal, luma, luma + plane, luma + 2 * plane, dst.width, tables, p))
    {
      ApplyRow(BT2020_TO_BT709, p, dst.width);
      ToneMapRow(tables.linearToSdr, *toneMap, p, dst.width);
    }

    StoreRow8<2, 0>(p, dst.width, dst.data[0] + static_cast<std::ptrdiff_t>(y) * dst.linesize[0]);
  }
  return true;
}
