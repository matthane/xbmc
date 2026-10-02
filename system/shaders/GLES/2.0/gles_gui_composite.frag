/*
 *  Copyright (C) 2026 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#version 100

#ifdef KODI_GUI_LUT3D
#extension GL_OES_texture_3D : require
#endif

precision mediump float;

// in mediump the HLG OOTF luminance scale and the OETF wobble by a code, which steps
// the output against the input in smooth gradients, and sqrt is not monotonic
#ifdef GL_FRAGMENT_PRECISION_HIGH
#define HLG_PRECISION highp
#define PQ_INDEX_PRECISION highp
#else
#define HLG_PRECISION mediump
#define PQ_INDEX_PRECISION mediump
#endif

varying vec2 v_tex;
#ifdef KODI_GUI_LUT3D
uniform mediump sampler2D u_samp; // GUI FBO texture (sRGB, rendered by GUI shaders)
#else
uniform sampler2D u_samp;       // GUI FBO texture (sRGB, rendered by GUI shaders)
#endif
uniform sampler2D u_lutDegamma; // sRGB -> linear LUT (IEC 61966-2-1)
uniform sampler2D u_lutTF;      // sqrt(linear) -> PQ LUT (PQ_LUT_SIZE entries, sdrPeak baked in)
uniform HLG_PRECISION float u_ootfGamma; // HLG: OOTF gamma (1.2 for BT.2100 1000-nit ref)
                                         // PQ: 0.0 (use LUT path instead)
uniform HLG_PRECISION float u_hlgWhite;  // HLG: GUI white / 1000-nit nominal peak

// texel-centre addressing: input x samples texel x * (KODI_LUT_SIZE - 1) exactly
const float LUT_SCALE = (KODI_LUT_SIZE - 1.0) / KODI_LUT_SIZE;
const float LUT_OFFSET = 0.5 / KODI_LUT_SIZE;
const PQ_INDEX_PRECISION float PQ_LUT_SCALE = (KODI_PQ_LUT_SIZE - 1.0) / KODI_PQ_LUT_SIZE;
const PQ_INDEX_PRECISION float PQ_LUT_OFFSET = 0.5 / KODI_PQ_LUT_SIZE;

#ifdef KODI_GUI_LUT3D
// sRGB -> output code in one fetch; node i sits at sRGB value (i / (size - 1))^2
uniform mediump sampler3D u_lut3d;
// texel-centre scale and offset for the size of the LUT
uniform highp vec2 u_lut3dMap;
#endif

// BT.709 -> BT.2020 color space conversion matrix (applied in linear light)
const mat3 bt709_to_bt2020 = mat3(
  0.6274,  0.0691,  0.0164,
  0.3293,  0.9195,  0.0880,
  0.0433,  0.0114,  0.8956
);

void main()
{
  vec4 gui = texture2D(u_samp, v_tex);

  // Skip pixels the GUI never wrote to. Blend unit would still preserve video
  // bit-exactly via DST*(1-0)+garbage*0=DST, but discard makes it structural
  // and avoids the tone-map math, BO read and BO write for those pixels.
  if (gui.a == 0.0)
    discard;

#ifdef KODI_GUI_LUT3D
  vec3 result = texture3D(u_lut3d, sqrt(gui.rgb) * u_lut3dMap.x + u_lut3dMap.y).rgb;
#else
  // sRGB -> linear via LUT (IEC 61966-2-1 EOTF, replaces inline pow)
  vec3 d = gui.rgb * LUT_SCALE + LUT_OFFSET;
  vec3 linear = vec3(
    texture2D(u_lutDegamma, vec2(d.r, 0.5)).r,
    texture2D(u_lutDegamma, vec2(d.g, 0.5)).r,
    texture2D(u_lutDegamma, vec2(d.b, 0.5)).r
  );

  vec3 result;

  if (u_ootfGamma > 0.0)
  {
    // HLG path: direct computation following libplacebo pl_color_delinearize.
    // BT.2100 reference display: 1000 nits. GUI white (linear 1.0) = u_hlgWhite
    // of that peak; 0.203 (BT.2408 reference white) maps to 75% HLG signal.
    //
    // Step 1: normalize to display-peak-relative units
    // Step 2: inverse OOTF with 12x prescale for OETF input domain
    // Step 3: HLG OETF (ARIB STD-B67) piecewise: sqrt for <=1, log for >1
    const float HLG_A = 0.17883277;
    const float HLG_B = 0.28466892;
    const float HLG_C = 0.55991073;

    // BT.709 -> BT.2020 gamut mapping
    HLG_PRECISION vec3 scene = linear;
    scene = bt709_to_bt2020 * scene * u_hlgWhite;
    HLG_PRECISION float Y = dot(scene, vec3(0.2627, 0.6780, 0.0593));
    scene *= 12.0 * pow(max(1e-6, Y), (1.0 - u_ootfGamma) / u_ootfGamma);

    // HLG OETF piecewise (threshold at scene-light 1.0)
    vec3 lo = vec3(0.5) * sqrt(max(scene, vec3(0.0)));
    vec3 hi = vec3(HLG_A) * log(max(scene - vec3(HLG_B), vec3(1e-6))) + vec3(HLG_C);
    result = mix(lo, hi, step(vec3(1.0), scene));
  }
  else
  {
    // BT.709 -> BT.2020 gamut mapping
    linear = bt709_to_bt2020 * linear;

    // PQ path: LUT lookup (sdrPeak baked into LUT range), indexed by sqrt so the
    // shadows get as many entries as the highlights; the LUT is read unfiltered,
    // since an fp16 blend of two entries is not monotonic either
    PQ_INDEX_PRECISION vec3 s = linear;
    s = sqrt(max(s, 0.0)) * PQ_LUT_SCALE + PQ_LUT_OFFSET;
    result = vec3(
      texture2D(u_lutTF, vec2(s.r, 0.5)).r,
      texture2D(u_lutTF, vec2(s.g, 0.5)).r,
      texture2D(u_lutTF, vec2(s.b, 0.5)).r
    );
  }
#endif

  // Limited-range encoding at the BO write boundary. Canonical normalized
  // ratios (bit-depth-agnostic in float space; BO write quantizes to the
  // active surface bit depth). The 3D LUT holds the range itself, since HLG
  // goes above 1.0 before the limited-range scale.
#if defined(KODI_LIMITED_RANGE) && !defined(KODI_GUI_LUT3D)
  result = result * ((235.0 - 16.0) / 255.0) + (16.0 / 255.0);
#endif

  gl_FragColor = vec4(result, gui.a);
}
