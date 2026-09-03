/*
 *  Copyright (C) 2011-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "utils/StreamDetails.h"

enum AML_SUPPORT_H264_4K2K
{
  AML_SUPPORT_H264_4K2K_UNINIT = -1,
  AML_NO_H264_4K2K,
  AML_HAS_H264_4K2K,
  AML_HAS_H264_4K2K_SAME_PROFILE
};

enum AML_DISPLAY_DV_LED
{
  AML_DV_TV_LED = 0,
  AML_DV_PLAYER_LED
};

#define AMDV_FOLLOW_SINK        (unsigned int)(0)
#define AMDV_FOLLOW_SOURCE      (unsigned int)(1)
#define AMDV_FORCE_OUTPUT_MODE  (unsigned int)(2)

#define AMDV_OUTPUT_MODE_IPT         (unsigned int)(0)
#define AMDV_OUTPUT_MODE_IPT_TUNNEL  (unsigned int)(1)
#define AMDV_OUTPUT_MODE_HDR10       (unsigned int)(2)
#define AMDV_OUTPUT_MODE_SDR10       (unsigned int)(3)
#define AMDV_OUTPUT_MODE_SDR8        (unsigned int)(4)
#define AMDV_OUTPUT_MODE_BYPASS      (unsigned int)(5)

#define DOLBY_VISION_LL_DISABLE (unsigned int)(0)
#define DOLBY_VISION_LL_YUV422  (unsigned int)(1)

#define AML_HDMI_CS_RGB         (unsigned int)(0)
#define AML_HDMI_CS_YUV422      (unsigned int)(1)
#define AML_HDMI_CS_YUV444      (unsigned int)(2)

#define HDR10_PLUS_CAP      (int)(1<<0)
#define HDR10_CAP           (int)(1<<2)
#define SMPTE_ST_2084_CAP   (int)(1<<3)
#define HLG_CAP             (int)(1<<4)

#define DV_2160p60Hz        (int)(1<<2)
#define DV_RGB_444_8BIT     (int)(1<<3)
#define LL_YCbCr_422_12BIT  (int)(1<<5)

#define AML_GXBB    0x1F
#define AML_GXL     0x21
#define AML_GXM     0x22
#define AML_G12A    0x28
#define AML_G12B    0x29
#define AML_SM1     0x2B
#define AML_SC2     0x32
#define AML_T7      0x36
#define AML_S4      0x37
#define AML_S5      0x3E
#define AML_S7      0x46
#define AML_S7D     0x47
#define AML_S6      0x48

int  aml_get_cpufamily_id();
std::string aml_get_cpufamily_name(int cpuid = -1);
bool aml_support_hevc();
bool aml_support_hevc_4k2k();
bool aml_support_hevc_8k4k();
bool aml_support_hevc_10bit();
bool aml_support_h266();
AML_SUPPORT_H264_4K2K aml_support_h264_4k2k();
bool aml_support_vp9();
bool aml_support_av1();
bool aml_support_avs2();
bool aml_support_avs3();
bool aml_support_dolby_vision();
bool aml_dolby_vision_enabled();
void aml_dv_restore_vs10_wire(void);
void aml_dv_set_vs10_mode(unsigned int mode, bool native_dv = false);
bool aml_convert_to_dv_by_vs_engine(StreamHdrType hdrType);
bool aml_video_started();
int aml_amdv_wait(StreamHdrType hdrType);
unsigned int aml_dv_resolve_tunnel_mode(unsigned int mode);
bool aml_dv_vs10_conversion_active(void);
void aml_dv_vs10_request_conversion(void);
void aml_dv_vs10_drop_conversion(void);
void aml_set_3d_video_mode(unsigned int mode, bool framepacking_support, int view_mode);
