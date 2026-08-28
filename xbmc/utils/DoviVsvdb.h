/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

/*
Dolby Vision VSVDB (EDID Vendor-Specific Video Data Block) layout, CTA-861
extended tag 0x01, IEEE OUI 00-D0-46 (little-endian on the wire: 46 D0 00).
This is the raw block as printed by the kernel's "VSVDB: <hex>" dv_cap line,
starting at the CTA tag/length byte.

Offset | Field             | Notes
-------------------------------------------------------------
  0    | Tag/length byte   | top 3 bits = 7 (extended tag), low 5 bits =
       |                   | declared length of everything from offset 1 on
  1    | Extended tag      | 0x01 (Vendor-Specific Video Data Block)
  2-4  | OUI               | 46 D0 00 (Dolby Laboratories, little-endian)
  5+   | Payload (x[0]...) | version dependent, x[0] = byte at offset 5

x[0] = byte at offset 5. version = (x[0] >> 5) & 7.

Version 0 (declared length 0x19, x[0..20]):
  x[0] bit 0        yuv422_12bit
  x[0] bit 1        sup_2160p60
  x[0] bit 2        global dimming
  x[1..12]          12-bit packed R/G/B/W primaries (value / 4096)
  x[13..15]         12-bit packed min/max PQ codewords
  x[16]             DM version, major = >>4, minor = &0xf

Version 1 (declared length 0x0B compact or 0x0E full primaries):
  x[0] bit 0        yuv422_12bit
  x[0] bit 1        sup_2160p60
  x[0] bits 2-4     DM version minor selector, major = value + 2
  x[1] bit 0        global dimming
  x[1] bits 1-7     max luminance code, nits = 100 + 50*code
  x[2] bit 0        colorimetry, 1 = P3-D65, 0 = BT.709
  x[2] bits 1-7     min luminance code, nits = (code/127)^2
  x[3] bit 0        low latency, compact layout only
  x[3..9] (full) or x[3..6] (compact) primaries

Version 2 (declared length 0x0B, longer declared lengths decode the same
first 7 bytes):
  x[0] bit 0        yuv422_12bit
  x[0] bit 1        backlight control supported
  x[0] bits 2-4     DM version minor selector, major = value + 2
  x[1] bits 0-1     backlight min luminance code, nits = 25 + 25*code
  x[1] bit 2        global dimming
  x[1] bits 3-7     min luminance code, pq = 20*code
  x[2] bits 0-1     interface / LLDV capability bits
  x[2] bits 3-7     max luminance code, pq = 2055 + 65*code
  x[3] bit 0, x[4] bit 0  10b/12b 444 support (2-bit combined value)
  x[3..6]           compact primaries

v0 and v2 min/max nits are the ST 2084 inverse EOTF applied to pq/4095. v1
uses the direct formulas above, no PQ math. Bit layout and nits formulas
verified against edid-decode (utils/edid-decode/parse-cta-block.cpp,
cta_dolby_video/pq2nits).
*/

#pragma once

#include <cstdint>
#include <optional>
#include <string>

enum class DoviVsvdb444Support : uint8_t
{
  NOT_SUPPORTED,
  BIT_10,
  BIT_12,
  RESERVED
};

struct DoviVsvdbPrimaries
{
  double rx{0};
  double ry{0};
  double gx{0};
  double gy{0};
  double bx{0};
  double by{0};
  std::optional<double> wx; // white point, version 0 only
  std::optional<double> wy;
};

struct DoviVsvdbInfo
{
  uint8_t version{0};

  std::optional<uint8_t> dmVersionMajor;
  std::optional<uint8_t> dmVersionMinor; // version 0 only

  std::optional<uint8_t> maxCodeRaw; // versions 1 and 2
  std::optional<uint16_t> maxPqCodeword; // versions 0 and 2
  double maxNits{0};

  std::optional<uint8_t> minCodeRaw; // versions 1 and 2
  std::optional<uint16_t> minPqCodeword; // versions 0 and 2
  double minNits{0};

  std::optional<uint8_t> interfaceBits; // version 2 only
  bool globalDimming{false};
  std::optional<bool> colorimetryP3D65; // version 1 only
  std::optional<bool> lowLatency; // version 1 compact only
  std::optional<bool> sup2160p60; // versions 0 and 1
  bool yuv42212Bit{false};
  std::optional<DoviVsvdb444Support> sup10b12b444; // version 2 only
  std::optional<bool> backlightControl; // version 2 only
  std::optional<uint16_t> backlightMinLumaNits; // version 2 only

  DoviVsvdbPrimaries primaries;
};

class CDoviVsvdb
{
public:
  /*!
   * @brief Decodes a raw Dolby Vision VSVDB block, starting at the CTA
   * tag/length byte.
   * \param data [in] raw block bytes, tag byte onward
   * \param size [in] byte count of data
   * \return decoded info, or std::nullopt if the block fails structural
   * validation (wrong extended tag, wrong OUI, undefined version, a
   * length/version mismatch, or truncation). There is no checksum to
   * validate against.
   */
  static std::optional<DoviVsvdbInfo> Decode(const uint8_t* data, size_t size);

  /*!
   * @brief Decodes a VSVDB block from its contiguous lowercase hex form, the
   * format the kernel prints after "VSVDB: " in dv_cap.
   * \param hex [in] contiguous lowercase hex, tag byte onward
   * \return decoded info, or std::nullopt if `hex` is empty, has odd
   * length, contains a non-hex character, or fails the same structural
   * validation as the byte overload
   */
  static std::optional<DoviVsvdbInfo> Decode(const std::string& hex);

  static std::string ToJson(const DoviVsvdbInfo& info);
};
