/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "DoviVsvdb.h"

#include "utils/JSONVariantWriter.h"
#include "utils/Variant.h"

#include <cctype>
#include <cmath>
#include <vector>

namespace
{
constexpr uint8_t EXTENDED_TAG_TYPE = 0x07;
constexpr uint8_t VSVDB_EXTENDED_TAG = 0x01;
constexpr uint8_t DOLBY_OUI[3] = {0x46, 0xd0, 0x00};
constexpr size_t HEADER_SIZE = 5; // tag byte, extended tag, 3 OUI bytes

constexpr size_t V0_PAYLOAD_LEN = 21;
constexpr size_t V1_COMPACT_PAYLOAD_LEN = 7;
constexpr size_t V1_FULL_PAYLOAD_LEN = 10;
constexpr size_t V2_PAYLOAD_LEN = 7;

// ST 2084 inverse EOTF, pq in [0, 1]. Matches edid-decode's pq2nits and
// dovi_tool.
double Pq2Nits(double pq)
{
  constexpr double m1 = 2610.0 / 16384.0;
  constexpr double m2 = 128.0 * (2523.0 / 4096.0);
  constexpr double c1 = 3424.0 / 4096.0;
  constexpr double c2 = 32.0 * (2413.0 / 4096.0);
  constexpr double c3 = 32.0 * (2392.0 / 4096.0);

  const double e = std::pow(pq, 1.0 / m2);
  double v = e - c1;
  if (v < 0)
    v = 0;
  v /= c2 - c3 * e;
  v = std::pow(v, 1.0 / m1);
  return v * 10000.0;
}

void DecodeV0(const uint8_t* x, DoviVsvdbInfo& info)
{
  info.yuv42212Bit = x[0] & 0x01;
  info.sup2160p60 = static_cast<bool>(x[0] & 0x02);
  info.globalDimming = x[0] & 0x04;

  info.primaries.rx = ((x[1] >> 4) | (x[2] << 4)) / 4096.0;
  info.primaries.ry = ((x[1] & 0x0f) | (x[3] << 4)) / 4096.0;
  info.primaries.gx = ((x[4] >> 4) | (x[5] << 4)) / 4096.0;
  info.primaries.gy = ((x[4] & 0x0f) | (x[6] << 4)) / 4096.0;
  info.primaries.bx = ((x[7] >> 4) | (x[8] << 4)) / 4096.0;
  info.primaries.by = ((x[7] & 0x0f) | (x[9] << 4)) / 4096.0;
  info.primaries.wx = ((x[10] >> 4) | (x[11] << 4)) / 4096.0;
  info.primaries.wy = ((x[10] & 0x0f) | (x[12] << 4)) / 4096.0;

  const auto pqMin = static_cast<uint16_t>((x[14] << 4) | (x[13] >> 4));
  const auto pqMax = static_cast<uint16_t>((x[15] << 4) | (x[13] & 0x0f));
  info.minPqCodeword = pqMin;
  info.maxPqCodeword = pqMax;
  info.minNits = Pq2Nits(pqMin / 4095.0);
  info.maxNits = Pq2Nits(pqMax / 4095.0);

  info.dmVersionMajor = static_cast<uint8_t>(x[16] >> 4);
  info.dmVersionMinor = static_cast<uint8_t>(x[16] & 0x0f);
}

void DecodeV1(const uint8_t* x, size_t payloadLen, DoviVsvdbInfo& info)
{
  info.yuv42212Bit = x[0] & 0x01;
  info.sup2160p60 = static_cast<bool>(x[0] & 0x02);
  info.dmVersionMajor = static_cast<uint8_t>(((x[0] >> 2) & 0x07) + 2);

  info.globalDimming = x[1] & 0x01;
  const uint8_t maxCode = x[1] >> 1;
  info.maxCodeRaw = maxCode;
  info.maxNits = 100.0 + maxCode * 50.0;

  info.colorimetryP3D65 = static_cast<bool>(x[2] & 0x01);
  const uint8_t minCode = x[2] >> 1;
  info.minCodeRaw = minCode;
  const double lm = minCode / 127.0;
  info.minNits = lm * lm;

  if (payloadLen == V1_COMPACT_PAYLOAD_LEN)
    info.lowLatency = static_cast<bool>(x[3] & 0x01);

  if (payloadLen == V1_FULL_PAYLOAD_LEN)
  {
    info.primaries.rx = x[4] / 256.0;
    info.primaries.ry = x[5] / 256.0;
    info.primaries.gx = x[6] / 256.0;
    info.primaries.gy = x[7] / 256.0;
    info.primaries.bx = x[8] / 256.0;
    info.primaries.by = x[9] / 256.0;
    return;
  }

  constexpr double xMinR = 0.625;
  constexpr double xStepR = (0.74609375 - xMinR) / 31.0;
  constexpr double yMinR = 0.25;
  constexpr double yStepR = (0.37109375 - yMinR) / 31.0;
  info.primaries.rx = xMinR + xStepR * (x[6] >> 3);
  info.primaries.ry =
      yMinR + yStepR * (((x[6] & 0x07) << 2) | (x[4] & 0x01) | ((x[5] & 0x01) << 1));

  constexpr double xStepG = 0.49609375 / 127.0;
  constexpr double yMinG = 0.5;
  constexpr double yStepG = (0.99609375 - yMinG) / 127.0;
  info.primaries.gx = xStepG * (x[4] >> 1);
  info.primaries.gy = yMinG + yStepG * (x[5] >> 1);

  constexpr double xMinB = 0.125;
  constexpr double xStepB = (0.15234375 - xMinB) / 7.0;
  constexpr double yMinB = 0.03125;
  constexpr double yStepB = (0.05859375 - yMinB) / 7.0;
  info.primaries.bx = xMinB + xStepB * (x[3] >> 5);
  info.primaries.by = yMinB + yStepB * ((x[3] >> 2) & 0x07);
}

void DecodeV2(const uint8_t* x, DoviVsvdbInfo& info)
{
  info.yuv42212Bit = x[0] & 0x01;
  info.backlightControl = static_cast<bool>(x[0] & 0x02);
  info.dmVersionMajor = static_cast<uint8_t>(((x[0] >> 2) & 0x07) + 2);

  const uint8_t backlightCode = x[1] & 0x03;
  info.backlightMinLumaNits = static_cast<uint16_t>(25 + backlightCode * 25);
  info.globalDimming = x[1] & 0x04;

  const uint8_t minCode = x[1] >> 3;
  info.minCodeRaw = minCode;
  const auto pqMin = static_cast<uint16_t>(20 * minCode);
  info.minPqCodeword = pqMin;
  info.minNits = Pq2Nits(pqMin / 4095.0);

  info.interfaceBits = static_cast<uint8_t>(x[2] & 0x03);

  const uint8_t maxCode = x[2] >> 3;
  info.maxCodeRaw = maxCode;
  const auto pqMax = static_cast<uint16_t>(2055 + 65 * maxCode);
  info.maxPqCodeword = pqMax;
  info.maxNits = Pq2Nits(pqMax / 4095.0);

  const auto support444 = static_cast<uint8_t>(((x[3] & 0x01) << 1) | (x[4] & 0x01));
  info.sup10b12b444 = static_cast<DoviVsvdb444Support>(support444);

  info.primaries.rx = 0.625 + (x[5] >> 3) / 256.0;
  info.primaries.ry = 0.25 + (x[6] >> 3) / 256.0;
  info.primaries.gx = (x[3] >> 1) / 256.0;
  info.primaries.gy = 0.5 + (x[4] >> 1) / 256.0;
  info.primaries.bx = 0.125 + (x[5] & 0x07) / 256.0;
  info.primaries.by = 0.03125 + (x[6] & 0x07) / 256.0;
}

} // unnamed namespace

std::optional<DoviVsvdbInfo> CDoviVsvdb::Decode(const uint8_t* data, size_t size)
{
  if (!data || size < HEADER_SIZE)
    return std::nullopt;

  if ((data[0] >> 5) != EXTENDED_TAG_TYPE)
    return std::nullopt;

  const size_t declaredLen = data[0] & 0x1f;
  if (declaredLen < HEADER_SIZE - 1 || size < declaredLen + 1)
    return std::nullopt;

  if (data[1] != VSVDB_EXTENDED_TAG)
    return std::nullopt;

  if (data[2] != DOLBY_OUI[0] || data[3] != DOLBY_OUI[1] || data[4] != DOLBY_OUI[2])
    return std::nullopt;

  const uint8_t* x = data + HEADER_SIZE;
  const size_t payloadLen = declaredLen - (HEADER_SIZE - 1);

  DoviVsvdbInfo info;
  info.version = (x[0] >> 5) & 0x07;

  switch (info.version)
  {
    case 0:
      if (payloadLen != V0_PAYLOAD_LEN)
        return std::nullopt;
      DecodeV0(x, info);
      break;
    case 1:
      if (payloadLen != V1_COMPACT_PAYLOAD_LEN && payloadLen != V1_FULL_PAYLOAD_LEN)
        return std::nullopt;
      DecodeV1(x, payloadLen, info);
      break;
    case 2:
      if (payloadLen < V2_PAYLOAD_LEN)
        return std::nullopt;
      DecodeV2(x, info);
      break;
    default:
      return std::nullopt;
  }

  return info;
}

std::optional<DoviVsvdbInfo> CDoviVsvdb::Decode(const std::string& hex)
{
  if (hex.empty() || (hex.length() % 2) != 0)
    return std::nullopt;

  std::vector<uint8_t> data;
  data.reserve(hex.length() / 2);
  for (size_t i = 0; i < hex.length(); i += 2)
  {
    if (!isxdigit(static_cast<unsigned char>(hex[i])) ||
        !isxdigit(static_cast<unsigned char>(hex[i + 1])))
      return std::nullopt;
    data.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
  }

  return Decode(data.data(), data.size());
}

std::string CDoviVsvdb::ToJson(const DoviVsvdbInfo& info)
{
  CVariant root(CVariant::VariantTypeObject);

  root["version"] = info.version;
  if (info.dmVersionMajor)
    root["dmVersionMajor"] = *info.dmVersionMajor;
  if (info.dmVersionMinor)
    root["dmVersionMinor"] = *info.dmVersionMinor;

  if (info.maxCodeRaw)
    root["maxCodeRaw"] = *info.maxCodeRaw;
  if (info.maxPqCodeword)
    root["maxPqCodeword"] = *info.maxPqCodeword;
  root["maxNits"] = info.maxNits;

  if (info.minCodeRaw)
    root["minCodeRaw"] = *info.minCodeRaw;
  if (info.minPqCodeword)
    root["minPqCodeword"] = *info.minPqCodeword;
  root["minNits"] = info.minNits;

  if (info.interfaceBits)
    root["interfaceBits"] = *info.interfaceBits;
  root["globalDimming"] = info.globalDimming;
  if (info.colorimetryP3D65)
    root["colorimetryP3D65"] = *info.colorimetryP3D65;
  if (info.lowLatency)
    root["lowLatency"] = *info.lowLatency;
  if (info.sup2160p60)
    root["sup2160p60"] = *info.sup2160p60;
  root["yuv42212Bit"] = info.yuv42212Bit;
  if (info.sup10b12b444)
    root["sup10b12b444"] = static_cast<int>(*info.sup10b12b444);
  if (info.backlightControl)
    root["backlightControl"] = *info.backlightControl;
  if (info.backlightMinLumaNits)
    root["backlightMinLumaNits"] = *info.backlightMinLumaNits;

  CVariant primaries(CVariant::VariantTypeObject);
  primaries["rx"] = info.primaries.rx;
  primaries["ry"] = info.primaries.ry;
  primaries["gx"] = info.primaries.gx;
  primaries["gy"] = info.primaries.gy;
  primaries["bx"] = info.primaries.bx;
  primaries["by"] = info.primaries.by;
  if (info.primaries.wx)
    primaries["wx"] = *info.primaries.wx;
  if (info.primaries.wy)
    primaries["wy"] = *info.primaries.wy;
  root["primaries"] = std::move(primaries);

  std::string json;
  CJSONVariantWriter::Write(root, json, true);
  return json;
}
