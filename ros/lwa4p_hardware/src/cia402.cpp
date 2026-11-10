// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#include "lwa4p_hardware/cia402.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>

namespace lwa4p_hardware
{

std::string StateName(uint16_t statusword)
{
  if ((statusword & 0x004F) == 0x0040) {
    return "switch on disabled";
  }
  if ((statusword & 0x006F) == 0x0021) {
    return "ready to switch on";
  }
  if ((statusword & 0x006F) == 0x0023) {
    return "switched on";
  }
  if ((statusword & 0x006F) == 0x0027) {
    return "operation enabled";
  }
  if (IsFault(statusword)) {
    return "fault";
  }
  return "not ready";
}

std::array<uint8_t, 8> PackRpdo1(uint16_t controlword, int32_t target_mdeg)
{
  const auto target = static_cast<uint32_t>(target_mdeg);
  return {
    static_cast<uint8_t>(controlword),
    static_cast<uint8_t>(controlword >> 8),
    0,
    0,
    static_cast<uint8_t>(target),
    static_cast<uint8_t>(target >> 8),
    static_cast<uint8_t>(target >> 16),
    static_cast<uint8_t>(target >> 24)};
}

Tpdo1 ParseTpdo1(const uint8_t * data)
{
  Tpdo1 out{};
  out.statusword = static_cast<uint16_t>(data[0] | (data[1] << 8));
  out.torque = static_cast<int16_t>(data[2] | (data[3] << 8));
  out.position_mdeg = static_cast<int32_t>(
    data[4] | (data[5] << 8) | (data[6] << 16) | (static_cast<uint32_t>(data[7]) << 24));
  return out;
}

std::string ErrorText(uint16_t code)
{
  char hex[16];
  std::snprintf(hex, sizeof(hex), "0x%04X", code);
  switch (code) {
    case 0x3222:
      return std::string(hex) + " (DC-link undervoltage: motor supply too weak / sagging)";
    case 0x7122:
      return std::string(hex) + " (commutation error: motor power dropped, enable again)";
    case 0x8611:
      return std::string(hex) + " (following error: the joint couldn't keep up)";
    default:
      return hex;
  }
}

}  // namespace lwa4p_hardware
