// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#ifndef LWA4P_HARDWARE__CIA402_HPP_
#define LWA4P_HARDWARE__CIA402_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace lwa4p_hardware
{

// CiA 402 controlwords
constexpr uint16_t CW_SHUTDOWN = 0x0006;
constexpr uint16_t CW_SWITCH_ON = 0x0007;
constexpr uint16_t CW_ENABLE = 0x000F;
constexpr uint16_t CW_NEW_SETPOINT = 0x001F;  // profile position: start the move
constexpr uint16_t CW_ENABLE_IP = 0x001F;     // interpolated position: follow the setpoints
constexpr uint16_t CW_FAULT_RESET = 0x0080;

constexpr int8_t MODE_PROFILE_POSITION = 1;
constexpr int8_t MODE_INTERPOLATED_POSITION = 7;

// objects
constexpr uint16_t OD_ERROR_HISTORY = 0x1003;
constexpr uint16_t OD_CONTROLWORD = 0x6040;
constexpr uint16_t OD_STATUSWORD = 0x6041;
constexpr uint16_t OD_MODE = 0x6060;
constexpr uint16_t OD_MODE_DISPLAY = 0x6061;
constexpr uint16_t OD_POSITION = 0x6064;
constexpr uint16_t OD_TARGET = 0x607A;
constexpr uint16_t OD_SW_LIMITS = 0x607D;
constexpr uint16_t OD_PROFILE_VELOCITY = 0x6081;
constexpr uint16_t OD_PROFILE_ACCEL = 0x6083;
constexpr uint16_t OD_IP_DATA = 0x60C1;
constexpr uint16_t OD_IP_PERIOD = 0x60C2;

// COB-IDs (the modules' default mapping: RPDO1 = controlword, torque offset, 0x60C1:01;
// TPDO1 = statusword, torque, position; both synchronous)
constexpr uint32_t COB_SYNC = 0x080;
constexpr uint32_t COB_EMCY = 0x080;
constexpr uint32_t COB_TPDO1 = 0x180;
constexpr uint32_t COB_RPDO1 = 0x200;
constexpr uint32_t COB_NMT = 0x000;

constexpr uint8_t NMT_START = 0x01;

/** @brief Operation enabled (drive powered, brake open). */
inline bool IsEnabled(uint16_t statusword) { return (statusword & 0x006F) == 0x0027; }

/** @brief Fault bit set. */
inline bool IsFault(uint16_t statusword) { return (statusword & 0x0008) != 0; }

/** @brief Interpolated position mode active (statusword bit 12 in mode 7). */
inline bool IsIpActive(uint16_t statusword) { return (statusword & 0x1000) != 0; }

/** @brief Readable CiA 402 state, e.g. "switch on disabled". */
std::string StateName(uint16_t statusword);

/** @brief RPDO1 payload: controlword, zero torque offset, target position in mdeg. */
std::array<uint8_t, 8> PackRpdo1(uint16_t controlword, int32_t target_mdeg);

/** @brief Fields of a TPDO1 payload. */
struct Tpdo1
{
  uint16_t statusword;
  int16_t torque;
  int32_t position_mdeg;
};

/** @brief Decode a TPDO1 payload (8 bytes). */
Tpdo1 ParseTpdo1(const uint8_t * data);

/** @brief Description of a CiA 301 error code seen on these modules, else its hex value. */
std::string ErrorText(uint16_t code);

/** @brief Joint angle (rad) from drive position (mdeg) for a joint with `sign` and `offset`. */
inline double ToRad(int32_t mdeg, double sign, double offset)
{
  return sign * mdeg * 1e-3 * 0.017453292519943295 + offset;
}

/** @brief Drive position (mdeg) for a joint angle (rad). */
inline int32_t ToMdeg(double rad, double sign, double offset)
{
  return static_cast<int32_t>(std::lround((rad - offset) / 0.017453292519943295 * 1e3 / sign));
}

}  // namespace lwa4p_hardware

#endif  // LWA4P_HARDWARE__CIA402_HPP_
