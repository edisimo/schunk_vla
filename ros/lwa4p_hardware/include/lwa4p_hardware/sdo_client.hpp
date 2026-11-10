// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#ifndef LWA4P_HARDWARE__SDO_CLIENT_HPP_
#define LWA4P_HARDWARE__SDO_CLIENT_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <string>

#include "lwa4p_hardware/can_socket.hpp"

namespace lwa4p_hardware
{

/** @brief Request frame for an expedited SDO upload (read) of index:sub. */
std::array<uint8_t, 8> SdoUploadRequest(uint16_t index, uint8_t sub);

/** @brief Request frame for an expedited SDO download (write) of 1, 2 or 4 bytes. */
std::array<uint8_t, 8> SdoDownloadRequest(
  uint16_t index, uint8_t sub, uint32_t value, uint8_t size);

/**
 * @brief Value of an expedited SDO response to index:sub.
 * @return false if the frame answers another object; throws on an SDO abort.
 */
bool ParseSdoResponse(const uint8_t * data, uint16_t index, uint8_t sub, uint32_t & value);

/** @brief Expedited SDO client (objects up to 4 bytes), for configuration and diagnostics. */
class SdoClient
{
public:
  explicit SdoClient(const std::string & interface);

  /** @brief Read index:sub of `node`; throws on abort or timeout. */
  uint32_t Upload(uint8_t node, uint16_t index, uint8_t sub);

  /** @brief Write `size` (1, 2 or 4) bytes of `value` to index:sub of `node`. */
  void Download(uint8_t node, uint16_t index, uint8_t sub, uint32_t value, uint8_t size);

private:
  uint32_t Transfer(
    uint8_t node, const std::array<uint8_t, 8> & request, uint16_t index, uint8_t sub);

  CanSocket socket_;
  std::mutex mutex_;
};

}  // namespace lwa4p_hardware

#endif  // LWA4P_HARDWARE__SDO_CLIENT_HPP_
