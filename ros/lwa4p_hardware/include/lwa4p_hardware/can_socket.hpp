// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#ifndef LWA4P_HARDWARE__CAN_SOCKET_HPP_
#define LWA4P_HARDWARE__CAN_SOCKET_HPP_

#include <linux/can.h>

#include <cstdint>
#include <string>
#include <vector>

namespace lwa4p_hardware
{

/** @brief Raw SocketCAN socket on one interface; each socket sees its own copy of the bus. */
class CanSocket
{
public:
  /** @brief Open and bind; only frames matching one of `filters` are received (none: all). */
  CanSocket(const std::string & interface, const std::vector<can_filter> & filters);
  ~CanSocket();
  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;

  /** @brief Queue one frame; false if the kernel refused it (TX queue full, bus off). */
  bool Send(uint32_t id, const uint8_t * data, uint8_t length);

  /** @brief Wait up to `timeout_ms` (0: don't wait) for one frame; false if none came. */
  bool Receive(can_frame & frame, int timeout_ms);

private:
  int fd_{-1};
};

}  // namespace lwa4p_hardware

#endif  // LWA4P_HARDWARE__CAN_SOCKET_HPP_
