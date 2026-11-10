// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#include "lwa4p_hardware/can_socket.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace lwa4p_hardware
{

CanSocket::CanSocket(const std::string & interface, const std::vector<can_filter> & filters)
{
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw std::runtime_error("CAN socket: " + std::string(strerror(errno)));
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    close(fd_);
    throw std::runtime_error(interface + " not found (PCAN plugged in, interface up?)");
  }
  if (!filters.empty()) {
    setsockopt(
      fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      static_cast<socklen_t>(filters.size() * sizeof(can_filter)));
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(fd_);
    throw std::runtime_error("bind " + interface + ": " + strerror(errno));
  }
}

CanSocket::~CanSocket()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool CanSocket::Send(uint32_t id, const uint8_t * data, uint8_t length)
{
  can_frame frame{};
  frame.can_id = id;
  frame.can_dlc = length;
  if (length > 0) {
    std::memcpy(frame.data, data, length);
  }
  return write(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CanSocket::Receive(can_frame & frame, int timeout_ms)
{
  pollfd pfd{fd_, POLLIN, 0};
  if (poll(&pfd, 1, timeout_ms) <= 0) {
    return false;
  }
  return read(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

}  // namespace lwa4p_hardware
