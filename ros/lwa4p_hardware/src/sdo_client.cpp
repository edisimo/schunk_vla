// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#include "lwa4p_hardware/sdo_client.hpp"

#include <linux/can.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "lwa4p_hardware/can_socket.hpp"

namespace lwa4p_hardware
{

namespace
{

constexpr uint32_t SDO_REQUEST = 0x600;
constexpr uint32_t SDO_RESPONSE = 0x580;
constexpr int SDO_TIMEOUT_MS = 300;

std::string Hex(uint32_t value, int width)
{
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "0x%0*X", width, value);
  return buffer;
}

}  // namespace

std::array<uint8_t, 8> SdoUploadRequest(uint16_t index, uint8_t sub)
{
  return {0x40, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), sub, 0, 0, 0, 0};
}

std::array<uint8_t, 8> SdoDownloadRequest(uint16_t index, uint8_t sub, uint32_t value, uint8_t size)
{
  // expedited download, size indicated: n = 4 - size unused bytes in bits 3..2
  const auto command = static_cast<uint8_t>(0x23 | ((4 - size) << 2));
  return {
    command,
    static_cast<uint8_t>(index),
    static_cast<uint8_t>(index >> 8),
    sub,
    static_cast<uint8_t>(value),
    static_cast<uint8_t>(value >> 8),
    static_cast<uint8_t>(value >> 16),
    static_cast<uint8_t>(value >> 24)};
}

bool ParseSdoResponse(const uint8_t * data, uint16_t index, uint8_t sub, uint32_t & value)
{
  const uint16_t answered = static_cast<uint16_t>(data[1] | (data[2] << 8));
  if (answered != index || data[3] != sub) {
    return false;
  }
  value = static_cast<uint32_t>(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
  if (data[0] == 0x80) {
    throw std::runtime_error(
      "SDO abort " + Hex(value, 8) + " on " + Hex(index, 4) + ":" + std::to_string(sub));
  }
  if ((data[0] & 0xE0) == 0x40 && (data[0] & 0x03) == 0x03) {  // expedited upload, size known
    const int unused = (data[0] >> 2) & 0x03;
    value &= unused == 0 ? 0xFFFFFFFFu : (1u << (8 * (4 - unused))) - 1u;
  }
  return true;
}

SdoClient::SdoClient(const std::string & interface)
: socket_(interface, {can_filter{SDO_RESPONSE, 0x780}})
{
}

uint32_t SdoClient::Upload(uint8_t node, uint16_t index, uint8_t sub)
{
  return Transfer(node, SdoUploadRequest(index, sub), index, sub);
}

void SdoClient::Download(uint8_t node, uint16_t index, uint8_t sub, uint32_t value, uint8_t size)
{
  Transfer(node, SdoDownloadRequest(index, sub, value, size), index, sub);
}

uint32_t SdoClient::Transfer(
  uint8_t node, const std::array<uint8_t, 8> & request, uint16_t index, uint8_t sub)
{
  std::lock_guard<std::mutex> lock(mutex_);
  can_frame frame{};
  while (socket_.Receive(frame, 0)) {
  }  // drop late answers to earlier, timed-out requests
  if (!socket_.Send(SDO_REQUEST + node, request.data(), 8)) {
    throw std::runtime_error("SDO request to node " + std::to_string(node) + " not sent");
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(SDO_TIMEOUT_MS);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!socket_.Receive(frame, 10) || frame.can_id != SDO_RESPONSE + node || frame.can_dlc < 8) {
      continue;
    }
    uint32_t value = 0;
    if (ParseSdoResponse(frame.data, index, sub, value)) {
      return value;
    }
  }
  throw std::runtime_error(
    "SDO timeout: node " + std::to_string(node) + " " + Hex(index, 4) + ":" + std::to_string(sub));
}

}  // namespace lwa4p_hardware
