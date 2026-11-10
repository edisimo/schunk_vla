// Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <stdexcept>

#include "lwa4p_hardware/cia402.hpp"
#include "lwa4p_hardware/sdo_client.hpp"

namespace lwa4p_hardware
{

TEST(Sdo, UploadRequest)
{
  const std::array<uint8_t, 8> expected{0x40, 0x7D, 0x60, 0x02, 0, 0, 0, 0};
  EXPECT_EQ(SdoUploadRequest(0x607D, 2), expected);
}

TEST(Sdo, DownloadRequestSizes)
{
  EXPECT_EQ(SdoDownloadRequest(0x6060, 0, 7, 1)[0], 0x2F);
  EXPECT_EQ(SdoDownloadRequest(0x6040, 0, 0x1F, 2)[0], 0x2B);
  const auto four = SdoDownloadRequest(0x60C1, 1, 0xFFFFFC18, 4);  // -1000 mdeg
  const std::array<uint8_t, 8> expected{0x23, 0xC1, 0x60, 0x01, 0x18, 0xFC, 0xFF, 0xFF};
  EXPECT_EQ(four, expected);
}

TEST(Sdo, ParseExpeditedUploads)
{
  uint32_t value = 0;
  const uint8_t one[8] = {0x4F, 0x61, 0x60, 0x00, 0x07, 0xAA, 0xBB, 0xCC};
  ASSERT_TRUE(ParseSdoResponse(one, 0x6061, 0, value));
  EXPECT_EQ(value, 7u);
  const uint8_t two[8] = {0x4B, 0x41, 0x60, 0x00, 0x37, 0x12, 0xAA, 0xBB};
  ASSERT_TRUE(ParseSdoResponse(two, 0x6041, 0, value));
  EXPECT_EQ(value, 0x1237u);
  const uint8_t four[8] = {0x43, 0x64, 0x60, 0x00, 0x18, 0xFC, 0xFF, 0xFF};
  ASSERT_TRUE(ParseSdoResponse(four, 0x6064, 0, value));
  EXPECT_EQ(static_cast<int32_t>(value), -1000);
}

TEST(Sdo, OtherObjectIsIgnoredAbortThrows)
{
  uint32_t value = 0;
  const uint8_t other[8] = {0x4B, 0x41, 0x60, 0x00, 0x37, 0x12, 0, 0};
  EXPECT_FALSE(ParseSdoResponse(other, 0x6064, 0, value));
  const uint8_t abort[8] = {0x80, 0x84, 0x60, 0x00, 0x00, 0x00, 0x02, 0x06};
  EXPECT_THROW(ParseSdoResponse(abort, 0x6084, 0, value), std::runtime_error);
}

TEST(Pdo, Rpdo1Layout)
{
  const std::array<uint8_t, 8> expected{0x1F, 0x00, 0x00, 0x00, 0x18, 0xFC, 0xFF, 0xFF};
  EXPECT_EQ(PackRpdo1(CW_ENABLE_IP, -1000), expected);
}

TEST(Pdo, Tpdo1Decoding)
{
  const uint8_t data[8] = {0x37, 0x12, 0xFE, 0xFF, 0xA9, 0x1A, 0x01, 0x00};  // 72361 mdeg
  const Tpdo1 tpdo = ParseTpdo1(data);
  EXPECT_EQ(tpdo.statusword, 0x1237);
  EXPECT_EQ(tpdo.torque, -2);
  EXPECT_EQ(tpdo.position_mdeg, 72361);
  EXPECT_TRUE(IsEnabled(tpdo.statusword));
  EXPECT_TRUE(IsIpActive(tpdo.statusword));
  EXPECT_FALSE(IsFault(tpdo.statusword));
}

TEST(Units, RoundTripWithSignAndOffset)
{
  for (const int32_t mdeg : {0, 1, -1, 72361, -170000}) {
    EXPECT_EQ(ToMdeg(ToRad(mdeg, -1.0, 0.2), -1.0, 0.2), mdeg);
  }
  EXPECT_NEAR(ToRad(90000, 1.0, 0.0), 1.5707963, 1e-6);
}

TEST(State, Names)
{
  EXPECT_EQ(StateName(0x0250), "switch on disabled");
  EXPECT_EQ(StateName(0x0231), "ready to switch on");
  EXPECT_EQ(StateName(0x1237), "operation enabled");
  EXPECT_EQ(StateName(0x0218), "fault");
}

}  // namespace lwa4p_hardware
