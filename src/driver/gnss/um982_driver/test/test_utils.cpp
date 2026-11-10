#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "um982_driver/utils.hpp"

namespace
{

// UNIHEADINGB message ID 972 captured from the actual UM982.  The frame
// layout and CRC cover Header (24) + Data (44), followed by little-endian
// CRC32, as specified by the N4 Reference Commands Manual R1.4.
constexpr std::array<uint8_t, 72> kCapturedUniheadingFrame = {
    0xaa, 0x44, 0xb5, 0x62, 0xcc, 0x03, 0x2c, 0x00,
    0x00, 0xc9, 0x01, 0x00, 0x50, 0x69, 0x0f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x02, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x40, 0xf8, 0x22, 0x10,
};

TEST(UnicoreBinaryProtocol, CapturedUniheadingFrameHasValidCrc)
{
    constexpr std::size_t kCrcSize = 4;
    const auto calculated = um982_driver::utils::calculate_unicore_crc32(
        kCapturedUniheadingFrame.data(), kCapturedUniheadingFrame.size() - kCrcSize);
    const auto received = um982_driver::utils::read_u32_le(
        kCapturedUniheadingFrame.data() + kCapturedUniheadingFrame.size() - kCrcSize);

    EXPECT_EQ(um982_driver::utils::read_u16_le(kCapturedUniheadingFrame.data() + 4), 972U);
    EXPECT_EQ(um982_driver::utils::read_u16_le(kCapturedUniheadingFrame.data() + 6), 44U);
    EXPECT_EQ(calculated, 0x1022f840U);
    EXPECT_EQ(calculated, received);
}

TEST(NumericParsing, AcceptsOnlyCompleteFiniteValues)
{
    double value = 0.0;
    EXPECT_TRUE(um982_driver::utils::parse_finite_double("-12.5", value));
    EXPECT_DOUBLE_EQ(value, -12.5);
    EXPECT_FALSE(um982_driver::utils::parse_finite_double("nan", value));
    EXPECT_FALSE(um982_driver::utils::parse_finite_double("inf", value));
    EXPECT_FALSE(um982_driver::utils::parse_finite_double("12.5junk", value));
    EXPECT_FALSE(um982_driver::utils::parse_finite_double("", value));

    int integer = 0;
    EXPECT_TRUE(um982_driver::utils::parse_int("4", integer));
    EXPECT_EQ(integer, 4);
    EXPECT_FALSE(um982_driver::utils::parse_int("4.0", integer));
    EXPECT_FALSE(um982_driver::utils::parse_int("invalid", integer));
}

TEST(HeadingMounting, ConvertsPrimaryToSecondaryBaselineToVesselYaw)
{
    constexpr double kBaselineYawRad = -0.6747409422235526;
    const double reported_baseline_yaw = um982_driver::utils::deg2rad(20.0);
    const double vessel_yaw = um982_driver::utils::normalize_angle_rad(
        reported_baseline_yaw - kBaselineYawRad);

    EXPECT_NEAR(vessel_yaw, um982_driver::utils::deg2rad(58.6598082541), 1e-10);
    EXPECT_NEAR(
        um982_driver::utils::normalize_angle_rad(4.0 * M_PI), 0.0, 1e-12);
}

TEST(NmeaParsing, ChecksumRequiresExactlyTwoHexDigits)
{
    EXPECT_TRUE(um982_driver::utils::validate_checksum("$GPGGA*56"));
    EXPECT_FALSE(um982_driver::utils::validate_checksum("$GPGGA*56junk"));
    EXPECT_FALSE(um982_driver::utils::validate_checksum("$GPGGA*5"));
    EXPECT_FALSE(um982_driver::utils::validate_checksum("$GPGGA*ZZ"));
}

TEST(NmeaParsing, ExtractsGstPositionStandardDeviations)
{
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    EXPECT_TRUE(um982_driver::utils::parse_gst_standard_deviations(
        "$GNGST,060458.00,0.71,1.62,1.44,9.1113,1.618,1.441,3.761*42",
        latitude, longitude, altitude));
    EXPECT_DOUBLE_EQ(latitude, 1.618);
    EXPECT_DOUBLE_EQ(longitude, 1.441);
    EXPECT_DOUBLE_EQ(altitude, 3.761);
}

TEST(NmeaParsing, RejectsNonFiniteMalformedAndOutOfRangeCoordinates)
{
    double coordinate = 0.0;
    EXPECT_TRUE(um982_driver::utils::convert_nmea_to_latlon("3541.6053", "N", coordinate));
    EXPECT_NEAR(coordinate, 35.6934216667, 1e-10);
    EXPECT_TRUE(um982_driver::utils::convert_nmea_to_latlon("13941.1234", "W", coordinate));
    EXPECT_LT(coordinate, 0.0);
    EXPECT_FALSE(um982_driver::utils::convert_nmea_to_latlon("nan", "N", coordinate));
    EXPECT_FALSE(um982_driver::utils::convert_nmea_to_latlon("3561.0", "N", coordinate));
    EXPECT_FALSE(um982_driver::utils::convert_nmea_to_latlon("18100.0", "E", coordinate));
    EXPECT_FALSE(um982_driver::utils::convert_nmea_to_latlon("3541.0", "X", coordinate));
}

TEST(NmeaParsing, ParsesUtcTimeForGgaGstMatching)
{
    double seconds = 0.0;
    EXPECT_TRUE(um982_driver::utils::parse_nmea_utc_seconds("123519.25", seconds));
    EXPECT_DOUBLE_EQ(seconds, 12 * 3600 + 35 * 60 + 19.25);
    EXPECT_FALSE(um982_driver::utils::parse_nmea_utc_seconds("246000", seconds));
    EXPECT_FALSE(um982_driver::utils::parse_nmea_utc_seconds("123460", seconds));
}

}  // namespace
