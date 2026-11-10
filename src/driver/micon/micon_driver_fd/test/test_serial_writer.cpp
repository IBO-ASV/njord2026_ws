#include <algorithm>
#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "micon_driver_fd/bms_serial_reader.hpp"
#include "micon_driver_fd/serial_writer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

namespace
{

uint16_t read_uint16_le(const uint8_t * bytes)
{
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8U);
}

float read_float32_le(const uint8_t * bytes)
{
  const uint32_t bits =
    static_cast<uint32_t>(bytes[0]) |
    (static_cast<uint32_t>(bytes[1]) << 8U) |
    (static_cast<uint32_t>(bytes[2]) << 16U) |
    (static_cast<uint32_t>(bytes[3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

uint16_t crc16_ccitt_false(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<uint8_t> cobs_decode(const uint8_t * encoded, size_t encoded_length)
{
  std::vector<uint8_t> decoded;
  size_t read_index = 0;
  while (read_index < encoded_length) {
    const uint8_t code = encoded[read_index++];
    if (code == 0) {
      return {};
    }
    const size_t bytes_to_copy = static_cast<size_t>(code) - 1U;
    if (bytes_to_copy > encoded_length - read_index) {
      return {};
    }
    for (size_t i = 0; i < bytes_to_copy; ++i) {
      decoded.push_back(encoded[read_index++]);
    }
    if (code != 0xFFU && read_index < encoded_length) {
      decoded.push_back(0);
    }
  }
  return decoded;
}

void expect_thruster_command_frame(
  const micon_driver_fd::Packet & packet,
  const std::array<float, 4> & thrust,
  uint8_t expected_flags,
  uint16_t expected_sequence,
  bool check_sequence = true)
{
  ASSERT_FALSE(packet.empty());
  EXPECT_EQ(packet.back(), 0);

  const std::vector<uint8_t> raw = cobs_decode(packet.data(), packet.size() - 1U);
  ASSERT_EQ(raw.size(), micon_driver_fd::kRawFrameSize);
  EXPECT_EQ(raw[0], micon_driver_fd::kProtocolVersion);
  EXPECT_EQ(raw[1], micon_driver_fd::kThrusterCommandType);
  if (check_sequence) {
    EXPECT_EQ(read_uint16_le(raw.data() + 2), expected_sequence);
  }
  EXPECT_EQ(raw[4], micon_driver_fd::kPayloadSize);

  for (size_t i = 0; i < thrust.size(); ++i) {
    EXPECT_FLOAT_EQ(
      read_float32_le(raw.data() + micon_driver_fd::kHeaderSize + i * sizeof(float)),
      thrust[i]);
  }
  EXPECT_EQ(raw[micon_driver_fd::kHeaderSize + 4U * sizeof(float)], expected_flags);

  const uint16_t received_crc =
    read_uint16_le(raw.data() + raw.size() - micon_driver_fd::kCrcSize);
  const uint16_t calculated_crc =
    crc16_ccitt_false(raw.data(), raw.size() - micon_driver_fd::kCrcSize);
  EXPECT_EQ(received_crc, calculated_crc);
}

}  // namespace

TEST(SerialPacket, EncodesFloatsAndFlags)
{
  const std::array<float, 4> thrust{{1.0F, -0.5F, 0.25F, 0.0F}};
  micon_driver_fd::Flags flags;
  flags.emergency = true;
  flags.green = true;
  flags.red = true;
  const auto packet = micon_driver_fd::encode_packet(thrust, flags, 42);
  expect_thruster_command_frame(packet, thrust, 0x0D, 42);
}

TEST(BmsCsv, ParsesCellVoltagesAndTemperature)
{
  micon_driver_fd::BmsTelemetry telemetry;
  ASSERT_TRUE(
    micon_driver_fd::parse_bms_csv_line(
      "1234,4.1010,4.0870,4.0930,4.0990,16.3800,25.5,41,38,44,40,OK", &telemetry));
  EXPECT_FLOAT_EQ(telemetry.cells[0], 4.1010F);
  EXPECT_FLOAT_EQ(telemetry.cells[1], 4.0870F);
  EXPECT_FLOAT_EQ(telemetry.cells[2], 4.0930F);
  EXPECT_FLOAT_EQ(telemetry.cells[3], 4.0990F);
  EXPECT_FLOAT_EQ(telemetry.temperature_c, 25.5F);
}

TEST(BmsCsv, RejectsHeaderAndPreservesUnavailableCellVoltage)
{
  micon_driver_fd::BmsTelemetry telemetry;
  EXPECT_FALSE(
    micon_driver_fd::parse_bms_csv_line(
      "ms,cell1_V,cell2_V,cell3_V,cell4_V,total_V,temperature_C,age1_ms,age2_ms,age3_ms,age4_ms,status",
      &telemetry));
  ASSERT_TRUE(
    micon_driver_fd::parse_bms_csv_line(
      "1234,4.1010,nan,4.0930,4.0990,nan,nan,41,38,44,40,STALE", &telemetry));
  EXPECT_FLOAT_EQ(telemetry.cells[0], 4.1010F);
  EXPECT_TRUE(std::isnan(telemetry.cells[1]));
  EXPECT_FLOAT_EQ(telemetry.cells[2], 4.0930F);
  EXPECT_FLOAT_EQ(telemetry.cells[3], 4.0990F);
  EXPECT_TRUE(std::isnan(telemetry.temperature_c));
}

TEST(SerialWriterIntegration, WritesRosInputsToPseudoTerminal)
{
  int master_fd = -1;
  int slave_fd = -1;
  char slave_name[128]{};
  ASSERT_EQ(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr), 0);
  close(slave_fd);
  fcntl(master_fd, F_SETFL, fcntl(master_fd, F_GETFL, 0) | O_NONBLOCK);

  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("serial_port", std::string(slave_name)),
    rclcpp::Parameter("ground_station_heartbeat_timeout_sec", 0.05),
  });
  auto writer = std::make_shared<micon_driver_fd::SerialWriter>(options);
  auto publisher_node = std::make_shared<rclcpp::Node>("serial_writer_test_publisher");
  auto thrust_pub = publisher_node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/thruster_command", 10);
  auto soft_emg_pub = publisher_node->create_publisher<std_msgs::msg::Bool>("/soft_emg", 10);
  auto sbus_pub = publisher_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_sbus", 10);
  auto green_pub = publisher_node->create_publisher<std_msgs::msg::Bool>("/green", 10);
  auto red_pub = publisher_node->create_publisher<std_msgs::msg::Bool>("/red", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(writer);
  executor.add_node(publisher_node);
  std_msgs::msg::Float32MultiArray thrust;
  thrust.data = {0.1F, -0.2F, 0.3F, -0.4F};
  const auto receive_deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < receive_deadline) {
    thrust_pub->publish(thrust);
    soft_emg_pub->publish(std_msgs::msg::Bool().set__data(false));
    sbus_pub->publish(geometry_msgs::msg::Twist{});
    green_pub->publish(std_msgs::msg::Bool().set__data(true));
    red_pub->publish(std_msgs::msg::Bool().set__data(true));
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  std::vector<uint8_t> received(1024);
  const ssize_t count = read(master_fd, received.data(), received.size());
  ASSERT_GT(count, 0);
  received.resize(static_cast<size_t>(count));
  const auto last_delimiter = std::find(received.rbegin(), received.rend(), 0);
  ASSERT_NE(last_delimiter, received.rend());
  const size_t last_delimiter_index =
    received.size() - 1U - static_cast<size_t>(last_delimiter - received.rbegin());
  const auto previous_delimiter = std::find(
    received.rbegin() + static_cast<std::ptrdiff_t>(received.size() - last_delimiter_index),
    received.rend(), 0);
  const size_t frame_begin = previous_delimiter == received.rend() ?
    0U :
    received.size() - static_cast<size_t>(previous_delimiter - received.rbegin());
  const micon_driver_fd::Packet packet(
    received.begin() + static_cast<std::ptrdiff_t>(frame_begin),
    received.begin() + static_cast<std::ptrdiff_t>(last_delimiter_index + 1U));
  expect_thruster_command_frame(packet, {0.1F, -0.2F, 0.3F, -0.4F}, 0x05, 0, false);

  close(master_fd);
  executor.remove_node(publisher_node);
  executor.remove_node(writer);
  writer.reset();
  publisher_node.reset();
  rclcpp::shutdown();
}
