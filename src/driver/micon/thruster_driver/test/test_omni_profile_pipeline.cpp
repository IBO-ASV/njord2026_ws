#include <gtest/gtest.h>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "micon_driver_fd/serial_writer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "thruster_driver/node.hpp"

using namespace std::chrono_literals;

namespace
{

struct Md10c3Frame
{
  std::array<float, 3> duty{};
  uint8_t flags{0U};
};

std::string readFile(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Unable to read test fixture: " + path);
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

uint32_t readUint32Le(const uint8_t * bytes)
{
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8U) |
         (static_cast<uint32_t>(bytes[2]) << 16U) |
         (static_cast<uint32_t>(bytes[3]) << 24U);
}

float readFloat32Le(const uint8_t * bytes)
{
  const uint32_t bits = readUint32Le(bytes);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::vector<uint8_t> cobsDecode(const uint8_t * encoded, size_t encodedLength)
{
  std::vector<uint8_t> decoded;
  size_t readIndex = 0;
  while (readIndex < encodedLength) {
    const uint8_t code = encoded[readIndex++];
    if (code == 0U) {
      return {};
    }
    const size_t bytesToCopy = static_cast<size_t>(code) - 1U;
    if (bytesToCopy > encodedLength - readIndex) {
      return {};
    }
    for (size_t i = 0; i < bytesToCopy; ++i) {
      decoded.push_back(encoded[readIndex++]);
    }
    if (code != 0xFFU && readIndex < encodedLength) {
      decoded.push_back(0U);
    }
  }
  return decoded;
}

bool decodeMd10c3Frame(const micon_driver_fd::Packet & encoded, Md10c3Frame * frame)
{
  if (encoded.empty() || encoded.back() != 0U) {
    return false;
  }
  const std::vector<uint8_t> raw = cobsDecode(encoded.data(), encoded.size() - 1U);
  if (raw.size() != micon_driver_fd::kOmniMd10c3RawFrameSize ||
    raw[0] != micon_driver_fd::kProtocolVersion ||
    raw[1] != micon_driver_fd::kOmniMd10c3CommandType ||
    raw[4] != micon_driver_fd::kOmniMd10c3PayloadSize)
  {
    return false;
  }
  for (size_t i = 0; i < frame->duty.size(); ++i) {
    frame->duty[i] = readFloat32Le(
      raw.data() + micon_driver_fd::kHeaderSize + i * sizeof(float));
  }
  frame->flags = raw[micon_driver_fd::kHeaderSize + frame->duty.size() * sizeof(float)];
  return true;
}

bool allZero(const Md10c3Frame & frame)
{
  for (const float duty : frame.duty) {
    if (std::fabs(duty) > 1.0e-6F) {
      return false;
    }
  }
  return true;
}

bool isFiniteBoundedNonzero(const Md10c3Frame & frame)
{
  bool nonzero = false;
  for (const float duty : frame.duty) {
    if (!std::isfinite(duty) || std::fabs(duty) > 0.50F + 1.0e-6F) {
      return false;
    }
    nonzero = nonzero || std::fabs(duty) > 1.0e-4F;
  }
  return nonzero;
}

void spinFor(rclcpp::executors::SingleThreadedExecutor & executor,
             std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

bool waitForFrame(
  int masterFd,
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool(const Md10c3Frame &)> & predicate,
  std::chrono::milliseconds timeout,
  Md10c3Frame * matched)
{
  micon_driver_fd::Packet encoded;
  std::array<uint8_t, 256> received{};
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    ssize_t count = 0;
    while ((count = read(masterFd, received.data(), received.size())) > 0) {
      for (ssize_t index = 0; index < count; ++index) {
        const uint8_t byte = received[static_cast<size_t>(index)];
        if (byte == 0U) {
          encoded.push_back(byte);
          Md10c3Frame candidate;
          if (decodeMd10c3Frame(encoded, &candidate) && predicate(candidate)) {
            *matched = candidate;
            return true;
          }
          encoded.clear();
        } else if (encoded.size() < micon_driver_fd::kOmniMd10c3RawFrameSize + 1U) {
          encoded.push_back(byte);
        } else {
          encoded.clear();
        }
      }
    }
    std::this_thread::sleep_for(2ms);
  }
  return false;
}

rclcpp::NodeOptions threeWheelHostOptions(bool confirmed)
{
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "--params-file",
    std::string(THRUSTER_DRIVER_SOURCE_DIR) + "/config/omni_md10c3.yaml",
  });
  options.parameter_overrides(
  {
    rclcpp::Parameter(
      "robot_description",
      readFile(std::string(THRUSTER_DRIVER_SOURCE_DIR) + "/../../../robot/urdf/omni_3wheel.urdf")),
    rclcpp::Parameter("control.use_velocity_feedback", false),
    rclcpp::Parameter("safety.actuator_configuration_confirmed", confirmed),
    rclcpp::Parameter("topics.cmd_vel", "/test_omni_profile_cmd_vel"),
    rclcpp::Parameter("topics.sim_thruster_command", "/test_omni_profile_thruster_command"),
  });
  return options;
}

rclcpp::NodeOptions threeWheelSerialOptions(const std::string & serialPort)
{
  rclcpp::NodeOptions options;
  options.arguments(
  {
    "--ros-args",
    "--params-file",
    std::string(MICON_DRIVER_FD_SOURCE_DIR) + "/config/omni_md10c3.yaml",
  });
  options.parameter_overrides(
  {
    rclcpp::Parameter("serial_port", serialPort),
    rclcpp::Parameter("command_topic", "/test_omni_profile_thruster_command"),
  });
  return options;
}

}  // namespace

TEST(ThreeWheelProfileLock, RefusesUnlockedPhysicalConfiguration)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}

  try {
    auto driver = std::make_shared<njord::thruster_driver::ThrusterDriverNode>(
      threeWheelHostOptions(false));
    (void)driver;
    FAIL() << "The physical-output profile unexpectedly started while locked.";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(
      std::string(error.what()).find("Actuator configuration is intentionally locked"),
      std::string::npos);
  }

  rclcpp::shutdown();
}

TEST(ThreeWheelProfilePipeline, UsesType02AndStopsOnInvalidOrStaleHostCommand)
{
  int masterFd = -1;
  int slaveFd = -1;
  char slaveName[128]{};
  ASSERT_EQ(openpty(&masterFd, &slaveFd, slaveName, nullptr, nullptr), 0);
  close(slaveFd);
  fcntl(masterFd, F_SETFL, fcntl(masterFd, F_GETFL, 0) | O_NONBLOCK);

  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto driver = std::make_shared<njord::thruster_driver::ThrusterDriverNode>(
    threeWheelHostOptions(true));
  auto serialWriter = std::make_shared<micon_driver_fd::SerialWriter>(
    threeWheelSerialOptions(slaveName));
  auto io = std::make_shared<rclcpp::Node>("three_wheel_profile_pipeline_test_io");
  auto commandPublisher = io->create_publisher<geometry_msgs::msg::Twist>(
    "/test_omni_profile_cmd_vel", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(serialWriter);
  executor.add_node(io);
  const auto discoveryDeadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < discoveryDeadline &&
    (commandPublisher->get_subscription_count() == 0U ||
    io->count_subscribers("/test_omni_profile_thruster_command") == 0U))
  {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_GT(commandPublisher->get_subscription_count(), 0U);
  ASSERT_GT(io->count_subscribers("/test_omni_profile_thruster_command"), 0U);

  geometry_msgs::msg::Twist finiteCommand;
  finiteCommand.linear.x = 0.05;
  finiteCommand.linear.y = -0.02;
  finiteCommand.angular.z = 0.05;
  const auto finiteDeadline = std::chrono::steady_clock::now() + 350ms;
  while (std::chrono::steady_clock::now() < finiteDeadline) {
    commandPublisher->publish(finiteCommand);
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  Md10c3Frame finiteFrame;
  ASSERT_TRUE(waitForFrame(
    masterFd, executor, isFiniteBoundedNonzero, 300ms, &finiteFrame));
  EXPECT_EQ(finiteFrame.flags & 0x08U, 0U);

  geometry_msgs::msg::Twist invalidCommand;
  invalidCommand.linear.x = std::numeric_limits<double>::quiet_NaN();
  commandPublisher->publish(invalidCommand);
  Md10c3Frame invalidFrame;
  ASSERT_TRUE(waitForFrame(masterFd, executor, allZero, 300ms, &invalidFrame));

  const auto recoveredDeadline = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < recoveredDeadline) {
    commandPublisher->publish(finiteCommand);
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  Md10c3Frame recoveredFrame;
  ASSERT_TRUE(waitForFrame(
    masterFd, executor, isFiniteBoundedNonzero, 300ms, &recoveredFrame));

  Md10c3Frame watchdogFrame;
  ASSERT_TRUE(waitForFrame(masterFd, executor, allZero, 600ms, &watchdogFrame));

  close(masterFd);
  executor.remove_node(io);
  executor.remove_node(serialWriter);
  executor.remove_node(driver);
  commandPublisher.reset();
  io.reset();
  serialWriter.reset();
  driver.reset();
  rclcpp::shutdown();
}
