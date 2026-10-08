#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "thruster_driver/node.hpp"

using namespace std::chrono_literals;

namespace
{

const char kThreeWheelUrdf[] = R"(
<robot name="omni_test">
  <link name="base_link"/>
  <link name="left_front"/>
  <link name="rear"/>
  <link name="right_front"/>
  <joint name="left_front_joint" type="fixed">
    <parent link="base_link"/><child link="left_front"/><origin xyz="0.15 0.259808 0"/>
  </joint>
  <joint name="rear_joint" type="fixed">
    <parent link="base_link"/><child link="rear"/><origin xyz="-0.3 0 0"/>
  </joint>
  <joint name="right_front_joint" type="fixed">
    <parent link="base_link"/><child link="right_front"/><origin xyz="0.15 -0.259808 0"/>
  </joint>
</robot>)";

void spinFor(rclcpp::executors::SingleThreadedExecutor & executor,
             std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

bool isAllZero(const std::vector<float> & values)
{
  return values.size() == 3U &&
         std::all_of(values.begin(), values.end(), [](float value) {return value == 0.0F;});
}

}  // namespace

TEST(ThrusterDriverNodeSafety, RejectsNonFiniteTwistAndUsesSteadyWatchdog)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}

  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("use_sim_time", true),
    rclcpp::Parameter("input_mode", "cmd_vel"),
    rclcpp::Parameter("actuator_model", "omni_wheel_duty"),
    rclcpp::Parameter("robot_description", std::string(kThreeWheelUrdf)),
    rclcpp::Parameter("input_scaling.max_linear_x", 0.20),
    rclcpp::Parameter("input_scaling.max_linear_y", 0.20),
    rclcpp::Parameter("input_scaling.max_angular_z", 0.30),
    rclcpp::Parameter("control.rate_hz", 100.0),
    rclcpp::Parameter("control.use_velocity_feedback", false),
    rclcpp::Parameter("safety.watchdog_timeout_sec", 0.05),
    rclcpp::Parameter("safety.actuator_configuration_confirmed", true),
    rclcpp::Parameter("output.mode", "duty_ratio"),
    rclcpp::Parameter("output.duty_limit", 0.50),
    rclcpp::Parameter("omni.wheel_radius_m", 0.0635),
    rclcpp::Parameter("omni.duty_per_wheel_rad_s", std::vector<double>{0.05, 0.05, 0.05}),
    rclcpp::Parameter("thrusters.ids", std::vector<std::string>{"LF", "REAR", "RF"}),
    rclcpp::Parameter(
      "thrusters.links", std::vector<std::string>{"left_front", "rear", "right_front"}),
    rclcpp::Parameter(
      "thrusters.angle_rad", std::vector<double>{2.617994, -1.570796, 0.523599}),
    rclcpp::Parameter("thrusters.max_thrust", std::vector<double>{1.0, 1.0, 1.0}),
    rclcpp::Parameter("thrusters.reverse", std::vector<bool>{false, false, false}),
    rclcpp::Parameter(
      "static_map.thrusters.forward_gain", std::vector<double>{1.0, 1.0, 1.0}),
    rclcpp::Parameter(
      "static_map.thrusters.reverse_gain", std::vector<double>{1.0, 1.0, 1.0}),
    rclcpp::Parameter("static_map.thrusters.offset", std::vector<double>{0.0, 0.0, 0.0}),
    rclcpp::Parameter("topics.cmd_vel", "/test_cmd_vel"),
    rclcpp::Parameter("topics.sim_thruster_command", "/test_thruster_command"),
  });
  auto driver = std::make_shared<njord::thruster_driver::ThrusterDriverNode>(options);
  auto io = std::make_shared<rclcpp::Node>("thruster_driver_node_safety_test_io");
  auto cmd_pub = io->create_publisher<geometry_msgs::msg::Twist>("/test_cmd_vel", 10);

  std::vector<float> latest_output;
  auto output_sub = io->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/test_thruster_command", 10,
    [&latest_output](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      latest_output = msg->data;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(io);
  // Let DDS discover both endpoints before sending the first command.
  spinFor(executor, 30ms);

  geometry_msgs::msg::Twist finite_command;
  finite_command.linear.x = 0.05;
  const auto finite_command_deadline = std::chrono::steady_clock::now() + 80ms;
  while (std::chrono::steady_clock::now() < finite_command_deadline) {
    cmd_pub->publish(finite_command);
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(latest_output.size(), 3U);
  EXPECT_TRUE(std::any_of(
    latest_output.begin(), latest_output.end(), [](float value) {return std::fabs(value) > 1e-4F;}));

  geometry_msgs::msg::Twist invalid_command;
  invalid_command.linear.x = std::numeric_limits<double>::quiet_NaN();
  cmd_pub->publish(invalid_command);
  spinFor(executor, 20ms);
  EXPECT_TRUE(isAllZero(latest_output));

  const auto recovered_command_deadline = std::chrono::steady_clock::now() + 30ms;
  while (std::chrono::steady_clock::now() < recovered_command_deadline) {
    cmd_pub->publish(finite_command);
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_TRUE(std::any_of(
    latest_output.begin(), latest_output.end(), [](float value) {return std::fabs(value) > 1e-4F;}));

  // `/clock` is intentionally never published.  If the output watchdog used
  // ROS time, the command would remain alive forever under use_sim_time.
  spinFor(executor, 90ms);
  EXPECT_TRUE(isAllZero(latest_output));

  executor.remove_node(io);
  executor.remove_node(driver);
  output_sub.reset();
  cmd_pub.reset();
  io.reset();
  driver.reset();
  rclcpp::shutdown();
}
