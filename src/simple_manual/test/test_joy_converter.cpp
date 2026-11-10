#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "simple_manual/joy_converter.hpp"

TEST(JoyConversion, UsesDefaultMapping)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {0.5F, -0.75F};
  joy.buttons.resize(8, 0);
  joy.buttons[0] = 1;
  joy.buttons[2] = 1;
  joy.buttons[6] = 1;

  const auto output = simple_manual::convert_joy(joy, simple_manual::JoyConfig{});
  EXPECT_DOUBLE_EQ(output.cmd_vel.linear.x, -0.15);
  EXPECT_DOUBLE_EQ(output.cmd_vel.linear.y, 0.1);
  EXPECT_DOUBLE_EQ(output.cmd_vel.angular.z, 0.2);
  EXPECT_TRUE(output.emergency);
  EXPECT_FALSE(output.green);
  EXPECT_TRUE(output.yellow);
  EXPECT_FALSE(output.red);
}

TEST(JoyConversion, UsesConfiguredMappingAndSafeMissingInputs)
{
  simple_manual::JoyConfig config;
  config.linear_x_axis = 0;
  config.linear_y_axis = 4;
  config.yaw_positive_button = 1;
  config.yaw_negative_button = 5;
  config.emergency_button = 3;
  config.green_button = 6;
  config.linear_x_scale = -0.4;
  config.angular_z_scale = 0.5;
  sensor_msgs::msg::Joy joy;
  joy.axes = {0.25F};
  joy.buttons = {0, 1};

  const auto output = simple_manual::convert_joy(joy, config);
  EXPECT_DOUBLE_EQ(output.cmd_vel.linear.x, -0.1);
  EXPECT_DOUBLE_EQ(output.cmd_vel.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd_vel.angular.z, 0.5);
  EXPECT_FALSE(output.emergency);
  EXPECT_FALSE(output.green);
  EXPECT_FALSE(output.yellow);
  EXPECT_FALSE(output.red);
}

TEST(JoyConversion, DetectsManualAndAutoModeButtons)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {0.5F, 0.5F};
  joy.buttons.resize(13, 0);

  auto output = simple_manual::convert_joy(joy, simple_manual::JoyConfig{});
  EXPECT_FALSE(output.manual_mode);
  EXPECT_FALSE(output.auto_mode);

  joy.buttons[4] = 1;
  output = simple_manual::convert_joy(joy, simple_manual::JoyConfig{});
  EXPECT_TRUE(output.manual_mode);
  EXPECT_FALSE(output.auto_mode);

  joy.buttons[4] = 0;
  joy.buttons[5] = 1;
  output = simple_manual::convert_joy(joy, simple_manual::JoyConfig{});
  EXPECT_FALSE(output.manual_mode);
  EXPECT_TRUE(output.auto_mode);
}

TEST(JoyConverterParameters, AcceptsValidAndRejectsInvalidValues)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<simple_manual::JoyConverter>();
  auto results = node->set_parameters(
  {
    rclcpp::Parameter("axis.linear_x", 3),
    rclcpp::Parameter("scale.angular_z", -0.7)});
  ASSERT_EQ(results.size(), 2U);
  EXPECT_TRUE(results[0].successful);
  EXPECT_TRUE(results[1].successful);
  EXPECT_EQ(node->get_parameter("axis.linear_x").as_int(), 3);
  EXPECT_DOUBLE_EQ(node->get_parameter("scale.angular_z").as_double(), -0.7);

  results = node->set_parameters({rclcpp::Parameter("button.red", -1)});
  ASSERT_EQ(results.size(), 1U);
  EXPECT_FALSE(results[0].successful);
  EXPECT_EQ(node->get_parameter("button.red").as_int(), 3);
  results = node->set_parameters({rclcpp::Parameter("button.manual_mode", -1)});
  ASSERT_EQ(results.size(), 1U);
  EXPECT_FALSE(results[0].successful);
  EXPECT_EQ(node->get_parameter("button.manual_mode").as_int(), 4);
  rclcpp::shutdown();
}
