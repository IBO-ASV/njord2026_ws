#include "gtest/gtest.h"
#include "simple_manual/sbus_joy_converter.hpp"

TEST(SbusJoyConverter, MapsAxesModesAndEmergency)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {0.222F, -0.555F, 0.0F, 0.0F, 0.9F, 0.444F};
  joy.buttons = {0};
  const auto output = simple_manual::convert_sbus_joy(joy, std::vector<double>(6, 0.0));
  EXPECT_DOUBLE_EQ(output.cmd_vel.linear.x, 0.555);
  EXPECT_DOUBLE_EQ(output.cmd_vel.linear.y, -0.222);
  EXPECT_DOUBLE_EQ(output.cmd_vel.angular.z, -0.444);
  EXPECT_TRUE(output.command_enabled);
  EXPECT_TRUE(output.soft_emg);

  joy.buttons[0] = 1;
  const auto disabled = simple_manual::convert_sbus_joy(joy, {});
  EXPECT_FALSE(disabled.command_enabled);
  EXPECT_FALSE(disabled.soft_emg);
  EXPECT_TRUE(disabled.operating_mode.empty());

  joy.buttons[0] = 0;
  joy.axes[4] = -0.8F;
  EXPECT_EQ(simple_manual::convert_sbus_joy(joy, {}).operating_mode, "auto");
  joy.axes[4] = 0.0F;
  EXPECT_EQ(simple_manual::convert_sbus_joy(joy, {}).operating_mode, "manual");

  joy.axes[4] = 0.9F;
  EXPECT_TRUE(simple_manual::convert_sbus_joy(joy, {0.0, 0.0, 0.0, 0.0, 0.9}).soft_emg);
}

TEST(SbusJoyConverter, FloorsAtThreeDecimalPlaces)
{
  EXPECT_DOUBLE_EQ(simple_manual::truncate_3(0.1239), 0.123);
  EXPECT_DOUBLE_EQ(simple_manual::truncate_3(-0.1231), -0.124);
}
