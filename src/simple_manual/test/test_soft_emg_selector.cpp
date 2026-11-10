#include "gtest/gtest.h"
#include "simple_manual/soft_emg_selector.hpp"

TEST(SoftEmgSelector, SelectsSbusOnlyWhenButtonZeroIsReleased)
{
  sensor_msgs::msg::Joy joy;
  joy.buttons = {0};
  EXPECT_TRUE(simple_manual::sbus_soft_emg_selected(joy));

  joy.buttons[0] = 1;
  EXPECT_FALSE(simple_manual::sbus_soft_emg_selected(joy));
}
