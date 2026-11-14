#include "gtest/gtest.h"
#include "emergency_stop_selector/soft_emg_selector.hpp"

TEST(SoftEmgSelector, SelectsSbusOnlyWhenButtonZeroIsReleased)
{
  sensor_msgs::msg::Joy joy;
  joy.buttons = {0};
  EXPECT_TRUE(emergency_stop_selector::sbus_soft_emg_selected(joy));

  joy.buttons[0] = 1;
  EXPECT_FALSE(emergency_stop_selector::sbus_soft_emg_selected(joy));
}
