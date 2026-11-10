#include "gtest/gtest.h"
#include "simple_manual/sbus_joy.hpp"

TEST(SbusJoy, ParsesSketchOutputAndFailsSafe)
{
  simple_manual::SbusFrame frame;
  ASSERT_TRUE(simple_manual::parse_sbus_line(
      "100,1,172,992,1811,1201,172,172,172,172,172,172,172,172,172,172,172,172,0,0,0,0", frame));
  const auto joy = simple_manual::sbus_to_joy(frame, 172, 992, 1811, 1200, 0);
  EXPECT_FLOAT_EQ(joy.axes[0], -1.0F);
  EXPECT_FLOAT_EQ(joy.axes[1], 0.0F);
  EXPECT_FLOAT_EQ(joy.axes[2], 1.0F);
  EXPECT_EQ(joy.buttons[3], 1);

  frame.lost_frame = true;
  const auto safe_joy = simple_manual::sbus_to_joy(frame, 172, 992, 1811, 1200, 0);
  EXPECT_EQ(safe_joy.axes[2], 0.0F);
  EXPECT_EQ(safe_joy.buttons[0], 1);

  frame.lost_frame = false;
  frame.failsafe = true;
  EXPECT_EQ(simple_manual::sbus_to_joy(frame, 172, 992, 1811, 1200, 0).buttons[0], 1);
}
