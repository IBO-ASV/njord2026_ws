#include <gtest/gtest.h>
#include "robot/se2_tracking.hpp"

using robot::State;
using robot::Command;
TEST(SE2Tracking, BodyFrameAndYawWrap)
{
  robot::TrackingLimits limits;
  limits.acceleration = limits.yaw_acceleration = 100;
  const auto cmd = robot::track({0, 0, M_PI/2}, {1, 0, M_PI/2}, {}, {}, 1, limits);
  EXPECT_NEAR(cmd.x, 0, 1e-8);
  EXPECT_NEAR(cmd.y, -limits.speed, 1e-8);
  const auto turn = robot::track({0, 0, M_PI-0.01}, {0, 0, -M_PI+0.01}, {}, {}, 1, limits);
  EXPECT_GT(turn.yaw, 0);
  EXPECT_NEAR(turn.yaw, 0.02*limits.yaw_gain, 1e-8);
}
TEST(SE2Tracking, AccelerationAndSpeedLimits)
{
  robot::TrackingLimits limits;
  auto cmd = robot::track({}, {100, 100, 2}, {}, {}, 0.02, limits);
  EXPECT_LE(std::hypot(cmd.x, cmd.y), limits.acceleration*0.02+1e-10);
  EXPECT_LE(std::abs(cmd.yaw), limits.yaw_acceleration*0.02+1e-10);
  for (int i=0; i<500; ++i) {
    cmd = robot::track({}, {100, 100, 2}, {}, cmd, 0.02, limits);
    EXPECT_LE(std::hypot(cmd.x, cmd.y), limits.speed+1e-10);
    EXPECT_LE(std::abs(cmd.yaw), limits.yaw_rate+1e-10);
  }
}
TEST(SE2Tracking, TerminalPoseConvergesFromDisturbance)
{
  robot::TrackingLimits limits;
  State vessel{0.4, -0.3, 3.0};
  const State goal{0, 0, -3.0};
  Command cmd;
  for (int i=0; i<1500; ++i) {
    cmd = robot::track(vessel, goal, {}, cmd, 0.02, limits);
    vessel.x += 0.02*(std::cos(vessel.yaw)*cmd.x-std::sin(vessel.yaw)*cmd.y);
    vessel.y += 0.02*(std::sin(vessel.yaw)*cmd.x+std::cos(vessel.yaw)*cmd.y);
    vessel.yaw += 0.02*cmd.yaw;
  }
  EXPECT_LT(std::hypot(vessel.x, vessel.y), 0.001);
  EXPECT_LT(std::abs(robot::angle(vessel.yaw-goal.yaw)), 0.001);
}
