#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "thruster_driver/allocation.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<njord::thruster_driver::ThrusterGeometry> xConfiguration()
{
  return {
    {0.353553, -0.353553, kPi / 4.0, 1.0, false},
    {0.353553, 0.353553, -kPi / 4.0, 1.0, false},
    {-0.353553, -0.353553, 3.0 * kPi / 4.0, 1.0, false},
    {-0.353553, 0.353553, -3.0 * kPi / 4.0, 1.0, false}};
}

std::vector<njord::thruster_driver::OmniWheelGeometry> threeWheelOmniKinematics()
{
  return {
    {0.150000, 0.259808, 5.0 * kPi / 6.0, 0.0635, 0.05, false},
    {-0.300000, 0.000000, -kPi / 2.0, 0.0635, 0.05, false},
    {0.150000, -0.259808, kPi / 6.0, 0.0635, 0.05, false}};
}

void expectTracksAxis(const std::array<double, 3> & requested)
{
  const auto geometry = xConfiguration();
  const std::vector<double> target(requested.begin(), requested.end());
  const auto commands = njord::thruster_driver::allocateWrench(geometry, target, 1e-9);
  const auto actual = njord::thruster_driver::commandToWrench(geometry, commands);
  EXPECT_NEAR(actual[0], requested[0], 1e-6);
  EXPECT_NEAR(actual[1], requested[1], 1e-6);
  EXPECT_NEAR(actual[2], requested[2], 1e-6);
}
}  // namespace

TEST(ThrusterAllocation, TracksSurgeSwayAndYaw)
{
  expectTracksAxis({0.5, 0.0, 0.0});
  expectTracksAxis({0.0, 0.5, 0.0});
  expectTracksAxis({0.0, 0.0, 0.25});
}

TEST(ThrusterAllocation, PreservesDirectionWhenSaturated)
{
  const auto geometry = xConfiguration();
  const auto commands = njord::thruster_driver::allocateWrench(geometry, {10.0, 0.0, 0.0}, 1e-9);
  const auto wrench = njord::thruster_driver::commandToWrench(geometry, commands);
  EXPECT_LE(*std::max_element(commands.begin(), commands.end()), 1.0);
  EXPECT_GE(*std::min_element(commands.begin(), commands.end()), -1.0);
  EXPECT_GT(wrench[0], 0.0);
  EXPECT_NEAR(wrench[1], 0.0, 1e-9);
  EXPECT_NEAR(wrench[2], 0.0, 1e-9);
}

TEST(ThrusterAllocation, AppliesWiringReverseWithoutChangingPhysicalWrench)
{
  auto geometry = xConfiguration();
  geometry[0].reverse = true;
  const auto commands = njord::thruster_driver::allocateWrench(geometry, {0.4, 0.0, 0.0}, 1e-9);
  EXPECT_LT(commands[0], 0.0);
  const auto wrench = njord::thruster_driver::commandToWrench(geometry, commands);
  EXPECT_NEAR(wrench[0], 0.4, 1e-6);
  EXPECT_NEAR(wrench[1], 0.0, 1e-6);
  EXPECT_NEAR(wrench[2], 0.0, 1e-6);
}

TEST(OmniKinematics, MapsPureAndMixedBodyTwistToSignedWheelDuty)
{
  const auto wheels = threeWheelOmniKinematics();
  const auto surge = njord::thruster_driver::bodyTwistToWheelDuty(wheels, 0.05, 0.0, 0.0);
  EXPECT_LT(surge[0], 0.0);
  EXPECT_NEAR(surge[1], 0.0, 1e-6);
  EXPECT_GT(surge[2], 0.0);

  const auto sway = njord::thruster_driver::bodyTwistToWheelDuty(wheels, 0.0, 0.05, 0.0);
  EXPECT_GT(sway[0], 0.0);
  EXPECT_LT(sway[1], 0.0);
  EXPECT_GT(sway[2], 0.0);

  const auto yaw = njord::thruster_driver::bodyTwistToWheelDuty(wheels, 0.0, 0.0, 0.10);
  EXPECT_GT(yaw[0], 0.0);
  EXPECT_GT(yaw[1], 0.0);
  EXPECT_GT(yaw[2], 0.0);

  const auto mixed = njord::thruster_driver::bodyTwistToWheelDuty(wheels, 0.05, -0.02, 0.10);
  EXPECT_NE(mixed[0], surge[0]);
  EXPECT_NE(mixed[1], surge[1]);
  EXPECT_NE(mixed[2], surge[2]);
}

TEST(OmniKinematics, SaturatesAndAppliesPerWheelReverse)
{
  const auto wheels = threeWheelOmniKinematics();
  const auto saturated = njord::thruster_driver::bodyTwistToWheelDuty(
    wheels, 100.0, 100.0, 100.0);
  EXPECT_LE(*std::max_element(saturated.begin(), saturated.end()), 1.0);
  EXPECT_GE(*std::min_element(saturated.begin(), saturated.end()), -1.0);

  auto reversed = wheels;
  reversed[1].reverse = true;
  const auto yaw = njord::thruster_driver::bodyTwistToWheelDuty(reversed, 0.0, 0.0, 0.10);
  EXPECT_LT(yaw[1], 0.0);
}
