#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "thruster_controller/allocation.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<njord::thruster_controller::ThrusterGeometry> xConfiguration()
{
  return {
    {0.353553, -0.353553, kPi / 4.0, 1.0, false},
    {0.353553, 0.353553, -kPi / 4.0, 1.0, false},
    {-0.353553, -0.353553, 3.0 * kPi / 4.0, 1.0, false},
    {-0.353553, 0.353553, -3.0 * kPi / 4.0, 1.0, false}};
}

void expectTracksAxis(const std::array<double, 3> & requested)
{
  const auto geometry = xConfiguration();
  const std::vector<double> target(requested.begin(), requested.end());
  const auto commands = njord::thruster_controller::allocateWrench(geometry, target, 1e-9);
  const auto actual = njord::thruster_controller::commandToWrench(geometry, commands);
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
  const auto commands = njord::thruster_controller::allocateWrench(geometry, {10.0, 0.0, 0.0}, 1e-9);
  const auto wrench = njord::thruster_controller::commandToWrench(geometry, commands);
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
  const auto commands = njord::thruster_controller::allocateWrench(geometry, {0.4, 0.0, 0.0}, 1e-9);
  EXPECT_LT(commands[0], 0.0);
  const auto wrench = njord::thruster_controller::commandToWrench(geometry, commands);
  EXPECT_NEAR(wrench[0], 0.4, 1e-6);
  EXPECT_NEAR(wrench[1], 0.0, 1e-6);
  EXPECT_NEAR(wrench[2], 0.0, 1e-6);
}
