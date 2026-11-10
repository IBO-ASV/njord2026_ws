#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "dutyed_tf_pub_with_disturbance/mmg_doyle_model.hpp"
#include "dutyed_tf_pub_with_disturbance/thruster_wrench.hpp"
#include "thruster_driver/allocation.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<njord::thruster_driver::ThrusterGeometry> driverGeometry()
{
  return {
    {-0.1803, -0.2500, 3.0 * kPi / 4.0, 51.5, false},
    {-0.1803, 0.2500, -3.0 * kPi / 4.0, 51.5, true},
    {0.1803, -0.2500, kPi / 4.0, 51.5, false},
    {0.1803, 0.2500, -kPi / 4.0, 51.5, true}};
}

std::vector<njord::sim::SimThrusterGeometry> simulatorGeometry()
{
  return {
    {-0.1803, -0.2500, 3.0 * kPi / 4.0},
    {-0.1803, 0.2500, -3.0 * kPi / 4.0},
    {0.1803, -0.2500, kPi / 4.0},
    {0.1803, 0.2500, -kPi / 4.0}};
}

std::vector<double> restorePhysicalForces(
  const std::vector<double> & commands,
  const std::vector<njord::thruster_driver::ThrusterGeometry> & geometry)
{
  std::vector<double> forces;
  forces.reserve(commands.size());
  for (std::size_t i = 0; i < commands.size(); ++i) {
    const double wiring_sign = geometry[i].reverse ? -1.0 : 1.0;
    forces.push_back(commands[i] * wiring_sign * geometry[i].force_per_duty);
  }
  return forces;
}

void expectMmgAxisMatchesAllocation(const std::array<double, 3> & requested)
{
  const auto commands = njord::thruster_driver::allocateWrench(
    driverGeometry(), {requested[0], requested[1], requested[2]}, 1e-9);
  const auto input = njord::sim::forcesToPlanarInput(
    restorePhysicalForces(commands, driverGeometry()), simulatorGeometry());

  const std::array<double, 3> actual = {
    input.surge_force, input.sway_force, input.yaw_moment};
  for (std::size_t axis = 0; axis < actual.size(); ++axis) {
    if (requested[axis] == 0.0) {
      EXPECT_NEAR(actual[axis], 0.0, 1e-8);
    } else {
      EXPECT_GT(actual[axis] * requested[axis], 0.0);
    }
  }

  njord::sim::DoyleParams params;
  params.R_0 = 0.0;
  params.Y_0 = 0.0;
  params.N_0 = 0.0;
  const njord::sim::MMGDoyleModel mmg(params);
  const auto accel = mmg.computeAccel({}, input);
  const std::array<double, 3> acceleration = {accel.du, accel.dv, accel.dr};
  for (std::size_t axis = 0; axis < acceleration.size(); ++axis) {
    if (requested[axis] != 0.0) {
      EXPECT_GT(acceleration[axis] * requested[axis], 0.0);
    }
  }
}
}  // namespace

TEST(ThrusterMmgConsistency, PreservesSurgeSwayAndYawAxes)
{
  expectMmgAxisMatchesAllocation({0.5, 0.0, 0.0});
  expectMmgAxisMatchesAllocation({0.0, 0.5, 0.0});
  expectMmgAxisMatchesAllocation({0.0, 0.0, 0.25});
}

TEST(ThrusterMmgConsistency, ForceTopicMatchesDriverAllocation)
{
  const auto geometry = driverGeometry();
  const auto sim_geometry = simulatorGeometry();
  for (const std::array<double, 3> requested : {
    std::array<double, 3>{0.5, 0.0, 0.0},
    std::array<double, 3>{0.0, 0.5, 0.0},
    std::array<double, 3>{0.0, 0.0, 0.25}}) {
    const auto commands = njord::thruster_driver::allocateWrench(
      geometry, {requested[0], requested[1], requested[2]}, 1e-9);
    const auto forces = restorePhysicalForces(commands, geometry);

    const auto driver_wrench = njord::thruster_driver::commandToWrench(geometry, commands);
    const auto sim_wrench = njord::sim::forcesToPlanarInput(forces, sim_geometry);
    EXPECT_NEAR(sim_wrench.surge_force, driver_wrench[0], 1e-9);
    EXPECT_NEAR(sim_wrench.sway_force, driver_wrench[1], 1e-9);
    EXPECT_NEAR(sim_wrench.yaw_moment, driver_wrench[2], 1e-9);
  }
}
