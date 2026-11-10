#include "dutyed_tf_pub_with_disturbance/thruster_wrench.hpp"

#include <cmath>
#include <stdexcept>

namespace njord
{
namespace sim
{

PlanarInput dutiesToPlanarInput(
  const std::vector<double> & duties,
  const std::vector<SimThrusterGeometry> & geometry,
  const T200Model & thrust_model)
{
  if (duties.size() != geometry.size()) {
    throw std::invalid_argument("duty and simulator thruster geometry counts must match");
  }

  PlanarInput input;
  for (std::size_t i = 0; i < duties.size(); ++i) {
    const double force = thrust_model.forceFromDuty(duties[i]);
    const double fx = force * std::cos(geometry[i].angle_rad);
    const double fy = force * std::sin(geometry[i].angle_rad);
    input.surge_force += fx;
    input.sway_force += fy;
    input.yaw_moment += geometry[i].x * fy - geometry[i].y * fx;
  }
  return input;
}

PlanarInput forcesToPlanarInput(
  const std::vector<double> & forces,
  const std::vector<SimThrusterGeometry> & geometry)
{
  if (forces.size() != geometry.size()) {
    throw std::invalid_argument("force and simulator thruster geometry counts must match");
  }

  PlanarInput input;
  for (std::size_t i = 0; i < forces.size(); ++i) {
    const double fx = forces[i] * std::cos(geometry[i].angle_rad);
    const double fy = forces[i] * std::sin(geometry[i].angle_rad);
    input.surge_force += fx;
    input.sway_force += fy;
    input.yaw_moment += geometry[i].x * fy - geometry[i].y * fx;
  }
  return input;
}

}  // namespace sim
}  // namespace njord
