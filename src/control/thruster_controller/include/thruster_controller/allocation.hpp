#ifndef THRUSTER_DRIVER__ALLOCATION_HPP_
#define THRUSTER_DRIVER__ALLOCATION_HPP_

#include <vector>

namespace njord
{
namespace thruster_controller
{

struct ThrusterGeometry
{
  double x{0.0};
  double y{0.0};
  double angle_rad{0.0};
  double force_per_duty{1.0};
  bool reverse{false};
};

std::vector<double> allocateWrench(
  const std::vector<ThrusterGeometry> & thrusters,
  const std::vector<double> & wrench,
  double regularization_lambda);

std::vector<double> commandToWrench(
  const std::vector<ThrusterGeometry> & thrusters,
  const std::vector<double> & commands);

}  // namespace thruster_controller
}  // namespace njord

#endif  // THRUSTER_DRIVER__ALLOCATION_HPP_
