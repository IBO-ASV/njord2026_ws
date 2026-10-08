#ifndef THRUSTER_DRIVER__ALLOCATION_HPP_
#define THRUSTER_DRIVER__ALLOCATION_HPP_

#include <vector>

namespace njord
{
namespace thruster_driver
{

struct ThrusterGeometry
{
  double x{0.0};
  double y{0.0};
  double angle_rad{0.0};
  double force_per_duty{1.0};
  bool reverse{false};
};

// This is intentionally separate from ThrusterGeometry/allocateWrench:
// underwater thrusters map a body wrench to force commands, while an omni
// wheel maps a body twist to wheel angular velocity and then an open-loop duty
// request.  duty_per_wheel_rad_s is a calibration parameter, not a motor or
// vehicle speed model.
struct OmniWheelGeometry
{
  double x{0.0};
  double y{0.0};
  double drive_angle_rad{0.0};
  double wheel_radius_m{1.0};
  double duty_per_wheel_rad_s{0.0};
  bool reverse{false};
};

std::vector<double> allocateWrench(
  const std::vector<ThrusterGeometry> & thrusters,
  const std::vector<double> & wrench,
  double regularization_lambda);

std::vector<double> commandToWrench(
  const std::vector<ThrusterGeometry> & thrusters,
  const std::vector<double> & commands);

std::vector<double> bodyTwistToWheelDuty(
  const std::vector<OmniWheelGeometry> & wheels,
  double linear_x_mps,
  double linear_y_mps,
  double angular_z_rad_s);

}  // namespace thruster_driver
}  // namespace njord

#endif  // THRUSTER_DRIVER__ALLOCATION_HPP_
