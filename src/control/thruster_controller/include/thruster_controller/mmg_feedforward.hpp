#ifndef THRUSTER_CONTROLLER__MMG_FEEDFORWARD_HPP_
#define THRUSTER_CONTROLLER__MMG_FEEDFORWARD_HPP_

#include <array>
#include <cmath>

namespace njord::thruster_controller
{
// Diagonal terms of the simulator's steady-state MMG inverse.
struct MmgFeedforwardParameters
{
  double Lpp{1.0}, D{0.25}, mass{10.0}, x_g{0.0}, m_x{1.0}, m_y{10.0};
  double R_0{-0.2}, Y_0{0.001}, Y_v{-0.2}, Y_r{0.02};
  double N_0{0.001}, N_v{-0.2}, N_r{-0.02};
};

inline std::array<double, 3> mmgSteadyWrench(
  const std::array<double, 3> & velocity, const MmgFeedforwardParameters & p)
{
  const auto [u, v, r] = velocity;
  const double scale = 0.5 * 1025.0 * p.Lpp * p.D;
  const double surge_speed = std::sqrt(u * u + 1e-6);
  const double sway_speed = std::sqrt(v * v + 1e-6);
  // With no translational velocity the simulator uses U = 0.001 m/s.
  const double yaw_speed = 0.001;
  return {-scale * p.R_0 * u * surge_speed,
    -scale * p.Y_v * v * sway_speed,
    -scale * p.Lpp * p.Lpp * p.N_r * r * yaw_speed};
}
}  // namespace njord::thruster_controller
#endif
