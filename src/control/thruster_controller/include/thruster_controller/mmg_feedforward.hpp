#ifndef THRUSTER_CONTROLLER__MMG_FEEDFORWARD_HPP_
#define THRUSTER_CONTROLLER__MMG_FEEDFORWARD_HPP_

#include <array>
#include <cmath>

namespace njord::thruster_controller
{
// Inverse of the simulator's MMG equations at constant body velocity.
struct MmgFeedforwardParameters
{
  double Lpp{1.0}, D{0.25}, mass{10.0}, I_zG{1.5}, x_g{0.0}, m_x{1.0}, m_y{10.0}, J_z{1.5};
  double R_0{-0.2}, Y_0{0.001}, Y_v{-0.2}, Y_r{0.02};
  double N_0{0.001}, N_v{-0.2}, N_r{-0.02};
};

inline std::array<double, 3> mmgSteadyWrench(
  const std::array<double, 3> & velocity, const MmgFeedforwardParameters & p)
{
  const auto [u, v, r] = velocity;
  const double speed = std::sqrt(u * u + v * v + 1e-6);
  const double scale = 0.5 * 1025.0 * p.Lpp * p.D * speed * speed;
  const double x = scale * (u / speed) * p.R_0;
  const double y = scale * (p.Y_0 + p.Y_v * v / speed + p.Y_r * r * p.Lpp / speed);
  const double n = scale * p.Lpp * (p.N_0 + p.N_v * v / speed + p.N_r * r * p.Lpp / speed);
  return {-x - (p.mass + p.m_y) * v * r - p.x_g * p.mass * r * r,
    -y + (p.mass + p.m_x) * u * r, -n + p.x_g * p.mass * u * r};
}
}  // namespace njord::thruster_controller
#endif
