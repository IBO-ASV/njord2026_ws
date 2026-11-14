#ifndef THRUSTER_DRIVER__VELOCITY_CONTROL_HPP_
#define THRUSTER_DRIVER__VELOCITY_CONTROL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace njord::thruster_controller
{
// Body-frame surge/sway [m/s], yaw rate [rad/s]; outputs [N,N,Nm].
// Integral gains default to zero, preserving existing hardware P control.
class VelocityControl
{
public:
  using Axes = std::array<double, 3>;
  void reset() {integral_ = {}; reference_ = {};}
  Axes step(const Axes & reference, const Axes & measured, double dt,
    const Axes & kp, const Axes & ki, const Axes & limits, double reference_tau = 0.0)
  {
    if (!std::isfinite(dt) || dt <= 0) throw std::invalid_argument("invalid control dt");
    if (!std::isfinite(reference_tau) || reference_tau < 0) throw std::invalid_argument("invalid reference filter");
    dt = std::min(dt, 0.1);
    Axes out{};
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(reference[i]) || !std::isfinite(measured[i]) ||
        !std::isfinite(kp[i]) || !std::isfinite(ki[i]) || !std::isfinite(limits[i]) ||
        kp[i] < 0 || ki[i] < 0 || limits[i] <= 0)
        throw std::invalid_argument("invalid velocity controller input");
      reference_[i] += (reference_tau > 0 ? 1-std::exp(-dt/reference_tau) : 1.)*(reference[i]-reference_[i]);
      const double error = reference_[i] - measured[i];
      if (ki[i] == 0) integral_[i] = 0;
      const double candidate = std::clamp(integral_[i] + ki[i] * error * dt, -limits[i], limits[i]);
      const double proposed = kp[i] * error + candidate;
      if (std::abs(proposed) <= limits[i] || proposed * error < 0) integral_[i] = candidate;
      out[i] = std::clamp(kp[i] * error + integral_[i], -limits[i], limits[i]);
    }
    return out;
  }
  // Back-calculate saturation introduced by allocation/static actuator mapping.
  void trackApplied(const Axes & requested, const Axes & applied, double dt, const Axes & ki)
  {
    for (size_t i = 0; i < 3; ++i)
      if (ki[i] > 0) integral_[i] += std::clamp(dt, 0.0, 0.1) * (applied[i] - requested[i]);
  }
private:
  Axes integral_{};
  Axes reference_{};
};
}  // namespace njord::thruster_controller
#endif
