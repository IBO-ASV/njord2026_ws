#ifndef THRUSTER_CONTROLLER__VELOCITY_CONTROL_HPP_
#define THRUSTER_CONTROLLER__VELOCITY_CONTROL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace njord::thruster_controller
{
// Body-frame velocity controller. Outputs surge/sway force and yaw moment.
class VelocityControl
{
public:
  using Axes = std::array<double, 3>;

  void reset()
  {
    integral_ = {};
    reference_ = {};
    previous_measurement_ = {};
    derivative_filtered_ = {};
    have_previous_measurement_ = false;
  }

  Axes step(
    const Axes & reference, const Axes & measured, double dt,
    const Axes & kp, const Axes & ki, const Axes & limits,
    double reference_tau = 0.0, const Axes & feedforward = {},
    const Axes & kd = {}, double derivative_tau = 0.0)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("invalid control dt");
    }
    if (!std::isfinite(reference_tau) || reference_tau < 0.0 ||
      !std::isfinite(derivative_tau) || derivative_tau < 0.0)
    {
      throw std::invalid_argument("invalid velocity filter");
    }
    dt = std::min(dt, 0.1);
    Axes out{};
    const double reference_alpha = reference_tau > 0.0 ?
      1.0 - std::exp(-dt / reference_tau) : 1.0;
    const double derivative_alpha = derivative_tau > 0.0 ?
      1.0 - std::exp(-dt / derivative_tau) : 1.0;

    for (std::size_t i = 0; i < 3U; ++i) {
      if (!std::isfinite(reference[i]) || !std::isfinite(measured[i]) ||
        !std::isfinite(kp[i]) || !std::isfinite(ki[i]) ||
        !std::isfinite(feedforward[i]) || !std::isfinite(kd[i]) ||
        !std::isfinite(limits[i]) || kp[i] < 0.0 || ki[i] < 0.0 ||
        feedforward[i] < 0.0 || kd[i] < 0.0 || limits[i] <= 0.0)
      {
        throw std::invalid_argument("invalid velocity controller input");
      }

      reference_[i] += reference_alpha * (reference[i] - reference_[i]);
      const double error = reference_[i] - measured[i];
      const double acceleration = have_previous_measurement_ ?
        (measured[i] - previous_measurement_[i]) / dt : 0.0;
      derivative_filtered_[i] += derivative_alpha *
        (acceleration - derivative_filtered_[i]);

      if (ki[i] == 0.0) {
        integral_[i] = 0.0;
      }
      const double candidate = std::clamp(
        integral_[i] + ki[i] * error * dt, -limits[i], limits[i]);
      const double proposed = feedforward[i] * reference_[i] +
        kp[i] * error - kd[i] * derivative_filtered_[i] + candidate;
      if (std::abs(proposed) <= limits[i] || proposed * error < 0.0) {
        integral_[i] = candidate;
      }
      out[i] = std::clamp(
        feedforward[i] * reference_[i] + kp[i] * error -
        kd[i] * derivative_filtered_[i] + integral_[i], -limits[i], limits[i]);
      previous_measurement_[i] = measured[i];
    }
    have_previous_measurement_ = true;
    return out;
  }

  void trackApplied(
    const Axes & requested, const Axes & applied, double dt, const Axes & ki)
  {
    for (std::size_t i = 0; i < 3U; ++i) {
      if (ki[i] > 0.0) {
        integral_[i] += std::clamp(dt, 0.0, 0.1) * (applied[i] - requested[i]);
      }
    }
  }

private:
  Axes integral_{};
  Axes reference_{};
  Axes previous_measurement_{};
  Axes derivative_filtered_{};
  bool have_previous_measurement_{false};
};
}  // namespace njord::thruster_controller
#endif  // THRUSTER_CONTROLLER__VELOCITY_CONTROL_HPP_
