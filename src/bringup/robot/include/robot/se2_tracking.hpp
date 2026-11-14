#ifndef ROBOT__SE2_TRACKING_HPP_
#define ROBOT__SE2_TRACKING_HPP_
#include <algorithm>
#include <cmath>

namespace robot
{
inline double angle(double value) {return std::atan2(std::sin(value), std::cos(value));}
struct State {double x{}, y{}, yaw{};};
struct Command {double x{}, y{}, yaw{};};
struct TrackingLimits
{
  double speed{0.25}, yaw_rate{0.35}, acceleration{0.25}, yaw_acceleration{0.5};
  double position_gain{0.8}, yaw_gain{1.2};
};
// World-frame reference velocity; output is in the vessel's current body frame.
inline Command track(
  State current, State reference, Command feedforward, Command previous,
  double dt, const TrackingLimits & limits)
{
  double wx = feedforward.x + limits.position_gain * (reference.x - current.x);
  double wy = feedforward.y + limits.position_gain * (reference.y - current.y);
  Command result{
    std::cos(current.yaw) * wx + std::sin(current.yaw) * wy,
    -std::sin(current.yaw) * wx + std::cos(current.yaw) * wy,
    feedforward.yaw + limits.yaw_gain * angle(reference.yaw - current.yaw)};
  const double speed = std::hypot(result.x, result.y);
  if (speed > limits.speed && speed > 0.0) {
    result.x *= limits.speed / speed;
    result.y *= limits.speed / speed;
  }
  result.yaw = std::clamp(result.yaw, -limits.yaw_rate, limits.yaw_rate);
  const double change = std::hypot(result.x - previous.x, result.y - previous.y);
  if (change > limits.acceleration * dt && change > 0.0) {
    const double scale = limits.acceleration * dt / change;
    result.x = previous.x + scale * (result.x - previous.x);
    result.y = previous.y + scale * (result.y - previous.y);
  }
  result.yaw = std::clamp(result.yaw,
    previous.yaw - limits.yaw_acceleration * dt,
    previous.yaw + limits.yaw_acceleration * dt);
  return result;
}
}  // namespace robot
#endif  // ROBOT__SE2_TRACKING_HPP_
