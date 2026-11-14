#include "status_lights/heartbeat_monitor.hpp"

#include <limits>

namespace status_lights
{

void HeartbeatMonitor::update(const std::string & name, const rclcpp::Time & stamp)
{
  last_seen_[name] = stamp;
}

bool HeartbeatMonitor::isAlive(
  const std::string & name, const rclcpp::Time & now, double timeout_sec) const
{
  return age(name, now) <= timeout_sec;
}

double HeartbeatMonitor::age(const std::string & name, const rclcpp::Time & now) const
{
  const auto found = last_seen_.find(name);
  if (found == last_seen_.end()) {
    return std::numeric_limits<double>::infinity();
  }
  return (now - found->second).seconds();
}

}  // namespace status_lights
