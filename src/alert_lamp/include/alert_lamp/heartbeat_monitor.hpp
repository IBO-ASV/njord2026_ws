#pragma once

#include <string>
#include <unordered_map>

#include "rclcpp/time.hpp"

namespace alert_lamp
{

class HeartbeatMonitor
{
public:
  void update(const std::string & name, const rclcpp::Time & stamp);
  bool isAlive(const std::string & name, const rclcpp::Time & now, double timeout_sec) const;
  double age(const std::string & name, const rclcpp::Time & now) const;

private:
  std::unordered_map<std::string, rclcpp::Time> last_seen_;
};

}  // namespace alert_lamp
