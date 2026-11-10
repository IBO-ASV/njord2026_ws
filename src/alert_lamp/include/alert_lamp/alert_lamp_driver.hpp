#pragma once

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "alert_lamp/msg/alert_lamp_command.hpp"

namespace alert_lamp
{

class AlertLampDriver : public rclcpp::Node
{
public:
  explicit AlertLampDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onCommand(const msg::AlertLampCommand::SharedPtr message);
  void onTimer();
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void apply(const msg::AlertLampCommand & command, bool fallback);

  rclcpp::Subscription<msg::AlertLampCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr red_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yellow_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr green_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
  msg::AlertLampCommand command_;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  bool received_command_{false};
  bool fallback_active_{true};
  bool red_output_{false};
  bool yellow_output_{false};
  bool green_output_{false};
  double command_timeout_sec_{1.0};
  std::string output_type_{"topic"};
};

}  // namespace alert_lamp
