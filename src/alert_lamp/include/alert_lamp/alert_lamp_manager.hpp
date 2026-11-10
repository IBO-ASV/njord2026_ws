#pragma once

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "njord_interfaces/msg/control_state.hpp"
#include "njord_interfaces/msg/health_state.hpp"
#include "njord_interfaces/msg/mission_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include "alert_lamp/msg/alert_lamp_command.hpp"
#include "alert_lamp/alert_lamp_state.hpp"
#include "alert_lamp/status_evaluator.hpp"

namespace alert_lamp
{

class AlertLampManager : public rclcpp::Node
{
public:
  explicit AlertLampManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTimer();
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
  SystemStatus collectStatus(const rclcpp::Time & now) const;
  LampDisplay displayFor(AlertState state, const SystemStatus & status) const;
  LampDisplay loadDisplay(const std::string & name, const LampDisplay & defaults);

  StatusEvaluator evaluator_;
  bool control_received_{false};
  bool mission_received_{false};
  bool health_received_{false};
  njord_interfaces::msg::ControlState control_state_{};
  njord_interfaces::msg::MissionStatus mission_status_{};
  njord_interfaces::msg::HealthState health_state_{};
  AlertState last_state_{AlertState::INITIALIZING};
  bool has_last_state_{false};

  rclcpp::Subscription<njord_interfaces::msg::ControlState>::SharedPtr control_sub_;
  rclcpp::Subscription<njord_interfaces::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Subscription<njord_interfaces::msg::HealthState>::SharedPtr health_sub_;
  rclcpp::Publisher<msg::AlertLampCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
  LampDisplay initializing_display_;
  LampDisplay manual_normal_display_;
  LampDisplay auto_normal_display_;
  LampDisplay autonomy_not_ready_display_;
  LampDisplay ground_link_lost_auto_display_;
  LampDisplay ground_link_lost_manual_display_;
  LampDisplay critical_fault_display_;
};

}  // namespace alert_lamp
