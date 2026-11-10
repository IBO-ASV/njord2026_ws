#include "alert_lamp/alert_lamp_manager.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <sstream>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace alert_lamp
{
namespace
{
uint8_t colorValue(LampColor color)
{
  return static_cast<uint8_t>(color);
}

uint8_t patternValue(LampPattern pattern)
{
  return static_cast<uint8_t>(pattern);
}

std::string stateName(AlertState state)
{
  switch (state) {
    case AlertState::INITIALIZING: return "INITIALIZING";
    case AlertState::MANUAL_NORMAL: return "MANUAL_NORMAL";
    case AlertState::AUTO_NORMAL: return "AUTO_NORMAL";
    case AlertState::AUTONOMY_NOT_READY: return "AUTONOMY_NOT_READY";
    case AlertState::GROUND_COMMUNICATION_LOST: return "GROUND_COMMUNICATION_LOST";
    case AlertState::CRITICAL_FAULT: return "CRITICAL_FAULT";
  }
  return "UNKNOWN";
}

bool hasInhibit(const njord_interfaces::msg::ControlState & state, uint16_t code)
{
  return std::find(state.inhibit_reason_codes.begin(), state.inhibit_reason_codes.end(), code) !=
         state.inhibit_reason_codes.end();
}

LampColor parseColor(const std::string & color)
{
  if (color == "off") {return LampColor::OFF;}
  if (color == "green") {return LampColor::GREEN;}
  if (color == "yellow") {return LampColor::YELLOW;}
  if (color == "red") {return LampColor::RED;}
  if (color == "green_yellow") {return LampColor::GREEN_YELLOW;}
  if (color == "green_red") {return LampColor::GREEN_RED;}
  if (color == "yellow_red") {return LampColor::YELLOW_RED;}
  throw std::invalid_argument("unknown alert-lamp color: " + color);
}

LampPattern parsePattern(const std::string & pattern)
{
  if (pattern == "off") {return LampPattern::OFF;}
  if (pattern == "solid") {return LampPattern::SOLID;}
  if (pattern == "blink") {return LampPattern::BLINK;}
  throw std::invalid_argument("unknown alert-lamp pattern: " + pattern);
}
}  // namespace

AlertLampManager::AlertLampManager(const rclcpp::NodeOptions & options)
: Node("alert_lamp_manager_node", options), updater_(this)
{
  const auto rate = declare_parameter<double>("evaluation_rate_hz", 10.0);
  const auto transient_qos = rclcpp::QoS(10).transient_local();
  control_sub_ = create_subscription<njord_interfaces::msg::ControlState>(
    declare_parameter<std::string>("topics.control_state", "/control/state"), transient_qos,
    [this](const njord_interfaces::msg::ControlState::SharedPtr message) {
      control_state_ = *message;
      control_received_ = true;
    });
  mission_sub_ = create_subscription<njord_interfaces::msg::MissionStatus>(
    declare_parameter<std::string>("topics.mission_status", "/mission/status"), transient_qos,
    [this](const njord_interfaces::msg::MissionStatus::SharedPtr message) {
      mission_status_ = *message;
      mission_received_ = true;
    });
  health_sub_ = create_subscription<njord_interfaces::msg::HealthState>(
    declare_parameter<std::string>("topics.health_state", "/health/state"), transient_qos,
    [this](const njord_interfaces::msg::HealthState::SharedPtr message) {
      health_state_ = *message;
      health_received_ = true;
    });

  initializing_display_ = loadDisplay(
    "initializing", {LampColor::RED, LampPattern::BLINK, 0.5F, 0.5F, "initializing"});
  manual_normal_display_ = loadDisplay(
    "manual_normal", {LampColor::YELLOW, LampPattern::SOLID, 0.0F, 0.5F,
      "manual mode normal"});
  auto_normal_display_ = loadDisplay(
    "auto_normal", {LampColor::GREEN, LampPattern::SOLID, 0.0F, 0.5F,
      "automatic mode normal"});
  autonomy_not_ready_display_ = loadDisplay(
    "autonomy_not_ready",
    {LampColor::GREEN_YELLOW, LampPattern::BLINK, 1.0F, 0.5F,
      "autonomy not ready"});
  ground_link_lost_auto_display_ = loadDisplay(
    "ground_link_lost_auto", {LampColor::GREEN_RED, LampPattern::SOLID, 0.0F, 0.5F,
      "ground station communication lost in automatic mode"});
  ground_link_lost_manual_display_ = loadDisplay(
    "ground_link_lost_manual", {LampColor::YELLOW_RED, LampPattern::SOLID, 0.0F, 0.5F,
      "ground station communication lost in manual mode"});
  critical_fault_display_ = loadDisplay(
    "critical_fault", {LampColor::RED, LampPattern::BLINK, 0.5F, 0.5F,
      "critical fault or unknown state"});
  command_pub_ = create_publisher<msg::AlertLampCommand>(
    declare_parameter<std::string>("topics.alert_command", "/alert_lamp/command"), 10);
  updater_.setHardwareID("alert_lamp_manager");
  updater_.add("Alert Lamp Manager", this, &AlertLampManager::updateDiagnostic);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
    std::bind(&AlertLampManager::onTimer, this));
}

LampDisplay AlertLampManager::loadDisplay(const std::string & name, const LampDisplay & defaults)
{
  const auto prefix = "patterns." + name + ".";
  LampDisplay display = defaults;
  display.color = parseColor(
    declare_parameter<std::string>(
      prefix + "color", [&defaults]() {
        switch (defaults.color) {
          case LampColor::OFF: return std::string("off");
          case LampColor::GREEN: return std::string("green");
          case LampColor::YELLOW: return std::string("yellow");
          case LampColor::RED: return std::string("red");
          case LampColor::GREEN_YELLOW: return std::string("green_yellow");
          case LampColor::GREEN_RED: return std::string("green_red");
          case LampColor::YELLOW_RED: return std::string("yellow_red");
        }
        return std::string("red");
      }()));
  display.pattern = parsePattern(
    declare_parameter<std::string>(
      prefix + "mode",
      defaults.pattern == LampPattern::SOLID ? "solid" :
      defaults.pattern == LampPattern::OFF ? "off" : "blink"));
  display.period =
    static_cast<float>(declare_parameter<double>(prefix + "period_sec", defaults.period));
  display.duty_ratio =
    static_cast<float>(declare_parameter<double>(prefix + "duty_ratio", defaults.duty_ratio));
  display.reason = declare_parameter<std::string>(prefix + "reason", defaults.reason);
  if (display.period < 0.0F || display.duty_ratio <= 0.0F || display.duty_ratio > 1.0F) {
    throw std::invalid_argument("invalid alert-lamp pattern parameters for " + name);
  }
  return display;
}

LampDisplay AlertLampManager::displayFor(AlertState state, const SystemStatus & status) const
{
  switch (state) {
    case AlertState::INITIALIZING: return initializing_display_;
    case AlertState::MANUAL_NORMAL: return manual_normal_display_;
    case AlertState::AUTO_NORMAL: return auto_normal_display_;
    case AlertState::AUTONOMY_NOT_READY: return autonomy_not_ready_display_;
    case AlertState::GROUND_COMMUNICATION_LOST:
      return status.mode == OperatingMode::MANUAL ?
             ground_link_lost_manual_display_ : ground_link_lost_auto_display_;
    case AlertState::CRITICAL_FAULT: return critical_fault_display_;
  }
  return critical_fault_display_;
}

SystemStatus AlertLampManager::collectStatus(const rclcpp::Time & current) const
{
  (void)current;
  SystemStatus status;
  status.state_unknown = !(control_received_ && mission_received_ && health_received_);

  // All operational decisions below are projections of canonical messages.
  // The lamp does not subscribe to raw heartbeats, GNSS, localization, or
  // DiagnosticArray and therefore cannot create a competing safety policy.
  status.driver_alive = true;
  status.high_level_alive = true;
  status.autonomy_alive = true;
  status.localization_alive = true;
  status.manual_control_alive = true;
  status.localization_stable = true;
  status.rtk_fix = true;
  status.ground_station_connected = true;

  if (control_received_) {
    status.mode = control_state_.requested_mode ==
      njord_interfaces::msg::ControlState::MODE_AUTO ? OperatingMode::AUTO : OperatingMode::MANUAL;
    status.emergency_stop = control_state_.emergency_stop ||
      control_state_.state == njord_interfaces::msg::ControlState::STATE_EMERGENCY_STOP;
    status.autonomy_ready = control_state_.auto_permitted;
    status.required_sensor_not_ready =
      status.mode == OperatingMode::AUTO && !control_state_.auto_permitted;
    status.ground_station_connected = !hasInhibit(
      control_state_, njord_interfaces::msg::ControlState::INHIBIT_GROUND_STATION_UNAVAILABLE);
    status.rtk_fix = !hasInhibit(
      control_state_, njord_interfaces::msg::ControlState::INHIBIT_RTK_FIX);
    status.critical_driver_failure = false;
  }

  if (mission_received_ && status.mode == OperatingMode::AUTO) {
    const auto waiting = mission_status_.state ==
      njord_interfaces::msg::MissionStatus::STATE_WAITING_FOR_READINESS ||
      mission_status_.state ==
      njord_interfaces::msg::MissionStatus::STATE_WAITING_FOR_AUTO_PERMISSION ||
      mission_status_.state == njord_interfaces::msg::MissionStatus::STATE_VALIDATING ||
      mission_status_.state == njord_interfaces::msg::MissionStatus::STATE_CONFIGURING ||
      mission_status_.state == njord_interfaces::msg::MissionStatus::STATE_REJECTED ||
      mission_status_.state == njord_interfaces::msg::MissionStatus::STATE_FAILED;
    status.required_sensor_not_ready = status.required_sensor_not_ready || waiting;
  }

  if (health_received_) {
    status.critical_diagnostics = health_state_.summary_state ==
      njord_interfaces::msg::HealthState::ERROR;
    status.state_unknown = status.state_unknown ||
      health_state_.summary_state == njord_interfaces::msg::HealthState::UNKNOWN ||
      health_state_.summary_state == njord_interfaces::msg::HealthState::STALE;
  }
  return status;
}

void AlertLampManager::onTimer()
{
  const auto current = now();
  const auto status = collectStatus(current);
  const auto state = evaluator_.evaluate(status);
  const auto display = displayFor(state, status);
  msg::AlertLampCommand command;
  command.header.stamp = current;
  command.color = colorValue(display.color);
  command.pattern = patternValue(display.pattern);
  command.period = display.period;
  command.duty_ratio = display.duty_ratio;
  command.reason = display.reason;
  command_pub_->publish(command);
  if (!has_last_state_ || state != last_state_) {
    RCLCPP_WARN(
      get_logger(), "%s -> %s: %s", has_last_state_ ? stateName(last_state_).c_str() : "START",
      stateName(state).c_str(), display.reason.c_str());
    last_state_ = state;
    has_last_state_ = true;
  }
  updater_.force_update();
}

void AlertLampManager::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto status = collectStatus(now());
  const auto state = evaluator_.evaluate(status);
  const auto display = displayFor(state, status);
  stat.summary(
    state == AlertState::CRITICAL_FAULT ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    state == AlertState::MANUAL_NORMAL || state == AlertState::AUTO_NORMAL ?
    diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN,
    display.reason);
  stat.add("alert_state", stateName(state));
  stat.add("reason", display.reason);
  stat.add("control_state_received", control_received_);
  stat.add("mission_status_received", mission_received_);
  stat.add("health_state_received", health_received_);
  if (health_received_) {
    stat.add("health_summary_state", static_cast<int>(health_state_.summary_state));
  }
  if (control_received_) {
    stat.add("control_state", static_cast<int>(control_state_.state));
    stat.add("inhibit_reason_count", static_cast<int>(control_state_.inhibit_reasons.size()));
  }
}

}  // namespace alert_lamp
