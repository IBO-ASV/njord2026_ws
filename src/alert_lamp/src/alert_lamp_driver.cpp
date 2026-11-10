#include "alert_lamp/alert_lamp_driver.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace alert_lamp
{

AlertLampDriver::AlertLampDriver(const rclcpp::NodeOptions & options)
: Node("alert_lamp_driver_node", options), updater_(this)
{
  command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 1.0);
  output_type_ = declare_parameter<std::string>("output_type", "topic");
  red_pub_ = create_publisher<std_msgs::msg::Bool>(declare_parameter<std::string>("topics.red",
      "/red"), 10);
  yellow_pub_ =
    create_publisher<std_msgs::msg::Bool>(declare_parameter<std::string>("topics.yellow",
      "/yellow"), 10);
  green_pub_ = create_publisher<std_msgs::msg::Bool>(declare_parameter<std::string>("topics.green",
      "/green"), 10);
  command_sub_ = create_subscription<msg::AlertLampCommand>(
    declare_parameter<std::string>("topics.alert_command", "/alert_lamp/command"), 10,
    std::bind(&AlertLampDriver::onCommand, this, std::placeholders::_1));
  updater_.setHardwareID("alert_lamp_driver");
  updater_.add("Alert Lamp Driver", this, &AlertLampDriver::updateDiagnostic);
  timer_ = create_wall_timer(std::chrono::milliseconds(25),
      std::bind(&AlertLampDriver::onTimer, this));
  if (output_type_ != "topic") {
    RCLCPP_ERROR(get_logger(),
        "output_type='%s' is not implemented; use topic output or add a hardware backend",
        output_type_.c_str());
  }
}

void AlertLampDriver::onCommand(const msg::AlertLampCommand::SharedPtr message)
{
  command_ = *message;
  last_command_ = now();
  received_command_ = true;
}

void AlertLampDriver::onTimer()
{
  const bool fallback = !received_command_ ||
    (now() - last_command_).seconds() > command_timeout_sec_;
  if (fallback) {
    msg::AlertLampCommand command;
    command.color = msg::AlertLampCommand::COLOR_RED;
    command.pattern = msg::AlertLampCommand::PATTERN_BLINK;
    command.period = 0.05F;
    command.duty_ratio = 0.5F;
    command.reason = "manager command timeout";
    apply(command, true);
  } else {
    apply(command_, false);
  }
  updater_.force_update();
}

void AlertLampDriver::apply(const msg::AlertLampCommand & command, bool fallback)
{
  fallback_active_ = fallback;
  bool on = command.pattern == msg::AlertLampCommand::PATTERN_SOLID;
  if (command.pattern == msg::AlertLampCommand::PATTERN_BLINK) {
    const double period = std::max(0.01, static_cast<double>(command.period));
    const double phase = std::fmod(now().seconds(), period) / period;
    on = phase < command.duty_ratio;
  }
  // Combined colors are intentional: they encode compound states such as
  // autonomy-not-ready and ground-station communication loss.
  red_output_ = on && (command.color & msg::AlertLampCommand::COLOR_RED) != 0;
  yellow_output_ = on && (command.color & msg::AlertLampCommand::COLOR_YELLOW) != 0;
  green_output_ = on && (command.color & msg::AlertLampCommand::COLOR_GREEN) != 0;
  std_msgs::msg::Bool red; red.data = red_output_; red_pub_->publish(red);
  std_msgs::msg::Bool yellow; yellow.data = yellow_output_; yellow_pub_->publish(yellow);
  std_msgs::msg::Bool green; green.data = green_output_; green_pub_->publish(green);
}

void AlertLampDriver::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const double age = received_command_ ? (now() - last_command_).seconds() : -1.0;
  stat.summary(fallback_active_ ||
      output_type_ != "topic" ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK,
      fallback_active_ ? "fallback active" : "output active");
  stat.add("last_command_age", age);
  stat.add("output_type", output_type_);
  stat.add("green_output", green_output_);
  stat.add("yellow_output", yellow_output_);
  stat.add("red_output", red_output_);
  stat.add("hardware_error", output_type_ != "topic");
  stat.add("fallback_active", fallback_active_);
}

}  // namespace alert_lamp
