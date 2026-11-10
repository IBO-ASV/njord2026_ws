#include "simple_manual/sbus_joy_converter.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace simple_manual
{
namespace
{

double axis_value(const sensor_msgs::msg::Joy & joy, size_t index)
{
  return index < joy.axes.size() ? joy.axes[index] : 0.0;
}

double offset_value(const std::vector<double> & offsets, size_t index)
{
  return index < offsets.size() ? offsets[index] : 0.0;
}

bool button_value(const sensor_msgs::msg::Joy & joy, size_t index)
{
  return index < joy.buttons.size() && joy.buttons[index] != 0;
}

}  // namespace

double truncate_3(double value)
{
  return std::floor(value * 1000.0 + 1e-4) / 1000.0;
}

SbusJoyOutput convert_sbus_joy(
  const sensor_msgs::msg::Joy & joy, const std::vector<double> & offsets)
{
  SbusJoyOutput output;
  output.cmd_vel.linear.x = truncate_3(-(axis_value(joy, 1) - offset_value(offsets, 1)));
  output.cmd_vel.linear.y = truncate_3(-(axis_value(joy, 0) - offset_value(offsets, 0)));
  output.cmd_vel.angular.z = truncate_3(-(axis_value(joy, 5) - offset_value(offsets, 5)));
  output.command_enabled = !button_value(joy, 0);
  if (!output.command_enabled) {
    return output;
  }
  const double axis4 = axis_value(joy, 4);
  output.soft_emg = axis4 >= 0.8;
  if (axis4 <= -0.8) {
    output.operating_mode = "auto";
  } else if (axis4 >= -0.1 && axis4 <= 0.1) {
    output.operating_mode = "manual";
  }
  return output;
}

class SbusJoyConverter : public rclcpp::Node
{
public:
  SbusJoyConverter()
  : Node("sbus_joy_converter")
  {
    zero_axes_on_startup_ = declare_parameter<bool>("zero_axes_on_startup", false);
    calibrated_ = !zero_axes_on_startup_;
    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&SbusJoyConverter::joy_cb, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_sbus", 10);
    soft_emg_pub_ = create_publisher<std_msgs::msg::Bool>("/soft_emg", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/operating_mode", rclcpp::QoS(1).transient_local());
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("/heartbeat/manual_control", 10);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {heartbeat_pub_->publish(std_msgs::msg::Empty{});});
  }

private:
  bool calibrate(const sensor_msgs::msg::Joy & joy)
  {
    if (button_value(joy, 0)) {
      calibration_start_.reset();
      calibration_sum_.clear();
      calibration_count_ = 0;
      return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!calibration_start_ || calibration_sum_.size() != joy.axes.size()) {
      calibration_start_ = now;
      calibration_sum_.assign(joy.axes.size(), 0.0);
      calibration_count_ = 0;
    }
    for (size_t i = 0; i < joy.axes.size(); ++i) {
      calibration_sum_[i] += joy.axes[i];
    }
    ++calibration_count_;
    if (now - *calibration_start_ < std::chrono::seconds(2)) {
      return false;
    }
    offsets_.resize(joy.axes.size());
    for (size_t i = 0; i < joy.axes.size(); ++i) {
      offsets_[i] = calibration_sum_[i] / calibration_count_;
    }
    return true;
  }

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    if (!calibrated_) {
      calibrated_ = calibrate(*joy);
      if (!calibrated_) {
        return;
      }
      RCLCPP_INFO(get_logger(), "SBUS axes calibrated");
    }
    const auto output = convert_sbus_joy(*joy, offsets_);
    if (output.command_enabled) {
      cmd_pub_->publish(output.cmd_vel);
      soft_emg_pub_->publish(std_msgs::msg::Bool().set__data(output.soft_emg));
      if (!output.operating_mode.empty()) {
        mode_pub_->publish(std_msgs::msg::String().set__data(output.operating_mode));
      }
    }
  }

  bool calibrated_{false};
  bool zero_axes_on_startup_{false};
  size_t calibration_count_{0};
  std::optional<std::chrono::steady_clock::time_point> calibration_start_;
  std::vector<double> calibration_sum_;
  std::vector<double> offsets_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr soft_emg_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace simple_manual

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_manual::SbusJoyConverter>());
  rclcpp::shutdown();
  return 0;
}
