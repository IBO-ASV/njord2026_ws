#include "simple_manual/joy_converter.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <string>

namespace simple_manual
{
namespace
{

double axis_value(const sensor_msgs::msg::Joy & msg, int64_t index)
{
  return static_cast<size_t>(index) < msg.axes.size() ? msg.axes[index] : 0.0;
}

bool button_value(const sensor_msgs::msg::Joy & msg, int64_t index)
{
  return static_cast<size_t>(index) < msg.buttons.size() && msg.buttons[index] != 0;
}

}  // namespace

JoyOutput convert_joy(const sensor_msgs::msg::Joy & msg, const JoyConfig & config)
{
  JoyOutput output;
  output.cmd_vel.linear.x = config.linear_x_scale * axis_value(msg, config.linear_x_axis);
  output.cmd_vel.linear.y = config.linear_y_scale * axis_value(msg, config.linear_y_axis);
  output.cmd_vel.angular.z = config.angular_z_scale *
    (static_cast<double>(button_value(msg, config.yaw_positive_button)) -
    static_cast<double>(button_value(msg, config.yaw_negative_button)));
  // Positive logic: pressing the designated button requests an emergency
  // stop; its normal released state keeps the communication E-stop clear.
  output.emergency = button_value(msg, config.emergency_button);
  output.green = button_value(msg, config.green_button);
  output.yellow = button_value(msg, config.yellow_button);
  output.red = button_value(msg, config.red_button);
  output.manual_mode = button_value(msg, config.manual_mode_button);
  output.auto_mode = button_value(msg, config.auto_mode_button);
  return output;
}

JoyConverter::JoyConverter(const rclcpp::NodeOptions & options)
: Node("joy_converter", options)
{
  config_.linear_x_axis = declare_parameter<int64_t>("axis.linear_x", 1);
  config_.linear_y_axis = declare_parameter<int64_t>("axis.linear_y", 0);
  config_.yaw_positive_button = declare_parameter<int64_t>("button.yaw_positive", 6);
  config_.yaw_negative_button = declare_parameter<int64_t>("button.yaw_negative", 7);
  config_.emergency_button = declare_parameter<int64_t>("button.emergency", 0);
  config_.green_button = declare_parameter<int64_t>("button.green", 1);
  config_.yellow_button = declare_parameter<int64_t>("button.yellow", 2);
  config_.red_button = declare_parameter<int64_t>("button.red", 3);
  config_.manual_mode_button = declare_parameter<int64_t>("button.manual_mode", 4);
  config_.auto_mode_button = declare_parameter<int64_t>("button.auto_mode", 5);
  config_.linear_x_scale = declare_parameter<double>("scale.linear_x", 0.2);
  config_.linear_y_scale = declare_parameter<double>("scale.linear_y", 0.2);
  config_.angular_z_scale = declare_parameter<double>("scale.angular_z", 0.2);

  parameter_callback_ = add_on_set_parameters_callback(
    std::bind(&JoyConverter::on_parameters, this, std::placeholders::_1));
  sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoyConverter::joy_cb, this, std::placeholders::_1));
  pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
  pub_soft_emg_ = create_publisher<std_msgs::msg::Bool>("/soft_emg", 10);
  pub_operating_mode_ = create_publisher<std_msgs::msg::String>(
    "/system/operating_mode", rclcpp::QoS(1).transient_local());
  pub_heartbeat_ = create_publisher<std_msgs::msg::Empty>(
    "/heartbeat/manual_control", 10);
  pub_green_ = create_publisher<std_msgs::msg::Bool>("/green", 10);
  pub_yellow_ = create_publisher<std_msgs::msg::Bool>("/yellow", 10);
  pub_red_ = create_publisher<std_msgs::msg::Bool>("/red", 10);
  heartbeat_timer_ = create_wall_timer(
    std::chrono::seconds(1), [this]() {pub_heartbeat_->publish(std_msgs::msg::Empty{});});
  RCLCPP_INFO(get_logger(), "joy_converter started");
}

void JoyConverter::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  JoyConfig config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config = config_;
  }
  const auto output = convert_joy(*msg, config);
  if (output.manual_mode && !previous_manual_button_) {
    manual_mode_ = true;
  } else if (output.auto_mode && !previous_auto_button_) {
    manual_mode_ = false;
  }
  previous_manual_button_ = output.manual_mode;
  previous_auto_button_ = output.auto_mode;

  pub_cmd_vel_->publish(output.cmd_vel);
  pub_soft_emg_->publish(std_msgs::msg::Bool().set__data(output.emergency));
  pub_operating_mode_->publish(
    std_msgs::msg::String().set__data(manual_mode_ ? "manual" : "auto"));
  pub_green_->publish(std_msgs::msg::Bool().set__data(output.green));
  pub_yellow_->publish(std_msgs::msg::Bool().set__data(output.yellow));
  pub_red_->publish(std_msgs::msg::Bool().set__data(output.red));
}

rcl_interfaces::msg::SetParametersResult JoyConverter::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  JoyConfig candidate = config_;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "axis.linear_x") {
      candidate.linear_x_axis = parameter.as_int();
    } else if (name == "axis.linear_y") {
      candidate.linear_y_axis = parameter.as_int();
    } else if (name == "button.yaw_positive") {
      candidate.yaw_positive_button = parameter.as_int();
    } else if (name == "button.yaw_negative") {
      candidate.yaw_negative_button = parameter.as_int();
    } else if (name == "button.emergency") {
      candidate.emergency_button = parameter.as_int();
    } else if (name == "button.green") {
      candidate.green_button = parameter.as_int();
    } else if (name == "button.yellow") {
      candidate.yellow_button = parameter.as_int();
    } else if (name == "button.red") {
      candidate.red_button = parameter.as_int();
    } else if (name == "button.manual_mode") {
      candidate.manual_mode_button = parameter.as_int();
    } else if (name == "button.auto_mode") {
      candidate.auto_mode_button = parameter.as_int();
    } else if (name == "scale.linear_x") {
      candidate.linear_x_scale = parameter.as_double();
    } else if (name == "scale.linear_y") {
      candidate.linear_y_scale = parameter.as_double();
    } else if (name == "scale.angular_z") {candidate.angular_z_scale = parameter.as_double();}
  }

  const bool valid_indices = candidate.linear_x_axis >= 0 && candidate.linear_y_axis >= 0 &&
    candidate.yaw_positive_button >= 0 && candidate.yaw_negative_button >= 0 &&
    candidate.emergency_button >= 0 && candidate.green_button >= 0 &&
    candidate.yellow_button >= 0 && candidate.red_button >= 0 &&
    candidate.manual_mode_button >= 0 && candidate.auto_mode_button >= 0;
  const bool valid_scales = std::isfinite(candidate.linear_x_scale) &&
    std::isfinite(candidate.linear_y_scale) && std::isfinite(candidate.angular_z_scale);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = valid_indices && valid_scales;
  if (!result.successful) {
    result.reason = "Joy indices must be non-negative and scales must be finite";
    return result;
  }
  config_ = candidate;
  return result;
}

}  // namespace simple_manual
