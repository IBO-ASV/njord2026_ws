#ifndef SIMPLE_MANUAL__JOY_CONVERTER_HPP_
#define SIMPLE_MANUAL__JOY_CONVERTER_HPP_

#include <mutex>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace simple_manual
{

struct JoyConfig
{
  int64_t linear_x_axis{1};
  int64_t linear_y_axis{0};
  int64_t yaw_positive_button{6};
  int64_t yaw_negative_button{7};
  int64_t emergency_button{0};
  int64_t green_button{1};
  int64_t yellow_button{2};
  int64_t red_button{3};
  int64_t manual_mode_button{4};
  int64_t auto_mode_button{5};
  double linear_x_scale{0.2};
  double linear_y_scale{0.2};
  double angular_z_scale{0.2};
};

struct JoyOutput
{
  geometry_msgs::msg::Twist cmd_vel;
  bool emergency{false};
  bool green{false};
  bool yellow{false};
  bool red{false};
  bool manual_mode{true};
  bool auto_mode{false};
};

JoyOutput convert_joy(const sensor_msgs::msg::Joy & msg, const JoyConfig & config);

class JoyConverter : public rclcpp::Node
{
public:
  explicit JoyConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  JoyConfig config_;
  std::mutex config_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_soft_emg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_operating_mode_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_green_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_yellow_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_red_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  bool manual_mode_{true};
  bool previous_manual_button_{false};
  bool previous_auto_button_{false};
};

}  // namespace simple_manual

#endif  // SIMPLE_MANUAL__JOY_CONVERTER_HPP_
