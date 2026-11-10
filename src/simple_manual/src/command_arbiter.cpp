#include <chrono>
#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace simple_manual
{
class CommandArbiter : public rclcpp::Node
{
public:
  CommandArbiter()
  : Node("command_arbiter")
  {
    timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    mode_ = declare_parameter<std::string>("initial_mode", "manual");
    auto_topic_ = declare_parameter<std::string>("auto_topic", "/cmd_vel_nav");
    require_autonomy_ready_ = declare_parameter<bool>("require_autonomy_ready", true);
    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_manual", 10, [this](geometry_msgs::msg::Twist::SharedPtr message) {
        manual_command_ = *message;
        manual_received_ = now();
      });
    auto_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      auto_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr message) {
        auto_command_ = *message;
        auto_received_ = now();
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/operating_mode", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr message) {
        if (message->data == "manual" || message->data == "auto") {
          mode_ = message->data;
        }
      });
    emergency_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/safety/emergency_stop", 10,
      [this](std_msgs::msg::UInt8::SharedPtr message) {
        emergency_stop_ = message->data != 0U;
      });
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/autonomy/ready", 10,
      [this](std_msgs::msg::Bool::SharedPtr message) {autonomy_ready_ = message->data;});
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    control_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/control_status", rclcpp::QoS(1).transient_local());
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("/heartbeat/high_level", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&CommandArbiter::publish, this));
    heartbeat_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
          heartbeat_pub_->publish(std_msgs::msg::Empty{});
    });
  }

private:
  bool fresh(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() != 0 && (now() - stamp).seconds() <= timeout_sec_;
  }

  void publish()
  {
    geometry_msgs::msg::Twist command;
    if (!emergency_stop_) {
      if (mode_ == "manual" && fresh(manual_received_)) {
        command = manual_command_;
      } else if (mode_ == "auto" &&
        (!require_autonomy_ready_ || autonomy_ready_) && fresh(auto_received_)) {
        command = auto_command_;
      }
    }
    command_pub_->publish(command);
    control_status_pub_->publish(std_msgs::msg::String().set__data(
        emergency_stop_ ? "emergency_stop" : mode_));
  }

  double timeout_sec_{0.5};
  std::string mode_{"manual"};
  std::string auto_topic_{"/cmd_vel_nav"};
  bool require_autonomy_ready_{true};
  bool emergency_stop_{true};
  bool autonomy_ready_{false};
  geometry_msgs::msg::Twist manual_command_;
  geometry_msgs::msg::Twist auto_command_;
  rclcpp::Time manual_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time auto_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};
}  // namespace simple_manual

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_manual::CommandArbiter>());
  rclcpp::shutdown();
  return 0;
}
