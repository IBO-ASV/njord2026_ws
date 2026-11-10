#include "simple_manual/soft_emg_selector.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace simple_manual
{

bool sbus_soft_emg_selected(const sensor_msgs::msg::Joy & joy)
{
  return joy.buttons.empty() || joy.buttons[0] == 0;
}

class SoftEmgSelector : public rclcpp::Node
{
public:
  SoftEmgSelector()
  : Node("soft_emg_selector")
  {
    const auto timeout = declare_parameter<double>("sbus_timeout_sec", 0.5);
    timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout));
    sbus_joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/sbus/joy", 10, std::bind(&SoftEmgSelector::sbus_joy_cb, this, std::placeholders::_1));
    ground_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/soft_emg_ground", 10, std::bind(&SoftEmgSelector::ground_cb, this, std::placeholders::_1));
    sbus_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/soft_emg_sbus", 10, std::bind(&SoftEmgSelector::sbus_cb, this, std::placeholders::_1));
    output_pub_ = create_publisher<std_msgs::msg::Bool>("/soft_emg", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&SoftEmgSelector::timeout_cb, this));
  }

private:
  void publish()
  {
    output_pub_->publish(std_msgs::msg::Bool().set__data(sbus_selected_ ? sbus_emg_ : ground_emg_));
  }

  void sbus_joy_cb(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    sbus_selected_ = sbus_soft_emg_selected(*joy);
    sbus_received_ = std::chrono::steady_clock::now();
    if (!sbus_selected_) {
      publish();
    }
  }

  void ground_cb(const std_msgs::msg::Bool::SharedPtr message)
  {
    ground_emg_ = message->data;
    if (!sbus_selected_) {
      publish();
    }
  }

  void sbus_cb(const std_msgs::msg::Bool::SharedPtr message)
  {
    sbus_emg_ = message->data;
    if (sbus_selected_) {
      publish();
    }
  }

  void timeout_cb()
  {
    if (sbus_selected_ && sbus_received_ &&
      std::chrono::steady_clock::now() - *sbus_received_ > timeout_)
    {
      sbus_selected_ = false;
      publish();
    }
  }

  bool sbus_selected_{false};
  bool ground_emg_{false};
  bool sbus_emg_{false};
  std::chrono::steady_clock::duration timeout_;
  std::optional<std::chrono::steady_clock::time_point> sbus_received_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sbus_joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ground_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sbus_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace simple_manual

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_manual::SoftEmgSelector>());
  rclcpp::shutdown();
  return 0;
}
