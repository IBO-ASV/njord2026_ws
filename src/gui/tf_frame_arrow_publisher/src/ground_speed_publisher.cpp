#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class GroundSpeedPublisher : public rclcpp::Node
{
public:
  GroundSpeedPublisher()
  : Node("ground_speed_publisher")
  {
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/odometry/feedback");
    output_topic_ = declare_parameter<std::string>("output_topic", "/gui/ground_speed_mps");
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 20, std::bind(&GroundSpeedPublisher::callback, this, std::placeholders::_1));
    pub_ = create_publisher<std_msgs::msg::Float32>(output_topic_, 20);
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
  {
    const auto & velocity = odometry->twist.twist.linear;
    pub_->publish(std_msgs::msg::Float32().set__data(
        static_cast<float>(std::hypot(velocity.x, velocity.y))));
  }

  std::string odometry_topic_;
  std::string output_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSpeedPublisher>());
  rclcpp::shutdown();
  return 0;
}
