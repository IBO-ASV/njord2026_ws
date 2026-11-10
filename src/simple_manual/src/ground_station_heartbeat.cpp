#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace simple_manual
{

class GroundStationHeartbeat : public rclcpp::Node
{
public:
  GroundStationHeartbeat()
  : Node("ground_station_heartbeat")
  {
    const auto topic = declare_parameter<std::string>(
      "topic", "/heartbeat/ground_station");
    const auto period_sec = declare_parameter<double>("period_sec", 1.0);
    publisher_ = create_publisher<std_msgs::msg::Empty>(topic, 10);
    const auto period = std::chrono::duration<double>(std::max(0.01, period_sec));
    timer_ = create_wall_timer(period, [this]() {publisher_->publish(std_msgs::msg::Empty{});});
  }

private:
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace simple_manual

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_manual::GroundStationHeartbeat>());
  rclcpp::shutdown();
  return 0;
}
