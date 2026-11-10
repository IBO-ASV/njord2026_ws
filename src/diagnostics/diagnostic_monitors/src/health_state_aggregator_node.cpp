#include "diagnostic_monitors/health_state_aggregator.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<njord::diagnostic_monitors::HealthStateAggregator>(
    rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
