#include <memory>

// Copyright 2026 IBO-ASV

#include "diagnostic_monitors/heartbeat_aggregator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<njord::diagnostic_monitors::HeartbeatAggregator>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
