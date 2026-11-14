#include "thruster_controller/node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord::thruster_controller::ThrusterControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
