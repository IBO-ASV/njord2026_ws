#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "parameter_gateway/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord_control::ParameterGatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
