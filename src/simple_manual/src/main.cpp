#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "simple_manual/joy_converter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_manual::JoyConverter>());
  rclcpp::shutdown();
  return 0;
}
