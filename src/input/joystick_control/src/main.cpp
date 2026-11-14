#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joystick_control/joy_converter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joystick_control::JoyConverter>());
  rclcpp::shutdown();
  return 0;
}
