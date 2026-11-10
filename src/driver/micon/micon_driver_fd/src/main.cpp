#include <memory>

#include "micon_driver_fd/serial_writer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<micon_driver_fd::SerialWriter>());
  rclcpp::shutdown();
  return 0;
}
