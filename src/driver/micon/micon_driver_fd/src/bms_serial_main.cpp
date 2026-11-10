#include <memory>

#include "micon_driver_fd/bms_serial_reader.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<micon_driver_fd::BmsSerialReader>());
  rclcpp::shutdown();
  return 0;
}
