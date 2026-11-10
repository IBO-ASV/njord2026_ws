#include "alert_lamp/alert_lamp_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<alert_lamp::AlertLampManager>());
  rclcpp::shutdown();
  return 0;
}
