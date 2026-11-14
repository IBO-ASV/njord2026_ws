#include "status_lights/alert_lamp_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<status_lights::AlertLampManager>());
  rclcpp::shutdown();
  return 0;
}
