#include "status_lights/alert_lamp_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<status_lights::AlertLampDriver>());
  rclcpp::shutdown();
  return 0;
}
