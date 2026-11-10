#include "alert_lamp/alert_lamp_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<alert_lamp::AlertLampDriver>());
  rclcpp::shutdown();
  return 0;
}
