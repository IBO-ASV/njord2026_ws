#ifndef BMS_NODE_HPP_
#define BMS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

#include <array>
#include <optional>

namespace njord
{
namespace bms
{

class BmsNode : public rclcpp::Node
{
public:
  explicit BmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cellsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void temperatureCallback(const std_msgs::msg::Float32::SharedPtr msg);

  void publishCells(const std::array<float, 4> & cells);
  void publishDiagnostics(const std::array<float, 4> & cells);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_cells_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_temperature_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_cells_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pack_voltage_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_battery_percent_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_temperature_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  double warning_voltage_{3.5};
  double error_voltage_{3.3};
  double full_voltage_{4.2};
  double diagnostic_publish_period_sec_{0.2};
  rclcpp::Time last_diagnostic_publish_time_{0, 0, RCL_ROS_TIME};
  std::optional<float> temperature_c_;
};

}  // namespace bms
}  // namespace njord

#endif  // BMS_NODE_HPP_
