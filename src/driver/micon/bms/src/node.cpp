#include "bms/node.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <cmath>
#include <sstream>
#include <string>

namespace njord
{
namespace bms
{

BmsNode::BmsNode(const rclcpp::NodeOptions & options)
: Node("bms_node", options)
{
  const std::string input_topic =
    this->declare_parameter<std::string>("topics.input", "micon/bms_cells");
  const std::string output_topic =
    this->declare_parameter<std::string>("topics.output", "bms/cell_voltages");
  const std::string diagnostics_topic =
    this->declare_parameter<std::string>("topics.diagnostics", "/diagnostics");
  const std::string temperature_input_topic =
    this->declare_parameter<std::string>("topics.temperature_input", "/micon/bms_temperature_c");
  const std::string temperature_output_topic =
    this->declare_parameter<std::string>("topics.temperature_output", "/bms/temperature_c");
  warning_voltage_ = this->declare_parameter<double>("low_voltage.warning", 3.5);
  error_voltage_ = this->declare_parameter<double>("low_voltage.error", 3.3);
  full_voltage_ = this->declare_parameter<double>("battery.full_cell_voltage", 4.2);
  const double diagnostic_publish_rate_hz =
    this->declare_parameter<double>("diagnostic_publish_rate_hz", 5.0);
  if (diagnostic_publish_rate_hz <= 0.0) {
    throw std::invalid_argument("diagnostic_publish_rate_hz must be positive");
  }
  diagnostic_publish_period_sec_ = 1.0 / diagnostic_publish_rate_hz;

  pub_cells_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic, 10);
  pub_pack_voltage_ = this->create_publisher<std_msgs::msg::Float32>("/gui/battery_voltage_v", 10);
  pub_battery_percent_ = this->create_publisher<std_msgs::msg::Float32>("/gui/battery_percent", 10);
  pub_temperature_ = this->create_publisher<std_msgs::msg::Float32>(temperature_output_topic, 10);
  pub_diagnostics_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
  sub_cells_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    input_topic, 10, std::bind(&BmsNode::cellsCallback, this, std::placeholders::_1));
  sub_temperature_ = this->create_subscription<std_msgs::msg::Float32>(
    temperature_input_topic, 10, std::bind(&BmsNode::temperatureCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "bms node started. input=%s output=%s",
    input_topic.c_str(), output_topic.c_str());
}

void BmsNode::temperatureCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "BMS temperature is not finite");
    return;
  }
  temperature_c_ = msg->data;
  pub_temperature_->publish(*msg);
}

void BmsNode::cellsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 4U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "BMS cell message size=%zu. Expected 4.",
      msg->data.size());
    return;
  }

  const std::array<float, 4> cells{
    msg->data[0],
    msg->data[1],
    msg->data[2],
    msg->data[3]};
  publishCells(cells);
  const auto now = this->now();
  if ((now - last_diagnostic_publish_time_).seconds() >= diagnostic_publish_period_sec_) {
    publishDiagnostics(cells);
    last_diagnostic_publish_time_ = now;
  }
}

void BmsNode::publishCells(const std::array<float, 4> & cells)
{
  std_msgs::msg::Float32MultiArray out;
  out.data = {cells[0], cells[1], cells[2], cells[3]};
  pub_cells_->publish(out);

  const float pack_voltage = cells[0] + cells[1] + cells[2] + cells[3];
  const float lowest_cell_voltage = *std::min_element(cells.begin(), cells.end());
  const double usable_range = std::max(0.01, full_voltage_ - error_voltage_);
  const float percentage = static_cast<float>(std::clamp(
    100.0 * (lowest_cell_voltage - error_voltage_) / usable_range, 0.0, 100.0));
  pub_pack_voltage_->publish(std_msgs::msg::Float32().set__data(pack_voltage));
  pub_battery_percent_->publish(std_msgs::msg::Float32().set__data(percentage));
}

void BmsNode::publishDiagnostics(const std::array<float, 4> & cells)
{
  const auto min_it = std::min_element(cells.begin(), cells.end());
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "bms/cell_voltage";
  status.hardware_id = "micon";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "OK";

  if (*min_it < error_voltage_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "cell voltage below error threshold";
  } else if (*min_it < warning_voltage_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "cell voltage below warning threshold";
  }

  for (size_t i = 0; i < cells.size(); ++i) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "cell" + std::to_string(i + 1) + "_voltage";
    std::ostringstream ss;
    ss << cells[i];
    kv.value = ss.str();
    status.values.push_back(kv);
  }

  if (temperature_c_) {
    diagnostic_msgs::msg::KeyValue temperature_kv;
    temperature_kv.key = "temperature_c";
    std::ostringstream temperature_ss;
    temperature_ss << *temperature_c_;
    temperature_kv.value = temperature_ss.str();
    status.values.push_back(temperature_kv);
  }

  diagnostic_msgs::msg::KeyValue min_kv;
  min_kv.key = "min_cell_voltage";
  std::ostringstream min_ss;
  min_ss << *min_it;
  min_kv.value = min_ss.str();
  status.values.push_back(min_kv);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.status.push_back(status);
  pub_diagnostics_->publish(array);
}

}  // namespace bms
}  // namespace njord

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(njord::bms::BmsNode)
