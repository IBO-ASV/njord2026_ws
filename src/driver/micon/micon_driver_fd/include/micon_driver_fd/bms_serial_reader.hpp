#ifndef MICON_DRIVER_FD__BMS_SERIAL_READER_HPP_
#define MICON_DRIVER_FD__BMS_SERIAL_READER_HPP_

#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace micon_driver_fd
{

struct BmsTelemetry
{
  std::array<float, 4> cells{};
  float temperature_c{};
};

bool parse_bms_csv_line(const std::string & line, BmsTelemetry * telemetry);

// Read-only driver for the BMS ESP32 serial protocol. It never sends a
// thruster-command frame and deliberately has no safety or relay topics.
class BmsSerialReader : public rclcpp::Node
{
public:
  explicit BmsSerialReader(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BmsSerialReader() override;

private:
  void timer_cb();
  int open_serial(const std::string & device, int baud);

  int fd_{-1};
  std::string serial_port_;
  int baud_{115200};
  std::string serial_rx_buffer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bms_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_bms_temperature_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace micon_driver_fd

#endif  // MICON_DRIVER_FD__BMS_SERIAL_READER_HPP_
