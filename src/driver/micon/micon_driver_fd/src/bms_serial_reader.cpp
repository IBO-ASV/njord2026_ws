#include "micon_driver_fd/bms_serial_reader.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <functional>
#include <sstream>
#include <termios.h>
#include <unistd.h>

namespace micon_driver_fd
{
namespace
{
bool parse_float(const std::string & field, float * value)
{
  char * end = nullptr;
  errno = 0;
  const float parsed = std::strtof(field.c_str(), &end);
  if (end == field.c_str() || errno == ERANGE || std::isinf(parsed)) {return false;}
  while (*end == ' ' || *end == '\t' || *end == '\r') {++end;}
  if (*end != '\0') {return false;}
  *value = parsed;
  return true;
}
}  // namespace

bool parse_bms_csv_line(const std::string & line, BmsTelemetry * telemetry)
{
  if (telemetry == nullptr) {return false;}
  std::stringstream stream(line);
  std::string field;
  if (!std::getline(stream, field, ',')) {return false;}
  for (float & cell : telemetry->cells) {
    if (!std::getline(stream, field, ',') || !parse_float(field, &cell)) {return false;}
  }
  if (!std::getline(stream, field, ',')) {return false;}
  return std::getline(stream, field, ',') && parse_float(field, &telemetry->temperature_c);
}

BmsSerialReader::BmsSerialReader(const rclcpp::NodeOptions & options)
: Node("bms_serial", options)
{
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  const auto bms_topic = declare_parameter<std::string>("bms_topic", "/micon/bms_cells");
  const auto temperature_topic = declare_parameter<std::string>(
    "bms_temperature_topic", "/micon/bms_temperature_c");
  pub_bms_ = create_publisher<std_msgs::msg::Float32MultiArray>(bms_topic, 10);
  pub_bms_temperature_ = create_publisher<std_msgs::msg::Float32>(temperature_topic, 10);
  fd_ = open_serial(serial_port_, baud_);
  timer_ =
    create_wall_timer(std::chrono::milliseconds(50), std::bind(&BmsSerialReader::timer_cb, this));
  RCLCPP_INFO(get_logger(), "bms_serial started, port: %s", serial_port_.c_str());
}

BmsSerialReader::~BmsSerialReader()
{
  if (fd_ >= 0) {close(fd_);}
}

void BmsSerialReader::timer_cb()
{
  if (fd_ < 0) {return;}
  char buffer[256];
  const ssize_t count = read(fd_, buffer, sizeof(buffer));
  if (count < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "serial read failed");
    }
    return;
  }
  if (count == 0) {return;}
  serial_rx_buffer_.append(buffer, static_cast<size_t>(count));
  size_t newline = 0;
  while ((newline = serial_rx_buffer_.find('\n')) != std::string::npos) {
    const std::string line = serial_rx_buffer_.substr(0, newline);
    serial_rx_buffer_.erase(0, newline + 1);
    BmsTelemetry telemetry;
    if (!parse_bms_csv_line(line, &telemetry)) {continue;}
    std_msgs::msg::Float32MultiArray cells;
    cells.data.assign(telemetry.cells.begin(), telemetry.cells.end());
    pub_bms_->publish(cells);
    pub_bms_temperature_->publish(std_msgs::msg::Float32().set__data(telemetry.temperature_c));
  }
  constexpr size_t kMaxPendingLineLength = 4096;
  if (serial_rx_buffer_.size() > kMaxPendingLineLength) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "discarding oversized serial line");
    serial_rx_buffer_.clear();
  }
}

int BmsSerialReader::open_serial(const std::string & device, int baud)
{
  const int fd = open(device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "failed to open serial port %s", device.c_str());
    return -1;
  }
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    RCLCPP_ERROR(get_logger(), "tcgetattr failed");
    return -1;
  }
  cfmakeraw(&tty);
  speed_t speed = B115200;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default:
      close(fd);
      RCLCPP_ERROR(get_logger(), "unsupported baud rate: %d", baud);
      return -1;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    RCLCPP_ERROR(get_logger(), "tcsetattr failed");
    return -1;
  }
  return fd;
}
}  // namespace micon_driver_fd
