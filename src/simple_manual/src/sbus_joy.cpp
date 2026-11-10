#include "simple_manual/sbus_joy.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace simple_manual
{

bool parse_sbus_line(std::string_view line, SbusFrame & frame)
{
  std::array<int, 22> values{};
  size_t offset = 0;
  for (auto & value : values) {
    const size_t end = line.find_first_of(",\r", offset);
    const auto token = line.substr(offset, end == std::string_view::npos ? end : end - offset);
    const auto result = std::from_chars(token.data(), token.data() + token.size(), value);
    if (result.ec != std::errc{} || result.ptr != token.data() + token.size()) {
      return false;
    }
    if (end == std::string_view::npos) {
      if (&value != &values.back()) {
        return false;
      }
      offset = line.size();
    } else {
      offset = end + 1;
    }
  }
  if (offset != line.size()) {
    return false;
  }
  std::copy_n(values.begin() + 2, frame.channels.size(), frame.channels.begin());
  frame.lost_frame = values[20] != 0;
  frame.failsafe = values[21] != 0;
  return true;
}

sensor_msgs::msg::Joy sbus_to_joy(
  const SbusFrame & frame, int minimum, int center, int maximum, int button_threshold,
  int failsafe_button)
{
  sensor_msgs::msg::Joy joy;
  joy.axes.resize(frame.channels.size());
  joy.buttons.resize(frame.channels.size());
  for (size_t i = 0; i < frame.channels.size(); ++i) {
    const double denominator = frame.channels[i] >= center ? maximum - center : center - minimum;
    joy.axes[i] = denominator > 0 ? static_cast<float>(std::clamp(
        (frame.channels[i] - center) / denominator, -1.0, 1.0)) : 0.0F;
    joy.buttons[i] = frame.channels[i] >= button_threshold;
  }
  if (frame.lost_frame || frame.failsafe) {
    std::fill(joy.axes.begin(), joy.axes.end(), 0.0F);
    std::fill(joy.buttons.begin(), joy.buttons.end(), 0);
    if (failsafe_button >= 0 && static_cast<size_t>(failsafe_button) < joy.buttons.size()) {
      joy.buttons[failsafe_button] = 1;
    }
  }
  return joy;
}

namespace
{

speed_t baud_flag(int baud)
{
  return baud == 230400 ? B230400 : 0;
}

int open_serial_port(const std::string & device, int baud)
{
  const auto speed = baud_flag(baud);
  if (speed == 0) {
    errno = EINVAL;
    return -1;
  }
  const int fd = open(device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    return -1;
  }
  termios options{};
  if (tcgetattr(fd, &options) != 0) {
    close(fd);
    return -1;
  }
  cfmakeraw(&options);
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~(CSTOPB | CRTSCTS);
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    close(fd);
    return -1;
  }
  tcflush(fd, TCIFLUSH);
  return fd;
}

}  // namespace

class SbusJoyNode : public rclcpp::Node
{
public:
  SbusJoyNode()
  : Node("sbus_joy")
  {
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 230400);
    minimum_ = declare_parameter<int>("minimum", 172);
    center_ = declare_parameter<int>("center", 992);
    maximum_ = declare_parameter<int>("maximum", 1811);
    button_threshold_ = declare_parameter<int>("button_threshold", 1200);
    failsafe_button_ = declare_parameter<int>("failsafe_button", 0);
    publisher_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    reader_ = std::thread([this] {read_loop();});
  }

  ~SbusJoyNode() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();
    }
  }

private:
  void read_loop()
  {
    while (rclcpp::ok() && running_) {
      const int fd = open_serial_port(device_, baud_);
      if (fd < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot open %s: %s",
          device_.c_str(), strerror(errno));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      RCLCPP_INFO(get_logger(), "Reading SBUS receiver from %s", device_.c_str());
      std::string buffer;
      std::array<char, 256> bytes{};
      while (rclcpp::ok() && running_) {
        const ssize_t count = read(fd, bytes.data(), bytes.size());
        if (count > 0) {
          buffer.append(bytes.data(), static_cast<size_t>(count));
          size_t newline = 0;
          while ((newline = buffer.find('\n')) != std::string::npos) {
            SbusFrame frame;
            if (parse_sbus_line(std::string_view(buffer).substr(0, newline), frame)) {
              publisher_->publish(sbus_to_joy(
                  frame, minimum_, center_, maximum_, button_threshold_, failsafe_button_));
            }
            buffer.erase(0, newline + 1);
          }
          if (buffer.size() > 256) {
            buffer.clear();
          }
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
      close(fd);
    }
  }

  std::string device_;
  int baud_{};
  int minimum_{};
  int center_{};
  int maximum_{};
  int button_threshold_{};
  int failsafe_button_{};
  std::atomic<bool> running_{true};
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  std::thread reader_;
};

}  // namespace simple_manual

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_manual::SbusJoyNode>());
  rclcpp::shutdown();
  return 0;
}
