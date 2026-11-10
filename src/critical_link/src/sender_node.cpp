#include "critical_link/io.hpp"
#include "critical_link/joy_codec.hpp"
#include "critical_link/operator_codec.hpp"
#include "critical_link/protocol.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "njord_interfaces/msg/operator_command.hpp"
#include "njord_interfaces/msg/operator_response.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"

namespace critical_link
{
namespace
{

uint64_t steady_milliseconds()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

int64_t unix_milliseconds()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

uint64_t make_session_id()
{
  std::random_device random;
  const uint64_t entropy = (static_cast<uint64_t>(random()) << 32U) ^ random();
  return entropy ^ (steady_milliseconds() << 16U) ^ static_cast<uint64_t>(getpid());
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

class CriticalLinkSender : public rclcpp::Node
{
public:
  CriticalLinkSender()
  : Node("critical_link_sender"), session_id_(make_session_id())
  {
    joy_topic_ = declare_parameter<std::string>(
      "joy_input_topic", "/critical_link/input/joy");
    heartbeat_topic_ = declare_parameter<std::string>(
      "heartbeat_input_topic", "/critical_link/input/heartbeat");
    command_topic_ = declare_parameter<std::string>(
      "operator_command_input_topic", "/critical_link/input/operator_command");
    response_topic_ = declare_parameter<std::string>(
      "operator_response_output_topic", "/critical_link/output/operator_response");
    serial_device_ = declare_parameter<std::string>("serial_device", "");
    serial_baud_ = declare_parameter<int>("serial_baud", 921600);
    const auto key = load_shared_key(declare_parameter<std::string>(
          "key_file", "/etc/njord/critical_link.key"));
    if (!key) throw std::runtime_error("critical-link shared key is missing or invalid");
    key_ = *key;

    for (const auto & text : declare_parameter<std::vector<std::string>>(
        "udp_paths", std::vector<std::string>{}))
    {
      const auto spec = parse_udp_sender_spec(text);
      if (!spec) {
        throw std::runtime_error(
                "invalid udp_paths entry; expected name|bind_address|destination_address|port: " + text);
      }
      udp_paths_.push_back(std::make_unique<UdpPath>(*spec));
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Joy::SharedPtr message) {send_joy(*message);});
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      heartbeat_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        send(StreamId::kGroundHeartbeat, {});
      });
    command_sub_ = create_subscription<njord_interfaces::msg::OperatorCommand>(
      command_topic_, 10, [this](const njord_interfaces::msg::OperatorCommand::SharedPtr message) {
        send(StreamId::kOperatorCommand, encode_operator_command(*message));
      });
    response_pub_ = create_publisher<njord_interfaces::msg::OperatorResponse>(response_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    probe_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
          send(StreamId::kLinkProbe, {});
          poll_responses();
          publish_diagnostics();
    });

    RCLCPP_INFO(
      get_logger(), "critical-link sender session=%llu udp_paths=%zu serial=%s",
      static_cast<unsigned long long>(session_id_), udp_paths_.size(),
      serial_device_.empty() ? "disabled" : serial_device_.c_str());
  }

  ~CriticalLinkSender() override
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  struct UdpPath
  {
    explicit UdpPath(UdpSenderSpec input_spec)
    : spec(std::move(input_spec)) {}

    ~UdpPath()
    {
      if (fd >= 0) {
        close(fd);
      }
    }

    bool send_frame(const std::vector<uint8_t> & bytes)
    {
      const auto now = std::chrono::steady_clock::now();
      if (fd < 0) {
        if (now < next_open_attempt) {
          ++failed;
          return false;
        }
        fd = open_udp_sender(spec);
        next_open_attempt = now + std::chrono::seconds(1);
        if (fd < 0) {
          last_errno = errno;
          ++failed;
          return false;
        }
      }
      const ssize_t result = ::send(fd, bytes.data(), bytes.size(), 0);
      if (result == static_cast<ssize_t>(bytes.size())) {
        ++sent;
        return true;
      }
      last_errno = errno;
      ++failed;
      close(fd);
      fd = -1;
      return false;
    }

    UdpSenderSpec spec;
    int fd{-1};
    uint64_t sent{0};
    uint64_t failed{0};
    int last_errno{0};
    std::chrono::steady_clock::time_point next_open_attempt{};
  };

  void send_joy(const sensor_msgs::msg::Joy & joy)
  {
    const auto payload = encode_joy_payload(joy);
    if (!payload) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting Joy with too many fields or non-finite axis");
      return;
    }
    send(StreamId::kJoy, *payload);
  }

  void poll_responses()
  {
    std::array<uint8_t, kMaxFrameSize> buffer{};
    for (const auto & path : udp_paths_) {
      if (path->fd < 0) continue;
      while (true) {
        const auto received = recv(path->fd, buffer.data(), buffer.size(), MSG_DONTWAIT);
        if (received <= 0) break;
        const auto frame = decode_frame(buffer.data(), static_cast<size_t>(received), key_, unix_milliseconds());
        if (!frame || frame->stream != StreamId::kOperatorResponse) continue;
        try {
          if (auto response = decode_operator_response(frame->payload)) response_pub_->publish(*response);
        } catch (const std::exception &) {}
      }
    }
  }

  void send(StreamId stream, std::vector<uint8_t> payload)
  {
    const size_t index = static_cast<size_t>(stream);
    Frame frame;
    frame.stream = stream;
    frame.session_id = session_id_;
    frame.sequence = ++sequences_[index];
    frame.source_unix_ms = unix_milliseconds();
    frame.payload = std::move(payload);
    const auto bytes = encode_frame(frame, key_);

    for (auto & path : udp_paths_) {
      path->send_frame(bytes);
    }
    send_serial(bytes);
    ++generated_[index];
  }

  void send_serial(const std::vector<uint8_t> & bytes)
  {
    if (serial_device_.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(serial_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (serial_fd_ < 0) {
      if (now < serial_next_open_attempt_) {
        ++serial_failed_;
        return;
      }
      serial_fd_ = open_serial_port(serial_device_, serial_baud_);
      serial_next_open_attempt_ = now + std::chrono::seconds(1);
      if (serial_fd_ < 0) {
        serial_last_errno_ = errno;
        ++serial_failed_;
        return;
      }
    }
    if (write_all(serial_fd_, bytes.data(), bytes.size())) {
      ++serial_sent_;
      return;
    }
    serial_last_errno_ = errno;
    ++serial_failed_;
    close(serial_fd_);
    serial_fd_ = -1;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    for (const auto & path : udp_paths_) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "critical_link/sender/" + path->spec.name;
      status.hardware_id = path->spec.bind_address;
      status.level = path->fd >= 0 ? status.OK : status.WARN;
      status.message = path->fd >= 0 ? "connected UDP path" : "UDP path unavailable";
      status.values.push_back(key_value("sent", std::to_string(path->sent)));
      status.values.push_back(key_value("failed", std::to_string(path->failed)));
      status.values.push_back(key_value("last_errno", std::to_string(path->last_errno)));
      array.status.push_back(std::move(status));
    }
    if (!serial_device_.empty()) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "critical_link/sender/espnow_serial";
      status.hardware_id = serial_device_;
      status.level = serial_fd_ >= 0 ? status.OK : status.WARN;
      status.message = serial_fd_ >= 0 ? "serial path open" : "serial path unavailable";
      status.values.push_back(key_value("sent", std::to_string(serial_sent_)));
      status.values.push_back(key_value("failed", std::to_string(serial_failed_)));
      status.values.push_back(key_value("last_errno", std::to_string(serial_last_errno_)));
      array.status.push_back(std::move(status));
    }
    diagnostics_pub_->publish(array);
  }

  std::string joy_topic_;
  std::string heartbeat_topic_;
  std::string command_topic_;
  std::string response_topic_;
  std::string serial_device_;
  int serial_baud_{921600};
  SharedKey key_{};
  uint64_t session_id_{0};
  std::array<uint32_t, static_cast<size_t>(StreamId::kOperatorResponse) + 1U> sequences_{};
  std::array<uint64_t, static_cast<size_t>(StreamId::kOperatorResponse) + 1U> generated_{};
  std::vector<std::unique_ptr<UdpPath>> udp_paths_;
  std::mutex serial_mutex_;
  int serial_fd_{-1};
  uint64_t serial_sent_{0};
  uint64_t serial_failed_{0};
  int serial_last_errno_{0};
  std::chrono::steady_clock::time_point serial_next_open_attempt_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<njord_interfaces::msg::OperatorCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<njord_interfaces::msg::OperatorResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr probe_timer_;
};

}  // namespace critical_link

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<critical_link::CriticalLinkSender>());
  rclcpp::shutdown();
  return 0;
}
