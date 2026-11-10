#include "critical_link/io.hpp"
#include "critical_link/joy_codec.hpp"
#include "critical_link/operator_codec.hpp"
#include "critical_link/protocol.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "njord_interfaces/msg/operator_command.hpp"
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

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

class CriticalLinkReceiver : public rclcpp::Node
{
public:
  CriticalLinkReceiver()
  : Node("critical_link_receiver")
  {
    joy_output_topic_ = declare_parameter<std::string>("joy_output_topic", "/joy");
    heartbeat_output_topic_ = declare_parameter<std::string>(
      "heartbeat_output_topic", "/heartbeat/ground_station");
    command_output_topic_ = declare_parameter<std::string>(
      "operator_command_output_topic", "/critical_link/operator_command");
    serial_device_ = declare_parameter<std::string>("serial_device", "");
    serial_baud_ = declare_parameter<int>("serial_baud", 921600);
    stale_after_sec_ = declare_parameter<double>("diagnostic_stale_after_sec", 3.0);
    max_frame_age_ms_ = declare_parameter<int64_t>("max_frame_age_ms", 2000);
    future_tolerance_ms_ = declare_parameter<int64_t>("future_tolerance_ms", 1000);
    const auto key = load_shared_key(declare_parameter<std::string>(
          "key_file", "/etc/njord/critical_link.key"));
    if (!key) throw std::runtime_error("critical-link shared key is missing or invalid");
    key_ = *key;

    for (const auto & text : declare_parameter<std::vector<std::string>>(
        "udp_paths", std::vector<std::string>{}))
    {
      const auto spec = parse_udp_receiver_spec(text);
      if (!spec) {
        throw std::runtime_error(
                "invalid udp_paths entry; expected name|bind_ipv4|port: " + text);
      }
      udp_paths_.push_back(std::make_unique<ReceivePath>(*spec));
    }

    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(joy_output_topic_, 10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>(heartbeat_output_topic_, 10);
    command_pub_ = create_publisher<njord_interfaces::msg::OperatorCommand>(command_output_topic_, 10);
    response_sub_ = create_subscription<njord_interfaces::msg::OperatorResponse>(
      "/critical_link/operator_response", 10,
      [this](const njord_interfaces::msg::OperatorResponse::SharedPtr response) { send_response(*response); });
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    for (auto & path : udp_paths_) {
      path->thread = std::thread([this, path_ptr = path.get()]() {udp_loop(*path_ptr);});
    }
    if (!serial_device_.empty()) {
      serial_thread_ = std::thread([this]() {serial_loop();});
    }
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(), "critical-link receiver udp_paths=%zu serial=%s",
      udp_paths_.size(), serial_device_.empty() ? "disabled" : serial_device_.c_str());
  }

  ~CriticalLinkReceiver() override
  {
    running_ = false;
    for (auto & path : udp_paths_) {
      if (path->thread.joinable()) {
        path->thread.join();
      }
    }
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }
  }

private:
  struct ReceivePath
  {
    explicit ReceivePath(UdpReceiverSpec input_spec)
    : spec(std::move(input_spec)) {}

    UdpReceiverSpec spec;
    std::thread thread;
    std::atomic<bool> open{false};
    int fd{-1};
    std::atomic<uint64_t> received{0};
    std::atomic<uint64_t> invalid{0};
    std::atomic<uint64_t> last_receive_ms{0};
    std::atomic<int> last_errno{0};
    sockaddr_in last_peer{};
    bool has_peer{false};
  };

  void udp_loop(ReceivePath & path)
  {
    std::array<uint8_t, kMaxFrameSize + 1U> buffer{};
    while (running_) {
      const int fd = open_udp_receiver(path.spec);
      if (fd < 0) {
        path.last_errno = errno;
        path.open = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      path.fd = fd;
      path.open = true;
      while (running_) {
        sockaddr_in peer{};
        socklen_t peer_size = sizeof(peer);
        const ssize_t size = recvfrom(fd, buffer.data(), buffer.size(), 0,
          reinterpret_cast<sockaddr *>(&peer), &peer_size);
        if (size > 0) {
          path.last_peer = peer;
          path.has_peer = true;
          path.last_receive_ms = steady_milliseconds();
          ++path.received;
          if (!handle_datagram(buffer.data(), static_cast<size_t>(size))) {
            ++path.invalid;
          }
        } else if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          path.last_errno = errno;
          break;
        }
      }
      path.open = false;
      path.fd = -1;
      close(fd);
    }
  }

  void serial_loop()
  {
    FrameStreamDecoder decoder;
    std::array<uint8_t, 512> buffer{};
    while (running_) {
      const int fd = open_serial_port(serial_device_, serial_baud_);
      if (fd < 0) {
        serial_last_errno_ = errno;
        serial_open_ = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      serial_open_ = true;
      decoder.clear();
      while (running_) {
        const ssize_t size = read(fd, buffer.data(), buffer.size());
        if (size > 0) {
          serial_last_receive_ms_ = steady_milliseconds();
          const auto frames = decoder.push(buffer.data(), static_cast<size_t>(size), key_, unix_milliseconds(), max_frame_age_ms_, future_tolerance_ms_);
          for (const auto & frame : frames) {
            ++serial_received_;
            handle_frame(frame);
          }
        } else if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          serial_last_errno_ = errno;
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
      serial_open_ = false;
      close(fd);
    }
  }

  bool handle_datagram(const uint8_t * data, size_t size)
  {
    const auto frame = decode_frame(data, size, key_, unix_milliseconds(), max_frame_age_ms_, future_tolerance_ms_);
    if (!frame) {
      return false;
    }
    handle_frame(*frame);
    return true;
  }

  void handle_frame(const Frame & frame)
  {
    std::optional<sensor_msgs::msg::Joy> joy;
    std::optional<njord_interfaces::msg::OperatorCommand> command;
    if (frame.stream == StreamId::kJoy) {
      joy = decode_joy_payload(frame.payload.data(), frame.payload.size());
      if (!joy) {
        ++invalid_payload_;
        return;
      }
    } else if (frame.stream == StreamId::kOperatorCommand) {
      command = decode_operator_command(frame.payload);
      if (!command) { ++invalid_payload_; return; }
    } else if (!frame.payload.empty()) {
      ++invalid_payload_;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      if (!gate_.accept(frame.session_id, frame.stream, frame.sequence)) {
        ++duplicates_or_old_;
        return;
      }
    }

    if (frame.stream == StreamId::kJoy) {
      joy->header.stamp = now();
      joy->header.frame_id = "critical_link";
      joy_pub_->publish(*joy);
      last_joy_ms_ = steady_milliseconds();
      ++accepted_joy_;
    } else if (frame.stream == StreamId::kGroundHeartbeat) {
      heartbeat_pub_->publish(std_msgs::msg::Empty{});
      last_heartbeat_ms_ = steady_milliseconds();
      ++accepted_heartbeat_;
    } else if (frame.stream == StreamId::kLinkProbe) {
      last_probe_ms_ = steady_milliseconds();
    } else if (frame.stream == StreamId::kOperatorCommand && command) {
      command_pub_->publish(*command);
    }
  }

  void send_response(const njord_interfaces::msg::OperatorResponse & response)
  {
    Frame frame;
    frame.stream = StreamId::kOperatorResponse;
    frame.session_id = response.request_id;
    frame.sequence = ++response_sequence_;
    frame.source_unix_ms = unix_milliseconds();
    frame.payload = encode_operator_response(response);
    const auto bytes = encode_frame(frame, key_);
    for (const auto & path : udp_paths_) {
      const sockaddr_in peer{path->last_peer};
      if (path->has_peer) sendto(path->fd, bytes.data(), bytes.size(), 0,
        reinterpret_cast<const sockaddr *>(&peer), sizeof(peer));
    }
  }

  diagnostic_msgs::msg::DiagnosticStatus make_path_status(
    const std::string & name, const std::string & hardware_id, bool open,
    uint64_t received, uint64_t invalid, uint64_t last_receive_ms, int last_errno) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "critical_link/receiver/" + name;
    status.hardware_id = hardware_id;
    const uint64_t now_ms = steady_milliseconds();
    const bool stale = last_receive_ms == 0U ||
      now_ms - last_receive_ms > static_cast<uint64_t>(stale_after_sec_ * 1000.0);
    status.level = !open ? status.WARN : stale ? status.WARN : status.OK;
    status.message = !open ? "path unavailable" : stale ? "path has no recent frames" : "receiving";
    status.values.push_back(key_value("received", std::to_string(received)));
    status.values.push_back(key_value("invalid", std::to_string(invalid)));
    status.values.push_back(key_value("last_errno", std::to_string(last_errno)));
    status.values.push_back(key_value("stale", stale ? "true" : "false"));
    return status;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    for (const auto & path : udp_paths_) {
      array.status.push_back(make_path_status(
          path->spec.name, path->spec.bind_address, path->open,
          path->received, path->invalid, path->last_receive_ms, path->last_errno));
    }
    if (!serial_device_.empty()) {
      array.status.push_back(make_path_status(
          "espnow_serial", serial_device_, serial_open_, serial_received_, 0,
          serial_last_receive_ms_, serial_last_errno_));
    }

    diagnostic_msgs::msg::DiagnosticStatus aggregate;
    aggregate.name = "critical_link/receiver/aggregate";
    aggregate.hardware_id = "critical_link";
    const uint64_t now_ms = steady_milliseconds();
    const bool joy_stale = last_joy_ms_ == 0U ||
      now_ms - last_joy_ms_ > static_cast<uint64_t>(stale_after_sec_ * 1000.0);
    aggregate.level = joy_stale ? aggregate.WARN : aggregate.OK;
    aggregate.message = joy_stale ? "no recent accepted Joy" : "accepted Joy is current";
    aggregate.values.push_back(key_value("accepted_joy", std::to_string(accepted_joy_)));
    aggregate.values.push_back(
      key_value("accepted_heartbeat", std::to_string(accepted_heartbeat_)));
    aggregate.values.push_back(
      key_value("duplicate_or_old", std::to_string(duplicates_or_old_)));
    aggregate.values.push_back(key_value("invalid_payload", std::to_string(invalid_payload_)));
    array.status.push_back(std::move(aggregate));
    diagnostics_pub_->publish(array);
  }

  std::string joy_output_topic_;
  std::string heartbeat_output_topic_;
  std::string command_output_topic_;
  std::string serial_device_;
  int serial_baud_{921600};
  double stale_after_sec_{3.0};
  int64_t max_frame_age_ms_{2000};
  int64_t future_tolerance_ms_{1000};
  SharedKey key_{};
  std::atomic<bool> running_{true};
  std::vector<std::unique_ptr<ReceivePath>> udp_paths_;
  std::thread serial_thread_;
  std::atomic<bool> serial_open_{false};
  std::atomic<uint64_t> serial_received_{0};
  std::atomic<uint64_t> serial_last_receive_ms_{0};
  std::atomic<int> serial_last_errno_{0};
  std::mutex gate_mutex_;
  SequenceGate gate_;
  std::atomic<uint64_t> accepted_joy_{0};
  std::atomic<uint64_t> accepted_heartbeat_{0};
  std::atomic<uint64_t> duplicates_or_old_{0};
  std::atomic<uint64_t> invalid_payload_{0};
  std::atomic<uint64_t> last_joy_ms_{0};
  std::atomic<uint64_t> last_heartbeat_ms_{0};
  std::atomic<uint64_t> last_probe_ms_{0};
  uint32_t response_sequence_{0};
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<njord_interfaces::msg::OperatorCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<njord_interfaces::msg::OperatorResponse>::SharedPtr response_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace critical_link

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<critical_link::CriticalLinkReceiver>());
  rclcpp::shutdown();
  return 0;
}
