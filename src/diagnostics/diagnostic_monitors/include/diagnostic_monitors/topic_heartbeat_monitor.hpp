#ifndef DIAGNOSTIC_MONITORS__TOPIC_HEARTBEAT_MONITOR_HPP_
#define DIAGNOSTIC_MONITORS__TOPIC_HEARTBEAT_MONITOR_HPP_

#include <deque>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "njord_interfaces/msg/health_signal.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

namespace njord
{
namespace diagnostic_monitors
{

class TopicHeartbeatMonitor : public rclcpp::Node
{
public:
  explicit TopicHeartbeatMonitor(const rclcpp::NodeOptions & options);

private:
  enum class Mode
  {
    RequiredFrequency,
    HeartbeatOnly,
    Optional
  };

  void onMessage(std::shared_ptr<rclcpp::SerializedMessage> message);
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void onTimer();
  njord_interfaces::msg::HealthSignal makeHealthSignal();

  Mode parseMode(const std::string & value) const;
  rclcpp::QoS makeQos() const;
  double measuredFrequency() const;
  double lastMessageAgeSec();
  void pruneSamples();

  diagnostic_updater::Updater updater_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string monitor_name_;
  std::string topic_name_;
  std::string topic_type_;
  std::string mode_text_;
  Mode mode_;

  double expected_frequency_hz_;
  double minimum_frequency_hz_;
  double timeout_sec_;
  double stale_timeout_sec_;
  double diagnostic_period_sec_;
  int window_size_;
  int qos_depth_;
  std::string qos_reliability_;
  std::string diagnostic_name_;
  std::string hardware_id_;
  std::string health_signal_topic_;

  std::deque<rclcpp::Time> samples_;
  rclcpp::Time last_message_time_;
  bool has_message_;
  rclcpp::Publisher<njord_interfaces::msg::HealthSignal>::SharedPtr health_signal_publisher_;
};

}  // namespace diagnostic_monitors
}  // namespace njord

#endif  // DIAGNOSTIC_MONITORS__TOPIC_HEARTBEAT_MONITOR_HPP_
