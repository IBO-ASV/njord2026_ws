// Copyright 2026 IBO-ASV

#include "diagnostic_monitors/heartbeat_aggregator.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace njord
{
namespace diagnostic_monitors
{

HeartbeatAggregator::HeartbeatAggregator(const rclcpp::NodeOptions & options)
: Node("heartbeat_aggregator", options),
  updater_(this)
{
  const auto topics = declare_parameter<std::vector<std::string>>(
    "input_topics", std::vector<std::string>{});
  const auto types = declare_parameter<std::vector<std::string>>(
    "input_types", std::vector<std::string>{});
  output_topic_ = declare_parameter<std::string>("output_topic", "");
  input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 2.0);
  startup_grace_sec_ = declare_parameter<double>("startup_grace_sec", 10.0);
  const auto publish_period_sec = declare_parameter<double>("publish_period_sec", 0.5);

  if (topics.empty() || topics.size() != types.size()) {
    throw std::invalid_argument("input_topics and input_types must be non-empty and equal-sized");
  }
  if (output_topic_.empty() || input_timeout_sec_ <= 0.0 || publish_period_sec <= 0.0) {
    throw std::invalid_argument("output_topic and positive timeout/period parameters are required");
  }

  const auto qos = rclcpp::SensorDataQoS();
  inputs_.resize(topics.size());
  for (std::size_t index = 0; index < topics.size(); ++index) {
    auto & input = inputs_[index];
    input.topic = topics[index];
    input.type = types[index];
    input.last_message = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    input.subscription = create_generic_subscription(
      input.topic, input.type, qos,
      [this, index](std::shared_ptr<rclcpp::SerializedMessage>) {onMessage(index);});
  }

  publisher_ = create_publisher<std_msgs::msg::Empty>(output_topic_, 10);
  start_time_ = now();
  updater_.setHardwareID(output_topic_);
  updater_.add("heartbeat_gate" + output_topic_, this, &HeartbeatAggregator::updateDiagnostic);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(publish_period_sec),
    std::bind(&HeartbeatAggregator::onTimer, this));
}

void HeartbeatAggregator::onMessage(std::size_t index)
{
  inputs_[index].last_message = now();
  inputs_[index].received = true;
}

bool HeartbeatAggregator::allInputsHealthy(
  const rclcpp::Time & current, std::string * reason) const
{
  for (const auto & input : inputs_) {
    if (count_publishers(input.topic) == 0U) {
      *reason = "no publisher: " + input.topic;
      return false;
    }
    if (!input.received) {
      *reason = "no message: " + input.topic;
      return false;
    }
    const double age = (current - input.last_message).seconds();
    if (age > input_timeout_sec_) {
      std::ostringstream stream;
      stream << "stale " << input.topic << " age=" << age << "s";
      *reason = stream.str();
      return false;
    }
  }
  *reason = "all inputs healthy";
  return true;
}

void HeartbeatAggregator::onTimer()
{
  std::string reason;
  if (allInputsHealthy(now(), &reason)) {
    publisher_->publish(std_msgs::msg::Empty{});
  }
  updater_.force_update();
}

void HeartbeatAggregator::updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto current = now();
  std::string reason;
  const bool healthy = allInputsHealthy(current, &reason);
  const bool in_grace = (current - start_time_).seconds() <= startup_grace_sec_;
  const int level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    in_grace ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat.summary(level, reason);
  stat.add("output_topic", output_topic_);
  stat.add("input_count", static_cast<int>(inputs_.size()));
  for (const auto & input : inputs_) {
    stat.add(input.topic + ".publishers", static_cast<int>(count_publishers(input.topic)));
    stat.add(
      input.topic + ".age_sec",
      input.received ? (current - input.last_message).seconds() : -1.0);
  }
}

}  // namespace diagnostic_monitors
}  // namespace njord

RCLCPP_COMPONENTS_REGISTER_NODE(njord::diagnostic_monitors::HeartbeatAggregator)
