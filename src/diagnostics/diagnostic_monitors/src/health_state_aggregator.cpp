#include "diagnostic_monitors/health_state_aggregator.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "rclcpp_components/register_node_macro.hpp"

namespace njord
{
namespace diagnostic_monitors
{

HealthStateAggregator::HealthStateAggregator(const rclcpp::NodeOptions & options)
: Node("health_state_aggregator", options),
  signal_timeout_sec_(10.0)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/health/signals");
  output_topic_ = declare_parameter<std::string>("output_topic", "/health/state");
  signal_timeout_sec_ = declare_parameter<double>("signal_timeout_sec", 10.0);
  const auto publish_period_sec = declare_parameter<double>("publish_period_sec", 1.0);
  if (input_topic_.empty() || output_topic_.empty() || signal_timeout_sec_ <= 0.0 ||
    publish_period_sec <= 0.0)
  {
    throw std::invalid_argument(
      "input_topic/output_topic and positive timeout/period parameters are required");
  }

  subscription_ = create_subscription<njord_interfaces::msg::HealthSignal>(
    input_topic_, rclcpp::QoS(100).transient_local(),
    std::bind(&HealthStateAggregator::onSignal, this, std::placeholders::_1));
  publisher_ = create_publisher<njord_interfaces::msg::HealthState>(
    output_topic_, rclcpp::QoS(10).transient_local());
  timer_ = create_wall_timer(
    std::chrono::duration<double>(publish_period_sec),
    std::bind(&HealthStateAggregator::onTimer, this));
}

void HealthStateAggregator::onSignal(
  const njord_interfaces::msg::HealthSignal::SharedPtr message)
{
  if (message->name.empty()) {
    RCLCPP_WARN(get_logger(), "Ignoring health signal with an empty name");
    return;
  }
  signals_[message->name] = SignalEntry{*message, now()};
}

njord_interfaces::msg::HealthState HealthStateAggregator::makeState() const
{
  auto state = njord_interfaces::msg::HealthState{};
  const auto current = now();
  const auto current_ns = current.nanoseconds();
  state.stamp.sec = static_cast<int32_t>(current_ns / 1000000000LL);
  state.stamp.nanosec = static_cast<uint32_t>(current_ns % 1000000000LL);
  state.summary_state = njord_interfaces::msg::HealthState::UNKNOWN;
  state.message = "no health signals received";

  bool has_unknown = false;
  bool has_degraded = false;
  bool has_stale = false;
  bool has_error = false;
  for (const auto & [name, entry] : signals_) {
    auto signal = entry.signal;
    const double received_age = (current - entry.received_at).seconds();
    if (received_age > signal_timeout_sec_ &&
      signal.state != njord_interfaces::msg::HealthSignal::DISABLED)
    {
      signal.state = njord_interfaces::msg::HealthSignal::STALE;
      signal.message = "monitor update is stale";
      signal.age_sec = static_cast<float>(std::max(received_age, 0.0));
    }
    state.signals.push_back(signal);
    has_unknown = has_unknown || signal.state == njord_interfaces::msg::HealthSignal::UNKNOWN;
    has_degraded = has_degraded || signal.state == njord_interfaces::msg::HealthSignal::DEGRADED;
    has_stale = has_stale || signal.state == njord_interfaces::msg::HealthSignal::STALE;
    has_error = has_error || signal.state == njord_interfaces::msg::HealthSignal::ERROR;
  }

  if (has_error) {
    state.summary_state = njord_interfaces::msg::HealthState::ERROR;
    state.message = "one or more health signals are in ERROR";
  } else if (has_stale) {
    state.summary_state = njord_interfaces::msg::HealthState::STALE;
    state.message = "one or more health signals are STALE";
  } else if (has_degraded) {
    state.summary_state = njord_interfaces::msg::HealthState::DEGRADED;
    state.message = "one or more health signals are DEGRADED";
  } else if (has_unknown) {
    state.summary_state = njord_interfaces::msg::HealthState::UNKNOWN;
    state.message = "one or more health signals are UNKNOWN";
  } else if (!signals_.empty()) {
    state.summary_state = njord_interfaces::msg::HealthState::OK;
    state.message = "all health signals are OK or DISABLED";
  }
  return state;
}

void HealthStateAggregator::onTimer()
{
  publisher_->publish(makeState());
}

}  // namespace diagnostic_monitors
}  // namespace njord

RCLCPP_COMPONENTS_REGISTER_NODE(njord::diagnostic_monitors::HealthStateAggregator)
