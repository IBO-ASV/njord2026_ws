#ifndef DIAGNOSTIC_MONITORS__HEALTH_STATE_AGGREGATOR_HPP_
#define DIAGNOSTIC_MONITORS__HEALTH_STATE_AGGREGATOR_HPP_

#include <map>
#include <string>

#include "njord_interfaces/msg/health_signal.hpp"
#include "njord_interfaces/msg/health_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace njord
{
namespace diagnostic_monitors
{

class HealthStateAggregator : public rclcpp::Node
{
public:
  explicit HealthStateAggregator(const rclcpp::NodeOptions & options);

private:
  struct SignalEntry
  {
    njord_interfaces::msg::HealthSignal signal;
    rclcpp::Time received_at;
  };

  void onSignal(const njord_interfaces::msg::HealthSignal::SharedPtr message);
  void onTimer();
  njord_interfaces::msg::HealthState makeState() const;

  std::map<std::string, SignalEntry> signals_;
  std::string input_topic_;
  std::string output_topic_;
  double signal_timeout_sec_;
  rclcpp::Subscription<njord_interfaces::msg::HealthSignal>::SharedPtr subscription_;
  rclcpp::Publisher<njord_interfaces::msg::HealthState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace diagnostic_monitors
}  // namespace njord

#endif  // DIAGNOSTIC_MONITORS__HEALTH_STATE_AGGREGATOR_HPP_
