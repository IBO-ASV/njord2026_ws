// Copyright 2026 IBO-ASV

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace njord
{
namespace diagnostic_monitors
{

class HeartbeatAggregator : public rclcpp::Node
{
public:
  explicit HeartbeatAggregator(const rclcpp::NodeOptions & options);

private:
  struct InputState
  {
    std::string topic;
    std::string type;
    rclcpp::Time last_message;
    bool received{false};
    std::shared_ptr<rclcpp::GenericSubscription> subscription;
  };

  void onMessage(std::size_t index);
  void onTimer();
  bool allInputsHealthy(const rclcpp::Time & current, std::string * reason) const;
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::vector<InputState> inputs_;
  std::string output_topic_;
  double input_timeout_sec_;
  double startup_grace_sec_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
};

}  // namespace diagnostic_monitors
}  // namespace njord
