#pragma once

#include <memory>
#include <string>
#include <thread>
#include <algorithm>
#include <vector>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "runtime_interfaces/action/configure_system.hpp"

namespace njord_control
{

using ConfigureSystem = runtime_interfaces::action::ConfigureSystem;
using GoalHandleConfigureSystem = rclcpp_action::ServerGoalHandle<ConfigureSystem>;

class ParameterGatewayNode : public rclcpp::Node
{
public:
  explicit ParameterGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ParameterGatewayNode() = default;

private:
  rclcpp_action::Server<ConfigureSystem>::SharedPtr action_server_;

  // --- Action Server Callbacks ---
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ConfigureSystem::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleConfigureSystem> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleConfigureSystem> goal_handle);

  // --- Execution Logic ---
  void execute(const std::shared_ptr<GoalHandleConfigureSystem> goal_handle);

  // --- Helpers ---
  bool parse_bool(const std::string & s);

  // テンプレート関数はヘッダーに実装を書く必要があります
  template <typename T>
  bool set_remote_parameter(
    const std::string & node_name, 
    const std::string & param_name, 
    const T & value)
  {
    auto client = this->create_client<rcl_interfaces::srv::SetParameters>(
      node_name + "/set_parameters");

    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service %s/set_parameters not available", node_name.c_str());
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter p;
    p.name = param_name;
    
    if constexpr (std::is_same_v<T, bool>) {
        p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        p.value.bool_value = value;
    } else if constexpr (std::is_same_v<T, double>) {
        p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        p.value.double_value = value;
    } else if constexpr (std::is_same_v<T, int>) {
        p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        p.value.integer_value = value;
    } else {
        p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        p.value.string_value = std::to_string(value);
    }

    request->parameters.push_back(p);

    auto future = client->async_send_request(request);
    
    // スレッドコンテキスト内なのでwait_forで待機しても安全
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Service call timed out");
      return false;
    }

    auto response = future.get();
    if (response->results.empty()) return false;
    
    if (!response->results[0].successful) {
      RCLCPP_ERROR(this->get_logger(), "Param set failed: %s", response->results[0].reason.c_str());
      return false;
    }

    return true;
  }
};

} // namespace njord_control
