#include "parameter_gateway/node.hpp"

namespace njord_control
{

ParameterGatewayNode::ParameterGatewayNode(const rclcpp::NodeOptions & options)
: Node("parameter_gateway", options)
{
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<ConfigureSystem>(
    this,
    "configure_system",
    std::bind(&ParameterGatewayNode::handle_goal, this, _1, _2),
    std::bind(&ParameterGatewayNode::handle_cancel, this, _1),
    std::bind(&ParameterGatewayNode::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "System Manager (C++) is ready.");
}

rclcpp_action::GoalResponse ParameterGatewayNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ConfigureSystem::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request: param='%s', value='%s'", 
    goal->parameter.c_str(), goal->value.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ParameterGatewayNode::handle_cancel(
  const std::shared_ptr<GoalHandleConfigureSystem> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ParameterGatewayNode::handle_accepted(const std::shared_ptr<GoalHandleConfigureSystem> goal_handle)
{
  // メインスレッドをブロックしないよう、別スレッドで実行してDetachする
  std::thread{std::bind(&ParameterGatewayNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ParameterGatewayNode::execute(const std::shared_ptr<GoalHandleConfigureSystem> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ConfigureSystem::Result>();
  
  std::string param_key = goal->parameter;
  std::string value_str = goal->value;
  bool success = false;
  std::string message;

  try {
    // === パラメータに応じた分岐処理 ===
    
    if (param_key == "avoidance") {
      bool val = parse_bool(value_str);
      // Nav2 Local Costmapの障害物回避 (bool)
      success = set_remote_parameter(
        "/local_costmap/local_costmap", 
        "obstacle_layer.enabled", 
        val);
      message = "Set avoidance to " + value_str;

    } else if (param_key == "virtual_wall") {
      bool val = parse_bool(value_str);
      // YOLOノードの仮想壁生成 (bool)
      success = set_remote_parameter(
        "/yolo_detector", 
        "enable_virtual_wall", 
        val);
      message = "Set virtual_wall to " + value_str;

    } else {
      message = "Unknown parameter key: " + param_key;
      RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
      result->success = false;
      result->message = message;
      goal_handle->abort(result);
      return;
    }

  } catch (const std::exception & e) {
    message = std::string("Exception: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    success = false;
  }

  result->success = success;
  result->message = success ? message : ("Failed: " + message);

  if (success) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", message.c_str());
  } else {
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Goal aborted: %s", message.c_str());
  }
}

bool ParameterGatewayNode::parse_bool(const std::string & s) {
  std::string low = s;
  std::transform(low.begin(), low.end(), low.begin(), ::tolower);
  return (low == "true" || low == "1" || low == "on");
}

} // namespace njord_control

// コンポーネント登録
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(njord_control::ParameterGatewayNode)