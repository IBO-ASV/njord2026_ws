#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "runtime_interfaces/action/configure_system.hpp"

namespace njord_behavior
{

// テンプレート引数に先ほど定義したAction型を指定します
class ConfigureSystemNode : public nav2_behavior_tree::BtActionNode<runtime_interfaces::action::ConfigureSystem>
{
public:
  ConfigureSystemNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<runtime_interfaces::action::ConfigureSystem>(xml_tag_name, action_name, conf)
  {
  }

  // XMLのパラメータを読み取り、ROS ActionのGoalにセットする関数
  void on_tick() override
  {
    std::string param, val;
    // XMLの属性 ("parameter" と "value") を取得
    getInput("parameter", param);
    getInput("value", val);

    // ROS ActionのGoalに詰める
    goal_.parameter = param;
    goal_.value = val;
  }

  // XMLで使えるポート(引数)の定義
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("parameter", "Name of the system parameter to configure"),
      BT::InputPort<std::string>("value", "Value to set")
    });
  }
};

} // namespace njord_behavior

// プラグイン登録
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<njord_behavior::ConfigureSystemNode>("ConfigureSystem");
}