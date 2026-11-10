#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "njord_interfaces/msg/nav2_runtime_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace njord::diagnostic_monitors
{
class AutonomySupervisor : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;

  explicit AutonomySupervisor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("autonomy_supervisor", options), updater_(this)
  {
    action_name_ = declare_parameter<std::string>(
      "action_name", "/navigate_through_poses");
    waypoint_topic_ = declare_parameter<std::string>(
      "waypoint_topic", "/task/plan");
    action_status_topic_ = declare_parameter<std::string>(
      "action_status_topic", action_name_ + "/_action/status");
    update_period_sec_ = declare_parameter<double>("update_period_sec", 0.2);
    waypoint_timeout_sec_ = declare_parameter<double>("waypoint_timeout_sec", 5.0);
    task2_path_timeout_sec_ = declare_parameter<double>("task2_path_timeout_sec", 2.0);

    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, action_name_);
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
      waypoint_topic_, rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr message) {
        waypoint_count_ = message->poses.size();
        last_waypoint_ = now();
      });
    task2_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path_pruned", 10,
      [this](nav_msgs::msg::Path::SharedPtr message) {
        if (message->poses.size() >= 2) {
          last_task2_path_ = now();
        }
      });
    action_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      action_status_topic_, 10,
      [this](action_msgs::msg::GoalStatusArray::SharedPtr message) {
        active_goals_ = 0;
        last_goal_status_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        for (const auto & status : message->status_list) {
          last_goal_status_ = status.status;
          if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
            status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
            status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
          {
            ++active_goals_;
          }
        }
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/operating_mode", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr message) {mode_ = message->data;});
    runtime_sub_ = create_subscription<njord_interfaces::msg::Nav2RuntimeStatus>(
      "/runtime/nav2/status", rclcpp::QoS(1).transient_local(),
      [this](njord_interfaces::msg::Nav2RuntimeStatus::SharedPtr message) {
        active_profile_ = message->active_profile;
      });

    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/autonomy/ready", 10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("/heartbeat/autonomy", 10);
    updater_.setHardwareID("autonomy_supervisor");
    updater_.add("Autonomy Supervisor", this, &AutonomySupervisor::updateDiagnostic);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(update_period_sec_)),
      std::bind(&AutonomySupervisor::update, this));
  }

private:
  void update()
  {
    action_server_ready_ = action_client_->wait_for_action_server(std::chrono::seconds(0));
    follow_path_server_ready_ = follow_path_client_->wait_for_action_server(std::chrono::seconds(0));
    const bool task2_path_fresh = last_task2_path_.nanoseconds() != 0 &&
      (now() - last_task2_path_).seconds() <= task2_path_timeout_sec_;
    const bool ready = active_profile_ == "task2" ?
      follow_path_server_ready_ && task2_path_fresh : action_server_ready_;
    ready_pub_->publish(std_msgs::msg::Bool().set__data(ready));
    heartbeat_pub_->publish(std_msgs::msg::Empty{});
    updater_.force_update();
  }

  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const auto age = (now() - last_waypoint_).seconds();
    status.add("action_name", action_name_);
    status.add("action_server_ready", action_server_ready_);
    status.add("active_nav2_profile", active_profile_);
    status.add("follow_path_server_ready", follow_path_server_ready_);
    status.add("waypoint_topic", waypoint_topic_);
    status.add("waypoint_count", static_cast<int>(waypoint_count_));
    status.add("waypoint_age_sec", age);
    status.add("active_goals", static_cast<int>(active_goals_));
    status.add("operating_mode", mode_);
    if (active_profile_ == "task2" && (!follow_path_server_ready_ || last_task2_path_.nanoseconds() == 0 ||
      (now() - last_task2_path_).seconds() > task2_path_timeout_sec_))
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Task 2 FollowPath action server or fresh path is unavailable");
    } else if (active_profile_ != "task2" && !action_server_ready_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "NavigateThroughPoses action server is unavailable");
    } else if (last_waypoint_.nanoseconds() == 0 ||
      age > waypoint_timeout_sec_ || waypoint_count_ == 0)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "No fresh non-empty waypoint plan is available");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Autonomy services are available");
    }
  }

  std::string action_name_;
  std::string waypoint_topic_;
  std::string action_status_topic_;
  std::string mode_{"manual"};
  std::string active_profile_;
  double update_period_sec_{0.2};
  double waypoint_timeout_sec_{5.0};
  double task2_path_timeout_sec_{2.0};
  bool action_server_ready_{false};
  bool follow_path_server_ready_{false};
  std::size_t waypoint_count_{0};
  std::size_t active_goals_{0};
  uint8_t last_goal_status_{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
  rclcpp::Time last_waypoint_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_task2_path_{0, 0, RCL_ROS_TIME};
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr task2_path_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<njord_interfaces::msg::Nav2RuntimeStatus>::SharedPtr runtime_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace njord::diagnostic_monitors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<njord::diagnostic_monitors::AutonomySupervisor>());
  rclcpp::shutdown();
  return 0;
}
