#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace robot
{

class PassThroughGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    const auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("goal checker lifecycle node expired");
    }
    plugin_name_ = plugin_name;
    node->declare_parameter(plugin_name_ + ".xy_goal_tolerance", 0.5);
    node->declare_parameter(plugin_name_ + ".yaw_goal_tolerance", 0.5);
    node->declare_parameter(plugin_name_ + ".exit_margin_m", 0.05);
    xy_tolerance_ = node->get_parameter(plugin_name_ + ".xy_goal_tolerance").as_double();
    yaw_tolerance_ = node->get_parameter(plugin_name_ + ".yaw_goal_tolerance").as_double();
    exit_margin_ = node->get_parameter(plugin_name_ + ".exit_margin_m").as_double();
    if (xy_tolerance_ <= 0.0 || yaw_tolerance_ < 0.0 || exit_margin_ < 0.0) {
      throw std::runtime_error("goal checker tolerances must be non-negative and xy positive");
    }
    reset();
  }

  void reset() override
  {
    entered_radius_ = false;
    last_goal_x_ = std::numeric_limits<double>::quiet_NaN();
    last_goal_y_ = std::numeric_limits<double>::quiet_NaN();
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    if (!std::isfinite(last_goal_x_) || !std::isfinite(last_goal_y_) ||
      goal_pose.position.x != last_goal_x_ || goal_pose.position.y != last_goal_y_)
    {
      entered_radius_ = false;
      last_goal_x_ = goal_pose.position.x;
      last_goal_y_ = goal_pose.position.y;
    }
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    const double distance = std::hypot(dx, dy);
    const bool inside = distance <= xy_tolerance_;
    if (inside) {
      entered_radius_ = true;
      if (std::abs(shortest_angle(yaw(query_pose), yaw(goal_pose))) <= yaw_tolerance_) {
        return true;
      }
    }
    // Do not command a surface vessel back to a waypoint it has already
    // crossed only because its yaw settled after the crossing.
    return entered_radius_ && distance > xy_tolerance_ + exit_margin_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & velocity_tolerance) override
  {
    pose_tolerance.position.x = xy_tolerance_;
    pose_tolerance.position.y = xy_tolerance_;
    pose_tolerance.position.z = 0.0;
    pose_tolerance.orientation.w = yaw_tolerance_;
    velocity_tolerance.linear.x = std::numeric_limits<double>::lowest();
    velocity_tolerance.linear.y = std::numeric_limits<double>::lowest();
    velocity_tolerance.linear.z = std::numeric_limits<double>::lowest();
    velocity_tolerance.angular.x = std::numeric_limits<double>::lowest();
    velocity_tolerance.angular.y = std::numeric_limits<double>::lowest();
    velocity_tolerance.angular.z = std::numeric_limits<double>::lowest();
    return true;
  }

private:
  static double yaw(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double shortest_angle(double from, double to)
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  std::string plugin_name_;
  double xy_tolerance_{0.5};
  double yaw_tolerance_{0.5};
  double exit_margin_{0.05};
  bool entered_radius_{false};
  double last_goal_x_{std::numeric_limits<double>::quiet_NaN()};
  double last_goal_y_{std::numeric_limits<double>::quiet_NaN()};
};

class SelectiveHeadingGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    const auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("goal checker lifecycle node expired");
    }
    node->declare_parameter(plugin_name + ".xy_goal_tolerance", 0.5);
    node->declare_parameter(plugin_name + ".position_only_xy_goal_tolerance", 1.5);
    node->declare_parameter(plugin_name + ".yaw_goal_tolerance", 0.5);
    node->declare_parameter(plugin_name + ".heading_required_goal_xs", std::vector<double>{});
    node->declare_parameter(plugin_name + ".heading_required_goal_ys", std::vector<double>{});
    node->declare_parameter(plugin_name + ".heading_required_goal_tolerance", 0.2);
    xy_tolerance_ = node->get_parameter(plugin_name + ".xy_goal_tolerance").as_double();
    position_only_xy_tolerance_ = node->get_parameter(
      plugin_name + ".position_only_xy_goal_tolerance").as_double();
    yaw_tolerance_ = node->get_parameter(plugin_name + ".yaw_goal_tolerance").as_double();
    required_xs_ = node->get_parameter(plugin_name + ".heading_required_goal_xs").as_double_array();
    required_ys_ = node->get_parameter(plugin_name + ".heading_required_goal_ys").as_double_array();
    required_tolerance_ = node->get_parameter(
      plugin_name + ".heading_required_goal_tolerance").as_double();
    if (xy_tolerance_ <= 0.0 || position_only_xy_tolerance_ < xy_tolerance_ ||
      yaw_tolerance_ < 0.0 || required_tolerance_ < 0.0 ||
      required_xs_.size() != required_ys_.size())
    {
      throw std::runtime_error("invalid selective heading goal checker parameters");
    }
  }

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    if (!requires_heading(goal_pose)) {
      return std::hypot(dx, dy) <= position_only_xy_tolerance_;
    }
    if (std::hypot(dx, dy) > xy_tolerance_) {
      return false;
    }
    return std::abs(shortest_angle(yaw(query_pose), yaw(goal_pose))) <= yaw_tolerance_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & velocity_tolerance) override
  {
    pose_tolerance.position.x = xy_tolerance_;
    pose_tolerance.position.y = xy_tolerance_;
    pose_tolerance.position.z = 0.0;
    pose_tolerance.orientation.w = yaw_tolerance_;
    velocity_tolerance.linear.x = std::numeric_limits<double>::lowest();
    velocity_tolerance.linear.y = std::numeric_limits<double>::lowest();
    velocity_tolerance.linear.z = std::numeric_limits<double>::lowest();
    velocity_tolerance.angular.x = std::numeric_limits<double>::lowest();
    velocity_tolerance.angular.y = std::numeric_limits<double>::lowest();
    velocity_tolerance.angular.z = std::numeric_limits<double>::lowest();
    return true;
  }

private:
  static double yaw(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double shortest_angle(double from, double to)
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  bool requires_heading(const geometry_msgs::msg::Pose & goal_pose) const
  {
    for (size_t index = 0; index < required_xs_.size(); ++index) {
      if (std::hypot(
          goal_pose.position.x - required_xs_[index],
          goal_pose.position.y - required_ys_[index]) <= required_tolerance_)
      {
        return true;
      }
    }
    return false;
  }

  double xy_tolerance_{0.5};
  double position_only_xy_tolerance_{1.5};
  double yaw_tolerance_{0.5};
  std::vector<double> required_xs_;
  std::vector<double> required_ys_;
  double required_tolerance_{0.2};
};

}  // namespace robot

PLUGINLIB_EXPORT_CLASS(robot::PassThroughGoalChecker, nav2_core::GoalChecker)
PLUGINLIB_EXPORT_CLASS(robot::SelectiveHeadingGoalChecker, nav2_core::GoalChecker)
