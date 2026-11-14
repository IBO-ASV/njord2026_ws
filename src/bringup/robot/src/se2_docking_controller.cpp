#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "robot/se2_tracking.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot
{
class SE2DockingController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = parent.lock();
    tf_ = tf;
    costmap_ = costmap;
    auto param = [&](const std::string & key, double value) {
        const std::string full = name + "." + key;
        if (!node_->has_parameter(full)) {node_->declare_parameter(full, value);}
        const double result = node_->get_parameter(full).as_double();
        if (!std::isfinite(result) || result <= 0.0) {
          throw std::runtime_error(full + " must be finite and positive");
        }
        return result;
      };
    limits_.speed = param("max_speed", 0.25);
    speed_limit_ = limits_.speed;
    limits_.yaw_rate = param("max_yaw_rate", 0.35);
    limits_.acceleration = param("max_acceleration", 0.25);
    limits_.yaw_acceleration = param("max_yaw_acceleration", 0.5);
    limits_.position_gain = param("position_gain", 0.8);
    limits_.yaw_gain = param("yaw_gain", 1.2);
    reference_speed_ = param("reference_speed", 0.2);
    position_lag_ = param("position_lag", 0.35);
    yaw_lag_ = param("yaw_lag", 0.25);
    horizon_ = param("collision_horizon", 1.5);
    target_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/docking_reference", 10);
    error_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/docking_tracking_error", 10);
  }
  void activate() override {target_pub_->on_activate(); error_pub_->on_activate();}
  void deactivate() override {target_pub_->on_deactivate(); error_pub_->on_deactivate();}
  void cleanup() override {path_.clear(); target_pub_.reset(); error_pub_.reset();}
  void setSpeedLimit(const double & value, const bool & percentage) override
  {
    speed_limit_ = value <= 0.0 ? limits_.speed :
      std::min(limits_.speed, percentage ? limits_.speed * value / 100.0 : value);
  }
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (path.poses.size() < 2 || path.header.frame_id.empty()) {
      throw nav2_core::InvalidPath("Docking requires a framed SE(2) reference");
    }
    path_ = path.poses;
    frame_ = path.header.frame_id;
    distances_ = {0.0};
    for (size_t i = 0; i < path_.size(); ++i) {
      const auto p = state(path_[i].pose);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) {
        throw nav2_core::InvalidPath("Non-finite docking reference");
      }
      if (i) {
        const auto a = state(path_[i-1].pose);
        // A small angular metric also supports pure heading correction.
        distances_.push_back(distances_.back() + std::max(
          std::hypot(p.x-a.x, p.y-a.y), 0.1*std::abs(angle(p.yaw-a.yaw))));
      }
    }
    progress_ = 0.0;
    initialized_ = false;
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker *) override
  {
    if (path_.empty()) {throw nav2_core::InvalidPath("No docking reference");}
    const auto now = node_->now();
    const double dt = initialized_ ? std::clamp((now-last_time_).seconds(), 0.001, 0.2) : 0.02;
    if (!initialized_) {previous_ = {velocity.linear.x, velocity.linear.y, velocity.angular.z};}
    last_time_ = now;
    initialized_ = true;
    geometry_msgs::msg::PoseStamped local;
    try {
      auto input = pose;
      input.header.stamp = rclcpp::Time(0);
      local = frame_ == pose.header.frame_id ? input : tf_->transform(input, frame_);
    } catch (const tf2::TransformException & ex) {throw nav2_core::ControllerTFError(ex.what());}
    const State current = state(local.pose);
    const auto old_ref = reference(progress_);
    const double position_error = std::hypot(old_ref.x-current.x, old_ref.y-current.y);
    const double yaw_error = std::abs(angle(old_ref.yaw-current.yaw));
    double speed = std::min(reference_speed_, std::min(limits_.speed, speed_limit_));
    const double lag = std::max(position_error/position_lag_, yaw_error/yaw_lag_);
    speed *= std::clamp(2.0-lag, 0.0, 1.0);
    const double remaining = distances_.back()-progress_;
    speed = std::min(speed, std::sqrt(2.0*limits_.acceleration*remaining));
    const double next = std::min(distances_.back(), progress_ + speed*dt);
    const auto ref = reference(next);
    // Derivatives of the sampled geometric reference provide feedforward.
    Command ff{(ref.x-old_ref.x)/dt, (ref.y-old_ref.y)/dt,
      angle(ref.yaw-old_ref.yaw)/dt};
    auto limits = limits_;
    limits.speed = std::min(limits.speed, speed_limit_);
    const Command cmd = track(current, ref, ff, previous_, dt, limits);
    if (!safe(pose, cmd)) {
      // ControllerServer publishes zero and bounds retries with failure_tolerance.
      previous_ = {};
      throw nav2_core::NoValidControl("Docking footprint prediction intersects obstacle");
    }
    previous_ = cmd;
    progress_ = next;
    geometry_msgs::msg::TwistStamped output;
    output.header = pose.header;
    output.header.stamp = now;
    output.header.frame_id = costmap_->getBaseFrameID();
    output.twist.linear.x = cmd.x;
    output.twist.linear.y = cmd.y;
    output.twist.angular.z = cmd.yaw;
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now;
    target.header.frame_id = frame_;
    target.pose.position.x = ref.x;
    target.pose.position.y = ref.y;
    target.pose.orientation.z = std::sin(ref.yaw/2.0);
    target.pose.orientation.w = std::cos(ref.yaw/2.0);
    target_pub_->publish(target);
    geometry_msgs::msg::TwistStamped error;
    error.header = target.header;
    error.twist.linear.x = std::hypot(ref.x-current.x, ref.y-current.y);
    error.twist.angular.z = angle(ref.yaw-current.yaw);
    error_pub_->publish(error);
    return output;
  }
private:
  static State state(const geometry_msgs::msg::Pose & p)
  {
    const auto & q = p.orientation;
    return {p.position.x, p.position.y,
      std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))};
  }
  State reference(double distance) const
  {
    auto it = std::upper_bound(distances_.begin(), distances_.end(), distance);
    if (it == distances_.end()) {return state(path_.back().pose);}
    const size_t b = it-distances_.begin(), a = b-1;
    const double t = (distance-distances_[a])/std::max(1e-9, distances_[b]-distances_[a]);
    const auto p = state(path_[a].pose), q = state(path_[b].pose);
    return {p.x+t*(q.x-p.x), p.y+t*(q.y-p.y), p.yaw+t*angle(q.yaw-p.yaw)};
  }
  bool safe(const geometry_msgs::msg::PoseStamped & pose, const Command & cmd)
  {
    geometry_msgs::msg::PoseStamped global;
    try {
      auto input = pose;
      input.header.stamp = rclcpp::Time(0);
      global = input.header.frame_id == costmap_->getGlobalFrameID() ? input :
        tf_->transform(input, costmap_->getGlobalFrameID());
    } catch (const tf2::TransformException & ex) {throw nav2_core::ControllerTFError(ex.what());}
    auto p = state(global.pose);
    auto * map = costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
    const auto footprint = costmap_->getRobotFootprint();
    // Check at <= 5 cm / 0.05 rad steps, including the current footprint.
    const double step = std::min({0.05, 0.05/std::max(0.01, std::hypot(cmd.x, cmd.y)),
      0.05/std::max(0.01, std::abs(cmd.yaw))});
    for (double t = 0.0; t <= horizon_+step; t += step) {
      const double cost = checker.footprintCostAtPose(p.x, p.y, p.yaw, footprint);
      if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {return false;}
      p.x += step*(std::cos(p.yaw)*cmd.x-std::sin(p.yaw)*cmd.y);
      p.y += step*(std::sin(p.yaw)*cmd.x+std::cos(p.yaw)*cmd.y);
      p.yaw += step*cmd.yaw;
    }
    return true;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr error_pub_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  std::vector<double> distances_;
  std::string frame_;
  TrackingLimits limits_;
  Command previous_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool initialized_{false};
  double progress_{0.0}, speed_limit_{0.25}, reference_speed_{0.2};
  double position_lag_{0.35}, yaw_lag_{0.25}, horizon_{1.5};
};
}  // namespace robot
PLUGINLIB_EXPORT_CLASS(robot::SE2DockingController, nav2_core::Controller)
