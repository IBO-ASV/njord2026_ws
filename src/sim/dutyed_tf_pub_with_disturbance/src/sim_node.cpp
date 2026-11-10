#include "dutyed_tf_pub_with_disturbance/sim_node.hpp"

#include "dutyed_tf_pub_with_disturbance/thruster_wrench.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace njord
{
namespace sim
{

namespace
{
double clamp(double value, double low, double high)
{
  return std::clamp(value, low, high);
}
}  // namespace

SimNode::SimNode(const rclcpp::NodeOptions & options)
: Node("dutyed_tf_pub_with_disturbance_node", options)
{
  update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 50.0);
  topic_thruster_command_ = this->declare_parameter<std::string>(
    "topic_thruster_command",
    "/thruster_command");
  thruster_force_sign_ = this->declare_parameter<std::vector<double>>(
    "thruster_force_sign", {1.0, 1.0, 1.0, 1.0});
  if (thruster_force_sign_.size() != 4U || std::any_of(
      thruster_force_sign_.begin(), thruster_force_sign_.end(),
      [](double sign) {return sign != -1.0 && sign != 1.0;}))
  {
    throw std::runtime_error("thruster_force_sign must contain four values, each -1 or 1");
  }
  topic_odom_ = this->declare_parameter<std::string>("topic_odom", "/odom");
  const auto initial_pose = this->declare_parameter<std::vector<double>>(
    "initial_pose_xyyaw", {0.0, 0.0, 0.0});
  if (initial_pose.size() >= 3U) {
    x_ = initial_pose[0];
    y_ = initial_pose[1];
    yaw_ = initial_pose[2];
  }

  frame_world_ = this->declare_parameter<std::string>("frame_world", "world");
  frame_map_ = this->declare_parameter<std::string>("frame_map", "map");
  frame_odom_ = this->declare_parameter<std::string>("frame_odom", "odom");
  frame_base_link_ = this->declare_parameter<std::string>("frame_base_link", "base_link");
  publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

  auto map_offset =
    this->declare_parameter<std::vector<double>>("map_offset_xyyaw", {0.0, 0.0, 0.0});
  auto odom_offset = this->declare_parameter<std::vector<double>>(
    "odom_offset_xyyaw", {0.0, 0.0,
      0.0});
  if (map_offset.size() >= 3U) {
    map_offset_x_ = map_offset[0];
    map_offset_y_ = map_offset[1];
    map_offset_yaw_ = map_offset[2];
  }
  if (odom_offset.size() >= 3U) {
    odom_offset_x_ = odom_offset[0];
    odom_offset_y_ = odom_offset[1];
    odom_offset_yaw_ = odom_offset[2];
  }


  DoyleParams mmg;
  mmg.Lpp = this->declare_parameter<double>("mmg.Lpp", 1.0);
  mmg.D = this->declare_parameter<double>("mmg.D", 0.25);
  mmg.mass_kg = this->declare_parameter<double>("mmg.mass_kg", 10.0);
  mmg.I_zG = this->declare_parameter<double>("mmg.I_zG", 1.5);
  mmg.x_G = this->declare_parameter<double>("mmg.x_G", 0.0);
  mmg.m_x = this->declare_parameter<double>("mmg.m_x", 1.0);
  mmg.m_y = this->declare_parameter<double>("mmg.m_y", 10.0);
  mmg.J_z = this->declare_parameter<double>("mmg.J_z", 1.5);
  mmg.R_0 = this->declare_parameter<double>("mmg.R_0", -0.2);
  mmg.Y_0 = this->declare_parameter<double>("mmg.Y_0", 0.001);
  mmg.Y_v = this->declare_parameter<double>("mmg.Y_v", -0.2);
  mmg.Y_r = this->declare_parameter<double>("mmg.Y_r", 0.02);
  mmg.N_0 = this->declare_parameter<double>("mmg.N_0", 0.001);
  mmg.N_v = this->declare_parameter<double>("mmg.N_v", -0.2);
  mmg.N_r = this->declare_parameter<double>("mmg.N_r", -0.02);

  const double disturbance_hz = this->declare_parameter<double>(
    "disturbance.natural_frequency_hz",
    0.18);
  const double disturbance_zeta = this->declare_parameter<double>("disturbance.damping_ratio", 0.8);
  const double disturbance_sigma_u = this->declare_parameter<double>("disturbance.sigma_u", 0.12);
  const double disturbance_sigma_v = this->declare_parameter<double>("disturbance.sigma_v", 0.12);
  const double disturbance_sigma_r = this->declare_parameter<double>("disturbance.sigma_r", 0.04);
  const int disturbance_seed = this->declare_parameter<int>("disturbance.seed", 2026);

  mmg_model_ = std::make_unique<MMGDoyleModel>(mmg);
  disturbance_model_ = std::make_unique<DisturbanceModel>(
    disturbance_hz,
    disturbance_zeta,
    disturbance_sigma_u,
    disturbance_sigma_v,
    disturbance_sigma_r,
    static_cast<std::uint32_t>(std::max(0, disturbance_seed)));

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom_, 10);
  sub_command_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    topic_thruster_command_,
    10,
    std::bind(&SimNode::commandCallback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  publishStaticTransforms();

  const double dt = 1.0 / std::max(1.0, update_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt),
    std::bind(&SimNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "dutyed_tf_pub_with_disturbance started");
}

void SimNode::commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 4U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "thruster_command requires four force values [RR, RL, FR, FL]");
    return;
  }

  latest_forces_.resize(msg->data.size());
  for (size_t i = 0; i < msg->data.size(); ++i) {
    latest_forces_[i] = static_cast<double>(msg->data[i]) * thruster_force_sign_[i];
  }
}

void SimNode::publishStaticTransforms()
{
  if (!publish_tf_) {
    return;
  }

  geometry_msgs::msg::TransformStamped world_to_map;
  world_to_map.header.stamp = this->now();
  world_to_map.header.frame_id = frame_world_;
  world_to_map.child_frame_id = frame_map_;
  world_to_map.transform.translation.x = map_offset_x_;
  world_to_map.transform.translation.y = map_offset_y_;
  world_to_map.transform.translation.z = 0.0;
  tf2::Quaternion q_map;
  q_map.setRPY(0.0, 0.0, map_offset_yaw_);
  world_to_map.transform.rotation.x = q_map.x();
  world_to_map.transform.rotation.y = q_map.y();
  world_to_map.transform.rotation.z = q_map.z();
  world_to_map.transform.rotation.w = q_map.w();

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = this->now();
  map_to_odom.header.frame_id = frame_map_;
  map_to_odom.child_frame_id = frame_odom_;
  map_to_odom.transform.translation.x = odom_offset_x_;
  map_to_odom.transform.translation.y = odom_offset_y_;
  map_to_odom.transform.translation.z = 0.0;
  tf2::Quaternion q_odom;
  q_odom.setRPY(0.0, 0.0, odom_offset_yaw_);
  map_to_odom.transform.rotation.x = q_odom.x();
  map_to_odom.transform.rotation.y = q_odom.y();
  map_to_odom.transform.rotation.z = q_odom.z();
  map_to_odom.transform.rotation.w = q_odom.w();

  tf_static_broadcaster_->sendTransform(world_to_map);
  tf_static_broadcaster_->sendTransform(map_to_odom);
}

void SimNode::onTimer()
{
  const auto now = this->now();
  if (first_tick_) {
    first_tick_ = false;
    last_stamp_ = now;
    publishDynamicTfAndOdom(now);
    return;
  }

  const double dt = clamp((now - last_stamp_).seconds(), 1e-4, 0.1);
  last_stamp_ = now;

  PlanarInput input;
  if (latest_forces_.size() == 4U) {
    const std::vector<SimThrusterGeometry> geometry = {
      // Match robot.urdf.xacro / robot.urdf_modified.urdf.
      {-0.1803, -0.2500, 2.35619},
      {-0.1803, 0.2500, -2.35619},
      {0.1803, -0.2500, 0.785398},
      {0.1803, 0.2500, -0.785398}};
    input = forcesToPlanarInput(latest_forces_, geometry);
  }

  PlanarAccel accel = mmg_model_->computeAccel(state_, input);
  const DisturbanceAccel disturbance = disturbance_model_->step(dt);
  accel.du += disturbance.du;
  accel.dv += disturbance.dv;
  accel.dr += disturbance.dr;

  state_.u += accel.du * dt;
  state_.v += accel.dv * dt;
  state_.r += accel.dr * dt;

  yaw_ += state_.r * dt;
  const double c = std::cos(yaw_);
  const double s = std::sin(yaw_);
  x_ += (state_.u * c - state_.v * s) * dt;
  y_ += (state_.u * s + state_.v * c) * dt;

  publishDynamicTfAndOdom(now);
}

void SimNode::publishDynamicTfAndOdom(const rclcpp::Time & stamp)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = frame_odom_;
  tf.child_frame_id = frame_base_link_;
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  if (publish_tf_) {
    tf_broadcaster_->sendTransform(tf);
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_odom_;
  odom.child_frame_id = frame_base_link_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = state_.u;
  odom.twist.twist.linear.y = state_.v;
  odom.twist.twist.angular.z = state_.r;
  pub_odom_->publish(odom);
}

}  // namespace sim
}  // namespace njord
