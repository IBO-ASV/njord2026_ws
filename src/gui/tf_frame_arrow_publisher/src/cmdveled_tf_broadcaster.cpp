#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class CmdVeledTfBroadcaster : public rclcpp::Node
{
public:
  CmdVeledTfBroadcaster()
  : Node("cmdveled_tf_broadcaster"),
    x_(0.0), y_(0.0), yaw_(0.0),
    vx_(0.0), vy_(0.0), wz_(0.0)
  {
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    cmd_vel_fallback_topic_ = this->declare_parameter<std::string>("cmd_vel_fallback_topic", "/cmd_vel");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry");

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.3);
    publish_odom_ = this->declare_parameter<bool>("publish_odom", true);
    emulate_reverse_circle_ = this->declare_parameter<bool>("emulate_reverse_circle", false);
    reverse_circle_radius_m_ = this->declare_parameter<double>("reverse_circle_radius_m", 0.4);
    reverse_circle_direction_ = this->declare_parameter<double>("reverse_circle_direction", 1.0);
    reverse_circle_min_vx_ = this->declare_parameter<double>("reverse_circle_min_vx", -0.02);

    x_ = this->declare_parameter<double>("initial_x", 0.0);
    y_ = this->declare_parameter<double>("initial_y", 0.0);
    yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 20,
      std::bind(&CmdVeledTfBroadcaster::cmd_vel_callback, this, std::placeholders::_1));

    if (cmd_vel_fallback_topic_ != cmd_vel_topic_) {
      sub_cmd_vel_fallback_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_fallback_topic_, 20,
        std::bind(&CmdVeledTfBroadcaster::cmd_vel_callback, this, std::placeholders::_1));
    }

    if (publish_odom_) {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    }

    const auto period_ms = std::max<int64_t>(
      5, static_cast<int64_t>(1000.0 / std::max(1e-3, publish_rate_hz_)));

    last_update_time_ = this->now();
    last_cmd_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CmdVeledTfBroadcaster::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "cmdveled_tf_broadcaster started. sub=%s fallback=%s tf=(%s->%s) odom_topic=%s",
      cmd_vel_topic_.c_str(),
      cmd_vel_fallback_topic_.c_str(),
      odom_frame_id_.c_str(),
      base_frame_id_.c_str(),
      odom_topic_.c_str());
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    wz_ = msg->angular.z;
    last_cmd_time_ = this->now();
  }

  void timer_callback()
  {
    const auto now = this->now();
    const double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    if ((now - last_cmd_time_).seconds() > cmd_timeout_sec_) {
      vx_ = 0.0;
      vy_ = 0.0;
      wz_ = 0.0;
    }

    double effective_wz = wz_;
    if (emulate_reverse_circle_ && vx_ < reverse_circle_min_vx_) {
      const double radius = std::max(0.05, std::abs(reverse_circle_radius_m_));
      const double dir = (reverse_circle_direction_ >= 0.0) ? 1.0 : -1.0;
      // 後退時に一定曲率(1/radius)を加えて、円弧トレースを再現する
      effective_wz += dir * (std::abs(vx_) / radius);
    }

    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);

    // Body速度(cmd_vel)をodom座標に変換して積分する
    x_ += (vx_ * cy - vy_ * sy) * dt;
    y_ += (vx_ * sy + vy_ * cy) * dt;
    yaw_ += effective_wz * dt;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

    publish_tf_and_odom(now, effective_wz);
    last_update_time_ = now;
  }

  void publish_tf_and_odom(const rclcpp::Time & stamp, double effective_wz)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = base_frame_id_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    if (!publish_odom_ || !odom_pub_) {
      return;
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = effective_wz;

    odom_pub_->publish(odom_msg);
  }

  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string cmd_vel_topic_;
  std::string cmd_vel_fallback_topic_;
  std::string odom_topic_;

  double publish_rate_hz_;
  double cmd_timeout_sec_;
  bool publish_odom_;
  bool emulate_reverse_circle_;
  double reverse_circle_radius_m_;
  double reverse_circle_direction_;
  double reverse_circle_min_vx_;

  double x_;
  double y_;
  double yaw_;
  double vx_;
  double vy_;
  double wz_;

  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_fallback_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVeledTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
