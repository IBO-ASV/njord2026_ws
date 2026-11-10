#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace njord
{
namespace sim
{

class Um982OdometrySimulatorNode : public rclcpp::Node
{
public:
  Um982OdometrySimulatorNode()
  : Node("um982_odometry_simulator"), rng_(std::random_device{}())
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/odom");
    output_odom_topic_ = declare_parameter<std::string>(
      "output_odom_topic", "/odometry/gps/um982");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
    position_noise_std_m_ = declare_parameter<double>("position_noise_std_m", 0.10);
    position_bias_rw_std_m_per_sqrt_s_ = declare_parameter<double>(
      "position_bias_random_walk_std_m_per_sqrt_s", 0.005);
    yaw_noise_std_rad_ = degreesToRadians(declare_parameter<double>("yaw_noise_std_deg", 1.0));
    yaw_bias_rw_std_rad_per_sqrt_s_ = degreesToRadians(declare_parameter<double>(
      "yaw_bias_random_walk_std_deg_per_sqrt_s", 0.05));

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 10,
      std::bind(&Um982OdometrySimulatorNode::odomCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const double period_s = 1.0 / std::max(1.0, publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s),
      std::bind(&Um982OdometrySimulatorNode::publishMeasurement, this));

    RCLCPP_INFO(
      get_logger(), "UM982 odometry simulator: %s -> %s (%s frame)",
      input_odom_topic_.c_str(), output_odom_topic_.c_str(), frame_id_.c_str());
  }

private:
  static double degreesToRadians(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_odom_ = message;
  }

  void advanceBias(double dt)
  {
    const double sqrt_dt = std::sqrt(std::max(0.0, dt));
    std::normal_distribution<double> position_bias_step(
      0.0, position_bias_rw_std_m_per_sqrt_s_ * sqrt_dt);
    std::normal_distribution<double> yaw_bias_step(
      0.0, yaw_bias_rw_std_rad_per_sqrt_s_ * sqrt_dt);

    position_bias_x_ += position_bias_step(rng_);
    position_bias_y_ += position_bias_step(rng_);
    yaw_bias_ += yaw_bias_step(rng_);

    // These are the variances of the random-walk component accumulated in
    // the reported observation, so the covariance tracks the configured
    // process instead of claiming fixed white-noise accuracy forever.
    position_bias_variance_ += std::pow(position_bias_rw_std_m_per_sqrt_s_, 2) * dt;
    yaw_bias_variance_ += std::pow(yaw_bias_rw_std_rad_per_sqrt_s_, 2) * dt;
  }

  bool transformTruthToMap(
    const nav_msgs::msg::Odometry & input,
    tf2::Vector3 * position_map,
    tf2::Quaternion * orientation_map)
  {
    tf2::Quaternion orientation_input(
      input.pose.pose.orientation.x,
      input.pose.pose.orientation.y,
      input.pose.pose.orientation.z,
      input.pose.pose.orientation.w);
    orientation_input.normalize();
    const tf2::Vector3 position_input(
      input.pose.pose.position.x,
      input.pose.pose.position.y,
      input.pose.pose.position.z);

    if (input.header.frame_id.empty() || input.header.frame_id == frame_id_) {
      *position_map = position_input;
      *orientation_map = orientation_input;
      return true;
    }

    try {
      const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        frame_id_, input.header.frame_id, tf2::TimePointZero);
      const tf2::Quaternion map_from_input(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      const tf2::Vector3 translation(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);

      *position_map = tf2::quatRotate(map_from_input, position_input) + translation;
      *orientation_map = map_from_input * orientation_input;
      orientation_map->normalize();
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for truth transform %s -> %s: %s",
        input.header.frame_id.c_str(), frame_id_.c_str(), exception.what());
      return false;
    }
  }

  void publishMeasurement()
  {
    if (!latest_odom_) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    double dt = 0.0;
    if (last_measurement_time_.nanoseconds() != 0) {
      dt = std::clamp((now - last_measurement_time_).seconds(), 0.0, 1.0);
    }
    last_measurement_time_ = now;
    advanceBias(dt);

    tf2::Vector3 truth_position;
    tf2::Quaternion truth_orientation;
    if (!transformTruthToMap(*latest_odom_, &truth_position, &truth_orientation)) {
      return;
    }

    std::normal_distribution<double> position_noise(0.0, position_noise_std_m_);
    std::normal_distribution<double> yaw_noise(0.0, yaw_noise_std_rad_);

    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(truth_orientation).getRPY(roll, pitch, yaw);
    const double noisy_yaw = yaw + yaw_bias_ + yaw_noise(rng_);
    tf2::Quaternion noisy_orientation;
    noisy_orientation.setRPY(roll, pitch, noisy_yaw);

    nav_msgs::msg::Odometry output;
    output.header.stamp = now;
    output.header.frame_id = frame_id_;
    output.child_frame_id = child_frame_id_;
    output.pose.pose.position.x = truth_position.x() + position_bias_x_ + position_noise(rng_);
    output.pose.pose.position.y = truth_position.y() + position_bias_y_ + position_noise(rng_);
    output.pose.pose.position.z = truth_position.z();
    output.pose.pose.orientation.x = noisy_orientation.x();
    output.pose.pose.orientation.y = noisy_orientation.y();
    output.pose.pose.orientation.z = noisy_orientation.z();
    output.pose.pose.orientation.w = noisy_orientation.w();

    const double position_variance = std::pow(position_noise_std_m_, 2) + position_bias_variance_;
    const double yaw_variance = std::pow(yaw_noise_std_rad_, 2) + yaw_bias_variance_;
    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] = position_variance;
    output.pose.covariance[7] = position_variance;
    // The simulator is planar; altitude is not a UM982 observation supplied
    // to this EKF path, so mark it unknown rather than advertising XY noise.
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = yaw_variance;
    output.twist.covariance.fill(0.0);

    publisher_->publish(output);
  }

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  double position_noise_std_m_;
  double position_bias_rw_std_m_per_sqrt_s_;
  double yaw_noise_std_rad_;
  double yaw_bias_rw_std_rad_per_sqrt_s_;
  double position_bias_x_{0.0};
  double position_bias_y_{0.0};
  double yaw_bias_{0.0};
  double position_bias_variance_{0.0};
  double yaw_bias_variance_{0.0};
  rclcpp::Time last_measurement_time_{0, 0, RCL_ROS_TIME};

  std::mt19937 rng_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace sim
}  // namespace njord

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<njord::sim::Um982OdometrySimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
