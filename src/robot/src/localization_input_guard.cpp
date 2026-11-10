#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
template<typename ContainerT>
bool all_finite(const ContainerT & values)
{
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const std::array<double, 4> values = {
    quaternion.x, quaternion.y, quaternion.z, quaternion.w};
  if (!all_finite(values)) {
    return false;
  }
  const double norm_squared = quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && std::abs(norm_squared - 1.0) < 1e-3;
}

bool finite_pose(const geometry_msgs::msg::PoseWithCovariance & pose)
{
  const std::array<double, 3> position = {
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
  return all_finite(position) && finite_quaternion(pose.pose.orientation) &&
         all_finite(pose.covariance);
}

bool finite_odometry(const nav_msgs::msg::Odometry & odometry)
{
  const std::array<double, 6> twist = {
    odometry.twist.twist.linear.x,
    odometry.twist.twist.linear.y,
    odometry.twist.twist.linear.z,
    odometry.twist.twist.angular.x,
    odometry.twist.twist.angular.y,
    odometry.twist.twist.angular.z};
  return finite_pose(odometry.pose) && all_finite(twist) &&
         all_finite(odometry.twist.covariance);
}
}  // namespace

class LocalizationInputGuard : public rclcpp::Node
{
public:
  LocalizationInputGuard()
  : Node("localization_input_guard")
  {
    const auto gps_input = declare_parameter<std::string>(
      "gps_odometry_input", "/odometry/gps/um982/raw");
    const auto gps_output = declare_parameter<std::string>(
      "gps_odometry_output", "/odometry/gps/um982");
    const auto compass_input = declare_parameter<std::string>(
      "compass_input", "/sensor/vehicle_gnss/compass/raw");
    const auto compass_output = declare_parameter<std::string>(
      "compass_output", "/sensor/vehicle_gnss/compass/validated");
    const auto imu_input = declare_parameter<std::string>("imu_input", "/livox/imu");
    const auto imu_output = declare_parameter<std::string>(
      "imu_output", "/livox/imu/validated");
    const auto glim_input = declare_parameter<std::string>("glim_odometry_input", "/odom");
    const auto glim_output = declare_parameter<std::string>(
      "glim_odometry_output", "/odom/validated");
    gps_publisher_ = create_publisher<nav_msgs::msg::Odometry>(gps_output, 10);
    compass_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(compass_output, 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_output, 10);
    glim_publisher_ = create_publisher<nav_msgs::msg::Odometry>(glim_output, 10);

    gps_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      gps_input, 10, [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        if (!finite_odometry(*message)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Rejected non-finite GPS Odometry");
          return;
        }
        gps_publisher_->publish(*message);
      });
    compass_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      compass_input, 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        if (!finite_pose(message->pose)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Rejected non-finite compass pose");
          return;
        }
        compass_publisher_->publish(*message);
      });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input, 10, [this](const sensor_msgs::msg::Imu::SharedPtr message) {
        const std::array<double, 6> observations = {
          message->angular_velocity.x,
          message->angular_velocity.y,
          message->angular_velocity.z,
          message->linear_acceleration.x,
          message->linear_acceleration.y,
          message->linear_acceleration.z};
        if (!all_finite(observations) ||
          !all_finite(message->angular_velocity_covariance) ||
          !all_finite(message->linear_acceleration_covariance))
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Rejected non-finite IMU observation");
          return;
        }
        imu_publisher_->publish(*message);
      });
    glim_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      glim_input, 10, [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        if (!finite_odometry(*message)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Rejected non-finite GLIM Odometry");
          return;
        }
        glim_publisher_->publish(*message);
      });
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr compass_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr glim_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    compass_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr glim_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationInputGuard>());
  rclcpp::shutdown();
  return 0;
}
