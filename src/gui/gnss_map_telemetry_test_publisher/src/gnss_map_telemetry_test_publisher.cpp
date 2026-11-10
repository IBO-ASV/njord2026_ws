#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kEarthRadiusMeters = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}
}  // namespace

class GnssMapTelemetryTestPublisher : public rclcpp::Node
{
public:
  GnssMapTelemetryTestPublisher()
  : Node("gnss_map_telemetry_test_publisher")
  {
    latitude_degrees_ = declare_parameter<double>("latitude", 35.609596);
    longitude_degrees_ = declare_parameter<double>("longitude", 139.683751);
    const auto heading_degrees = declare_parameter<double>("heading_degrees", 90.0);
    speed_mps_ = declare_parameter<double>("speed_mps", 2.0);
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 2.0);
    const auto fix_topic = declare_parameter<std::string>(
      "fix_topic", "/sensor/vehicle_gnss/fix/raw");
    const auto speed_topic = declare_parameter<std::string>(
      "speed_topic", "/gui/ground_speed_mps");

    if (latitude_degrees_ < -90.0 || latitude_degrees_ > 90.0) {
      throw std::invalid_argument("latitude must be between -90 and 90 degrees");
    }
    if (longitude_degrees_ < -180.0 || longitude_degrees_ > 180.0) {
      throw std::invalid_argument("longitude must be between -180 and 180 degrees");
    }
    if (speed_mps_ < 0.0) {
      throw std::invalid_argument("speed_mps must be non-negative");
    }
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    heading_radians_ = degreesToRadians(heading_degrees);
    yaw_radians_ = degreesToRadians(90.0 - heading_degrees);
    fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic, 10);
    speed_publisher_ = create_publisher<std_msgs::msg::Float32>(speed_topic, 10);
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    publish();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GnssMapTelemetryTestPublisher::advanceAndPublish, this));

    RCLCPP_INFO(
      get_logger(), "Publishing %.2f m/s at %.1f degrees from (%.6f, %.6f)",
      speed_mps_, heading_degrees, latitude_degrees_, longitude_degrees_);
  }

private:
  void advanceAndPublish()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_seconds = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;

    const double distance_meters = speed_mps_ * elapsed_seconds;
    const double north_meters = distance_meters * std::cos(heading_radians_);
    const double east_meters = distance_meters * std::sin(heading_radians_);
    const double latitude_radians = degreesToRadians(latitude_degrees_);

    latitude_degrees_ += radiansToDegrees(north_meters / kEarthRadiusMeters);
    longitude_degrees_ += radiansToDegrees(
      east_meters / (kEarthRadiusMeters * std::cos(latitude_radians)));
    east_meters_ += east_meters;
    north_meters_ += north_meters;
    publish();
  }

  void publish()
  {
    sensor_msgs::msg::NavSatFix fix;
    const auto stamp = get_clock()->now();
    fix.header.stamp = stamp;
    fix.header.frame_id = "gnss";
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = latitude_degrees_;
    fix.longitude = longitude_degrees_;
    fix.altitude = 0.0;
    fix.position_covariance = std::array<double, 9>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0};
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_publisher_->publish(fix);

    std_msgs::msg::Float32 speed;
    speed.data = static_cast<float>(speed_mps_);
    speed_publisher_->publish(speed);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = east_meters_;
    transform.transform.translation.y = north_meters_;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_radians_);
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    transform_broadcaster_->sendTransform(transform);
  }

  double latitude_degrees_;
  double longitude_degrees_;
  double heading_radians_;
  double yaw_radians_;
  double speed_mps_;
  double east_meters_{0.0};
  double north_meters_{0.0};
  std::chrono::steady_clock::time_point last_update_{std::chrono::steady_clock::now()};
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssMapTelemetryTestPublisher>());
  rclcpp::shutdown();
  return 0;
}
