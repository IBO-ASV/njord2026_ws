#include <random>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

namespace njord
{
namespace sim
{

class GnssNoiseSimulatorNode : public rclcpp::Node
{
public:
  explicit GnssNoiseSimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gnss_noise_simulator", options),
    rng_(std::random_device{}())
  {
    // Retrieve parameters
    gps_origin_lat_ = this->declare_parameter<double>("gps_origin_lat", 35.652832);
    gps_origin_lon_ = this->declare_parameter<double>("gps_origin_lon", 139.839478);
    gps_origin_alt_ = this->declare_parameter<double>("gps_origin_alt", 0.0);

    gps_pos_noise_std_ = this->declare_parameter<double>("gps_pos_noise_std", 0.02); // 2 cm
    gps_heading_noise_std_deg_ = this->declare_parameter<double>("gps_heading_noise_std_deg", 2.5); // 2.5 deg
    // Direction of geographical true north expressed in the simulation map
    // frame (ENU default: +Y == pi/2).  Keeping this separate from sensor
    // noise lets Task1 exercise sites whose navigation axes are rotated from
    // true north, as on the real vessel.
    true_north_yaw_rad_ = this->declare_parameter<double>(
      "true_north_yaw_rad", M_PI / 2.0);

    double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 10.0);

    frame_id_gps_ = this->declare_parameter<std::string>("frame_id_gps", "base_link");
    frame_id_compass_ = this->declare_parameter<std::string>("frame_id_compass", "base_link");

    // Initialize publishers
    pub_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    pub_compass_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensor/vehicle_gnss/compass/raw", 10);

    // Initialize subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&GnssNoiseSimulatorNode::odomCallback, this, std::placeholders::_1));

    // Setup timer
    const double dt = 1.0 / std::max(1.0, publish_rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&GnssNoiseSimulatorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "GnssNoiseSimulatorNode started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
  }

  void timerCallback()
  {
    const auto now = this->now();
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;

    if (latest_odom_) {
      const auto & pose = latest_odom_->pose.pose;
      x = pose.position.x;
      y = pose.position.y;
      tf2::Quaternion q(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }
    // ponytail: publishes the configured origin until /odom arrives; add a
    // trajectory parameter only if standalone motion simulation is needed.

    // Generate Gaussian Noise
    std::normal_distribution<double> noise_gps_pos(0.0, gps_pos_noise_std_);
    std::normal_distribution<double> noise_gps_yaw(0.0, gps_heading_noise_std_deg_ * M_PI / 180.0);

    // Publish GPS fix using Flat Earth projection
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = now;
    gps_msg.header.frame_id = frame_id_gps_;

    const double R_EARTH = 6378137.0; // Equatorial radius
    double lat_rad = gps_origin_lat_ * M_PI / 180.0;

    double dx = x + noise_gps_pos(rng_);
    double dy = y + noise_gps_pos(rng_);

    double dlat = dy / R_EARTH;
    double dlon = dx / (R_EARTH * std::cos(lat_rad));

    gps_msg.latitude = gps_origin_lat_ + dlat * 180.0 / M_PI;
    gps_msg.longitude = gps_origin_lon_ + dlon * 180.0 / M_PI;
    gps_msg.altitude = gps_origin_alt_ + noise_gps_pos(rng_);

    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    double var_gps_pos = gps_pos_noise_std_ * gps_pos_noise_std_;
    for (int i = 0; i < 9; ++i) {
      gps_msg.position_covariance[i] = 0.0;
    }
    gps_msg.position_covariance[0] = var_gps_pos;
    gps_msg.position_covariance[4] = var_gps_pos;
    gps_msg.position_covariance[8] = var_gps_pos;
    gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    pub_gps_->publish(gps_msg);

    // Publish Compass Yaw as PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped compass_msg;
    compass_msg.header.stamp = now;
    compass_msg.header.frame_id = frame_id_compass_;

    // The simulated odometry yaw is expressed in the map frame.  Convert it
    // to an ENU heading reported by a true-north-referenced dual-antenna
    // GNSS: yaw=0 points along map +X, whose bearing from north is
    // pi/2-true_north_yaw_rad_.
    double compass_yaw = yaw + M_PI / 2.0 - true_north_yaw_rad_ + noise_gps_yaw(rng_);
    tf2::Quaternion q_compass;
    q_compass.setRPY(0.0, 0.0, compass_yaw);
    compass_msg.pose.pose.orientation.x = q_compass.x();
    compass_msg.pose.pose.orientation.y = q_compass.y();
    compass_msg.pose.pose.orientation.z = q_compass.z();
    compass_msg.pose.pose.orientation.w = q_compass.w();

    double var_gps_yaw = (gps_heading_noise_std_deg_ * M_PI / 180.0) * (gps_heading_noise_std_deg_ * M_PI / 180.0);
    for (int i = 0; i < 36; ++i) {
      compass_msg.pose.covariance[i] = 0.0;
    }
    compass_msg.pose.covariance[35] = var_gps_yaw;

    pub_compass_->publish(compass_msg);
  }

  // Parameters
  double gps_origin_lat_;
  double gps_origin_lon_;
  double gps_origin_alt_;
  double gps_pos_noise_std_;
  double gps_heading_noise_std_deg_;
  double true_north_yaw_rad_;
  std::string frame_id_gps_;
  std::string frame_id_compass_;

  // Publishers & Subscribers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_compass_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  // Random number generator
  std::mt19937 rng_;
};

} // namespace sim
} // namespace njord

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<njord::sim::GnssNoiseSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
