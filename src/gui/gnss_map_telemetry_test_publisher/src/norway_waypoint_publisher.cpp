#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace
{
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kStartLatitudeDegrees = 63.4420936;
constexpr double kStartLongitudeDegrees = 10.4242793;
// Unit vectors of the supplied GPS 1 -> GPS 2 baseline in local ENU.
constexpr double kCourseEast = -0.341615;
constexpr double kCourseNorth = -0.939856;
constexpr double kLeftEast = 0.939856;
constexpr double kLeftNorth = -0.341615;
// The rulebook-shaped Task 1 template spans 50 m from GPS 1 to GPS 2.
constexpr double kTask1Scale = 57.878 / 50.0;

double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

struct Waypoint
{
  std::string topic_suffix;
  double latitude_degrees;
  double longitude_degrees;
};

Waypoint localWaypoint(const std::string & topic_suffix, double east_meters, double north_meters)
{
  const double latitude = kStartLatitudeDegrees + radiansToDegrees(north_meters / kEarthRadiusMeters);
  const double longitude = kStartLongitudeDegrees + radiansToDegrees(
    east_meters / (kEarthRadiusMeters * std::cos(degreesToRadians(kStartLatitudeDegrees))));
  return {topic_suffix, latitude, longitude};
}

Waypoint courseWaypoint(const std::string & topic_suffix, double along_meters, double left_meters)
{
  return localWaypoint(
    topic_suffix,
    along_meters * kCourseEast + left_meters * kLeftEast,
    along_meters * kCourseNorth + left_meters * kLeftNorth);
}
}  // namespace

class NorwayWaypointPublisher : public rclcpp::Node
{
public:
  NorwayWaypointPublisher()
  : Node("norway_waypoint_publisher")
  {
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    // Rulebook-shaped, visualization-only course proposals.  They are expressed
    // in the supplied GPS 1 -> GPS 2 local ENU baseline; see
    // Docs/norway_task_waypoint_geometry.md.
    const std::vector<Waypoint> waypoints{
      {"start", kStartLatitudeDegrees, kStartLongitudeDegrees},
      // Task 1.1: GPS 1 -> 1.1...1.10 -> GPS 2 (rulebook zig-zag).
      courseWaypoint("task1/gps1", 0.0, 0.0),
      courseWaypoint("task1/wp_1_1", 10.0 * kTask1Scale, 5.0 * kTask1Scale),
      courseWaypoint("task1/wp_1_2", 13.0 * kTask1Scale, 0.0),
      courseWaypoint("task1/wp_1_3", 16.0 * kTask1Scale, -5.0 * kTask1Scale),
      courseWaypoint("task1/wp_1_4", 19.0 * kTask1Scale, -0.5 * kTask1Scale),
      courseWaypoint("task1/wp_1_5", 22.0 * kTask1Scale, 6.0 * kTask1Scale),
      courseWaypoint("task1/wp_1_6", 25.0 * kTask1Scale, 0.0),
      courseWaypoint("task1/wp_1_7", 28.0 * kTask1Scale, -5.0 * kTask1Scale),
      courseWaypoint("task1/wp_1_8", 31.0 * kTask1Scale, -1.0 * kTask1Scale),
      courseWaypoint("task1/wp_1_9", 34.0 * kTask1Scale, 2.0 * kTask1Scale),
      courseWaypoint("task1/wp_1_10", 40.0 * kTask1Scale, 0.0),
      {"task1/gps2", 63.4416044, 10.4238816},
      // Task 1.2: return lane, passing S/N/S cardinal marks alternately.
      courseWaypoint("task1/gps3", 50.0 * kTask1Scale, -25.0 * kTask1Scale),
      courseWaypoint("task1/wp_3_1_pass_south", 24.0 * kTask1Scale, -35.0 * kTask1Scale),
      courseWaypoint("task1/wp_3_2_pass_north", 15.0 * kTask1Scale, -15.0 * kTask1Scale),
      courseWaypoint("task1/wp_3_3_pass_south", 8.0 * kTask1Scale, -35.0 * kTask1Scale),
      courseWaypoint("task1/gps4", 0.0, -25.0 * kTask1Scale),
      // Task 2: two 5 m-wide red/green gates with 20 m gate spacing.
      courseWaypoint("task2/gps5", 0.0, 0.0),
      courseWaypoint("task2/gate1_red", 18.939, 2.5),
      courseWaypoint("task2/gate1_center", 18.939, 0.0),
      courseWaypoint("task2/gate1_green", 18.939, -2.5),
      courseWaypoint("task2/gate2_red", 38.939, 2.5),
      courseWaypoint("task2/gate2_center", 38.939, 0.0),
      courseWaypoint("task2/gate2_green", 38.939, -2.5),
      {"task2/gps6", 63.4416044, 10.4238816},
      // Task 3.1: buoy corridor -> GPS 8 gate -> normal-dock mouth/berth.
      courseWaypoint("task3_1/gps7", 0.0, 0.0),
      courseWaypoint("task3_1/corridor_gate", 12.0, 0.0),
      courseWaypoint("task3_1/gps8_gate", 18.0, 10.0),
      courseWaypoint("task3_1/berth_approach", 18.25, 1.065),
      courseWaypoint("task3_1/berth", 20.0, 1.065),
      courseWaypoint("task3_1/undock_exit", 18.25, 1.065),
      // Task 3.2 is its own point-symmetric, parallel-dock course.
      courseWaypoint("task3_2/gps9", 0.0, 0.0),
      courseWaypoint("task3_2/corridor_gate", -12.0, 0.0),
      courseWaypoint("task3_2/gps9_gate", -18.0, -10.0),
      courseWaypoint("task3_2/berth_approach", -18.25, 0.0),
      courseWaypoint("task3_2/berth", -20.0, 0.0),
      courseWaypoint("task3_2/gps10_finish", -18.0, -11.0),
    };

    rclcpp::QoS qos(1);
    for (const auto & waypoint : waypoints) {
      publishers_.emplace_back(
        waypoint,
        create_publisher<sensor_msgs::msg::NavSatFix>(
          "/visualization/norway_waypoints/" + waypoint.topic_suffix,
          qos.transient_local().reliable()));
    }

    publish();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NorwayWaypointPublisher::publish, this));
  }

private:
  void publish()
  {
    const auto stamp = get_clock()->now();
    for (const auto & [waypoint, publisher] : publishers_) {
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = stamp;
      fix.header.frame_id = "wgs84";
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      fix.latitude = waypoint.latitude_degrees;
      fix.longitude = waypoint.longitude_degrees;
      fix.altitude = 0.0;
      fix.position_covariance = std::array<double, 9>{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 4.0};
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      publisher->publish(fix);
    }
  }

  using PublisherEntry = std::pair<Waypoint, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr>;
  std::vector<PublisherEntry> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NorwayWaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}
