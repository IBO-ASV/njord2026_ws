#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class CmdVelMarkerPublisher : public rclcpp::Node
{
public:
  CmdVelMarkerPublisher()
  : Node("cmd_vel_marker_publisher")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/cmd_vel_markers");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    display_seconds_ = declare_parameter<double>("display_seconds", 1.0);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 20, std::bind(&CmdVelMarkerPublisher::callback, this, std::placeholders::_1));
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

private:
  visualization_msgs::msg::Marker marker(int id, int type) const
  {
    visualization_msgs::msg::Marker value;
    value.header.frame_id = frame_id_;
    value.header.stamp = now();
    value.ns = "cmd_vel";
    value.id = id;
    value.type = type;
    value.action = visualization_msgs::msg::Marker::ADD;
    value.pose.orientation.w = 1.0;
    value.color.a = 1.0;
    return value;
  }

  static geometry_msgs::msg::Point point(double x, double y, double z = 0.05)
  {
    geometry_msgs::msg::Point value;
    value.x = x;
    value.y = y;
    value.z = z;
    return value;
  }

  void callback(const geometry_msgs::msg::Twist::SharedPtr command)
  {
    constexpr double kRadius = 10.0;
    constexpr int kSegments = 48;
    constexpr double kMotionEpsilon = 1e-3;
    visualization_msgs::msg::MarkerArray markers;
    const auto erase = [this, &markers](int id) {
      auto value = marker(id, visualization_msgs::msg::Marker::ARROW);
      value.action = visualization_msgs::msg::Marker::DELETE;
      markers.markers.push_back(value);
    };

    if (std::hypot(command->linear.x, command->linear.y) > kMotionEpsilon) {
      auto translation = marker(0, visualization_msgs::msg::Marker::ARROW);
      translation.scale.x = 0.20;
      translation.scale.y = 0.40;
      translation.scale.z = 0.40;
      translation.color.g = 1.0;
      translation.color.b = 0.2;
      translation.points = {point(0.0, 0.0), point(
        command->linear.x * display_seconds_, command->linear.y * display_seconds_)};
      markers.markers.push_back(translation);
    } else {
      erase(0);
    }

    auto circle = marker(1, visualization_msgs::msg::Marker::LINE_STRIP);
    circle.scale.x = 0.05;
    circle.color.r = circle.color.g = circle.color.b = 0.65;
    for (int index = 0; index <= kSegments; ++index) {
      const double angle = 2.0 * M_PI * index / kSegments;
      circle.points.push_back(point(kRadius * std::cos(angle), kRadius * std::sin(angle), 0.02));
    }
    markers.markers.push_back(circle);

    const double sweep = std::clamp(command->angular.z * display_seconds_, -2.0 * M_PI, 2.0 * M_PI);
    if (std::abs(sweep) > kMotionEpsilon) {
      auto arc = marker(2, visualization_msgs::msg::Marker::LINE_STRIP);
      arc.scale.x = 0.15;
      arc.color.r = 1.0;
      arc.color.g = 0.45;
      const int arc_segments = std::max(1, static_cast<int>(kSegments * std::abs(sweep) / (2.0 * M_PI)));
      for (int index = 0; index <= arc_segments; ++index) {
        const double angle = sweep * index / arc_segments;
        arc.points.push_back(point(kRadius * std::cos(angle), kRadius * std::sin(angle), 0.04));
      }
      markers.markers.push_back(arc);

      auto turn_head = marker(3, visualization_msgs::msg::Marker::ARROW);
      turn_head.scale.x = 0.20;
      turn_head.scale.y = 0.40;
      turn_head.scale.z = 0.40;
      turn_head.color.r = 1.0;
      turn_head.color.g = 0.45;
      const double direction = sweep >= 0.0 ? 1.0 : -1.0;
      const double end_x = kRadius * std::cos(sweep);
      const double end_y = kRadius * std::sin(sweep);
      turn_head.points = {point(
        end_x - 0.50 * direction * std::sin(sweep), end_y + 0.50 * direction * std::cos(sweep), 0.04),
        point(end_x, end_y, 0.04)};
      markers.markers.push_back(turn_head);
    } else {
      erase(2);
      erase(3);
    }
    pub_->publish(markers);
  }

  std::string cmd_vel_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  double display_seconds_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
