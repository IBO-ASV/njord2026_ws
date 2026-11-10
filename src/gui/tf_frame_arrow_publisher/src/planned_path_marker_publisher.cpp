#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PlannedPathMarkerPublisher : public rclcpp::Node
{
public:
  PlannedPathMarkerPublisher()
  : Node("planned_path_marker_publisher")
  {
    path_topic_ = declare_parameter<std::string>("path_topic", "/plan_smoothed");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "planned_path_marker");
    sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10, std::bind(&PlannedPathMarkerPublisher::path_callback, this, std::placeholders::_1));
    pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
  }

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr path)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path->header;
    marker.ns = "planned_route";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.r = 0.25;
    marker.color.g = 1.0;
    marker.color.b = 0.35;
    marker.color.a = 1.0;
    marker.points.reserve(path->poses.size());
    for (const auto & pose : path->poses) {
      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = pose.pose.position.z + 0.03;
      marker.points.push_back(point);
    }
    pub_->publish(marker);
  }

  std::string path_topic_;
  std::string marker_topic_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannedPathMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
