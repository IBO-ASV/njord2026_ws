#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class FullPathPublisher : public rclcpp::Node
{
public:
  FullPathPublisher()
  : Node("full_path_publisher")
  {
    marker_topic_ = declare_parameter<std::string>("marker_topic", "actual_path_marker");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

    // TFリスナーのセットアップ (odom座標を取得するため)
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 0.1秒ごとに位置をチェックして軌跡を更新
    timer_ = this->create_wall_timer(
      100ms, std::bind(&FullPathPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;

    try {
      // "odom" フレームから見た "base_link" の位置を取得
      t = tf_buffer_->lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      // TFがまだ来ていないときはスキップ
      return;
    }

    geometry_msgs::msg::Point p;
    p.x = t.transform.translation.x;
    p.y = t.transform.translation.y;
    p.z = t.transform.translation.z; // 地面スレスレにしたければ 0.0 固定でも可

    // 【軽量化】前回記録した点から十分(5cm)動いていなければ記録しない
    if (!points_.empty()) {
      double dist = std::hypot(p.x - points_.back().x, p.y - points_.back().y);
      if (dist < 0.05) {
        return;
      }
    }
    points_.push_back(p);

    // マーカー作成
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = parent_frame_;
    marker.header.stamp = this->now();
    marker.ns = "path_history";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 線でつなぐ
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.025; // 2.5 cm: planned route remains visually dominant.

    // Actual route: thin purple line.
    marker.color.r = 0.72;
    marker.color.g = 0.20;
    marker.color.b = 0.90;
    marker.color.a = 1.0;

    marker.points = points_; // 蓄積した全ての点を入れる

    publisher_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 軌跡の点をすべて保存するベクター
  std::vector<geometry_msgs::msg::Point> points_;
  std::string marker_topic_;
  std::string parent_frame_;
  std::string child_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FullPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
