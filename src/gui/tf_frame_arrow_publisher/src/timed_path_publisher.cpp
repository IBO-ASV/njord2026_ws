#include <memory>
#include <deque>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

// 履歴として保存するデータ構造 (時刻と座標)
struct PathPoint {
  rclcpp::Time timestamp;
  geometry_msgs::msg::Point point;
};

class TimedPathPublisher : public rclcpp::Node
{
public:
  TimedPathPublisher()
  : Node("timed_path_publisher")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("timed_path_marker", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 更新頻度 (20Hz) - 滑らかに見えるよう少し早め
    timer_ = this->create_wall_timer(
      50ms, std::bind(&TimedPathPublisher::timer_callback, this));

    // 保持期間の設定 (例: 5秒間)
    history_duration_ = 5.0;
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return;
    }

    auto now = this->now();

    // 現在位置を記録
    PathPoint current_pp;
    current_pp.timestamp = now;
    current_pp.point.x = t.transform.translation.x;
    current_pp.point.y = t.transform.translation.y;
    current_pp.point.z = t.transform.translation.z;

    // 【軽量化】前回位置からあまり動いていなければ追加しない (ただし時間は更新しても良いが今回はスキップ)
    if (!path_queue_.empty()) {
      auto last_p = path_queue_.back().point;
      double dist = std::hypot(current_pp.point.x - last_p.x, current_pp.point.y - last_p.y);
      if (dist > 0.02) { // 2cm以上動いたら追加
         path_queue_.push_back(current_pp);
      }
    } else {
      path_queue_.push_back(current_pp);
    }

    // 古いデータを削除する処理
    // "現在時刻 - データの時刻" が 設定時間を超えていたら削除
    while (!path_queue_.empty()) {
      double age = (now - path_queue_.front().timestamp).seconds();
      if (age > history_duration_) {
        path_queue_.pop_front();
      } else {
        break; // 先頭が新しければ、それ以降も新しいので終了
      }
    }

    // マーカー作成
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = now;
    marker.ns = "timed_path";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.03; // 線は少し細め (3cm)

    // 色 (黄色)
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    // dequeからMarker用配列へコピー
    for (const auto & pp : path_queue_) {
      marker.points.push_back(pp.point);
    }

    publisher_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<PathPoint> path_queue_;
  double history_duration_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimedPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
