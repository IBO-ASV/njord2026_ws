#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class CircleTfBroadcaster : public rclcpp::Node
{
public:
  CircleTfBroadcaster()
  : Node("circle_tf_broadcaster")
  {
    // TF配信用のブロードキャスターを作成
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 50Hz (20ms) で更新
    timer_ = this->create_wall_timer(
      20ms, std::bind(&CircleTfBroadcaster::timer_callback, this));

    // 初期化
    theta_ = 0.0;
  }

private:
  void timer_callback()
  {
    // 時間経過(dt)と角速度(omega)から角度を更新
    // 半径1mで、適度な速度(0.5 rad/s)で回る設定
    double dt = 0.02;     // 20ms = 0.02s
    double omega = 0.5;   // 角速度 rad/s
    double radius = 1.0;  // 半径 1m

    theta_ += omega * dt;

    // 角度が 2pi を超えたらリセット（数値安定のため）
    if (theta_ > 2.0 * M_PI) {
      theta_ -= 2.0 * M_PI;
    }

    // --- 座標計算 ---
    // 原点(0,0)からスタートして、左(Y軸方向)へ旋回していく軌道
    double x = radius * std::sin(theta_);
    double y = radius * (1.0 - std::cos(theta_));

    // --- TFメッセージの作成 ---
    geometry_msgs::msg::TransformStamped t;

    // ヘッダー情報
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";       // 親フレーム（固定）
    t.child_frame_id = "base_link";   // 子フレーム（動くロボット）

    // 並進 (Translation)
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    // 回転 (Rotation) -> クォータニオン変換
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_); // Roll, Pitch, Yaw
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // --- 配信 ---
    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
