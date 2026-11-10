#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class NavArrowPublisher : public rclcpp::Node
{
public:
  NavArrowPublisher()
  : Node("nav_arrow_publisher")
  {
    // 通常のPublisher設定に戻しました
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);

    // 【軽量化設定】
    // 1000ms (1秒) に1回の配信に設定。
    // マーカー自体は base_link に追従するため、この頻度でもカクつくことはありません。
    timer_ = this->create_wall_timer(
      10ms, std::bind(&NavArrowPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto marker = visualization_msgs::msg::Marker();

    // 常に最新の時刻を入れる
    marker.header.stamp = this->now();
    marker.header.frame_id = "base_link";

    marker.ns = "nav_icon";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 位置・姿勢
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 全体のスケール (TRIANGLE_LISTの場合は1.0固定が扱いやすい)
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // 色設定（シアン系）
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    marker.color.a = 0.9;

    // カーナビ風矢印（シェブロン/紙飛行機型）の定義
    // 頂点座標を変更することで形を微調整できます
    geometry_msgs::msg::Point p_tip;         // 先端
    p_tip.x = 0.5; p_tip.y = 0.0; p_tip.z = 0.0;

    geometry_msgs::msg::Point p_left;        // 左翼の後ろ
    p_left.x = -0.3; p_left.y = 0.4; p_left.z = 0.0;

    geometry_msgs::msg::Point p_center_back; // 後ろの凹み
    p_center_back.x = -0.1; p_center_back.y = 0.0; p_center_back.z = 0.0;

    geometry_msgs::msg::Point p_right;       // 右翼の後ろ
    p_right.x = -0.3; p_right.y = -0.4; p_right.z = 0.0;

    // 三角形を2つ組み合わせて形を作ります

    // 1. 左側の三角形 (先端 -> 左 -> 中心凹み)
    marker.points.push_back(p_tip);
    marker.points.push_back(p_left);
    marker.points.push_back(p_center_back);

    // 2. 右側の三角形 (先端 -> 中心凹み -> 右)
    marker.points.push_back(p_tip);
    marker.points.push_back(p_center_back);
    marker.points.push_back(p_right);

    // ライフタイムを0(無限)にしておくと、通信が途切れても表示が消えにくくなりますが、
    // 更新されないと困る場合は「1.2秒」など周期より少し長く設定することもあります。
    // 今回はupdateしているので0でOKです。
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    publisher_->publish(marker);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavArrowPublisher>());
  rclcpp::shutdown();
  return 0;
}
