#include <chrono>
#include <array>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace
{
constexpr size_t kNumThrusters = 4;
constexpr std::chrono::seconds kHoldDuration{10};
constexpr std::array<const char *, kNumThrusters> kThrusterIds = {"RR", "RL", "FR", "FL"};
}  // namespace

class ThrusterIdTest : public rclcpp::Node
{
public:
  ThrusterIdTest()
  : Node("thruster_id_test")
  {
    thrust_value_ = declare_parameter<double>("thrust_value", 2.0);
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/thruster_command", 10);
    start_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&ThrusterIdTest::timer_cb, this));
  }

private:
  void timer_cb()
  {
    const auto elapsed = now() - start_time_;
    const auto hold_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kHoldDuration).count();
    const size_t index =
      static_cast<size_t>(elapsed.nanoseconds() / hold_ns) % kNumThrusters;

    if (index != last_index_) {
      RCLCPP_INFO(
        get_logger(), "Testing thruster channel %zu (%s)", index, kThrusterIds[index]);
      last_index_ = index;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(kNumThrusters, 0.0f);
    msg.data[index] = static_cast<float>(thrust_value_);
    pub_->publish(msg);
  }

  double thrust_value_{0.7};
  size_t last_index_{kNumThrusters};
  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterIdTest>());
  rclcpp::shutdown();
  return 0;
}
