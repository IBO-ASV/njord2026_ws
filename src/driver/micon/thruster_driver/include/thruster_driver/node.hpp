#ifndef THRUSTER_DRIVER_NODE_HPP_
#define THRUSTER_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <string>
#include <vector>

namespace njord
{
namespace thruster_driver
{

class ThrusterDriverNode : public rclcpp::Node
{
public:
  explicit ThrusterDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct ThrusterConfig
  {
    std::string id;
    std::string link;
    double x{0.0};
    double y{0.0};
    double angle_rad{0.0};
    double max_thrust{40.0};
    double force_per_duty{1.0};
    double forward_gain{1.0};
    double reverse_gain{1.0};
    double offset{0.0};
    bool reverse{false};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void dutyArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlTimerCallback();

  void loadThrusterConfigs();
  void loadThrusterPosesFromUrdf(const std::string & robot_description);
  void validateThrusterConfigs() const;

  std::vector<double> computeWrench(double dt);
  std::vector<double> allocateWrench(const std::vector<double> & wrench) const;
  std::vector<double> computeOmniWheelDuty() const;
  double applyStaticMap(double value, const ThrusterConfig & thruster) const;
  double applyDeadzone(double value, double deadzone) const;
  void publishCommands(const std::vector<double> & commands);
  double clamp(double value, double min_value, double max_value) const;
  std::vector<double> getDoubleVector(
    const std::string & name,
    const std::vector<double> & defaults);
  std::vector<std::string> getStringVector(
    const std::string & name,
    const std::vector<std::string> & defaults);
  std::vector<bool> getBoolVector(const std::string & name, const std::vector<bool> & defaults);
  static std::string toLower(std::string value);

  // Input subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_duty_array_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_thruster_command_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_current_force_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_dob_estimate_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string input_mode_;

  std::vector<ThrusterConfig> thrusters_;

  double max_linear_x_{1.0};
  double max_linear_y_{1.0};
  double max_angular_z_{1.0};
  double control_rate_hz_{50.0};
  double watchdog_timeout_sec_{0.5};
  bool use_velocity_feedback_{true};
  double feedback_timeout_sec_{0.5};
  bool stop_on_feedback_timeout_{true};

  double kp_surge_{1.0};
  double kp_sway_{1.0};
  double kp_yaw_{1.0};
  double max_surge_wrench_{1.0};
  double max_sway_wrench_{1.0};
  double max_yaw_wrench_{1.0};

  bool dob_enable_{false};
  double dob_observer_gain_{1.0};
  double dob_filter_tau_{0.1};
  double mass_{10.0};
  double iz_{1.0};
  double damping_linear_surge_{0.0};
  double damping_linear_sway_{0.0};
  double damping_linear_yaw_{0.0};
  double damping_quadratic_surge_{0.0};
  double damping_quadratic_sway_{0.0};
  double damping_quadratic_yaw_{0.0};

  double allocation_regularization_{1e-4};
  std::vector<double> allocation_wrench_sign_{1.0, 1.0, 1.0};
  double deadzone_pos_{0.0};
  double deadzone_neg_{0.0};

  int duty_resolution_{1000};

  // `force_newton` publishes the allocated command multiplied by each
  // thruster's calibrated max_thrust.  `duty_ratio` publishes the normalized
  // command directly, clamped by output.duty_limit.  The latter is for motor
  // drivers such as MD10C that do not accept a force command.
  std::string output_mode_{"force_newton"};
  double duty_limit_{1.0};
  bool actuator_configuration_confirmed_{true};
  std::string actuator_model_{"wrench_allocation"};
  double omni_wheel_radius_m_{0.0};
  std::vector<double> omni_duty_per_wheel_rad_s_;

  geometry_msgs::msg::Twist latest_cmd_;
  double meas_surge_{0.0};
  double meas_sway_{0.0};
  double meas_yaw_{0.0};

  double prev_meas_surge_{0.0};
  double prev_meas_sway_{0.0};
  double prev_meas_yaw_{0.0};
  bool have_prev_meas_{false};
  bool have_feedback_{false};

  std::vector<double> prev_wrench_{0.0, 0.0, 0.0};
  std::vector<double> dob_hat_{0.0, 0.0, 0.0};
  std::vector<double> dob_lpf_{0.0, 0.0, 0.0};

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_feedback_time_;
  rclcpp::Time last_control_time_;
};

}  // namespace thruster_driver
}  // namespace njord

#endif  // THRUSTER_DRIVER_NODE_HPP_
