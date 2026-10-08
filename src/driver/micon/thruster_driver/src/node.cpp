#include "thruster_driver/node.hpp"

#include "thruster_driver/allocation.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>

namespace njord
{
namespace thruster_driver
{

namespace
{
std::string captureFirst(const std::string & text, const std::regex & pattern)
{
  std::smatch match;
  if (std::regex_search(text, match, pattern) && match.size() > 1U) {
    return match[1].str();
  }
  return {};
}

std::vector<double> parseDoubleList(const std::string & text)
{
  std::vector<double> values;
  const std::regex number_pattern(R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), number_pattern);
    it != std::sregex_iterator();
    ++it)
  {
    values.push_back(std::stod(it->str()));
  }
  return values;
}
}  // namespace

ThrusterDriverNode::ThrusterDriverNode(const rclcpp::NodeOptions & options)
: Node("thruster_driver_node", options),
  last_cmd_time_(this->now()),
  last_feedback_time_(this->now()),
  last_control_time_(this->now())
{
  input_mode_ = toLower(this->declare_parameter<std::string>("input_mode", "cmd_vel"));

  control_rate_hz_ = this->declare_parameter<double>("control.rate_hz", 50.0);
  max_linear_x_ = this->declare_parameter<double>("input_scaling.max_linear_x", 1.0);
  max_linear_y_ = this->declare_parameter<double>("input_scaling.max_linear_y", 1.0);
  max_angular_z_ = this->declare_parameter<double>("input_scaling.max_angular_z", 1.0);
  if (!std::isfinite(max_linear_x_) || max_linear_x_ <= 0.0 ||
    !std::isfinite(max_linear_y_) || max_linear_y_ <= 0.0 ||
    !std::isfinite(max_angular_z_) || max_angular_z_ <= 0.0)
  {
    throw std::runtime_error("input_scaling limits must be finite and greater than zero");
  }
  watchdog_timeout_sec_ = this->declare_parameter<double>("safety.watchdog_timeout_sec", 0.5);
  use_velocity_feedback_ =
    this->declare_parameter<bool>("control.use_velocity_feedback", true);
  feedback_timeout_sec_ = this->declare_parameter<double>("control.feedback_timeout_sec", 0.5);
  stop_on_feedback_timeout_ =
    this->declare_parameter<bool>("control.stop_on_feedback_timeout", true);

  kp_surge_ = this->declare_parameter<double>("control.p.surge", 1.0);
  kp_sway_ = this->declare_parameter<double>("control.p.sway", 1.0);
  kp_yaw_ = this->declare_parameter<double>("control.p.yaw", 1.0);
  max_surge_wrench_ = std::max(
    1.0, this->declare_parameter<double>("control.max_surge_wrench", 1.0));
  max_sway_wrench_ = std::max(
    1.0, this->declare_parameter<double>("control.max_sway_wrench", 1.0));
  max_yaw_wrench_ = std::max(
    1.0, this->declare_parameter<double>("control.max_yaw_wrench", 1.0));

  dob_enable_ = this->declare_parameter<bool>("control.dob.enable", false);
  dob_observer_gain_ = this->declare_parameter<double>("control.dob.observer_gain", 1.0);
  dob_filter_tau_ = this->declare_parameter<double>("control.dob.filter_tau_sec", 0.1);

  mass_ = this->declare_parameter<double>("control.nominal.mass", 10.0);
  iz_ = this->declare_parameter<double>("control.nominal.iz", 1.0);
  damping_linear_surge_ = this->declare_parameter<double>(
    "control.nominal.damping.linear.surge",
    0.0);
  damping_linear_sway_ =
    this->declare_parameter<double>("control.nominal.damping.linear.sway", 0.0);
  damping_linear_yaw_ = this->declare_parameter<double>("control.nominal.damping.linear.yaw", 0.0);
  damping_quadratic_surge_ = this->declare_parameter<double>(
    "control.nominal.damping.quadratic.surge", 0.0);
  damping_quadratic_sway_ = this->declare_parameter<double>(
    "control.nominal.damping.quadratic.sway", 0.0);
  damping_quadratic_yaw_ = this->declare_parameter<double>(
    "control.nominal.damping.quadratic.yaw",
    0.0);

  allocation_regularization_ = std::max(
    1e-9,
    this->declare_parameter<double>("allocation.regularization_lambda", 1e-4));
  allocation_wrench_sign_ = getDoubleVector(
    "allocation.wrench_sign", {1.0, 1.0, 1.0});
  if (allocation_wrench_sign_.size() != 3U ||
    !std::all_of(
      allocation_wrench_sign_.begin(), allocation_wrench_sign_.end(),
      [](double sign) {return sign == -1.0 || sign == 1.0;}))
  {
    throw std::runtime_error("allocation.wrench_sign must contain three values, each -1 or 1");
  }
  deadzone_pos_ = this->declare_parameter<double>("static_map.deadzone_pos", 0.0);
  deadzone_neg_ = this->declare_parameter<double>("static_map.deadzone_neg", 0.0);

  duty_resolution_ = this->declare_parameter<int>("duty_resolution", 1000);
  output_mode_ = toLower(
    this->declare_parameter<std::string>("output.mode", "force_newton"));
  duty_limit_ = clamp(
    this->declare_parameter<double>("output.duty_limit", 1.0), 0.0, 1.0);
  actuator_configuration_confirmed_ = this->declare_parameter<bool>(
    "safety.actuator_configuration_confirmed", true);
  if (output_mode_ != "force_newton" && output_mode_ != "duty_ratio") {
    throw std::runtime_error("output.mode must be 'force_newton' or 'duty_ratio'");
  }
  actuator_model_ = toLower(
    this->declare_parameter<std::string>("actuator_model", "wrench_allocation"));
  if (actuator_model_ != "wrench_allocation" && actuator_model_ != "omni_wheel_duty") {
    throw std::runtime_error(
            "actuator_model must be 'wrench_allocation' or 'omni_wheel_duty'");
  }

  loadThrusterConfigs();
  const std::string robot_description =
    this->declare_parameter<std::string>("robot_description", "");
  loadThrusterPosesFromUrdf(robot_description);
  if (actuator_model_ == "omni_wheel_duty") {
    omni_wheel_radius_m_ = this->declare_parameter<double>("omni.wheel_radius_m", 0.0);
    omni_duty_per_wheel_rad_s_ = getDoubleVector(
      "omni.duty_per_wheel_rad_s", std::vector<double>(thrusters_.size(), 0.0));
  }
  validateThrusterConfigs();

  const std::string cmd_vel_topic =
    this->declare_parameter<std::string>("topics.cmd_vel", "cmd_vel");
  const std::string duty_array_topic =
    this->declare_parameter<std::string>("topics.duty_array", "thruster_command");
  const std::string odom_topic =
    this->declare_parameter<std::string>("topics.feedback_odometry", "/odometry/filtered/local");
  const std::string sim_command_topic =
    this->declare_parameter<std::string>("topics.sim_thruster_command", "/thruster_command");

  pub_thruster_command_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>(sim_command_topic, 10);

  pub_current_force_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>(
      output_mode_ == "duty_ratio" ? "/debug/current_duty" : "/debug/current_force", 10);
  pub_dob_estimate_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>("/debug/dob_estimate", 10);

  if (input_mode_ == "cmd_vel") {
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      10,
      std::bind(&ThrusterDriverNode::cmdVelCallback, this, std::placeholders::_1));
    if (use_velocity_feedback_) {
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        10,
        std::bind(&ThrusterDriverNode::odomCallback, this, std::placeholders::_1));
    }
    const double period_sec = 1.0 / std::max(1.0, control_rate_hz_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          period_sec)),
      std::bind(&ThrusterDriverNode::controlTimerCallback, this));
  } else if (input_mode_ == "duty_array") {
    sub_duty_array_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      duty_array_topic,
      10,
      std::bind(&ThrusterDriverNode::dutyArrayCallback, this, std::placeholders::_1));
  } else {
    throw std::runtime_error("input_mode must be 'cmd_vel' or 'duty_array'");
  }

  RCLCPP_INFO(
    this->get_logger(),
    "thruster_driver started. mode=%s output=%s/%s model=%s thrusters=%zu velocity_feedback=%s dob=%s",
    input_mode_.c_str(),
    sim_command_topic.c_str(),
    output_mode_.c_str(),
    actuator_model_.c_str(),
    thrusters_.size(),
    use_velocity_feedback_ ? "on" : "off",
    dob_enable_ ? "on" : "off");
}

void ThrusterDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_ = *msg;
  last_cmd_time_ = this->now();
}

void ThrusterDriverNode::dutyArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  if (msg->data.size() != thrusters_.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Received duty_array with size=%zu. Expected %zu.",
      msg->data.size(),
      thrusters_.size());
    return;
  }

  const double resolution = std::max(1, duty_resolution_);
  std::vector<double> commands;
  commands.reserve(msg->data.size());
  for (const auto value : msg->data) {
    commands.push_back(clamp(static_cast<double>(value) / resolution, -1.0, 1.0));
  }
  publishCommands(commands);
}

void ThrusterDriverNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  meas_surge_ = msg->twist.twist.linear.x;
  meas_sway_ = msg->twist.twist.linear.y;
  meas_yaw_ = msg->twist.twist.angular.z;
  have_feedback_ = true;
  last_feedback_time_ = this->now();
}

void ThrusterDriverNode::controlTimerCallback()
{
  const auto now = this->now();
  const double dt = std::max(1e-4, (now - last_control_time_).seconds());
  last_control_time_ = now;

  const bool cmd_timeout = (now - last_cmd_time_).seconds() > watchdog_timeout_sec_;
  const bool feedback_timeout = use_velocity_feedback_ &&
    (!have_feedback_ || (now - last_feedback_time_).seconds() > feedback_timeout_sec_);

  if (cmd_timeout || (feedback_timeout && stop_on_feedback_timeout_)) {
    publishCommands(std::vector<double>(thrusters_.size(), 0.0));
    have_prev_meas_ = false;
    prev_wrench_ = {0.0, 0.0, 0.0};
    return;
  }

  std::vector<double> commands;
  if (actuator_model_ == "omni_wheel_duty") {
    commands = computeOmniWheelDuty();
  } else {
    const std::vector<double> wrench = computeWrench(dt);
    std::vector<double> allocation_wrench = wrench;
    for (std::size_t i = 0; i < allocation_wrench.size(); ++i) {
      allocation_wrench[i] *= allocation_wrench_sign_[i];
    }
    commands = allocateWrench(allocation_wrench);
    prev_wrench_ = wrench;
  }

  for (std::size_t i = 0; i < commands.size(); ++i) {
    commands[i] = applyStaticMap(commands[i], thrusters_[i]);
  }

  publishCommands(commands);
}

void ThrusterDriverNode::loadThrusterConfigs()
{
  const std::vector<std::string> ids =
    getStringVector("thrusters.ids", {"FR", "FL", "RR", "RL"});
  const std::vector<std::string> links = getStringVector(
    "thrusters.links",
    {"thruster_FR", "thruster_FL", "thruster_RR", "thruster_RL"});
  const std::vector<double> angle_rad = getDoubleVector(
    "thrusters.angle_rad",
    {0.785398, -0.785398, 2.356194, -2.356194});
  const std::vector<double> max_thrust =
    getDoubleVector("thrusters.max_thrust", std::vector<double>(ids.size(), 40.0));
  const std::vector<bool> reverse =
    getBoolVector("thrusters.reverse", std::vector<bool>(ids.size(), false));
  const std::vector<double> forward_gain =
    getDoubleVector("static_map.thrusters.forward_gain", std::vector<double>(ids.size(), 1.0));
  const std::vector<double> reverse_gain =
    getDoubleVector("static_map.thrusters.reverse_gain", std::vector<double>(ids.size(), 1.0));
  const std::vector<double> offset =
    getDoubleVector("static_map.thrusters.offset", std::vector<double>(ids.size(), 0.0));

  const std::size_t n = ids.size();
  if (
    links.size() != n || angle_rad.size() != n || max_thrust.size() != n ||
    reverse.size() != n || forward_gain.size() != n || reverse_gain.size() != n ||
    offset.size() != n)
  {
    throw std::runtime_error("All thruster config arrays must match thrusters.ids size");
  }

  thrusters_.clear();
  thrusters_.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    ThrusterConfig config;
    config.id = ids[i];
    config.link = links[i];
    config.angle_rad = angle_rad[i];
    config.max_thrust = max_thrust[i];
    config.reverse = reverse[i];
    config.forward_gain = forward_gain[i];
    config.reverse_gain = reverse_gain[i];
    config.offset = offset[i];
    thrusters_.push_back(config);
  }
}

void ThrusterDriverNode::loadThrusterPosesFromUrdf(const std::string & robot_description)
{
  if (robot_description.empty()) {
    throw std::runtime_error("robot_description parameter is required for thruster pose loading");
  }

  const std::regex joint_pattern(R"(<joint\b[^>]*>[\s\S]*?<\/joint>)");
  const std::regex parent_pattern(R"(<parent\s+link=["']([^"']+)["'])");
  const std::regex child_pattern(R"(<child\s+link=["']([^"']+)["'])");
  const std::regex origin_pattern(R"(<origin\b[^>]*xyz=["']([^"']+)["'])");

  std::vector<bool> found(thrusters_.size(), false);
  for (auto it =
    std::sregex_iterator(robot_description.begin(), robot_description.end(), joint_pattern);
    it != std::sregex_iterator();
    ++it)
  {
    const std::string joint_xml = it->str();
    const std::string parent = captureFirst(joint_xml, parent_pattern);
    const std::string child = captureFirst(joint_xml, child_pattern);
    if (parent != "base_link" || child.empty()) {
      continue;
    }

    for (std::size_t i = 0; i < thrusters_.size(); ++i) {
      if (thrusters_[i].link != child) {
        continue;
      }
      const std::string xyz_text = captureFirst(joint_xml, origin_pattern);
      const std::vector<double> xyz = parseDoubleList(xyz_text);
      if (xyz.size() < 2U) {
        throw std::runtime_error("Thruster joint for " + child + " must include origin xyz");
      }
      thrusters_[i].x = xyz[0];
      thrusters_[i].y = xyz[1];
      found[i] = true;
    }
  }

  for (std::size_t i = 0; i < thrusters_.size(); ++i) {
    if (!found[i]) {
      throw std::runtime_error(
              "Missing fixed base_link joint for thruster link: " + thrusters_[i].link);
    }
  }
}

void ThrusterDriverNode::validateThrusterConfigs() const
{
  if (thrusters_.size() < 3U) {
    throw std::runtime_error("At least three thrusters are required for surge/sway/yaw allocation");
  }

  if (!actuator_configuration_confirmed_) {
    throw std::runtime_error(
            "Actuator configuration is intentionally locked. Verify physical wheel direction and "
            "set safety.actuator_configuration_confirmed:=true before enabling this profile.");
  }

  if (actuator_model_ == "omni_wheel_duty") {
    if (!std::isfinite(omni_wheel_radius_m_) || omni_wheel_radius_m_ <= 0.0 ||
      omni_duty_per_wheel_rad_s_.size() != thrusters_.size())
    {
      throw std::runtime_error(
              "omni.wheel_radius_m and omni.duty_per_wheel_rad_s must match the wheel config");
    }
    for (const double gain : omni_duty_per_wheel_rad_s_) {
      if (!std::isfinite(gain) || gain < 0.0) {
        throw std::runtime_error("omni.duty_per_wheel_rad_s must be finite and non-negative");
      }
    }
  }

  for (std::size_t i = 0; i < thrusters_.size(); ++i) {
    for (std::size_t j = i + 1U; j < thrusters_.size(); ++j) {
      if (thrusters_[i].id == thrusters_[j].id || thrusters_[i].link == thrusters_[j].link) {
        throw std::runtime_error("Thruster ids and links must be unique");
      }
    }
  }
}

std::vector<double> ThrusterDriverNode::computeWrench(double dt)
{
  const double ref_surge = clamp(latest_cmd_.linear.x, -max_linear_x_, max_linear_x_);
  const double ref_sway = clamp(latest_cmd_.linear.y, -max_linear_y_, max_linear_y_);
  const double ref_yaw = clamp(latest_cmd_.angular.z, -max_angular_z_, max_angular_z_);

  std::vector<double> wrench = {
    kp_surge_ * (ref_surge - meas_surge_),
    kp_sway_ * (ref_sway - meas_sway_),
    kp_yaw_ * (ref_yaw - meas_yaw_)};

  if (dob_enable_ && have_prev_meas_) {
    const std::vector<double> meas = {meas_surge_, meas_sway_, meas_yaw_};
    const std::vector<double> prev = {prev_meas_surge_, prev_meas_sway_, prev_meas_yaw_};
    const std::vector<double> inertia = {mass_, mass_, iz_};
    const std::vector<double> damping_linear = {
      damping_linear_surge_, damping_linear_sway_, damping_linear_yaw_};
    const std::vector<double> damping_quadratic = {
      damping_quadratic_surge_, damping_quadratic_sway_, damping_quadratic_yaw_};

    const double alpha = dt / (std::max(1e-6, dob_filter_tau_) + dt);
    for (std::size_t i = 0; i < 3U; ++i) {
      const double measured_dot = (meas[i] - prev[i]) / dt;
      const double damping =
        -(damping_linear[i] * meas[i] + damping_quadratic[i] * std::abs(meas[i]) * meas[i]);
      dob_hat_[i] = inertia[i] * measured_dot - prev_wrench_[i] + damping;
      dob_lpf_[i] += alpha * (dob_hat_[i] - dob_lpf_[i]);
      wrench[i] = clamp(wrench[i] + dob_observer_gain_ * dob_lpf_[i], -1.0, 1.0);
    }
  } else {
    wrench[0] = clamp(wrench[0], -max_surge_wrench_, max_surge_wrench_);
    wrench[1] = clamp(wrench[1], -max_sway_wrench_, max_sway_wrench_);
    wrench[2] = clamp(wrench[2], -max_yaw_wrench_, max_yaw_wrench_);
  }

  prev_meas_surge_ = meas_surge_;
  prev_meas_sway_ = meas_sway_;
  prev_meas_yaw_ = meas_yaw_;
  have_prev_meas_ = true;

  return wrench;
}

std::vector<double> ThrusterDriverNode::allocateWrench(const std::vector<double> & wrench) const
{
  std::vector<ThrusterGeometry> geometry;
  geometry.reserve(thrusters_.size());
  for (const auto & thruster : thrusters_) {
    geometry.push_back(
        {
          thruster.x, thruster.y, thruster.angle_rad, thruster.max_thrust, thruster.reverse});
  }
  return njord::thruster_driver::allocateWrench(geometry, wrench, allocation_regularization_);
}

std::vector<double> ThrusterDriverNode::computeOmniWheelDuty() const
{
  std::vector<OmniWheelGeometry> wheels;
  wheels.reserve(thrusters_.size());
  for (std::size_t i = 0; i < thrusters_.size(); ++i) {
    const auto & wheel = thrusters_[i];
    wheels.push_back({
      wheel.x, wheel.y, wheel.angle_rad, omni_wheel_radius_m_,
      omni_duty_per_wheel_rad_s_[i], wheel.reverse});
  }
  return bodyTwistToWheelDuty(
    wheels,
    clamp(latest_cmd_.linear.x, -max_linear_x_, max_linear_x_),
    clamp(latest_cmd_.linear.y, -max_linear_y_, max_linear_y_),
    clamp(latest_cmd_.angular.z, -max_angular_z_, max_angular_z_));
}

double ThrusterDriverNode::applyStaticMap(double value, const ThrusterConfig & thruster) const
{
  double mapped = value;
  if (mapped >= 0.0) {
    mapped = applyDeadzone(mapped, deadzone_pos_);
    mapped *= thruster.forward_gain;
  } else {
    mapped = applyDeadzone(mapped, deadzone_neg_);
    mapped *= thruster.reverse_gain;
  }
  mapped += thruster.offset;
  return clamp(mapped, -1.0, 1.0);
}

double ThrusterDriverNode::applyDeadzone(double value, double deadzone) const
{
  const double dz = std::max(0.0, deadzone);
  if (std::abs(value) <= dz) {
    return 0.0;
  }
  return std::copysign(std::abs(value), value);
}

void ThrusterDriverNode::publishCommands(const std::vector<double> & commands)
{
  if (pub_thruster_command_) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(commands.size());
    for (std::size_t i = 0; i < commands.size(); ++i) {
      const double normalized = clamp(commands[i], -1.0, 1.0);
      const double output = output_mode_ == "duty_ratio" ?
        clamp(normalized, -duty_limit_, duty_limit_) :
        normalized * thrusters_[i].max_thrust;
      msg.data.push_back(static_cast<float>(output));
    }
    pub_thruster_command_->publish(msg);
  }

  std_msgs::msg::Float32MultiArray force_msg;
  force_msg.data.reserve(commands.size());
  for (double command : commands) {
    force_msg.data.push_back(static_cast<float>(command));
  }
  pub_current_force_->publish(force_msg);

  std_msgs::msg::Float32MultiArray dob_msg;
  dob_msg.data = {
    static_cast<float>(dob_lpf_[0]),
    static_cast<float>(dob_lpf_[1]),
    static_cast<float>(dob_lpf_[2])};
  pub_dob_estimate_->publish(dob_msg);
}

double ThrusterDriverNode::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

std::vector<double> ThrusterDriverNode::getDoubleVector(
  const std::string & name,
  const std::vector<double> & defaults)
{
  return this->declare_parameter<std::vector<double>>(name, defaults);
}

std::vector<std::string> ThrusterDriverNode::getStringVector(
  const std::string & name,
  const std::vector<std::string> & defaults)
{
  return this->declare_parameter<std::vector<std::string>>(name, defaults);
}

std::vector<bool> ThrusterDriverNode::getBoolVector(
  const std::string & name,
  const std::vector<bool> & defaults)
{
  return this->declare_parameter<std::vector<bool>>(name, defaults);
}

std::string ThrusterDriverNode::toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

}  // namespace thruster_driver
}  // namespace njord

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(njord::thruster_driver::ThrusterDriverNode)
