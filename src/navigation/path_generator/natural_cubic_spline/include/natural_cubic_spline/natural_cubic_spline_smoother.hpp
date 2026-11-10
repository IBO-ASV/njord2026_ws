#ifndef NATURAL_CUBIC_SPLINE_SMOOTHER_HPP
#define NATURAL_CUBIC_SPLINE_SMOOTHER_HPP

#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/smoother.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace natural_cubic_spline {

struct SplineSegment {
  double a0, a1, a2, a3;  // Coefficients for the cubic equation
};

class NaturalCubicSplineSmoother : public nav2_core::Smoother {
public:
  NaturalCubicSplineSmoother() = default;
  ~NaturalCubicSplineSmoother() override = default;

  // Plugin lifecycle methods
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  /**
   * @brief Method to smooth given path
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

private:
  // ROS 2 components
  rclcpp::Logger logger_{rclcpp::get_logger("NaturalCubicSplineSmoother")};
  std::string name_;

  // Configuration parameters
  double path_point_spacing_ = 0.05;    // Distance between output path points (meters)
  double control_point_spacing_ = 0.5;  // Distance between control points (meters)
  rclcpp::Clock::SharedPtr clock_;

  /**
   * @brief Solve the natural cubic spline system for a single coordinate axis.
   * @param points The coordinate values at control points
   * @return Vector of SplineSegment objects for each interval
   */
  std::vector<SplineSegment> solveNaturalCubicSpline(
    const std::vector<double> & points);

  /**
   * @brief Evaluate a spline segment at a normalized parameter u ∈ [0, 1]
   * @param segment The cubic spline segment
   * @param u Normalized parameter
   * @return Interpolated value
   */
  double evaluateSpline(const SplineSegment & segment, double u);
};

}  // namespace natural_cubic_spline

#endif  // NATURAL_CUBIC_SPLINE_SMOOTHER_HPP
