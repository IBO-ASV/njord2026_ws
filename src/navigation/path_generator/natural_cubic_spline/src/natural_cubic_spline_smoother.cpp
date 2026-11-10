#include "natural_cubic_spline/natural_cubic_spline_smoother.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>

namespace natural_cubic_spline {

void NaturalCubicSplineSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
{
  (void)tf;
  (void)costmap_sub;
  (void)footprint_sub;
  name_ = name;

  auto parent_ptr = parent.lock();
  if (!parent_ptr) {
    throw std::runtime_error("Parent is invalid");
  }

  RCLCPP_INFO(logger_, "Configuring %s as Smoother Plugin", name_.c_str());
  clock_ = parent_ptr->get_clock();

  // Declare and retrieve parameters
  if (!parent_ptr->has_parameter(name_ + ".path_point_spacing")) {
    parent_ptr->declare_parameter(name_ + ".path_point_spacing", 0.05);
  }
  if (!parent_ptr->has_parameter(name_ + ".control_point_spacing")) {
    parent_ptr->declare_parameter(name_ + ".control_point_spacing", 0.5);
  }

  parent_ptr->get_parameter(name_ + ".path_point_spacing", path_point_spacing_);
  parent_ptr->get_parameter(name_ + ".control_point_spacing", control_point_spacing_);

  RCLCPP_INFO(logger_, "%s: path_point_spacing = %.3f, control_point_spacing = %.3f",
    name_.c_str(), path_point_spacing_, control_point_spacing_);
}

void NaturalCubicSplineSmoother::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void NaturalCubicSplineSmoother::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

void NaturalCubicSplineSmoother::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

bool NaturalCubicSplineSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  (void)max_time;  // Unused

  if (path.poses.size() < 3) {
    // Too few points to smooth
    return true;
  }

  // 1. Calculate cumulative distances along the input path
  double total_distance = 0.0;
  std::vector<double> accumulated_dist;
  accumulated_dist.reserve(path.poses.size());
  accumulated_dist.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    total_distance += std::sqrt(dx * dx + dy * dy);
    accumulated_dist.push_back(total_distance);
  }

  if (total_distance < 1e-4) {
    // Virtually zero-length path
    return true;
  }

  // 2. Determine number of control points
  int num_control_points = static_cast<int>(std::ceil(total_distance / control_point_spacing_)) + 1;
  num_control_points = std::max(3, num_control_points);
  
  // If the input path has fewer poses than num_control_points, cap it
  if (static_cast<int>(path.poses.size()) < num_control_points) {
    num_control_points = path.poses.size();
  }

  // 3. Sample control points at uniform distances along the input path
  std::vector<double> x_controls;
  std::vector<double> y_controls;
  x_controls.reserve(num_control_points);
  y_controls.reserve(num_control_points);

  for (int i = 0; i < num_control_points; ++i) {
    double target_d = static_cast<double>(i) / (num_control_points - 1) * total_distance;
    
    // Find segment in accumulated_dist
    auto it = std::lower_bound(accumulated_dist.begin(), accumulated_dist.end(), target_d);
    int idx = std::distance(accumulated_dist.begin(), it);
    
    if (idx == 0) {
      x_controls.push_back(path.poses[0].pose.position.x);
      y_controls.push_back(path.poses[0].pose.position.y);
    } else if (idx >= static_cast<int>(path.poses.size())) {
      x_controls.push_back(path.poses.back().pose.position.x);
      y_controls.push_back(path.poses.back().pose.position.y);
    } else {
      // Interpolate between idx - 1 and idx
      double d0 = accumulated_dist[idx - 1];
      double d1 = accumulated_dist[idx];
      double t = (d1 > d0) ? (target_d - d0) / (d1 - d0) : 0.0;
      
      double x = (1.0 - t) * path.poses[idx - 1].pose.position.x + t * path.poses[idx].pose.position.x;
      double y = (1.0 - t) * path.poses[idx - 1].pose.position.y + t * path.poses[idx].pose.position.y;
      x_controls.push_back(x);
      y_controls.push_back(y);
    }
  }

  // 4. Solve natural cubic splines
  std::vector<SplineSegment> x_splines = solveNaturalCubicSpline(x_controls);
  std::vector<SplineSegment> y_splines = solveNaturalCubicSpline(y_controls);

  if (x_splines.empty() || y_splines.empty()) {
    return false;
  }

  std_msgs::msg::Header output_header = path.header;
  if (output_header.frame_id.empty() && !path.poses.empty()) {
    output_header.frame_id = path.poses.front().header.frame_id;
  }
  if (clock_) {
    output_header.stamp = clock_->now();
  }

  // 5. Generate smoothed path
  int num_path_points = static_cast<int>(std::ceil(total_distance / path_point_spacing_)) + 1;
  num_path_points = std::max(3, num_path_points);

  std::vector<geometry_msgs::msg::PoseStamped> smoothed_poses;
  smoothed_poses.reserve(num_path_points);

  for (int i = 0; i < num_path_points; ++i) {
    double alpha = static_cast<double>(i) / (num_path_points - 1);
    
    // Scale parameter to spline segment indices
    double spline_param = alpha * (num_control_points - 1);
    int segment_idx = static_cast<int>(spline_param);
    segment_idx = std::max(0, std::min(segment_idx, static_cast<int>(x_splines.size()) - 1));
    
    double u = spline_param - segment_idx;
    u = std::max(0.0, std::min(1.0, u));

    double x = evaluateSpline(x_splines[segment_idx], u);
    double y = evaluateSpline(y_splines[segment_idx], u);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = output_header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    // Interpolate orientation from the input path based on cumulative distance
    double target_d = alpha * total_distance;
    auto it = std::lower_bound(accumulated_dist.begin(), accumulated_dist.end(), target_d);
    int idx = std::distance(accumulated_dist.begin(), it);

    if (idx == 0) {
      pose.pose.orientation = path.poses[0].pose.orientation;
    } else if (idx >= static_cast<int>(path.poses.size())) {
      pose.pose.orientation = path.poses.back().pose.orientation;
    } else {
      double d0 = accumulated_dist[idx - 1];
      double d1 = accumulated_dist[idx];
      double t = (d1 > d0) ? (target_d - d0) / (d1 - d0) : 0.0;

      double qx = (1.0 - t) * path.poses[idx - 1].pose.orientation.x + t * path.poses[idx].pose.orientation.x;
      double qy = (1.0 - t) * path.poses[idx - 1].pose.orientation.y + t * path.poses[idx].pose.orientation.y;
      double qz = (1.0 - t) * path.poses[idx - 1].pose.orientation.z + t * path.poses[idx].pose.orientation.z;
      double qw = (1.0 - t) * path.poses[idx - 1].pose.orientation.w + t * path.poses[idx].pose.orientation.w;

      double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
      if (norm > 1e-6) {
        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;
      }
      pose.pose.orientation.x = qx;
      pose.pose.orientation.y = qy;
      pose.pose.orientation.z = qz;
      pose.pose.orientation.w = qw;
    }

    smoothed_poses.push_back(pose);
  }

  path.poses = std::move(smoothed_poses);
  path.header = output_header;
  return true;
}

std::vector<SplineSegment> NaturalCubicSplineSmoother::solveNaturalCubicSpline(
  const std::vector<double> & points)
{
  int n = points.size() - 1;
  if (n < 1) {
    return {};
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n + 1, n + 1);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n + 1);

  A(0, 0) = 1.0;
  b(0) = 0.0;
  A(n, n) = 1.0;
  b(n) = 0.0;

  double h = 1.0;  // Uniform normalized spacing
  for (int i = 1; i < n; ++i) {
    A(i, i - 1) = 1.0;
    A(i, i) = 4.0;
    A(i, i + 1) = 1.0;
    b(i) = 6.0 * (points[i + 1] - 2.0 * points[i] + points[i - 1]) / (h * h);
  }

  Eigen::VectorXd M = A.colPivHouseholderQr().solve(b);

  std::vector<SplineSegment> segments;
  for (int i = 0; i < n; ++i) {
    SplineSegment seg;
    double h_seg = h;
    double f_i = points[i];
    double f_ip1 = points[i + 1];
    double M_i = M(i);
    double M_ip1 = M(i + 1);

    seg.a0 = f_i;
    seg.a1 = (f_ip1 - f_i) / h_seg - h_seg * (2.0 * M_i + M_ip1) / 6.0;
    seg.a2 = M_i / 2.0;
    seg.a3 = (M_ip1 - M_i) / (6.0 * h_seg);

    segments.push_back(seg);
  }

  return segments;
}

double NaturalCubicSplineSmoother::evaluateSpline(
  const SplineSegment & segment,
  double u)
{
  return segment.a0 + segment.a1 * u + segment.a2 * u * u + segment.a3 * u * u * u;
}

}  // namespace natural_cubic_spline

PLUGINLIB_EXPORT_CLASS(
  natural_cubic_spline::NaturalCubicSplineSmoother,
  nav2_core::Smoother)
