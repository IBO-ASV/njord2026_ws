#pragma once

#include "zed2i_driver/perception_types.hpp"

#include <vector>

namespace zed2i_driver
{

// Selects one already clustered LiDAR candidate. The returned index is -1 when
// the candidate does not satisfy the range/ray policy.
int select_lidar_cluster(
  const std::vector<LidarCluster> & clusters, const Point3f & zed_position,
  bool zed_position_valid, const Detection2D & detection, const FusionPolicy & policy);

// Assigns each projected point to at most one detection, choosing the closest
// normalized bbox-centre ray for overlapping boxes.
std::vector<std::vector<Point3f>> assign_projected_points(
  const std::vector<Point3f> & camera_points, const std::vector<Detection2D> & detections,
  int image_width, int image_height, float fx, float fy, float cx, float cy,
  float depth_center_ratio = 0.5F);

}  // namespace zed2i_driver
