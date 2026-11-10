#include "zed2i_driver/lidar_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace zed2i_driver
{
namespace
{
float range(const Point3f & p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float ray_angle(const Point3f & p, float u, float v, float cx, float cy, float fx, float fy)
{
  const float du = u - cx;
  const float dv = v - cy;
  const float dz = fx > 0.0F && fy > 0.0F ? p.z : 1.0F;
  const float px = du / fx * dz;
  const float py = dv / fy * dz;
  const float dot = p.x * px + p.y * py + p.z * dz;
  const float denom = range(p) * std::sqrt(px * px + py * py + dz * dz);
  if (denom <= 0.0F) { return std::numeric_limits<float>::infinity(); }
  return std::acos(std::clamp(dot / denom, -1.0F, 1.0F));
}
}  // namespace

int select_lidar_cluster(
  const std::vector<LidarCluster> & clusters, const Point3f & zed, bool zed_valid,
  const Detection2D & detection, const FusionPolicy & policy)
{
  int best = -1;
  if (zed_valid) {
    const float zed_range = range(zed);
    float best_delta = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < clusters.size(); ++i) {
      const float delta = std::fabs(clusters[i].range - zed_range);
      if (delta <= policy.max_range_delta_m && delta < best_delta) {
        best_delta = delta;
        best = static_cast<int>(i);
      }
    }
    return best;
  }

  const float center_u = (detection.x1 + detection.x2) * 0.5F;
  const float center_v = (detection.y1 + detection.y2) * 0.5F;
  float best_angle = policy.max_fallback_angle_rad;
  float best_range = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto & cluster = clusters[i];
    if (cluster.ray_angle_rad < best_angle ||
      (cluster.ray_angle_rad == best_angle && cluster.range < best_range))
    {
      best_angle = cluster.ray_angle_rad;
      best_range = cluster.range;
      best = static_cast<int>(i);
    }
  }
  (void)center_u;
  (void)center_v;
  return best;
}

std::vector<std::vector<Point3f>> assign_projected_points(
  const std::vector<Point3f> & points, const std::vector<Detection2D> & detections,
  int width, int height, float fx, float fy, float cx, float cy, float ratio)
{
  std::vector<std::vector<Point3f>> result(detections.size());
  if (width <= 0 || height <= 0 || fx <= 0.0F || fy <= 0.0F) { return result; }
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || point.z <= 0.0F) { continue; }
    const float u = fx * point.x / point.z + cx;
    const float v = fy * point.y / point.z + cy;
    if (u < 0.0F || v < 0.0F || u >= width || v >= height) { continue; }
    int best = -1;
    float best_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto & d = detections[i];
      const float dcx = (d.x1 + d.x2) * 0.5F;
      const float dcy = (d.y1 + d.y2) * 0.5F;
      const float half_w = std::max(0.0F, (d.x2 - d.x1) * ratio * 0.5F);
      const float half_h = std::max(0.0F, (d.y2 - d.y1) * ratio * 0.5F);
      if (std::fabs(u - dcx) > half_w || std::fabs(v - dcy) > half_h) { continue; }
      const float nx = (u - dcx) / std::max(half_w, 1.0e-6F);
      const float ny = (v - dcy) / std::max(half_h, 1.0e-6F);
      const float distance = nx * nx + ny * ny;
      if (distance < best_distance) { best_distance = distance; best = static_cast<int>(i); }
    }
    if (best >= 0) { result[static_cast<size_t>(best)].push_back(point); }
  }
  return result;
}

}  // namespace zed2i_driver
