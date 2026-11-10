#pragma once

#include "zed2i_driver/perception_types.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace zed2i_driver
{

std::vector<Detection2D> decode_detections(
  const std::vector<float> & output, int detection_count, bool channels_first,
  float confidence_threshold, const LetterboxTransform & letterbox, int max_detections = 32);

// Decodes the pre-NMS YOLO output used by a one-class YOLO-World export:
// [1,5,N] or [1,N,5], where each candidate is cx, cy, width, height, score.
// NMS stays on the CPU because only the compact output tensor leaves TensorRT.
std::vector<Detection2D> decode_yolo_world_raw_detections(
  const std::vector<float> & output, int detection_count, bool channels_first,
  float confidence_threshold, const LetterboxTransform & letterbox, int max_detections = 32);

DepthRoi central_depth_roi(
  const Detection2D & detection, float ratio, int image_width, int image_height);

std::optional<float> depth_median_cpu(
  const std::vector<float> & depth, int width, int height, const DepthRoi & roi,
  int sample_stride, int min_samples, float min_depth_m, float max_depth_m);

// Selects the cluster according to the ZED-first/fallback policy. The input
// clusters are already projected and clustered; this function is deliberately
// ROS/TF independent so it can be used as a deterministic test oracle.
std::optional<LidarCluster> select_lidar_cluster(
  const std::vector<LidarCluster> & clusters, bool zed_position_valid,
  const std::array<float, 3> & zed_position, float max_range_delta_m,
  float max_ray_angle_rad);

bool point_in_central_bbox_roi(
  float u, float v, const Detection2D & detection, float ratio);

bool valid_position(const std::array<float, 3> & position);

// Compact label for the ground-video overlay. Z and L identify the final
// position source; ? means that this detection has no usable range.
std::string detection_range_label(const PositionedDetection & detection);

struct WallPoint { float x; float y; float z; };

std::vector<WallPoint> virtual_wall_points(
  int class_id, float buoy_x, float buoy_y, float channel_heading_rad,
  float radius_m = 2.0F, int points_per_full_circle = 40);

// Connect adjacent buoys on one lateral-mark boundary.  The points are
// ordered along channel_heading_rad, so a row of same-colour buoys becomes a
// continuous Nav2 obstacle without joining detections across a large gap.
std::vector<WallPoint> same_color_wall_points(
  const std::vector<WallPoint> & buoy_positions, float channel_heading_rad,
  float max_gap_m = 12.0F, float point_spacing_m = 0.2F);

}  // namespace zed2i_driver
