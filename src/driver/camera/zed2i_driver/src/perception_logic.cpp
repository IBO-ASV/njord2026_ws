#include "zed2i_driver/perception_logic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace zed2i_driver
{
namespace
{
constexpr float kPi = 3.14159265358979323846F;

float value_at(
  const std::vector<float> & output, int index, int field, int count, bool channels_first,
  int fields_per_detection = 6)
{
  return channels_first ? output[static_cast<size_t>(field * count + index)] :
         output[static_cast<size_t>(index * fields_per_detection + field)];
}

float intersection_over_union(const Detection2D & first, const Detection2D & second)
{
  const float x1 = std::max(first.x1, second.x1);
  const float y1 = std::max(first.y1, second.y1);
  const float x2 = std::min(first.x2, second.x2);
  const float y2 = std::min(first.y2, second.y2);
  const float intersection = std::max(0.0F, x2 - x1) * std::max(0.0F, y2 - y1);
  const float first_area = std::max(0.0F, first.x2 - first.x1) * std::max(0.0F, first.y2 - first.y1);
  const float second_area = std::max(0.0F, second.x2 - second.x1) * std::max(0.0F, second.y2 - second.y1);
  const float union_area = first_area + second_area - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

float normalize_angle(float angle)
{
  while (angle <= -kPi) { angle += 2.0F * kPi; }
  while (angle > kPi) { angle -= 2.0F * kPi; }
  return angle;
}

}  // namespace

std::vector<Detection2D> decode_detections(
  const std::vector<float> & output, int count, bool channels_first, float threshold,
  const LetterboxTransform & letterbox, int max_detections)
{
  if (count <= 0 || max_detections <= 0 || letterbox.scale <= 0.0F ||
    output.size() < static_cast<size_t>(count * 6))
  {
    return {};
  }
  std::vector<Detection2D> decoded;
  decoded.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const float confidence = value_at(output, i, 4, count, channels_first);
    const float class_value = value_at(output, i, 5, count, channels_first);
    if (!std::isfinite(confidence) || confidence < threshold || confidence < 0.0F || confidence > 1.0F ||
      !std::isfinite(class_value) || class_value < 0.0F || class_value > 5.0F ||
      std::floor(class_value) != class_value) { continue; }
    Detection2D detection{
      static_cast<int>(class_value), confidence,
      (value_at(output, i, 0, count, channels_first) - letterbox.pad_x) / letterbox.scale,
      (value_at(output, i, 1, count, channels_first) - letterbox.pad_y) / letterbox.scale,
      (value_at(output, i, 2, count, channels_first) - letterbox.pad_x) / letterbox.scale,
      (value_at(output, i, 3, count, channels_first) - letterbox.pad_y) / letterbox.scale};
    detection.x1 = std::clamp(detection.x1, 0.0F, static_cast<float>(letterbox.source_width));
    detection.x2 = std::clamp(detection.x2, 0.0F, static_cast<float>(letterbox.source_width));
    detection.y1 = std::clamp(detection.y1, 0.0F, static_cast<float>(letterbox.source_height));
    detection.y2 = std::clamp(detection.y2, 0.0F, static_cast<float>(letterbox.source_height));
    if (detection.x2 - detection.x1 >= 1.0F && detection.y2 - detection.y1 >= 1.0F) { decoded.push_back(detection); }
  }
  std::sort(decoded.begin(), decoded.end(), [](const auto & a, const auto & b) { return a.confidence > b.confidence; });
  if (decoded.size() > static_cast<size_t>(max_detections)) { decoded.resize(static_cast<size_t>(max_detections)); }
  return decoded;
}

std::vector<Detection2D> decode_yolo_world_raw_detections(
  const std::vector<float> & output, int count, bool channels_first, float threshold,
  const LetterboxTransform & letterbox, int max_detections)
{
  if (count <= 0 || max_detections <= 0 || letterbox.scale <= 0.0F ||
    output.size() < static_cast<size_t>(count * 5))
  {
    return {};
  }
  std::vector<Detection2D> candidates;
  candidates.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const float confidence = value_at(output, i, 4, count, channels_first, 5);
    if (!std::isfinite(confidence) || confidence < threshold || confidence > 1.0F) { continue; }
    const float cx = value_at(output, i, 0, count, channels_first, 5);
    const float cy = value_at(output, i, 1, count, channels_first, 5);
    const float width = value_at(output, i, 2, count, channels_first, 5);
    const float height = value_at(output, i, 3, count, channels_first, 5);
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(width) || !std::isfinite(height) ||
      width <= 0.0F || height <= 0.0F)
    {
      continue;
    }
    Detection2D detection{0, confidence,
      (cx - width * 0.5F - letterbox.pad_x) / letterbox.scale,
      (cy - height * 0.5F - letterbox.pad_y) / letterbox.scale,
      (cx + width * 0.5F - letterbox.pad_x) / letterbox.scale,
      (cy + height * 0.5F - letterbox.pad_y) / letterbox.scale};
    detection.x1 = std::clamp(detection.x1, 0.0F, static_cast<float>(letterbox.source_width));
    detection.x2 = std::clamp(detection.x2, 0.0F, static_cast<float>(letterbox.source_width));
    detection.y1 = std::clamp(detection.y1, 0.0F, static_cast<float>(letterbox.source_height));
    detection.y2 = std::clamp(detection.y2, 0.0F, static_cast<float>(letterbox.source_height));
    if (detection.x2 - detection.x1 >= 1.0F && detection.y2 - detection.y1 >= 1.0F) {
      candidates.push_back(detection);
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.confidence > b.confidence;
  });
  std::vector<Detection2D> result;
  result.reserve(static_cast<size_t>(max_detections));
  for (const auto & candidate : candidates) {
    const bool overlaps = std::any_of(result.begin(), result.end(), [&candidate](const auto & kept) {
      return intersection_over_union(candidate, kept) >= 0.45F;
    });
    if (!overlaps) { result.push_back(candidate); }
    if (result.size() >= static_cast<size_t>(max_detections)) { break; }
  }
  return result;
}

DepthRoi central_depth_roi(const Detection2D & d, float ratio, int width, int height)
{
  width = std::max(0, width);
  height = std::max(0, height);
  if (!std::isfinite(ratio) || ratio <= 0.0F) { return {}; }
  ratio = std::min(ratio, 1.0F);
  const float cx = (d.x1 + d.x2) * 0.5F;
  const float cy = (d.y1 + d.y2) * 0.5F;
  const float half_w = (d.x2 - d.x1) * ratio * 0.5F;
  const float half_h = (d.y2 - d.y1) * ratio * 0.5F;
  return {std::clamp(static_cast<int>(std::floor(cx - half_w)), 0, width),
    std::clamp(static_cast<int>(std::floor(cy - half_h)), 0, height),
    std::clamp(static_cast<int>(std::ceil(cx + half_w)), 0, width),
    std::clamp(static_cast<int>(std::ceil(cy + half_h)), 0, height)};
}

std::optional<float> depth_median_cpu(
  const std::vector<float> & depth, int width, int height, const DepthRoi & roi,
  int stride, int min_samples, float min_depth_m, float max_depth_m)
{
  if (width <= 0 || height <= 0 || stride <= 0 || min_samples <= 0 ||
    depth.size() < static_cast<size_t>(width * height)) { return std::nullopt; }
  std::vector<float> values;
  for (int y = roi.y0; y < roi.y1; y += stride) for (int x = roi.x0; x < roi.x1; x += stride) {
    const auto value = depth[static_cast<size_t>(y * width + x)];
    if (std::isfinite(value) && value >= min_depth_m && value <= max_depth_m) { values.push_back(value); }
  }
  if (values.size() < static_cast<size_t>(min_samples)) { return std::nullopt; }
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  return values.size() % 2 ? values[middle] : (values[middle - 1] + values[middle]) * 0.5F;
}

bool valid_position(const std::array<float, 3> & position)
{
  return std::all_of(position.begin(), position.end(), [](float value) {
    return std::isfinite(value);
  });
}

std::string detection_range_label(const PositionedDetection & detection)
{
  const char source = detection.source == PositionSource::kZedDepth ? 'Z' :
    detection.source == PositionSource::kLidarFused ? 'L' : '?';
  if (source == '?' || !valid_position(detection.position_base)) {
    return "?";
  }
  const float range = std::hypot(
    detection.position_base[0], detection.position_base[1], detection.position_base[2]);
  if (!std::isfinite(range)) {
    return "?";
  }
  char label[16];
  std::snprintf(label, sizeof(label), "%c %.1fm", source, static_cast<double>(range));
  return label;
}

bool point_in_central_bbox_roi(float u, float v, const Detection2D & detection, float ratio)
{
  if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(ratio) || ratio <= 0.0F) {
    return false;
  }
  const float cx = (detection.x1 + detection.x2) * 0.5F;
  const float cy = (detection.y1 + detection.y2) * 0.5F;
  const float half_w = std::abs(detection.x2 - detection.x1) * std::min(ratio, 1.0F) * 0.5F;
  const float half_h = std::abs(detection.y2 - detection.y1) * std::min(ratio, 1.0F) * 0.5F;
  return u >= cx - half_w && u <= cx + half_w && v >= cy - half_h && v <= cy + half_h;
}

std::optional<LidarCluster> select_lidar_cluster(
  const std::vector<LidarCluster> & clusters, bool zed_position_valid,
  const std::array<float, 3> & zed_position, float max_range_delta_m,
  float max_ray_angle_rad)
{
  if (clusters.empty() || max_range_delta_m < 0.0F || max_ray_angle_rad < 0.0F) {
    return std::nullopt;
  }
  if (zed_position_valid && valid_position(zed_position)) {
    const float zed_range = std::sqrt(
      zed_position[0] * zed_position[0] + zed_position[1] * zed_position[1] +
      zed_position[2] * zed_position[2]);
    auto candidate = std::min_element(clusters.begin(), clusters.end(), [zed_range](const auto & a, const auto & b) {
      return std::abs(a.range - zed_range) < std::abs(b.range - zed_range);
    });
    if (candidate != clusters.end() && std::abs(candidate->range - zed_range) <= max_range_delta_m) {
      return *candidate;
    }
    return std::nullopt;
  }

  auto candidate = std::min_element(clusters.begin(), clusters.end(), [](const auto & a, const auto & b) {
    const float a_angle = std::abs(a.ray_angle_rad);
    const float b_angle = std::abs(b.ray_angle_rad);
    if (a_angle == b_angle) { return a.range < b.range; }
    return a_angle < b_angle;
  });
  if (candidate != clusters.end() && std::abs(candidate->ray_angle_rad) <= max_ray_angle_rad) {
    return *candidate;
  }
  return std::nullopt;
}

std::vector<WallPoint> virtual_wall_points(
  int class_id, float x, float y, float heading, float radius, int density)
{
  if (!std::isfinite(x) || !std::isfinite(y) || radius < 0.0F || density <= 0) { return {}; }
  float start{};
  float sweep{};
  switch (class_id) {
    case 2: start = kPi * 0.75F; sweep = kPi * 1.5F; break;  // north safe sector +y
    case 3: start = kPi * 0.25F; sweep = kPi * 1.5F; break;  // east safe sector +x
    case 4: start = -kPi * 0.25F; sweep = kPi * 1.5F; break; // south safe sector -y
    case 5: start = kPi * 1.25F; sweep = kPi * 1.5F; break;  // west safe sector -x
    case 0: if (!std::isfinite(heading)) return {}; start = heading - kPi; sweep = kPi; break;
    case 1: if (!std::isfinite(heading)) return {}; start = heading; sweep = kPi; break;
    default: return {};
  }
  const int points = static_cast<int>(std::lround((sweep / (2.0F * kPi)) * density)) + 1;
  std::vector<WallPoint> result;
  result.reserve(static_cast<size_t>(points));
  for (int i = 0; i < points; ++i) {
    const float fraction = points > 1 ? static_cast<float>(i) / static_cast<float>(points - 1) : 0.0F;
    const float angle = normalize_angle(start + sweep * fraction);
    result.push_back({x + radius * std::cos(angle), y + radius * std::sin(angle), 0.0F});
  }
  return result;
}

std::vector<WallPoint> same_color_wall_points(
  const std::vector<WallPoint> & buoy_positions, float heading, float max_gap, float spacing)
{
  if (!std::isfinite(heading) || max_gap <= 0.0F || spacing <= 0.0F) { return {}; }

  std::vector<WallPoint> ordered;
  ordered.reserve(buoy_positions.size());
  for (const auto & buoy : buoy_positions) {
    if (std::isfinite(buoy.x) && std::isfinite(buoy.y)) { ordered.push_back(buoy); }
  }
  if (ordered.size() < 2U) { return {}; }

  const float along_x = std::cos(heading);
  const float along_y = std::sin(heading);
  std::sort(ordered.begin(), ordered.end(), [along_x, along_y](const auto & lhs, const auto & rhs) {
    return lhs.x * along_x + lhs.y * along_y < rhs.x * along_x + rhs.y * along_y;
  });

  std::vector<WallPoint> result;
  for (size_t index = 1; index < ordered.size(); ++index) {
    const auto & first = ordered[index - 1];
    const auto & second = ordered[index];
    const float dx = second.x - first.x;
    const float dy = second.y - first.y;
    const float distance = std::hypot(dx, dy);
    if (distance <= 0.0F || distance > max_gap) { continue; }

    const int intervals = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
    for (int step = 0; step <= intervals; ++step) {
      const float fraction = static_cast<float>(step) / static_cast<float>(intervals);
      result.push_back({first.x + fraction * dx, first.y + fraction * dy, 0.0F});
    }
  }
  return result;
}

}  // namespace zed2i_driver
