#include "zed2i_driver/detection_message_utils.hpp"

#include "zed2i_driver/perception_logic.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace zed2i_driver
{

njord_interfaces::msg::BuoyDetectionArray to_detection_array(
  const std::vector<PositionedDetection> & detections, const std_msgs::msg::Header & header)
{
  njord_interfaces::msg::BuoyDetectionArray message;
  message.header = header;
  message.detections.reserve(detections.size());
  for (const auto & item : detections) {
    njord_interfaces::msg::BuoyDetection detection;
    detection.class_id = static_cast<uint8_t>(std::clamp(item.detection.class_id, 0, 255));
    detection.confidence = std::clamp(item.detection.confidence, 0.0F, 1.0F);
    detection.position_source = static_cast<uint8_t>(item.source);
    const bool valid = item.source != PositionSource::kNone &&
      std::all_of(item.position_base.begin(), item.position_base.end(),
      [](float value) {return std::isfinite(value);});
    if (valid) {
      detection.position.x = item.position_base[0];
      detection.position.y = item.position_base[1];
      detection.position.z = item.position_base[2];
    } else {
      const auto nan = std::numeric_limits<double>::quiet_NaN();
      detection.position.x = nan;
      detection.position.y = nan;
      detection.position.z = nan;
      detection.position_source = static_cast<uint8_t>(PositionSource::kNone);
    }
    message.detections.push_back(detection);
  }
  return message;
}

sensor_msgs::msg::PointCloud2 to_virtual_wall_cloud(
  const std::vector<PositionedDetection> & detections, const std_msgs::msg::Header & header,
  const std::string & wall_frame, float heading, float radius, int density,
  bool connect_same_color_buoys, float same_color_max_gap, float same_color_point_spacing)
{
  std::vector<WallPoint> points;
  std::vector<WallPoint> green_buoys;
  std::vector<WallPoint> red_buoys;
  for (const auto & detection : detections) {
    if (detection.source == PositionSource::kNone ||
      !std::isfinite(detection.position_base[0]) || !std::isfinite(detection.position_base[1])) { continue; }
    const auto wall = virtual_wall_points(
      detection.detection.class_id, detection.position_base[0], detection.position_base[1],
      heading, radius, density);
    points.insert(points.end(), wall.begin(), wall.end());
    if (detection.detection.class_id == 0) {
      green_buoys.push_back({detection.position_base[0], detection.position_base[1], 0.0F});
    } else if (detection.detection.class_id == 1) {
      red_buoys.push_back({detection.position_base[0], detection.position_base[1], 0.0F});
    }
  }
  if (connect_same_color_buoys) {
    const auto green_wall = same_color_wall_points(
      green_buoys, heading, same_color_max_gap, same_color_point_spacing);
    const auto red_wall = same_color_wall_points(
      red_buoys, heading, same_color_max_gap, same_color_point_spacing);
    points.insert(points.end(), green_wall.begin(), green_wall.end());
    points.insert(points.end(), red_wall.begin(), red_wall.end());
  }
  sensor_msgs::msg::PointCloud2 message;
  message.header = header;
  message.header.frame_id = wall_frame;
  message.height = 1;
  message.is_bigendian = false;
  message.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  for (const auto & point : points) {
    *x = point.x; *y = point.y; *z = 0.0F;
    ++x; ++y; ++z;
  }
  return message;
}

}  // namespace zed2i_driver
