#pragma once

#include "zed2i_driver/perception_types.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <njord_interfaces/msg/buoy_detection_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <string>
#include <vector>

namespace zed2i_driver
{

njord_interfaces::msg::BuoyDetectionArray to_detection_array(
  const std::vector<PositionedDetection> & detections, const std_msgs::msg::Header & header);

sensor_msgs::msg::PointCloud2 to_virtual_wall_cloud(
  const std::vector<PositionedDetection> & detections, const std_msgs::msg::Header & header,
  const std::string & wall_frame, float channel_heading_rad, float radius_m,
  int points_per_full_circle, bool connect_same_color_buoys = true,
  float same_color_max_gap_m = 12.0F, float same_color_point_spacing_m = 0.2F);

}  // namespace zed2i_driver
