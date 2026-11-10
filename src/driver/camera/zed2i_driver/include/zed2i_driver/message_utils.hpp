#pragma once

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <njord_interfaces/msg/buoy_detection_array.hpp>

#include <memory>
#include <string>
#include <vector>

#include "zed2i_driver/perception_types.hpp"
#include "zed2i_driver/perception_logic.hpp"

namespace zed2i_driver
{

sensor_msgs::msg::Image::UniquePtr mat_to_image_msg(
  const cv::Mat & image,
  const std::string & encoding,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

sensor_msgs::msg::CameraInfo make_camera_info_msg(
  int width,
  int height,
  double fx,
  double fy,
  double cx,
  double cy,
  double baseline_m,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

sensor_msgs::msg::PointCloud2::UniquePtr depth_to_point_cloud_msg(
  const cv::Mat & depth_m,
  double fx,
  double fy,
  double cx,
  double cy,
  int stride,
  double depth_min_m,
  double depth_max_m,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

njord_interfaces::msg::BuoyDetectionArray to_detection_array_msg(
  const std::vector<PositionedDetection> & detections,
  const std::string & frame_id, const rclcpp::Time & stamp);

sensor_msgs::msg::PointCloud2::UniquePtr wall_points_to_cloud_msg(
  const std::vector<WallPoint> & points, const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace zed2i_driver
