#include "zed2i_driver/message_utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace zed2i_driver
{
namespace
{

int bytes_per_pixel(const cv::Mat & image)
{
  return static_cast<int>(image.elemSize());
}

}  // namespace

sensor_msgs::msg::Image::UniquePtr mat_to_image_msg(
  const cv::Mat & image,
  const std::string & encoding,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (image.empty()) {
    throw std::runtime_error("cannot convert an empty cv::Mat to sensor_msgs::msg::Image");
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->height = static_cast<uint32_t>(image.rows);
  msg->width = static_cast<uint32_t>(image.cols);
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(image.cols * bytes_per_pixel(image));
  msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
  if (image.isContinuous() && image.step == msg->step) {
    std::memcpy(msg->data.data(), image.data, msg->data.size());
  } else {
    for (int row = 0; row < image.rows; ++row) {
      std::memcpy(
        msg->data.data() + static_cast<size_t>(row) * msg->step,
        image.ptr(row), msg->step);
    }
  }
  return msg;
}

sensor_msgs::msg::CameraInfo make_camera_info_msg(
  int width,
  int height,
  double fx,
  double fy,
  double cx,
  double cy,
  double baseline_m,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  msg.distortion_model = "plumb_bob";
  msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  msg.k = {
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0};
  msg.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  msg.p = {
    fx, 0.0, cx, -fx * baseline_m,
    0.0, fy, cy, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return msg;
}

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
  const rclcpp::Time & stamp)
{
  if (depth_m.empty() || depth_m.type() != CV_32FC1) {
    throw std::runtime_error("depth_to_point_cloud_msg expects a non-empty CV_32FC1 image");
  }

  stride = std::max(1, stride);
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->height = 1;
  msg->is_bigendian = false;
  msg->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  const auto point_rows = static_cast<size_t>((depth_m.rows + stride - 1) / stride);
  const auto point_columns = static_cast<size_t>((depth_m.cols + stride - 1) / stride);
  const auto max_points = point_rows * point_columns;
  modifier.resize(max_points);

  size_t point_count = 0;

  for (int v = 0; v < depth_m.rows; v += stride) {
    const float * row = depth_m.ptr<float>(v);
    for (int u = 0; u < depth_m.cols; u += stride) {
      const float z = row[u];
      if (!std::isfinite(z) || z < depth_min_m || z > depth_max_m) {
        continue;
      }

      const float x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
      const float y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);
      auto * point_data = msg->data.data() + point_count * msg->point_step;
      std::memcpy(point_data + msg->fields[0].offset, &x, sizeof(x));
      std::memcpy(point_data + msg->fields[1].offset, &y, sizeof(y));
      std::memcpy(point_data + msg->fields[2].offset, &z, sizeof(z));
      ++point_count;
    }
  }

  msg->width = static_cast<uint32_t>(point_count);
  msg->row_step = msg->width * msg->point_step;
  msg->data.resize(msg->row_step);

  return msg;
}

njord_interfaces::msg::BuoyDetectionArray to_detection_array_msg(
  const std::vector<PositionedDetection> & detections,
  const std::string & frame_id, const rclcpp::Time & stamp)
{
  njord_interfaces::msg::BuoyDetectionArray message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.detections.reserve(detections.size());
  for (const auto & item : detections) {
    njord_interfaces::msg::BuoyDetection detection;
    detection.class_id = static_cast<uint8_t>(std::clamp(item.detection.class_id, 0, 255));
    detection.confidence = std::clamp(item.detection.confidence, 0.0F, 1.0F);
    detection.position.x = item.position_base[0];
    detection.position.y = item.position_base[1];
    detection.position.z = item.position_base[2];
    switch (item.source) {
      case PositionSource::kZedDepth:
        detection.position_source = njord_interfaces::msg::BuoyDetection::POSITION_ZED_DEPTH;
        break;
      case PositionSource::kLidarFused:
        detection.position_source = njord_interfaces::msg::BuoyDetection::POSITION_LIDAR_FUSED;
        break;
      default:
        detection.position_source = njord_interfaces::msg::BuoyDetection::POSITION_NONE;
        detection.position.x = std::numeric_limits<double>::quiet_NaN();
        detection.position.y = std::numeric_limits<double>::quiet_NaN();
        detection.position.z = std::numeric_limits<double>::quiet_NaN();
        break;
    }
    if (detection.position_source != njord_interfaces::msg::BuoyDetection::POSITION_NONE &&
      (!std::isfinite(detection.position.x) || !std::isfinite(detection.position.y) ||
      !std::isfinite(detection.position.z))) {
      detection.position_source = njord_interfaces::msg::BuoyDetection::POSITION_NONE;
      detection.position.x = std::numeric_limits<double>::quiet_NaN();
      detection.position.y = std::numeric_limits<double>::quiet_NaN();
      detection.position.z = std::numeric_limits<double>::quiet_NaN();
    }
    message.detections.push_back(detection);
  }
  return message;
}

sensor_msgs::msg::PointCloud2::UniquePtr wall_points_to_cloud_msg(
  const std::vector<WallPoint> & points, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
  message->header.stamp = stamp;
  message->header.frame_id = frame_id;
  message->height = 1;
  message->is_bigendian = false;
  message->is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(*message);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  size_t count = 0;
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    auto * data = message->data.data() + count * message->point_step;
    std::memcpy(data + message->fields[0].offset, &point.x, sizeof(float));
    std::memcpy(data + message->fields[1].offset, &point.y, sizeof(float));
    std::memcpy(data + message->fields[2].offset, &point.z, sizeof(float));
    ++count;
  }
  message->width = static_cast<uint32_t>(count);
  message->row_step = message->width * message->point_step;
  message->data.resize(message->row_step);
  return message;
}

}  // namespace zed2i_driver
