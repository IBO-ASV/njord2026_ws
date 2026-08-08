#include "zed2i_driver/message_utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace zed2i_driver
{
namespace
{

int bytes_per_pixel(const cv::Mat & image)
{
  return static_cast<int>(image.elemSize());
}

}  // namespace

sensor_msgs::msg::Image mat_to_image_msg(
  const cv::Mat & image,
  const std::string & encoding,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (image.empty()) {
    throw std::runtime_error("cannot convert an empty cv::Mat to sensor_msgs::msg::Image");
  }

  const cv::Mat contiguous = image.isContinuous() ? image : image.clone();

  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = static_cast<uint32_t>(contiguous.rows);
  msg.width = static_cast<uint32_t>(contiguous.cols);
  msg.encoding = encoding;
  msg.is_bigendian = false;
  msg.step = static_cast<uint32_t>(contiguous.cols * bytes_per_pixel(contiguous));
  msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
  std::memcpy(msg.data.data(), contiguous.data, msg.data.size());
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

sensor_msgs::msg::PointCloud2 depth_to_point_cloud_msg(
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
  std::vector<cv::Vec3f> points;
  points.reserve(static_cast<size_t>((depth_m.rows / stride) * (depth_m.cols / stride)));

  for (int v = 0; v < depth_m.rows; v += stride) {
    const float * row = depth_m.ptr<float>(v);
    for (int u = 0; u < depth_m.cols; u += stride) {
      const float z = row[u];
      if (!std::isfinite(z) || z < depth_min_m || z > depth_max_m) {
        continue;
      }

      const float x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
      const float y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);
      points.emplace_back(x, y, z);
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(msg, "z");
  for (const auto & point : points) {
    *x = point[0];
    *y = point[1];
    *z = point[2];
    ++x;
    ++y;
    ++z;
  }

  return msg;
}

}  // namespace zed2i_driver
