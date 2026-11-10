#include "zed2i_driver/message_utils.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace zed2i_driver
{
namespace
{

int normalize_num_disparities(int value)
{
  value = std::max(16, value);
  return (value / 16) * 16;
}

int normalize_block_size(int value)
{
  value = std::max(3, value);
  if (value % 2 == 0) {
    ++value;
  }
  return value;
}

template<typename MessageT>
bool has_subscribers(const std::shared_ptr<rclcpp::Publisher<MessageT>> & publisher)
{
  return publisher->get_subscription_count() > 0 ||
         publisher->get_intra_process_subscription_count() > 0;
}

}  // namespace

class CpuStereoNode : public rclcpp::Node
{
public:
  explicit CpuStereoNode(const rclcpp::NodeOptions & options)
  : Node("zed2i_cpu_node", options)
  {
    left_device_ = declare_parameter<std::string>("left_device", "/dev/video0");
    right_device_ = declare_parameter<std::string>("right_device", "/dev/video1");
    left_frame_id_ = declare_parameter<std::string>("left_frame_id", "zed2i_left_camera_frame");
    right_frame_id_ = declare_parameter<std::string>("right_frame_id", "zed2i_right_camera_frame");
    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", left_frame_id_);
    image_width_ = declare_parameter<int>("image_width", 1280);
    image_height_ = declare_parameter<int>("image_height", 720);
    framerate_ = declare_parameter<int>("framerate", 15);
    baseline_m_ = declare_parameter<double>("baseline_m", 0.12);
    fx_ = declare_parameter<double>("fx", static_cast<double>(image_width_));
    fy_ = declare_parameter<double>("fy", fx_);
    cx_ = declare_parameter<double>("cx", static_cast<double>(image_width_) / 2.0);
    cy_ = declare_parameter<double>("cy", static_cast<double>(image_height_) / 2.0);
    depth_min_m_ = declare_parameter<double>("depth_min_m", 0.3);
    depth_max_m_ = declare_parameter<double>("depth_max_m", 20.0);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    pointcloud_stride_ = declare_parameter<int>("pointcloud_stride", 2);
    const auto num_disparities =
      normalize_num_disparities(declare_parameter<int>("num_disparities", 128));
    const auto block_size = normalize_block_size(declare_parameter<int>("block_size", 5));

    rclcpp::QoS image_qos(rclcpp::KeepLast(5));
    image_qos.best_effort();
    image_qos.durability_volatile();

    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_rect", image_qos);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_rect", image_qos);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", image_qos);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info",
        image_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image", image_qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", image_qos);

    stereo_ = cv::StereoSGBM::create(0, num_disparities, block_size);
    stereo_->setP1(8 * block_size * block_size);
    stereo_->setP2(32 * block_size * block_size);
    stereo_->setMode(cv::StereoSGBM::MODE_SGBM);

    combined_stereo_device_ = left_device_ == right_device_;
    open_capture(
      left_capture_, left_device_, "left",
      combined_stereo_device_ ? 2 * image_width_ : image_width_);
    if (!combined_stereo_device_) {
      open_capture(right_capture_, right_device_, "right", image_width_);
    }

    const auto period_ms = std::max(1, 1000 / std::max(1, framerate_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CpuStereoNode::capture_and_publish, this));
  }

private:
  void open_capture(
    cv::VideoCapture & capture, const std::string & device, const char * name, int capture_width)
  {
    capture.open(device, cv::CAP_ANY);
    if (!capture.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s camera device '%s'", name, device.c_str());
      return;
    }

    capture.set(cv::CAP_PROP_FRAME_WIDTH, capture_width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    capture.set(cv::CAP_PROP_FPS, framerate_);
    RCLCPP_INFO(get_logger(), "Opened %s camera device '%s'", name, device.c_str());
  }

  void capture_and_publish()
  {
    const bool publish_left = has_subscribers(left_image_pub_);
    const bool publish_right = has_subscribers(right_image_pub_);
    const bool publish_left_info = has_subscribers(left_info_pub_);
    const bool publish_right_info = has_subscribers(right_info_pub_);
    const bool publish_depth = has_subscribers(depth_pub_);
    const bool publish_points = publish_pointcloud_ && has_subscribers(pointcloud_pub_);
    const bool needs_depth = publish_depth || publish_points;
    const bool needs_left_frame = publish_left || needs_depth;
    const bool needs_right_frame = publish_right || needs_depth;

    if (!needs_left_frame && !needs_right_frame && !publish_left_info && !publish_right_info) {
      return;
    }

    if ((needs_left_frame && !left_capture_.isOpened()) ||
      (needs_right_frame &&
      !(combined_stereo_device_ ? left_capture_.isOpened() : right_capture_.isOpened())))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "A camera device required by an active ZED output is not readable");
      return;
    }

    cv::Mat left_bgr;
    cv::Mat right_bgr;
    bool left_ok = true;
    bool right_ok = true;
    if (combined_stereo_device_) {
      if (needs_left_frame || needs_right_frame) {
        cv::Mat stereo_bgr;
        const bool stereo_ok = left_capture_.read(stereo_bgr) &&
          !stereo_bgr.empty() && stereo_bgr.cols >= 2 && stereo_bgr.cols % 2 == 0;
        left_ok = stereo_ok;
        right_ok = stereo_ok;
        if (stereo_ok) {
          const int eye_width = stereo_bgr.cols / 2;
          left_bgr = stereo_bgr(cv::Rect(0, 0, eye_width, stereo_bgr.rows)).clone();
          right_bgr = stereo_bgr(cv::Rect(eye_width, 0, eye_width, stereo_bgr.rows)).clone();
        }
      }
    } else {
      left_ok = !needs_left_frame ||
        (left_capture_.read(left_bgr) && !left_bgr.empty());
      right_ok = !needs_right_frame ||
        (right_capture_.read(right_bgr) && !right_bgr.empty());
    }
    if (!left_ok || !right_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read a stereo frame");
      return;
    }

    if (needs_depth && left_bgr.size() != right_bgr.size()) {
      cv::resize(right_bgr, right_bgr, left_bgr.size());
    }

    const auto stamp = now();
    const auto width = !left_bgr.empty() ? left_bgr.cols :
      (!right_bgr.empty() ? right_bgr.cols : image_width_);
    const auto height = !left_bgr.empty() ? left_bgr.rows :
      (!right_bgr.empty() ? right_bgr.rows : image_height_);

    if (publish_left) {
      left_image_pub_->publish(mat_to_image_msg(left_bgr, "bgr8", left_frame_id_, stamp));
    }
    if (publish_right) {
      right_image_pub_->publish(mat_to_image_msg(right_bgr, "bgr8", right_frame_id_, stamp));
    }
    if (publish_left_info) {
      left_info_pub_->publish(make_camera_info_msg(
          width, height, fx_, fy_, cx_, cy_, 0.0, left_frame_id_, stamp));
    }
    if (publish_right_info) {
      right_info_pub_->publish(make_camera_info_msg(
          width, height, fx_, fy_, cx_, cy_, baseline_m_, right_frame_id_, stamp));
    }
    if (!needs_depth) {
      return;
    }

    cv::Mat left_gray;
    cv::Mat right_gray;
    cv::cvtColor(left_bgr, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_bgr, right_gray, cv::COLOR_BGR2GRAY);

    cv::Mat disparity_fixed;
    stereo_->compute(left_gray, right_gray, disparity_fixed);

    cv::Mat depth(height, width, CV_32FC1);
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    for (int v = 0; v < height; ++v) {
      const int16_t * disparity_row = disparity_fixed.ptr<int16_t>(v);
      float * depth_row = depth.ptr<float>(v);
      for (int u = 0; u < width; ++u) {
        const float disparity_px = static_cast<float>(disparity_row[u]) / 16.0F;
        if (disparity_px <= 0.0F) {
          depth_row[u] = invalid;
          continue;
        }
        const float z = static_cast<float>((fx_ * baseline_m_) / disparity_px);
        depth_row[u] = (z >= depth_min_m_ && z <= depth_max_m_) ? z : invalid;
      }
    }

    if (publish_depth) {
      depth_pub_->publish(mat_to_image_msg(depth, "32FC1", depth_frame_id_, stamp));
    }

    if (publish_points) {
      pointcloud_pub_->publish(
        depth_to_point_cloud_msg(
          depth, fx_, fy_, cx_, cy_, pointcloud_stride_, depth_min_m_, depth_max_m_,
          depth_frame_id_, stamp));
    }
  }

  std::string left_device_;
  std::string right_device_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string depth_frame_id_;
  int image_width_;
  int image_height_;
  int framerate_;
  double baseline_m_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double depth_min_m_;
  double depth_max_m_;
  bool publish_pointcloud_;
  bool combined_stereo_device_{false};
  int pointcloud_stride_;

  cv::VideoCapture left_capture_;
  cv::VideoCapture right_capture_;
  cv::Ptr<cv::StereoSGBM> stereo_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

}  // namespace zed2i_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed2i_driver::CpuStereoNode)
