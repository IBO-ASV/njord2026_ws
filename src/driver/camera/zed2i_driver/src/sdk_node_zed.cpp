#include "zed2i_driver/message_utils.hpp"
#include "zed2i_driver/detection_message_utils.hpp"
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
#include "zed2i_driver/gpu_depth_median.hpp"
#include "zed2i_driver/perception_logic.hpp"
#include "zed2i_driver/tensor_rt_detector.hpp"
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
#include "zed2i_driver/ground_video_streamer.hpp"
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sl/Camera.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>
#include <mutex>
#include <optional>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace zed2i_driver
{

namespace
{

sl::RESOLUTION parse_camera_resolution(const std::string & value)
{
  if (value == "HD2K") return sl::RESOLUTION::HD2K;
  if (value == "HD1080") return sl::RESOLUTION::HD1080;
  if (value == "HD720") return sl::RESOLUTION::HD720;
  if (value == "VGA") return sl::RESOLUTION::VGA;
  throw std::invalid_argument("camera_resolution must be one of HD2K, HD1080, HD720, or VGA");
}

// Per-row [x_start, x_end) column bounds of the human-FOV-like elliptical
// region of interest. Precomputing this once (per camera-info change) lets
// every per-frame consumer (RGB, depth, point cloud) skip whole outside
// spans with a memset/fill instead of testing every pixel against the
// ellipse equation.
struct EllipseRowSpans
{
  std::vector<int> x_start;
  std::vector<int> x_end;  // exclusive
};

EllipseRowSpans compute_ellipse_row_spans(
  int width, int height, double cx, double cy, double a, double b)
{
  EllipseRowSpans spans;
  spans.x_start.assign(static_cast<size_t>(height), 0);
  spans.x_end.assign(static_cast<size_t>(height), 0);
  a = std::max(a, 1e-6);
  b = std::max(b, 1e-6);
  for (int y = 0; y < height; ++y) {
    const double dy = (static_cast<double>(y) - cy) / b;
    const double term = 1.0 - dy * dy;
    if (term < 0.0) {
      continue;
    }
    const double dx = a * std::sqrt(term);
    int x0 = static_cast<int>(std::ceil(cx - dx));
    int x1 = static_cast<int>(std::floor(cx + dx)) + 1;
    x0 = std::clamp(x0, 0, width);
    x1 = std::clamp(x1, 0, width);
    if (x1 < x0) {
      x1 = x0;
    }
    spans.x_start[static_cast<size_t>(y)] = x0;
    spans.x_end[static_cast<size_t>(y)] = x1;
  }
  return spans;
}

// Blacks out the columns outside the precomputed per-row span of a BGRA8
// image (4 bytes/pixel), leaving the inside untouched.
void apply_ellipse_mask_bgra(cv::Mat & image, const EllipseRowSpans & spans)
{
  for (int y = 0; y < image.rows; ++y) {
    auto * row = image.ptr<std::uint8_t>(y);
    const int x0 = spans.x_start[static_cast<size_t>(y)];
    const int x1 = spans.x_end[static_cast<size_t>(y)];
    if (x0 > 0) {
      std::memset(row, 0, static_cast<size_t>(x0) * 4);
    }
    if (x1 < image.cols) {
      std::memset(
        row + static_cast<size_t>(x1) * 4, 0,
        static_cast<size_t>(image.cols - x1) * 4);
    }
  }
}

// Invalidates (NaN) the columns outside the precomputed per-row span of a
// 32FC1 depth image. NaN matches the depth invalid convention already
// checked by depth_to_point_cloud_msg (std::isfinite), so the point-cloud
// builder automatically skips masked-out pixels with no further changes.
void apply_ellipse_mask_depth(cv::Mat & depth, const EllipseRowSpans & spans)
{
  constexpr float kInvalid = std::numeric_limits<float>::quiet_NaN();
  for (int y = 0; y < depth.rows; ++y) {
    auto * row = depth.ptr<float>(y);
    const int x0 = spans.x_start[static_cast<size_t>(y)];
    const int x1 = spans.x_end[static_cast<size_t>(y)];
    std::fill(row, row + x0, kInvalid);
    std::fill(row + x1, row + depth.cols, kInvalid);
  }
}

struct DepthAnnotatedDetection
{
  Detection2D detection;
  float depth_m{};
  bool is_vessel{false};
};

struct DetectionVisualStyle
{
  const char * label;
  cv::Scalar color;
};

// The TensorRT buoy model uses the same class contract as virtual-wall
// generation: class 0 is green and class 1 is red.  Keep the visual mapping
// here, beside the Foxglove annotation, so an operator can immediately see
// that the image and the downstream navigation classification agree.
DetectionVisualStyle detection_visual_style(int class_id, bool is_vessel)
{
  if (is_vessel) {
    return {"boat", cv::Scalar(255, 0, 0, 255)};
  }
  switch (class_id) {
    case 0:
      return {"green buoy", cv::Scalar(0, 255, 0, 255)};
    case 1:
      return {"red buoy", cv::Scalar(0, 0, 255, 255)};
    default:
      return {"unknown buoy", cv::Scalar(0, 255, 255, 255)};
  }
}

void draw_depth_annotations(
  cv::Mat & image, const std::vector<DepthAnnotatedDetection> & detections)
{
  for (const auto & item : detections) {
    const auto & box = item.detection;
    const auto style = detection_visual_style(box.class_id, item.is_vessel);
    const cv::Point top_left{cvRound(box.x1), cvRound(box.y1)};
    const cv::Point bottom_right{cvRound(box.x2), cvRound(box.y2)};
    cv::rectangle(image, top_left, bottom_right, style.color, 2);
    char label[96];
    if (std::isfinite(item.depth_m)) {
      std::snprintf(
        label, sizeof(label), "%s %.0f%%  ZED %.2f m", style.label,
        box.confidence * 100.0F, item.depth_m);
    } else {
      std::snprintf(
        label, sizeof(label), "%s %.0f%%  ZED N/A", style.label,
        box.confidence * 100.0F);
    }
    constexpr double kFontScale = 0.5;
    constexpr int kThickness = 1;
    int baseline = 0;
    const auto text_size = cv::getTextSize(
      label, cv::FONT_HERSHEY_SIMPLEX, kFontScale, kThickness, &baseline);
    const int text_y = std::max(text_size.height + baseline + 4, top_left.y - 6);
    const cv::Point text_origin{top_left.x, text_y};
    cv::rectangle(
      image,
      {text_origin.x, text_origin.y - text_size.height - baseline - 3},
      {text_origin.x + text_size.width + 4, text_origin.y + 3},
      cv::Scalar(0, 0, 0, 255), cv::FILLED);
    cv::putText(
      image, label, text_origin, cv::FONT_HERSHEY_SIMPLEX,
      kFontScale, style.color, kThickness, cv::LINE_AA);
  }
}

sensor_msgs::msg::PointCloud2::UniquePtr depth_to_detection_point_cloud_msg(
  const cv::Mat & depth_m, const std::vector<Detection2D> & detections,
  double fx, double fy, double cx, double cy, int stride,
  double depth_min_m, double depth_max_m, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp = stamp;
  cloud->header.frame_id = frame_id;
  cloud->height = 1;
  cloud->is_bigendian = false;
  cloud->is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(static_cast<size_t>(depth_m.rows / stride + 1) *
    static_cast<size_t>(depth_m.cols / stride + 1));

  size_t count = 0;
  for (int v = 0; v < depth_m.rows; v += stride) {
    const auto * row = depth_m.ptr<float>(v);
    for (int u = 0; u < depth_m.cols; u += stride) {
      if (std::none_of(detections.begin(), detections.end(), [u, v](const auto & box) {
            return u >= box.x1 && u < box.x2 && v >= box.y1 && v < box.y2;
          })) continue;
      const float z = row[u];
      if (!std::isfinite(z) || z < depth_min_m || z > depth_max_m) continue;
      const float x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
      const float y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);
      auto * data = cloud->data.data() + count * cloud->point_step;
      std::memcpy(data + cloud->fields[0].offset, &x, sizeof(x));
      std::memcpy(data + cloud->fields[1].offset, &y, sizeof(y));
      std::memcpy(data + cloud->fields[2].offset, &z, sizeof(z));
      ++count;
    }
  }
  cloud->width = static_cast<uint32_t>(count);
  cloud->row_step = cloud->width * cloud->point_step;
  cloud->data.resize(cloud->row_step);
  return cloud;
}

}  // namespace

class SdkNode : public rclcpp::Node
{
public:
  explicit SdkNode(const rclcpp::NodeOptions & options)
  : Node("zed2i_sdk_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    left_frame_id_ = declare_parameter<std::string>("left_frame_id", "zed2i_left_camera_frame");
    right_frame_id_ = declare_parameter<std::string>("right_frame_id", "zed2i_right_camera_frame");
    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", left_frame_id_);
    framerate_ = declare_parameter<int>("framerate", 15);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    pointcloud_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int>("pointcloud_stride", 2)));
    depth_min_m_ = declare_parameter<double>("depth_min_m", 0.3);
    depth_max_m_ = declare_parameter<double>("depth_max_m", 20.0);
    const bool disable_self_calibration = declare_parameter<bool>("disable_self_calibration", true);
    aec_agc_enable_ = declare_parameter<bool>("aec_agc_enable", true);
    aec_agc_roi_enable_ = declare_parameter<bool>("aec_agc_roi_enable", true);
    aec_agc_roi_x_ratio_ = declare_parameter<double>("aec_agc_roi_x_ratio", 0.0);
    aec_agc_roi_y_ratio_ = declare_parameter<double>("aec_agc_roi_y_ratio", 0.5);
    aec_agc_roi_width_ratio_ = declare_parameter<double>("aec_agc_roi_width_ratio", 1.0);
    aec_agc_roi_height_ratio_ = declare_parameter<double>("aec_agc_roi_height_ratio", 0.5);
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    fov_ellipse_enable_ = declare_parameter<bool>("fov_ellipse_enable", false);
    fov_ellipse_cx_ratio_ = declare_parameter<double>("fov_ellipse_cx_ratio", 0.5);
    fov_ellipse_cy_ratio_ = declare_parameter<double>("fov_ellipse_cy_ratio", 0.5);
    fov_ellipse_a_ratio_ = declare_parameter<double>("fov_ellipse_a_ratio", 0.5);
    fov_ellipse_b_ratio_ = declare_parameter<double>("fov_ellipse_b_ratio", 0.5);
    const auto camera_resolution = declare_parameter<std::string>("camera_resolution", "HD720");
    const bool enable_ground_video = declare_parameter<bool>("enable_ground_video", false);
    const auto ground_video_codec = declare_parameter<std::string>("ground_video_codec", "jpeg");
    ground_video_config_.width = declare_parameter<int>("ground_video_width", 480);
    ground_video_config_.height = declare_parameter<int>("ground_video_height", 360);
    ground_video_config_.fps = declare_parameter<double>("ground_video_fps", 4.0);
    ground_video_config_.jpeg_quality = declare_parameter<int>("ground_video_jpeg_quality", 70);
    ground_video_config_.host = declare_parameter<std::string>("ground_video_host", "");
    ground_video_config_.port = declare_parameter<int>("ground_video_port", 5600);
    ground_video_config_.max_pending_frames = declare_parameter<int>("ground_video_max_pending_frames", 1);
    ground_video_config_.mtu = declare_parameter<int>("ground_video_mtu", 1200);
    ground_video_draw_detections_ = declare_parameter<bool>("ground_video_draw_detections", false);
    const bool enable_gpu_perception = declare_parameter<bool>("enable_gpu_perception", false);
    const bool enable_vessel_perception = declare_parameter<bool>("enable_vessel_perception", false);
    if (enable_gpu_perception) {
#ifndef ZED2I_DRIVER_HAS_GPU_PERCEPTION
      RCLCPP_FATAL(
        get_logger(),
        "enable_gpu_perception=true requires a build with ZED, CUDA, and TensorRT support.");
      rclcpp::shutdown();
      return;
#else
      if (engine_path_.empty()) {
        RCLCPP_FATAL(get_logger(), "enable_gpu_perception=true requires a non-empty engine_path");
        rclcpp::shutdown();
        return;
      }
      confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.25);
      max_detections_ = declare_parameter<int>("max_detections", 32);
      detection_topic_ = declare_parameter<std::string>("detection_topic", "/buoy_detections_3d");
      output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
      publish_debug_detections_ = declare_parameter<bool>("publish_debug_detections", false);
      publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
      depth_center_ratio_ = declare_parameter<double>("depth_center_ratio", 0.5);
      depth_sample_stride_ = declare_parameter<int>("depth_sample_stride", 2);
      min_valid_depth_samples_ = declare_parameter<int>("min_valid_depth_samples", 16);
      enable_virtual_wall_ = declare_parameter<bool>("enable_virtual_wall", true);
      virtual_wall_topic_ = declare_parameter<std::string>(
        "virtual_wall_topic", "/virtual_obstacles");
      wall_frame_ = declare_parameter<std::string>("wall_frame", "map");
      channel_heading_rad_ = declare_parameter<double>("channel_heading_rad", NAN);
      wall_radius_m_ = declare_parameter<double>("wall_radius_m", 2.0);
      wall_points_per_full_circle_ = declare_parameter<int>("wall_points_per_full_circle", 40);
      connect_same_color_buoys_ = declare_parameter<bool>("connect_same_color_buoys", true);
      same_color_wall_max_gap_m_ = declare_parameter<double>("same_color_wall_max_gap_m", 12.0);
      same_color_wall_point_spacing_m_ = declare_parameter<double>(
        "same_color_wall_point_spacing_m", 0.2);
      lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
      lidar_max_age_sec_ = declare_parameter<double>("lidar_max_age_sec", 0.15);
      lidar_cluster_tolerance_m_ = declare_parameter<double>("lidar_cluster_tolerance_m", 0.15);
      lidar_min_cluster_points_ = declare_parameter<int>("lidar_min_cluster_points", 5);
      lidar_max_cluster_points_ = declare_parameter<int>("lidar_max_cluster_points", 5000);
      detector_ = std::make_unique<TensorRtDetector>(engine_path_, confidence_threshold_, max_detections_);
      // Canonical output for Nav2-side perception consumers.  This must not
      // depend on the optional debug-detection setting.
      detection_pub_ = create_publisher<njord_interfaces::msg::BuoyDetectionArray>(
        detection_topic_, rclcpp::QoS(10));
      if (enable_virtual_wall_) {
        virtual_wall_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          virtual_wall_topic_, rclcpp::QoS(10));
      }
      lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(lidar_mutex_);
          latest_lidar_ = std::move(message);
        });
#endif
    }

    if (enable_vessel_perception) {
#ifndef ZED2I_DRIVER_HAS_GPU_PERCEPTION
      RCLCPP_FATAL(
        get_logger(),
        "enable_vessel_perception=true requires a build with ZED, CUDA, and TensorRT support.");
      rclcpp::shutdown();
      return;
#else
      vessel_engine_path_ = declare_parameter<std::string>("vessel_engine_path", "");
      vessel_confidence_threshold_ = declare_parameter<double>("vessel_confidence_threshold", 0.25);
      vessel_max_detections_ = declare_parameter<int>("vessel_max_detections", 16);
      if (!enable_gpu_perception) {
        publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
        depth_center_ratio_ = declare_parameter<double>("depth_center_ratio", 0.5);
        depth_sample_stride_ = declare_parameter<int>("depth_sample_stride", 2);
        min_valid_depth_samples_ = declare_parameter<int>("min_valid_depth_samples", 16);
      }
      if (vessel_engine_path_.empty()) {
        RCLCPP_FATAL(
          get_logger(), "enable_vessel_perception=true requires a non-empty vessel_engine_path");
        rclcpp::shutdown();
        return;
      }
      vessel_detector_ = std::make_unique<TensorRtDetector>(
        vessel_engine_path_, static_cast<float>(vessel_confidence_threshold_), vessel_max_detections_);
#endif
    }

    rclcpp::QoS image_qos(rclcpp::KeepLast(5));
    image_qos.best_effort();
    image_qos.durability_volatile();

    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_rect", image_qos);
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    if (detector_ || vessel_detector_) {
      debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "debug/detections_image", image_qos);
    }
#endif
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_rect", image_qos);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", image_qos);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info",
        image_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image", image_qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", image_qos);
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    detection_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "points/detections", image_qos);
#endif

    sl::InitParameters init_params;
    try {
      init_params.camera_resolution = parse_camera_resolution(camera_resolution);
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(get_logger(), "%s", error.what());
      rclcpp::shutdown();
      return;
    }
    init_params.camera_fps = framerate_;
    init_params.camera_disable_self_calib = disable_self_calibration;
    init_params.depth_mode = sl::DEPTH_MODE::QUALITY;
    init_params.coordinate_units = sl::UNIT::METER;

    const auto error = camera_.open(init_params);
    if (error != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_FATAL(get_logger(), "Failed to open ZED camera: %s", sl::toString(error).c_str());
      rclcpp::shutdown();
      return;
    }

    const auto info = camera_.getCameraInformation();
    const auto calibration = info.camera_configuration.calibration_parameters;
    width_ = static_cast<int>(info.camera_configuration.resolution.width);
    height_ = static_cast<int>(info.camera_configuration.resolution.height);
    fx_ = calibration.left_cam.fx;
    fy_ = calibration.left_cam.fy;
    cx_ = calibration.left_cam.cx;
    cy_ = calibration.left_cam.cy;
    baseline_m_ = std::abs(calibration.stereo_transform.getTranslation().tx);
    if (baseline_m_ > 10.0) {
      baseline_m_ *= 0.001;
    }
    configure_auto_exposure_gain();
    ellipse_cx_ = fov_ellipse_cx_ratio_ * width_;
    ellipse_cy_ = fov_ellipse_cy_ratio_ * height_;
    ellipse_a_ = fov_ellipse_a_ratio_ * width_;
    ellipse_b_ = fov_ellipse_b_ratio_ * height_;
    if (fov_ellipse_enable_) {
      ellipse_row_spans_ = compute_ellipse_row_spans(
        width_, height_, ellipse_cx_, ellipse_cy_, ellipse_a_, ellipse_b_);
    }
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    if (detector_ || vessel_detector_) {
      left_gpu_.alloc(width_, height_, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
      depth_gpu_.alloc(width_, height_, sl::MAT_TYPE::F32_C1, sl::MEM::GPU);
    }
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
    if (enable_ground_video && ground_video_codec == "jpeg") {
      GroundVideoConfig config;
      config.source_width = width_;
      config.source_height = height_;
      config.width = ground_video_config_.width;
      config.height = ground_video_config_.height;
      config.fps = ground_video_config_.fps;
      config.jpeg_quality = ground_video_config_.jpeg_quality;
      config.host = ground_video_config_.host;
      config.port = ground_video_config_.port;
      config.max_pending_frames = ground_video_config_.max_pending_frames;
      config.mtu = ground_video_config_.mtu;
      config.fov_ellipse_enable = fov_ellipse_enable_;
      config.fov_ellipse_cx_ratio = fov_ellipse_cx_ratio_;
      config.fov_ellipse_cy_ratio = fov_ellipse_cy_ratio_;
      config.fov_ellipse_a_ratio = fov_ellipse_a_ratio_;
      config.fov_ellipse_b_ratio = fov_ellipse_b_ratio_;
      try {
        ground_video_streamer_ = std::make_unique<GroundVideoStreamer>(config);
        if (!left_gpu_.isInit()) {
          left_gpu_.alloc(width_, height_, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Ground-video streaming disabled: %s", error.what());
      }
    }
    if (enable_ground_video && ground_video_codec != "jpeg") {
      RCLCPP_ERROR(get_logger(), "Ground-video streaming disabled: only JPEG is supported");
    }
#else
    if (enable_ground_video) {
      RCLCPP_ERROR(
        get_logger(),
        "Ground-video streaming disabled: this build requires ZED, CUDA, nvJPEG, and GStreamer appsrc.");
    }
#endif

    const auto period_ms = std::max(1, 1000 / std::max(1, framerate_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SdkNode::grab_and_publish, this));
  }

  ~SdkNode() override
  {
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
    // Stop the worker before releasing any camera-backed CUDA storage.
    ground_video_streamer_.reset();
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    detector_.reset();
    vessel_detector_.reset();
#endif
#if defined(ZED2I_DRIVER_HAS_GPU_PERCEPTION) || defined(ZED2I_DRIVER_HAS_GROUND_VIDEO)
    // GPU sl::Mat storage must be released while the camera's CUDA context is alive.
    left_gpu_.free();
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    depth_gpu_.free();
#endif
    camera_.close();
  }

private:
  void configure_auto_exposure_gain()
  {
    const auto aec_agc_error = camera_.setCameraSettings(
      sl::VIDEO_SETTINGS::AEC_AGC, aec_agc_enable_ ? 1 : 0);
    if (aec_agc_error != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN(
        get_logger(), "Failed to %s ZED automatic exposure/gain: %s",
        aec_agc_enable_ ? "enable" : "disable", sl::toString(aec_agc_error).c_str());
      return;
    }

    if (!aec_agc_enable_ || !aec_agc_roi_enable_) {
      return;
    }

    const auto x_ratio = std::clamp(aec_agc_roi_x_ratio_, 0.0, 1.0);
    const auto y_ratio = std::clamp(aec_agc_roi_y_ratio_, 0.0, 1.0);
    const auto width_ratio = std::clamp(aec_agc_roi_width_ratio_, 0.0, 1.0 - x_ratio);
    const auto height_ratio = std::clamp(aec_agc_roi_height_ratio_, 0.0, 1.0 - y_ratio);
    const int x = static_cast<int>(x_ratio * width_);
    const int y = static_cast<int>(y_ratio * height_);
    const int roi_width = std::max(1, static_cast<int>(width_ratio * width_));
    const int roi_height = std::max(1, static_cast<int>(height_ratio * height_));
    const sl::Rect roi(x, y, std::min(roi_width, width_ - x), std::min(roi_height, height_ - y));
    const auto roi_error = camera_.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC_ROI, roi);
    if (roi_error != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN(
        get_logger(), "Failed to set ZED automatic exposure/gain ROI: %s",
        sl::toString(roi_error).c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "ZED automatic exposure/gain enabled with ROI x=%d y=%d width=%d height=%d",
      roi.x, roi.y, roi.width, roi.height);
  }

#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
  std::optional<geometry_msgs::msg::PointStamped> lidar_fallback_position(
    const Detection2D & detection, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      cloud = latest_lidar_;
    }
    if (!cloud || cloud->header.frame_id.empty()) {
      return std::nullopt;
    }
    const auto age = std::abs((stamp - rclcpp::Time(cloud->header.stamp)).seconds());
    if (age > lidar_max_age_sec_) {
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped cloud_to_camera;
    try {
      cloud_to_camera = tf_buffer_.lookupTransform(
        left_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping LiDAR fallback: %s -> %s TF unavailable: %s",
        cloud->header.frame_id.c_str(), left_frame_id_.c_str(), error.what());
      return std::nullopt;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZ>());
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        geometry_msgs::msg::PointStamped point_in;
        point_in.header = cloud->header;
        point_in.point.x = *x;
        point_in.point.y = *y;
        point_in.point.z = *z;
        geometry_msgs::msg::PointStamped point_camera;
        tf2::doTransform(point_in, point_camera, cloud_to_camera);
        if (point_camera.point.z <= 0.0) {
          continue;
        }
        const auto u = fx_ * point_camera.point.x / point_camera.point.z + cx_;
        const auto v = fy_ * point_camera.point.y / point_camera.point.z + cy_;
        if (point_in_central_bbox_roi(
            static_cast<float>(u), static_cast<float>(v), detection,
            static_cast<float>(depth_center_ratio_))) {
          candidates->push_back({
            static_cast<float>(point_camera.point.x),
            static_cast<float>(point_camera.point.y),
            static_cast<float>(point_camera.point.z)});
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping LiDAR fallback: PointCloud2 lacks xyz fields: %s", error.what());
      return std::nullopt;
    }
    if (candidates->size() < static_cast<size_t>(lidar_min_cluster_points_)) {
      return std::nullopt;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(candidates);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(lidar_cluster_tolerance_m_);
    extraction.setMinClusterSize(lidar_min_cluster_points_);
    extraction.setMaxClusterSize(lidar_max_cluster_points_);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(candidates);
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);
    if (clusters.empty()) {
      return std::nullopt;
    }
    const auto center_u = (detection.x1 + detection.x2) * 0.5F;
    const auto center_v = (detection.y1 + detection.y2) * 0.5F;
    float best_distance = std::numeric_limits<float>::infinity();
    Eigen::Vector4f selected;
    bool found = false;
    for (const auto & cluster : clusters) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*candidates, cluster.indices, centroid);
      const auto u = static_cast<float>(fx_) * centroid.x() / centroid.z() + static_cast<float>(cx_);
      const auto v = static_cast<float>(fy_) * centroid.y() / centroid.z() + static_cast<float>(cy_);
      const auto distance = (u - center_u) * (u - center_u) + (v - center_v) * (v - center_v);
      if (distance < best_distance) {
        best_distance = distance;
        selected = centroid;
        found = true;
      }
    }
    if (!found) {
      return std::nullopt;
    }
    geometry_msgs::msg::PointStamped camera_point;
    camera_point.header.stamp = stamp;
    camera_point.header.frame_id = left_frame_id_;
    camera_point.point.x = selected.x();
    camera_point.point.y = selected.y();
    camera_point.point.z = selected.z();
    try {
      const auto camera_to_output = tf_buffer_.lookupTransform(
        output_frame_, left_frame_id_, stamp, rclcpp::Duration::from_seconds(0.05));
      geometry_msgs::msg::PointStamped output_point;
      tf2::doTransform(camera_point, output_point, camera_to_output);
      return output_point;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping LiDAR fallback: %s -> %s TF unavailable: %s",
        left_frame_id_.c_str(), output_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }
#endif

  template<typename MessageT>
  static bool has_subscribers(const std::shared_ptr<rclcpp::Publisher<MessageT>> & publisher)
  {
    return publisher && (publisher->get_subscription_count() > 0 ||
           publisher->get_intra_process_subscription_count() > 0);
  }

  static cv::Mat sl_mat_to_cv_bgra_view(const sl::Mat & mat)
  {
    const auto width = static_cast<int>(mat.getWidth());
    const auto height = static_cast<int>(mat.getHeight());
    return cv::Mat(
      height, width, CV_8UC4, mat.getPtr<sl::uchar1>(sl::MEM::CPU),
      mat.getStepBytes(sl::MEM::CPU));
  }

  // Cheap per-detection ellipse membership test used to gate GPU perception
  // output. width_/height_ and the ellipse_* members are fixed after the
  // camera opens, so this is a handful of flops per detection.
  bool ellipse_contains(double x, double y) const
  {
    const double dx = (x - ellipse_cx_) / std::max(ellipse_a_, 1e-6);
    const double dy = (y - ellipse_cy_) / std::max(ellipse_b_, 1e-6);
    return dx * dx + dy * dy <= 1.0;
  }

  static cv::Mat sl_mat_to_cv_depth_view(const sl::Mat & mat)
  {
    const auto width = static_cast<int>(mat.getWidth());
    const auto height = static_cast<int>(mat.getHeight());
    return cv::Mat(
      height, width, CV_32FC1, mat.getPtr<sl::float1>(sl::MEM::CPU),
      mat.getStepBytes(sl::MEM::CPU));
  }

  void grab_and_publish()
  {
    bool run_gpu = false;
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    run_gpu = static_cast<bool>(detector_) || static_cast<bool>(vessel_detector_);
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
    run_gpu = run_gpu || static_cast<bool>(ground_video_streamer_);
#endif
    bool publish_debug_image = false;
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    publish_debug_image = publish_debug_image_ && has_subscribers(debug_image_pub_);
#endif
    const bool publish_left = has_subscribers(left_image_pub_);
    const bool publish_right = has_subscribers(right_image_pub_);
    const bool publish_left_info = has_subscribers(left_info_pub_);
    const bool publish_right_info = has_subscribers(right_info_pub_);
    const bool publish_depth = has_subscribers(depth_pub_);
    const bool publish_points = publish_pointcloud_ && has_subscribers(pointcloud_pub_);
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    const bool publish_detection_points = has_subscribers(detection_pointcloud_pub_);
    std::vector<Detection2D> detection_boxes;
#endif

    if (!publish_left && !publish_right && !publish_left_info && !publish_right_info &&
      !publish_depth && !publish_points && !publish_debug_image && !run_gpu)
    {
      return;
    }

    sl::RuntimeParameters runtime_params;
    if (camera_.grab(runtime_params) != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to grab a ZED frame");
      return;
    }
    const auto stamp = now();
    if (publish_left_info) {
      left_info_pub_->publish(make_camera_info_msg(
          width_, height_, fx_, fy_, cx_, cy_, 0.0, left_frame_id_, stamp));
    }
    if (publish_right_info) {
      right_info_pub_->publish(make_camera_info_msg(
          width_, height_, fx_, fy_, cx_, cy_, baseline_m_, right_frame_id_, stamp));
    }

    bool left_published = false;
#if defined(ZED2I_DRIVER_HAS_GPU_PERCEPTION) || defined(ZED2I_DRIVER_HAS_GROUND_VIDEO)
    if (run_gpu) {
      const auto image_error = camera_.retrieveImage(left_gpu_, sl::VIEW::LEFT, sl::MEM::GPU);
      if (image_error != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to retrieve the left ZED image on GPU: %s",
          sl::toString(image_error).c_str());
      } else {
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
        std::vector<GroundVideoBox> ground_video_boxes;
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
        std::vector<DepthAnnotatedDetection> debug_detections;
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
        if (detector_) {
          camera_.retrieveMeasure(depth_gpu_, sl::MEASURE::DEPTH, sl::MEM::GPU);
          const auto * pointer = left_gpu_.getPtr<sl::uchar1>(sl::MEM::GPU);
          try {
            const auto detections = detector_->infer(
              pointer, left_gpu_.getStepBytes(sl::MEM::GPU), width_, height_, nullptr);
            const auto depth_pointer = depth_gpu_.getPtr<sl::float1>(sl::MEM::GPU);
            std::vector<PositionedDetection> positioned;
            positioned.reserve(detections.size());
            for (const auto & detection : detections) {
              if (fov_ellipse_enable_) {
                const auto center_u = (detection.x1 + detection.x2) * 0.5F;
                const auto center_v = (detection.y1 + detection.y2) * 0.5F;
                if (!ellipse_contains(center_u, center_v)) {
                  continue;
                }
              }
              if (publish_detection_points) detection_boxes.push_back(detection);
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
#endif
              PositionedDetection item{detection, nan_position(), PositionSource::kNone};
              const auto roi = central_depth_roi(
                detection, static_cast<float>(depth_center_ratio_), width_, height_);
              const auto depth = depth_median_gpu(
                depth_pointer, depth_gpu_.getStepBytes(sl::MEM::GPU), width_, height_,
                roi.x0, roi.y0, roi.x1, roi.y1, depth_sample_stride_,
                static_cast<float>(depth_min_m_), static_cast<float>(depth_max_m_),
                min_valid_depth_samples_, nullptr);
              if (publish_debug_image) {
                debug_detections.push_back({detection, depth, false});
              }
              if (std::isfinite(depth)) {
                RCLCPP_INFO_THROTTLE(
                  get_logger(), *get_clock(), 1000,
                  "Buoy detected: class=%d confidence=%.2f ZED depth=%.2f m",
                  detection.class_id, detection.confidence, depth);
                geometry_msgs::msg::PointStamped camera_point;
                camera_point.header.stamp = stamp;
                camera_point.header.frame_id = left_frame_id_;
                const auto u = (detection.x1 + detection.x2) * 0.5F;
                const auto v = (detection.y1 + detection.y2) * 0.5F;
                camera_point.point.x = (u - cx_) * depth / fx_;
                camera_point.point.y = (v - cy_) * depth / fy_;
                camera_point.point.z = depth;
                try {
                  const auto transform = tf_buffer_.lookupTransform(
                    output_frame_, left_frame_id_, stamp, rclcpp::Duration::from_seconds(0.05));
                  geometry_msgs::msg::PointStamped output_point;
                  tf2::doTransform(camera_point, output_point, transform);
                  item.position_base = {
                    static_cast<float>(output_point.point.x),
                    static_cast<float>(output_point.point.y),
                    static_cast<float>(output_point.point.z)};
                  item.source = PositionSource::kZedDepth;
                } catch (const tf2::TransformException & error) {
                  RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Skipping buoy position: %s -> %s TF unavailable: %s",
                    left_frame_id_.c_str(), output_frame_.c_str(), error.what());
                }
              } else {
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 1000,
                  "Buoy detected: class=%d confidence=%.2f ZED depth unavailable",
                  detection.class_id, detection.confidence);
              }
              if (item.source == PositionSource::kNone) {
                if (const auto fallback = lidar_fallback_position(detection, stamp)) {
                  item.position_base = {
                    static_cast<float>(fallback->point.x),
                    static_cast<float>(fallback->point.y),
                    static_cast<float>(fallback->point.z)};
                  item.source = PositionSource::kLidarFused;
                }
              }
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
              if (ground_video_draw_detections_) {
                GroundVideoBox box{detection.x1, detection.y1, detection.x2, detection.y2};
                std::snprintf(
                  box.label, sizeof(box.label), "%s", detection_range_label(item).c_str());
                ground_video_boxes.push_back(box);
              }
#endif
              positioned.push_back(item);
            }
            std_msgs::msg::Header header;
            header.stamp = stamp;
            header.frame_id = output_frame_;
            if (detection_pub_) {
              detection_pub_->publish(to_detection_array(positioned, header));
            }
            if (virtual_wall_pub_) {
              std::vector<PositionedDetection> wall_detections;
              wall_detections.reserve(positioned.size());
              for (const auto & item : positioned) {
                if (item.source == PositionSource::kNone) {
                  continue;
                }
                try {
                  geometry_msgs::msg::PointStamped output_point;
                  output_point.header = header;
                  output_point.point.x = item.position_base[0];
                  output_point.point.y = item.position_base[1];
                  output_point.point.z = item.position_base[2];
                  const auto transform = tf_buffer_.lookupTransform(
                    wall_frame_, output_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
                  geometry_msgs::msg::PointStamped wall_point;
                  tf2::doTransform(output_point, wall_point, transform);
                  auto wall_item = item;
                  wall_item.position_base = {
                    static_cast<float>(wall_point.point.x),
                    static_cast<float>(wall_point.point.y),
                    static_cast<float>(wall_point.point.z)};
                  wall_detections.push_back(wall_item);
                } catch (const tf2::TransformException & error) {
                  RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Skipping virtual wall: %s -> %s TF unavailable: %s",
                    output_frame_.c_str(), wall_frame_.c_str(), error.what());
                }
              }
              auto wall_header = header;
              wall_header.frame_id = wall_frame_;
              virtual_wall_pub_->publish(to_virtual_wall_cloud(
                wall_detections, wall_header, wall_frame_,
                static_cast<float>(channel_heading_rad_), static_cast<float>(wall_radius_m_),
                wall_points_per_full_circle_, connect_same_color_buoys_,
                static_cast<float>(same_color_wall_max_gap_m_),
                static_cast<float>(same_color_wall_point_spacing_m_)));
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 2000, "GPU perception stopped: %s", error.what());
            detector_.reset();
          }
        }
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
        if (vessel_detector_) {
          camera_.retrieveMeasure(depth_gpu_, sl::MEASURE::DEPTH, sl::MEM::GPU);
          const auto * pointer = left_gpu_.getPtr<sl::uchar1>(sl::MEM::GPU);
          const auto * depth_pointer = depth_gpu_.getPtr<sl::float1>(sl::MEM::GPU);
          try {
            const auto detections = vessel_detector_->infer(
              pointer, left_gpu_.getStepBytes(sl::MEM::GPU), width_, height_, nullptr);
            for (const auto & detection : detections) {
              if (fov_ellipse_enable_) {
                const auto center_u = (detection.x1 + detection.x2) * 0.5F;
                const auto center_v = (detection.y1 + detection.y2) * 0.5F;
                if (!ellipse_contains(center_u, center_v)) {
                  continue;
                }
              }
              if (publish_detection_points) detection_boxes.push_back(detection);
              const auto roi = central_depth_roi(
                detection, static_cast<float>(depth_center_ratio_), width_, height_);
              const auto depth = depth_median_gpu(
                depth_pointer, depth_gpu_.getStepBytes(sl::MEM::GPU), width_, height_,
                roi.x0, roi.y0, roi.x1, roi.y1, depth_sample_stride_,
                static_cast<float>(depth_min_m_), static_cast<float>(depth_max_m_),
                min_valid_depth_samples_, nullptr);
              if (publish_debug_image) {
                debug_detections.push_back({detection, depth, true});
              }
              if (std::isfinite(depth)) {
                RCLCPP_INFO_THROTTLE(
                  get_logger(), *get_clock(), 1000,
                  "Boat detected: confidence=%.2f ZED depth=%.2f m",
                  detection.confidence, depth);
              } else {
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 1000,
                  "Boat detected: confidence=%.2f ZED depth unavailable", detection.confidence);
              }
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 2000, "Vessel TensorRT perception stopped: %s", error.what());
            vessel_detector_.reset();
          }
        }
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
        // Use the same TensorRT detections for the ground video; no second YOLO pass.
        if (ground_video_streamer_) {
          ground_video_streamer_->submit(
            left_gpu_.getPtr<sl::uchar1>(sl::MEM::GPU), left_gpu_.getStepBytes(sl::MEM::GPU),
            width_, height_, ground_video_boxes);
        }
#endif
        if (publish_left || publish_debug_image) {
          const auto download_error = left_gpu_.updateCPUfromGPU();
          if (download_error == sl::ERROR_CODE::SUCCESS) {
            auto left_bgra = sl_mat_to_cv_bgra_view(left_gpu_);
            if (fov_ellipse_enable_) {
              apply_ellipse_mask_bgra(left_bgra, ellipse_row_spans_);
            }
            if (publish_left) {
              left_image_pub_->publish(
                mat_to_image_msg(left_bgra, "bgra8", left_frame_id_, stamp));
              left_published = true;
            }
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
            if (publish_debug_image) {
              auto debug_image = left_bgra.clone();
              draw_depth_annotations(debug_image, debug_detections);
              debug_image_pub_->publish(
                mat_to_image_msg(debug_image, "bgra8", left_frame_id_, stamp));
            }
#endif
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Failed to download the left ZED image after ground-video submission: %s",
              sl::toString(download_error).c_str());
          }
        }
      }
    }
#endif

    if (publish_left && !left_published) {
      sl::Mat left;
      camera_.retrieveImage(left, sl::VIEW::LEFT);
      auto left_bgra = sl_mat_to_cv_bgra_view(left);
      if (fov_ellipse_enable_) {
        apply_ellipse_mask_bgra(left_bgra, ellipse_row_spans_);
      }
      left_image_pub_->publish(mat_to_image_msg(left_bgra, "bgra8", left_frame_id_, stamp));
    }

    if (publish_right) {
      sl::Mat right;
      camera_.retrieveImage(right, sl::VIEW::RIGHT);
      auto right_bgra = sl_mat_to_cv_bgra_view(right);
      if (fov_ellipse_enable_) {
        apply_ellipse_mask_bgra(right_bgra, ellipse_row_spans_);
      }
      right_image_pub_->publish(mat_to_image_msg(right_bgra, "bgra8", right_frame_id_, stamp));
    }

    if (publish_depth || publish_points
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
      || publish_detection_points
#endif
    ) {
      sl::Mat depth;
      camera_.retrieveMeasure(depth, sl::MEASURE::DEPTH);
      // This is a non-owning view. Each requested ROS output copies it directly into its
      // final message allocation before the SDK buffer expires.
      auto depth_m = sl_mat_to_cv_depth_view(depth);
      if (fov_ellipse_enable_) {
        // NaN matches the existing invalid-depth convention, so the point
        // cloud loop below (which already tests std::isfinite) needs no
        // further change to skip masked-out pixels.
        apply_ellipse_mask_depth(depth_m, ellipse_row_spans_);
      }
      if (publish_depth) {
        depth_pub_->publish(mat_to_image_msg(depth_m, "32FC1", depth_frame_id_, stamp));
      }
      if (publish_points) {
        pointcloud_pub_->publish(
          depth_to_point_cloud_msg(
            depth_m, fx_, fy_, cx_, cy_, pointcloud_stride_, depth_min_m_, depth_max_m_,
            depth_frame_id_, stamp));
      }
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
      if (publish_detection_points && !detection_boxes.empty()) {
        detection_pointcloud_pub_->publish(depth_to_detection_point_cloud_msg(
            depth_m, detection_boxes, fx_, fy_, cx_, cy_, pointcloud_stride_,
            depth_min_m_, depth_max_m_, depth_frame_id_, stamp));
      }
#endif
    }
  }

  sl::Camera camera_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string depth_frame_id_;
  int framerate_;
  bool publish_pointcloud_;
  int pointcloud_stride_;
  double depth_min_m_;
  double depth_max_m_;
  bool aec_agc_enable_{true};
  bool aec_agc_roi_enable_{true};
  double aec_agc_roi_x_ratio_{0.0};
  double aec_agc_roi_y_ratio_{0.5};
  double aec_agc_roi_width_ratio_{1.0};
  double aec_agc_roi_height_ratio_{0.5};
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double baseline_m_;
  std::string engine_path_;
  std::string vessel_engine_path_;
  bool fov_ellipse_enable_{false};
  double fov_ellipse_cx_ratio_{0.5};
  double fov_ellipse_cy_ratio_{0.5};
  double fov_ellipse_a_ratio_{0.5};
  double fov_ellipse_b_ratio_{0.5};
  double ellipse_cx_{0.0};
  double ellipse_cy_{0.0};
  double ellipse_a_{0.0};
  double ellipse_b_{0.0};
  EllipseRowSpans ellipse_row_spans_;
  struct GroundVideoParameterStorage
  {
    int source_width{};
    int source_height{};
    int width{640};
    int height{360};
    double fps{5.0};
    int jpeg_quality{70};
    std::string host;
    int port{5600};
    int max_pending_frames{1};
    int mtu{1200};
  } ground_video_config_;
  bool ground_video_draw_detections_{false};
#if defined(ZED2I_DRIVER_HAS_GPU_PERCEPTION) || defined(ZED2I_DRIVER_HAS_GROUND_VIDEO)
  sl::Mat left_gpu_;
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
  std::unique_ptr<TensorRtDetector> detector_;
  std::unique_ptr<TensorRtDetector> vessel_detector_;
  sl::Mat depth_gpu_;
  std::string detection_topic_;
  std::string output_frame_;
  double confidence_threshold_{};
  double vessel_confidence_threshold_{};
  int max_detections_{};
  int vessel_max_detections_{};
  bool publish_debug_detections_{};
  bool publish_debug_image_{true};
  double depth_center_ratio_{};
  int depth_sample_stride_{};
  int min_valid_depth_samples_{};
  bool enable_virtual_wall_{};
  std::string virtual_wall_topic_;
  std::string wall_frame_;
  double channel_heading_rad_{};
  double wall_radius_m_{};
  int wall_points_per_full_circle_{};
  bool connect_same_color_buoys_{};
  double same_color_wall_max_gap_m_{};
  double same_color_wall_point_spacing_m_{};
  rclcpp::Publisher<njord_interfaces::msg::BuoyDetectionArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr virtual_wall_pub_;
  std::string lidar_topic_;
  double lidar_max_age_sec_{};
  double lidar_cluster_tolerance_m_{};
  int lidar_min_cluster_points_{};
  int lidar_max_cluster_points_{};
  std::mutex lidar_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
  std::unique_ptr<GroundVideoStreamer> ground_video_streamer_;
#endif

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detection_pointcloud_pub_;
#endif
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace zed2i_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed2i_driver::SdkNode)
