#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <zed2i_driver/cv_bridge_include.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

class BackCamH26xGroundVideoStreamer : public rclcpp::Node
{
public:
  BackCamH26xGroundVideoStreamer()
  : Node("back_cam_h26x_ground_video_streamer")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/back_cam/image_raw");
    host_ = declare_parameter<std::string>("host", "");
    port_ = declare_parameter<int>("port", 5601);
    codec_ = declare_parameter<std::string>("codec", "h264");
    fps_ = declare_parameter<double>("fps", 5.0);
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    bitrate_kbps_ = declare_parameter<int>("bitrate_kbps", 800);
    keyframe_interval_ = declare_parameter<int>("keyframe_interval", 5);
    mtu_ = declare_parameter<int>("mtu", 1200);

    if (image_topic_.empty() || port_ < 1 || port_ > 65535 ||
      (codec_ != "h264" && codec_ != "h265") || fps_ <= 0.0 || width_ <= 0 || height_ <= 0 ||
      bitrate_kbps_ <= 0 || keyframe_interval_ < 1 || mtu_ < 256 || mtu_ > 65507)
    {
      throw std::invalid_argument("invalid back-camera H.26x ground-video configuration");
    }

    // An empty host is deliberately a no-op. This keeps the miniPC bringup usable
    // without a ground station while making H.264 the selected path once a host is set.
    if (host_.empty()) {
      RCLCPP_INFO(get_logger(), "Back-camera H.26x ground-video disabled: host is empty");
      return;
    }

    gst_init(nullptr, nullptr);
    const std::string encoder = codec_ == "h264" ? "vaapih264enc" : "vaapih265enc";
    const std::string parser = codec_ == "h264" ? "h264parse" : "h265parse";
    const std::string payloader = codec_ == "h264" ? "rtph264pay" : "rtph265pay";
    GError * error = nullptr;
    const std::string pipeline_description =
      "appsrc name=source is-live=true format=time block=false do-timestamp=false "
      "! videoconvert ! video/x-raw,format=NV12 "
      "! " + encoder + " bitrate=" + std::to_string(bitrate_kbps_) +
      " keyframe-period=" + std::to_string(keyframe_interval_) +
      " cpb-length=100 max-bframes=0 " +
      " ! " + parser + " config-interval=1 ! " + payloader + " pt=96 mtu=" +
      std::to_string(mtu_) + " ! udpsink host=" + host_ + " port=" +
      std::to_string(port_) + " sync=false async=false";
    pipeline_ = gst_parse_launch(pipeline_description.c_str(), &error);
    if (error != nullptr || pipeline_ == nullptr) {
      const std::string message = error == nullptr ? "unknown GStreamer error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error(
              "could not create VA-API " + codec_ + " ground-video pipeline: " + message +
              ". Install a GStreamer VA-API encoder and verify it with gst-inspect-1.0 " + encoder);
    }
    appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
    if (appsrc_ == nullptr) {
      throw std::runtime_error("could not find back-camera H.26x appsrc element");
    }
    int fps_numerator = 0;
    int fps_denominator = 1;
    gst_util_double_to_fraction(fps_, &fps_numerator, &fps_denominator);
    GstCaps * caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT, width_,
      "height", G_TYPE_INT, height_, "framerate", GST_TYPE_FRACTION,
      fps_numerator, fps_denominator, nullptr);
    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("could not start VA-API H.26x ground-video pipeline");
    }

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) { latest_image_ = std::move(message); });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / fps_)),
      std::bind(&BackCamH26xGroundVideoStreamer::publish_latest, this));
    RCLCPP_INFO(get_logger(), "Sending %s as VA-API %s to %s:%d at %.1f FPS, %d kbps",
      image_topic_.c_str(), codec_.c_str(), host_.c_str(), port_, fps_, bitrate_kbps_);
  }

  ~BackCamH26xGroundVideoStreamer() override
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsrc_ != nullptr) {
      gst_object_unref(appsrc_);
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
    }
  }

private:
  void publish_latest()
  {
    if (!latest_image_) {
      return;
    }
    try {
      cv::Mat bgr = cv_bridge::toCvCopy(latest_image_, "bgr8")->image;
      cv::Mat resized;
      cv::resize(bgr, resized, cv::Size(width_, height_), 0.0, 0.0, cv::INTER_AREA);
      const auto bytes = resized.total() * resized.elemSize();
      GstBuffer * buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
      gst_buffer_fill(buffer, 0, resized.data, bytes);
      GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(frame_index_ * GST_SECOND / fps_);
      GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
      GST_BUFFER_DURATION(buffer) = static_cast<GstClockTime>(GST_SECOND / fps_);
      ++frame_index_;
      if (gst_app_src_push_buffer(appsrc_, buffer) != GST_FLOW_OK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Back-camera VA-API H.26x stream is not accepting frames");
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Could not convert back-camera image for H.26x: %s", error.what());
    }
  }

  std::string image_topic_;
  std::string host_;
  std::string codec_;
  int port_{};
  double fps_{};
  int width_{};
  int height_{};
  int bitrate_kbps_{};
  int keyframe_interval_{};
  int mtu_{};
  GstElement * pipeline_{nullptr};
  GstAppSrc * appsrc_{nullptr};
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t frame_index_{0};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BackCamH26xGroundVideoStreamer>());
  rclcpp::shutdown();
  return 0;
}
