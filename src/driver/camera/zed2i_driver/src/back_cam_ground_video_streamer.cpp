#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <zed2i_driver/cv_bridge_include.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

class BackCamGroundVideoStreamer : public rclcpp::Node
{
public:
  BackCamGroundVideoStreamer()
  : Node("back_cam_ground_video_streamer")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/back_cam/image_raw");
    host_ = declare_parameter<std::string>("host", "");
    port_ = declare_parameter<int>("port", 5601);
    fps_ = declare_parameter<double>("fps", 5.0);
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 70);
    mtu_ = declare_parameter<int>("mtu", 1200);

    if (host_.empty() || port_ < 1 || port_ > 65535 || fps_ <= 0.0 || width_ <= 0 ||
      height_ <= 0 || jpeg_quality_ < 1 || jpeg_quality_ > 100 || mtu_ < 256 || mtu_ > 65507)
    {
      throw std::invalid_argument("invalid back-camera ground-video configuration");
    }

    gst_init(nullptr, nullptr);
    GError * error = nullptr;
    const std::string pipeline_description =
      "appsrc name=source is-live=true format=time block=false do-timestamp=false "
      "! jpegparse ! rtpjpegpay mtu=" + std::to_string(mtu_) +
      " ! udpsink host=" + host_ + " port=" + std::to_string(port_) +
      " sync=false async=false";
    pipeline_ = gst_parse_launch(pipeline_description.c_str(), &error);
    if (error != nullptr || pipeline_ == nullptr) {
      const std::string message = error == nullptr ? "unknown GStreamer error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("could not create back-camera ground-video pipeline: " + message);
    }
    appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
    if (appsrc_ == nullptr) {
      throw std::runtime_error("could not find back-camera ground-video appsrc element");
    }
    // Caps MUST be set before the PLAYING transition: appsrc negotiates its
    // source pad caps as part of that state change, and setting them
    // afterwards can leave the pad unnegotiated, silently dropping every
    // buffer downstream with no error ever surfacing here.
    GstCaps * caps = gst_caps_new_empty_simple("image/jpeg");
    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("could not start back-camera ground-video pipeline");
    }

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) { latest_image_ = std::move(message); });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / fps_)),
      std::bind(&BackCamGroundVideoStreamer::publish_latest, this));
    RCLCPP_INFO(get_logger(), "Sending %s to %s:%d at %.1f FPS", image_topic_.c_str(),
      host_.c_str(), port_, fps_);
  }

  ~BackCamGroundVideoStreamer() override
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
    const auto image = latest_image_;
    if (!image) {
      return;
    }
    try {
      cv::Mat bgr = cv_bridge::toCvCopy(image, "bgr8")->image;
      cv::Mat resized;
      cv::resize(bgr, resized, cv::Size(width_, height_), 0.0, 0.0, cv::INTER_AREA);
      std::vector<unsigned char> jpeg;
      if (!cv::imencode(".jpg", resized, jpeg, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) {
        RCLCPP_WARN(get_logger(), "Could not JPEG encode back-camera image");
        return;
      }
      GstBuffer * buffer = gst_buffer_new_allocate(nullptr, jpeg.size(), nullptr);
      gst_buffer_fill(buffer, 0, jpeg.data(), jpeg.size());
      GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(frame_index_ * GST_SECOND / fps_);
      GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
      GST_BUFFER_DURATION(buffer) = static_cast<GstClockTime>(GST_SECOND / fps_);
      ++frame_index_;
      if (gst_app_src_push_buffer(appsrc_, buffer) != GST_FLOW_OK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Back-camera RTP/JPEG stream is not accepting frames");
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Could not convert back-camera image: %s", error.what());
    }
  }

  std::string image_topic_;
  std::string host_;
  int port_{};
  double fps_{};
  int width_{};
  int height_{};
  int jpeg_quality_{};
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
  rclcpp::spin(std::make_shared<BackCamGroundVideoStreamer>());
  rclcpp::shutdown();
  return 0;
}
