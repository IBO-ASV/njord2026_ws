#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class PreviewNode final : public rclcpp::Node
{
public:
  PreviewNode() : Node("foxglove_image_preview")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/zed2i/stereo/image_raw");
    output_prefix_ = declare_parameter<std::string>(
      "output_prefix", "/zed2i/left/preview");
    output_raw_topic_ = declare_parameter<std::string>("output_raw_topic", "");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "");
    split_stereo_ = declare_parameter<bool>("split_stereo", true);
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 360);
    max_fps_ = declare_parameter<double>("max_fps", 10.0);
    qualities_ = declare_parameter<std::vector<int64_t>>(
      "jpeg_qualities", std::vector<int64_t>{40, 70, 90});

    if (width_ <= 0 || height_ <= 0 || max_fps_ <= 0.0 || qualities_.empty()) {
      throw std::invalid_argument("width, height, max_fps and jpeg_qualities must be positive");
    }

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    for (const auto quality : qualities_) {
      if (quality < 1 || quality > 100) {
        throw std::invalid_argument("jpeg_qualities entries must be in [1, 100]");
      }
      const auto topic = output_prefix_ + "/q" + std::to_string(quality) + "/compressed";
      outputs_.push_back(Output{
        static_cast<int>(quality),
        create_publisher<sensor_msgs::msg::CompressedImage>(topic, output_qos),
        camera_info_topic_.empty() ? nullptr : create_publisher<sensor_msgs::msg::CameraInfo>(
          output_prefix_ + "/q" + std::to_string(quality) + "/camera_info", output_qos)});
      RCLCPP_INFO(get_logger(), "JPEG quality %ld -> %s", quality, topic.c_str());
    }
    if (!output_raw_topic_.empty()) {
      raw_output_ = create_publisher<sensor_msgs::msg::Image>(
        output_raw_topic_, rclcpp::SensorDataQoS().keep_last(1));
    }

    minimum_period_ = std::chrono::duration<double>(1.0 / max_fps_);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PreviewNode::on_image, this, std::placeholders::_1));
    if (!camera_info_topic_.empty()) {
      camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { camera_info_ = std::move(info); });
    }

    RCLCPP_INFO(
      get_logger(), "Previewing left eye from %s at %dx%d, max %.1f fps",
      input_topic_.c_str(), width_, height_, max_fps_);
  }

private:
  struct Output
  {
    int quality;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;
  };

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_publish_.time_since_epoch().count() != 0 && now - last_publish_ < minimum_period_) {
      return;
    }
    last_publish_ = now;

    if (msg->encoding != "rgb8" && msg->encoding != "bgr8" && msg->encoding != "bgra8") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    if (msg->width < (split_stereo_ ? 2U : 1U) || msg->height == 0 ||
      msg->step < msg->width * (msg->encoding == "bgra8" ? 4U : 3U))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid input image geometry");
      return;
    }

    cv::Mat input(
      static_cast<int>(msg->height), static_cast<int>(msg->width),
      msg->encoding == "bgra8" ? CV_8UC4 : CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
    cv::Mat source = input;
    if (split_stereo_) {
      source = input(cv::Rect(
        0, 0, static_cast<int>(msg->width / 2U), static_cast<int>(msg->height)));
    }
    cv::Mat resized;
    cv::resize(source, resized, cv::Size(width_, height_), 0.0, 0.0, cv::INTER_AREA);

    cv::Mat bgr;
    if (msg->encoding == "rgb8") {
      cv::cvtColor(resized, bgr, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "bgra8") {
      cv::cvtColor(resized, bgr, cv::COLOR_BGRA2BGR);
    } else {
      bgr = resized;
    }
    if (raw_output_) {
      sensor_msgs::msg::Image raw;
      raw.header = msg->header;
      raw.header.frame_id = "zed2i_left_camera_frame";
      raw.height = static_cast<uint32_t>(bgr.rows);
      raw.width = static_cast<uint32_t>(bgr.cols);
      raw.encoding = "bgr8";
      raw.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.step);
      raw.data.assign(bgr.datastart, bgr.dataend);
      raw_output_->publish(std::move(raw));
    }

    for (const auto & output : outputs_) {
      auto compressed = sensor_msgs::msg::CompressedImage();
      compressed.header = msg->header;
      compressed.header.frame_id = "zed2i_left_camera_frame";
      compressed.format = "jpeg";
      const std::vector<int> options{cv::IMWRITE_JPEG_QUALITY, output.quality};
      if (cv::imencode(".jpg", bgr, compressed.data, options)) {
        output.publisher->publish(std::move(compressed));
      }
      if (output.camera_info_publisher && camera_info_) {
        auto info = *camera_info_;
        const double scale_x = static_cast<double>(width_) / source.cols;
        const double scale_y = static_cast<double>(height_) / source.rows;
        info.header = msg->header;
        info.header.frame_id = "zed2i_left_camera_frame";
        info.width = width_;
        info.height = height_;
        info.k[0] *= scale_x;
        info.k[2] *= scale_x;
        info.k[4] *= scale_y;
        info.k[5] *= scale_y;
        info.p[0] *= scale_x;
        info.p[2] *= scale_x;
        info.p[5] *= scale_y;
        info.p[6] *= scale_y;
        output.camera_info_publisher->publish(std::move(info));
      }
    }
  }

  std::string input_topic_;
  std::string output_prefix_;
  std::string output_raw_topic_;
  std::string camera_info_topic_;
  bool split_stereo_;
  int width_;
  int height_;
  double max_fps_;
  std::vector<int64_t> qualities_;
  std::vector<Output> outputs_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_output_;
  std::chrono::duration<double> minimum_period_{0.1};
  std::chrono::steady_clock::time_point last_publish_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreviewNode>());
  rclcpp::shutdown();
  return 0;
}
