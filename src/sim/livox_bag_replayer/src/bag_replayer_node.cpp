// Copyright (c) 2026
// Replays LiDAR/IMU data from an mcap rosbag as an intra-process composable
// node, so it can be loaded into an existing component_container_mt in place
// of the real Livox driver for pipeline performance testing.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace livox_bag_replayer
{

int64_t bagMessageTimestamp(const rosbag2_storage::SerializedBagMessage & message)
{
#if LIVOX_BAG_REPLAYER_HAS_RECV_TIMESTAMP
  return message.recv_timestamp;
#else
  return message.time_stamp;
#endif
}

class BagReplayerNode : public rclcpp::Node
{
public:
  explicit BagReplayerNode(const rclcpp::NodeOptions & options)
  : Node("bag_replayer_node", options)
  {
    bag_path_ = declare_parameter<std::string>(
      "bag_path", "/home/ibo/njord2026_ws/rosbag2_2026_07_21-16_22_30");
    storage_id_ = declare_parameter<std::string>("storage_id", "mcap");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    loop_ = declare_parameter<bool>("loop", true);
    rate_ = declare_parameter<double>("rate", 1.0);
    restamp_ = declare_parameter<bool>("restamp", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");

    RCLCPP_INFO(
      get_logger(),
      "livox_bag_replayer starting: bag_path='%s' lidar_topic='%s' imu_topic='%s' "
      "loop=%s rate=%.3f restamp=%s",
      bag_path_.c_str(), lidar_topic_.c_str(), imu_topic_.c_str(),
      loop_ ? "true" : "false", rate_, restamp_ ? "true" : "false");

    auto qos = rclcpp::SensorDataQoS();
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_, qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, qos);

    stop_ = false;
    replay_thread_ = std::thread(&BagReplayerNode::replayLoop, this);
  }

  ~BagReplayerNode() override
  {
    stop_ = true;
    if (replay_thread_.joinable()) {
      replay_thread_.join();
    }
  }

private:
  void replayLoop()
  {
    do {
      if (!replayOnce()) {
        // Opening or reading the bag failed outright; do not spin forever.
        return;
      }
    } while (loop_ && !stop_);
  }

  // Reads and publishes the whole bag once (subject to loop-external stop_).
  // Returns false on a hard failure (bag could not be opened at all).
  bool replayOnce()
  {
    rosbag2_cpp::Reader reader;
    try {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_path_;
      storage_options.storage_id = storage_id_;
      reader.open(storage_options);

      rosbag2_storage::StorageFilter filter;
      filter.topics = {lidar_topic_, imu_topic_};
      reader.set_filter(filter);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open bag '%s': %s", bag_path_.c_str(), e.what());
      return false;
    }

    if (!reader.has_next()) {
      RCLCPP_ERROR(
        get_logger(), "Bag '%s' contains no messages on topics '%s' / '%s'",
        bag_path_.c_str(), lidar_topic_.c_str(), imu_topic_.c_str());
      return false;
    }

    bool have_offset = false;
    std::chrono::system_clock::time_point wall_start;
    int64_t first_bag_ns = 0;

    while (!stop_ && reader.has_next()) {
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_msg;
      try {
        bag_msg = reader.read_next();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Error reading next bag message: %s", e.what());
        break;
      }

      if (!have_offset) {
        wall_start = std::chrono::system_clock::now();
        first_bag_ns = bagMessageTimestamp(*bag_msg);
        have_offset = true;
      }

      const double rate = rate_ > 0.0 ? rate_ : 1.0;
      const double elapsed_s =
        static_cast<double>(bagMessageTimestamp(*bag_msg) - first_bag_ns) / 1e9 / rate;
      const auto target = wall_start +
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::duration<double>(elapsed_s));

      std::this_thread::sleep_until(target);
      if (stop_) {
        break;
      }

      if (bag_msg->topic_name == lidar_topic_) {
        publishLidar(bag_msg);
      } else if (bag_msg->topic_name == imu_topic_) {
        publishImu(bag_msg);
      }
    }

    reader.close();
    return true;
  }

  void publishLidar(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_msg)
  {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
    pc2_serialization_.deserialize_message(&extracted, msg.get());

    if (restamp_) {
      msg->header.stamp = now();
      if (!frame_id_.empty()) {
        msg->header.frame_id = frame_id_;
      }
    }

    lidar_pub_->publish(std::move(msg));
  }

  void publishImu(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_msg)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
    imu_serialization_.deserialize_message(&extracted, msg.get());

    if (restamp_) {
      msg->header.stamp = now();
      if (!frame_id_.empty()) {
        msg->header.frame_id = frame_id_;
      }
    }

    imu_pub_->publish(std::move(msg));
  }

  // Parameters
  std::string bag_path_;
  std::string storage_id_;
  std::string lidar_topic_;
  std::string imu_topic_;
  bool loop_{true};
  double rate_{1.0};
  bool restamp_{true};
  std::string frame_id_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Serialization helpers
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serialization_;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization_;

  // Replay thread
  std::thread replay_thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace livox_bag_replayer

RCLCPP_COMPONENTS_REGISTER_NODE(livox_bag_replayer::BagReplayerNode)
