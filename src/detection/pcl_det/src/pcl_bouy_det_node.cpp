#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "njord_interfaces/msg/buoy_roi.hpp"
#include "pcl/common/centroid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kEpsilon = 1e-6;

double AngleDiff(double a, double b)
{
  return std::fabs(std::atan2(std::sin(a - b), std::cos(a - b)));
}
}

namespace pcl_det
{

class PclBuoyDetectionNode : public rclcpp::Node
{
public:
  explicit PclBuoyDetectionNode(const rclcpp::NodeOptions & options)
  : Node("pcl_bouy_det_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    declare_parameter<std::string>("input_topic", "/livox/lidar");
    declare_parameter<std::string>("roi_topic", "/buoy_roi");
    declare_parameter<std::string>("output_topic", "/buoy_detections");
    declare_parameter<std::string>("frame_id", "base_link");
    declare_parameter<double>("height_min", -1.0);
    declare_parameter<double>("height_max", 0.5);
    declare_parameter<double>("distance_threshold", 0.05);
    declare_parameter<int>("min_points_per_cluster", 5);
    declare_parameter<double>("roi_timeout_sec", 0.5);

    input_topic_ = get_parameter("input_topic").as_string();
    roi_topic_ = get_parameter("roi_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    base_frame_ = get_parameter("frame_id").as_string();
    height_min_ = get_parameter("height_min").as_double();
    height_max_ = get_parameter("height_max").as_double();
    distance_threshold_ = get_parameter("distance_threshold").as_double();
    min_points_per_cluster_ = get_parameter("min_points_per_cluster").as_int();
    roi_timeout_sec_ = get_parameter("roi_timeout_sec").as_double();

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PclBuoyDetectionNode::PointCloudCallback, this, std::placeholders::_1));

    sub_roi_ = create_subscription<njord_interfaces::msg::BuoyRoi>(
      roi_topic_, 10,
      std::bind(&PclBuoyDetectionNode::RoiCallback, this, std::placeholders::_1));

    pub_detection_ = create_publisher<geometry_msgs::msg::PointStamped>(output_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "PCL Buoy Detection Node initialized. input=%s roi=%s output=%s base=%s",
      input_topic_.c_str(), roi_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str());
  }

private:
  void RoiCallback(const njord_interfaces::msg::BuoyRoi::SharedPtr msg)
  {
    latest_roi_ = *msg;
    have_roi_ = true;
  }

  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_roi_) {
      return;
    }

    if ((msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ||
      (latest_roi_.header.stamp.sec == 0 && latest_roi_.header.stamp.nanosec == 0))
    {
      return;
    }

    const auto roi_age = rclcpp::Time(msg->header.stamp) - rclcpp::Time(latest_roi_.header.stamp);
    if (roi_age.seconds() > roi_timeout_sec_) {
      return;
    }

    geometry_msgs::msg::TransformStamped base_to_cloud;
    geometry_msgs::msg::TransformStamped cloud_to_base;
    try {
      base_to_cloud = tf_buffer_.lookupTransform(
        msg->header.frame_id, base_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      cloud_to_base = tf_buffer_.lookupTransform(
        base_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    const auto origin_x = base_to_cloud.transform.translation.x;
    const auto origin_y = base_to_cloud.transform.translation.y;
    const auto origin_z = base_to_cloud.transform.translation.z;

    geometry_msgs::msg::PointStamped roi_point_base;
    roi_point_base.header.frame_id = base_frame_;
    roi_point_base.header.stamp = msg->header.stamp;
    roi_point_base.point.x = std::cos(latest_roi_.theta_predict);
    roi_point_base.point.y = std::sin(latest_roi_.theta_predict);
    roi_point_base.point.z = 0.0;

    geometry_msgs::msg::PointStamped roi_point_cloud;
    tf2::doTransform(roi_point_base, roi_point_cloud, base_to_cloud);

    const double dir_x = roi_point_cloud.point.x - origin_x;
    const double dir_y = roi_point_cloud.point.y - origin_y;
    const double theta_predict_cloud = std::atan2(dir_y, dir_x);

    const double r_min = std::max(0.0,
        static_cast<double>(latest_roi_.r_predict - latest_roi_.r_range));
    const double r_max = static_cast<double>(latest_roi_.r_predict + latest_roi_.r_range);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    filtered->reserve(cloud->size());
    for (const auto & pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const double dx = pt.x - origin_x;
      const double dy = pt.y - origin_y;
      const double dz = pt.z - origin_z;
      if (dz < height_min_ || dz > height_max_) {
        continue;
      }
      const double r = std::hypot(dx, dy);
      if (r < r_min || r > r_max) {
        continue;
      }
      const double theta = std::atan2(dy, dx);
      if (AngleDiff(theta, theta_predict_cloud) > latest_roi_.theta_range) {
        continue;
      }
      filtered->push_back(pt);
    }

    if (filtered->size() < static_cast<size_t>(min_points_per_cluster_)) {
      return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(distance_threshold_);
    ec.setMinClusterSize(min_points_per_cluster_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);

    int cluster_id = 0;
    for (const auto & indices : cluster_indices) {
      if (indices.indices.empty()) {
        continue;
      }
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*filtered, indices, centroid);

      geometry_msgs::msg::PointStamped centroid_cloud;
      centroid_cloud.header.frame_id = msg->header.frame_id;
      centroid_cloud.header.stamp = msg->header.stamp;
      centroid_cloud.point.x = centroid[0];
      centroid_cloud.point.y = centroid[1];
      centroid_cloud.point.z = centroid[2];

      geometry_msgs::msg::PointStamped centroid_base;
      tf2::doTransform(centroid_cloud, centroid_base, cloud_to_base);

      geometry_msgs::msg::PointStamped detection;
      detection.header.stamp = msg->header.stamp;
      detection.header.frame_id = base_frame_;
      detection.point.x = centroid_base.point.x;
      detection.point.y = centroid_base.point.y;
      detection.point.z = 0.0;
      pub_detection_->publish(detection);

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = base_frame_;
      transform.child_frame_id = "buoy_" + std::to_string(cluster_id);
      transform.transform.translation.x = centroid_base.point.x;
      transform.transform.translation.y = centroid_base.point.y;
      transform.transform.translation.z = centroid_base.point.z;
      transform.transform.rotation.w = 1.0;
      tf_broadcaster_.sendTransform(transform);

      cluster_id++;
    }
  }

  std::string input_topic_;
  std::string roi_topic_;
  std::string output_topic_;
  std::string base_frame_;
  double height_min_{};
  double height_max_{};
  double distance_threshold_{};
  int min_points_per_cluster_{};
  double roi_timeout_sec_{};

  bool have_roi_{false};
  njord_interfaces::msg::BuoyRoi latest_roi_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<njord_interfaces::msg::BuoyRoi>::SharedPtr sub_roi_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_detection_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace pcl_det

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_det::PclBuoyDetectionNode)
