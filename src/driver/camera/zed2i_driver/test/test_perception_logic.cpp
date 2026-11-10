#include "zed2i_driver/perception_logic.hpp"
#include "zed2i_driver/detection_message_utils.hpp"
#include "zed2i_driver/lidar_fusion.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace zed2i_driver
{
namespace
{

TEST(PerceptionLogic, DecodesBothTensorLayoutsAndCapsResults)
{
  LetterboxTransform transform{1.0F, 0.0F, 0.0F, 640, 640};
  std::vector<float> rows;
  for (int i = 0; i < 33; ++i) {
    rows.insert(rows.end(), {10.0F, 20.0F, 30.0F, 50.0F, 0.5F + i * 0.01F, static_cast<float>(i % 6)});
  }
  auto decoded = decode_detections(rows, 33, false, 0.25F, transform);
  EXPECT_EQ(decoded.size(), 32U);
  EXPECT_EQ(decoded.front().class_id, 2);

  std::vector<float> channels(12);
  const std::vector<float> two_rows{10, 20, 30, 40, .9F, 1, 50, 60, 80, 100, .8F, 5};
  for (int i = 0; i < 2; ++i) for (int field = 0; field < 6; ++field) channels[field * 2 + i] = two_rows[i * 6 + field];
  decoded = decode_detections(channels, 2, true, 0.25F, transform);
  ASSERT_EQ(decoded.size(), 2U);
  EXPECT_EQ(decoded[0].class_id, 1);
  EXPECT_EQ(decoded[1].class_id, 5);
}

TEST(PerceptionLogic, ReversesLetterboxAndDropsDegenerateBoxes)
{
  LetterboxTransform transform{0.5F, 0.0F, 140.0F, 1280, 720};
  const std::vector<float> output{10, 150, 100, 300, .9F, 0, 1, 1, 1.5F, 2, .8F, 1};
  const auto decoded = decode_detections(output, 2, false, .25F, transform);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_FLOAT_EQ(decoded[0].x1, 20.0F);
  EXPECT_FLOAT_EQ(decoded[0].y1, 20.0F);
}

TEST(PerceptionLogic, ComputesDepthMedianAndMinimumSamplePolicy)
{
  std::vector<float> depth(64, 3.0F);
  depth[0] = NAN;
  depth[1] = INFINITY;
  depth[2] = 100.0F;
  const DepthRoi roi{0, 0, 8, 8};
  EXPECT_FALSE(depth_median_cpu(depth, 8, 8, roi, 1, 63, .3F, 20.0F).has_value());
  const auto median = depth_median_cpu(depth, 8, 8, roi, 1, 16, .3F, 20.0F);
  ASSERT_TRUE(median.has_value());
  EXPECT_FLOAT_EQ(*median, 3.0F);
}

TEST(PerceptionLogic, LabelsTheFinalRangeSourceForGroundVideo)
{
  PositionedDetection detection;
  detection.position_base = {3.0F, 4.0F, 0.0F};
  detection.source = PositionSource::kZedDepth;
  EXPECT_EQ(detection_range_label(detection), "Z 5.0m");

  detection.source = PositionSource::kLidarFused;
  EXPECT_EQ(detection_range_label(detection), "L 5.0m");

  detection.source = PositionSource::kNone;
  EXPECT_EQ(detection_range_label(detection), "?");
}

TEST(PerceptionLogic, CentralDepthRoiClipsImageBounds)
{
  const Detection2D detection{0, .9F, -10.0F, 0.0F, 30.0F, 20.0F};
  const auto roi = central_depth_roi(detection, .5F, 32, 24);
  EXPECT_EQ(roi.x0, 0);
  EXPECT_EQ(roi.x1, 20);
  EXPECT_EQ(roi.y0, 5);
  EXPECT_EQ(roi.y1, 15);
}

TEST(PerceptionLogic, GeneratesSpecifiedWallGeometries)
{
  const auto north = virtual_wall_points(2, 0.0F, 0.0F, NAN, 2.0F, 40);
  EXPECT_EQ(north.size(), 31U);
  for (const auto & point : north) EXPECT_FALSE(point.x > 0.0F && point.y > 0.0F && std::fabs(point.x) < point.y);
  EXPECT_EQ(virtual_wall_points(0, 0.0F, 0.0F, NAN).size(), 0U);
  EXPECT_EQ(virtual_wall_points(1, 0.0F, 0.0F, 0.0F).size(), 21U);
}

TEST(PerceptionLogic, LateralWallsLeaveTheChannelInteriorOpen)
{
  // Heading east: Region-A red is the north (left) boundary and green is the
  // south (right) boundary.  Each half-circle must occupy only its outer side.
  const auto red_wall = virtual_wall_points(1, 0.0F, 4.0F, 0.0F, 2.0F, 40);
  const auto green_wall = virtual_wall_points(0, 0.0F, -4.0F, 0.0F, 2.0F, 40);
  ASSERT_EQ(red_wall.size(), 21U);
  ASSERT_EQ(green_wall.size(), 21U);
  for (const auto & point : red_wall) EXPECT_GE(point.y, 4.0F - 1.0e-5F);
  for (const auto & point : green_wall) EXPECT_LE(point.y, -4.0F + 1.0e-5F);
}

TEST(PerceptionLogic, ConnectsSameColourBuoysAlongTheChannelOnly)
{
  const std::vector<WallPoint> buoys{{4.0F, 1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {8.0F, 1.0F, 0.0F}};
  const auto wall = same_color_wall_points(buoys, 0.0F, 10.0F, 1.0F);
  ASSERT_EQ(wall.size(), 10U);
  EXPECT_FLOAT_EQ(wall.front().x, 0.0F);
  EXPECT_FLOAT_EQ(wall.back().x, 8.0F);
  for (const auto & point : wall) EXPECT_FLOAT_EQ(point.y, 1.0F);

  const std::vector<WallPoint> separated{{0.0F, 1.0F, 0.0F}, {15.0F, 1.0F, 0.0F}};
  EXPECT_TRUE(same_color_wall_points(separated, 0.0F, 10.0F, 1.0F).empty());
  EXPECT_TRUE(same_color_wall_points(buoys, NAN, 10.0F, 1.0F).empty());
}

TEST(PerceptionLogic, AddsSameColourConnectionsToTheVirtualObstacleCloud)
{
  const auto positioned = [](int class_id, float x, float y) {
      PositionedDetection detection;
      detection.detection.class_id = class_id;
      detection.detection.confidence = 0.9F;
      detection.position_base = {x, y, 0.0F};
      detection.source = PositionSource::kZedDepth;
      return detection;
    };
  std_msgs::msg::Header header;
  header.frame_id = "map";
  const std::vector<PositionedDetection> detections{
    positioned(0, 0.0F, -4.0F), positioned(0, 4.0F, -4.0F),
    positioned(1, 0.0F, 4.0F), positioned(1, 4.0F, 4.0F)};

  const auto disconnected = to_virtual_wall_cloud(
    detections, header, "map", 0.0F, 2.0F, 40, false, 10.0F, 1.0F);
  const auto connected = to_virtual_wall_cloud(
    detections, header, "map", 0.0F, 2.0F, 40, true, 10.0F, 1.0F);
  EXPECT_EQ(disconnected.width, 84U);  // Four half-circles of 21 points.
  EXPECT_EQ(connected.width, 94U);     // Plus two same-colour 4 m segments.
}

TEST(PerceptionLogic, SelectsLidarByRangeThenFallsBackToRay)
{
  const std::vector<LidarCluster> clusters{
    {{0.0F, 0.0F, 4.0F}, 5, 4.0F, 0.2F},
    {{0.0F, 0.0F, 8.0F}, 5, 8.0F, 0.05F}};
  auto selected = select_lidar_cluster(clusters, true, {0.0F, 0.0F, 4.1F}, 2.0F, .1F);
  ASSERT_TRUE(selected.has_value());
  EXPECT_FLOAT_EQ(selected->range, 4.0F);

  selected = select_lidar_cluster(clusters, true, {0.0F, 0.0F, 20.0F}, 2.0F, .1F);
  EXPECT_FALSE(selected.has_value());
  selected = select_lidar_cluster(clusters, false, {NAN, NAN, NAN}, 2.0F, .1F);
  ASSERT_TRUE(selected.has_value());
  EXPECT_FLOAT_EQ(selected->range, 8.0F);
}

TEST(PerceptionLogic, CentralRoiDoesNotAssignProjectedPointTwiceByItself)
{
  const Detection2D left{0, .8F, 0.0F, 0.0F, 100.0F, 100.0F};
  const Detection2D right{1, .7F, 50.0F, 0.0F, 150.0F, 100.0F};
  EXPECT_TRUE(point_in_central_bbox_roi(25.0F, 50.0F, left, .5F));
  EXPECT_TRUE(point_in_central_bbox_roi(125.0F, 50.0F, right, .5F));
}

TEST(PerceptionLogic, ConvertsInvalidPositionToNaNAndPublishesEmptyWalls)
{
  PositionedDetection detection;
  detection.detection.class_id = 0;
  detection.detection.confidence = 1.2F;
  detection.source = PositionSource::kNone;
  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  const auto message = to_detection_array({detection}, header);
  ASSERT_EQ(message.detections.size(), 1U);
  EXPECT_TRUE(std::isnan(message.detections[0].position.x));
  EXPECT_EQ(message.detections[0].position_source, 0U);
  const auto cloud = to_virtual_wall_cloud({detection}, header, "map", 0.0F, 2.0F, 40);
  EXPECT_EQ(cloud.width, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(PerceptionLogic, AssignsOverlappingLidarPointsOnlyOnce)
{
  const std::vector<Detection2D> detections{
    {0, .9F, 10, 10, 50, 50}, {1, .8F, 30, 10, 70, 50}};
  const std::vector<Point3f> points{{0, 0, 10}, {1, 0, 10}};
  const auto assigned = assign_projected_points(points, detections, 100, 100, 10, 10, 50, 50, 1.0F);
  ASSERT_EQ(assigned.size(), 2U);
  EXPECT_EQ(assigned[0].size() + assigned[1].size(), 2U);
}

TEST(PerceptionLogic, FusesMatchingClusterAndRejectsStaleRange)
{
  const Detection2D detection{0, .9F, 0, 0, 20, 20};
  const std::vector<LidarCluster> clusters{{{0, 0, 5}, 10, 5.1F, .1F}, {{0, 0, 10}, 10, 10, .1F}};
  EXPECT_EQ(select_lidar_cluster(clusters, {0, 0, 5}, true, detection, {.2F, .5F}), 0);
  EXPECT_EQ(select_lidar_cluster(clusters, {0, 0, 5}, true, detection, {.01F, .5F}), -1);
  EXPECT_EQ(select_lidar_cluster({{{0, 0, 5}, 10, 5, .2F}}, {0, 0, 0}, false, detection, {.2F, .3F}), 0);
  EXPECT_EQ(select_lidar_cluster({{{0, 0, 5}, 10, 5, .4F}}, {0, 0, 0}, false, detection, {.2F, .3F}), -1);
}

}  // namespace
}  // namespace zed2i_driver
