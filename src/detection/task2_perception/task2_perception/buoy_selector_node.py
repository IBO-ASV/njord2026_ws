#!/usr/bin/env python3
"""Select stationary Task 2 buoys from the shared LiDAR track stream.

The route GPS5 -> GPS6 defines the colours geometrically: a stationary track
near +2.5 m (left) is red, and one near -2.5 m (right) is green.  This avoids
the old buoy-specific detector while retaining the tracker confirmation and
TF/ego-motion handling used for other vessels.
"""

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from njord_interfaces.msg import BuoyDetection, BuoyDetectionArray

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

from ship_perception_msgs.msg import TrackedObjectArray
from task2_perception import buoy_glue, cloud_ops, tracking_glue
from task2_perception.tracking_glue import SelectionParams, Track


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BuoySelectorNode(Node):
    def __init__(self):
        super().__init__("task2_buoy_selector")
        self.declare_parameters("", [
            ("tracked_objects_topic", "/tracked_objects"),
            ("tracked_objects_in_map_frame", False),
            ("ego_odom_topic", "/task2/ego_odom"),
            ("output_topic", "/task2/buoy_detections"),
            ("legacy_point_topic", "/buoy_detections"),
            # Disabled for normal operation.  The rosbag launch enables this
            # so an operator can see precisely which tracked clusters passed
            # the buoy-selection gates.
            ("publish_detection_markers", False),
            ("detection_markers_topic", "/task2/buoy_detection_markers"),
            ("map_frame", "map"), ("base_frame", "base_link"),
            ("waypoint_start_topic", "/waypoint1_pose"),
            ("waypoint_end_topic", "/waypoint2_pose"),
            ("expected_lateral_offset_m", 2.5),
            ("lateral_tolerance_m", 1.0),
            ("start_margin_m", 2.0), ("end_margin_m", 2.0),
            ("confirmed_only", True), ("min_hit_count", 5),
            ("max_distance_m", 40.0), ("min_point_count", 5),
            ("min_length_m", 0.0), ("max_length_m", 1.2),
            ("min_width_m", 0.0), ("max_width_m", 1.2),
            ("min_height_m", 0.15), ("max_height_m", 1.5),
            ("stale_timeout_sec", 2.0),
            ("stationary_speed_max_mps", 0.35),
            ("stationary_confirmation_window_sec", 3.0),
            ("stationary_confirmation_min_observations", 5),
            ("publish_rate_hz", 5.0),
        ])
        p = lambda name: self.get_parameter(name).value
        self.map_frame, self.base_frame = str(p("map_frame")), str(p("base_frame"))
        self.tracked_objects_in_map_frame = bool(p("tracked_objects_in_map_frame"))
        self.expected_offset_m, self.lateral_tolerance_m = float(p("expected_lateral_offset_m")), float(p("lateral_tolerance_m"))
        self.start_margin_m, self.end_margin_m = float(p("start_margin_m")), float(p("end_margin_m"))
        self.stationary_speed_max_mps = float(p("stationary_speed_max_mps"))
        self.stationary_confirmation_window_sec = float(
            p("stationary_confirmation_window_sec"))
        self.stationary_confirmation_min_observations = int(
            p("stationary_confirmation_min_observations"))
        if (self.expected_offset_m <= 0.0 or self.lateral_tolerance_m < 0.0
                or self.stationary_speed_max_mps < 0.0
                or self.stationary_confirmation_window_sec <= 0.0
                or self.stationary_confirmation_min_observations < 1):
            raise ValueError("Buoy side-line offset/tolerance/speed parameters must be non-negative (offset > 0)")
        self.selection_params = SelectionParams(
            confirmed_only=bool(p("confirmed_only")), max_distance_m=(
                float("inf") if self.tracked_objects_in_map_frame
                else float(p("max_distance_m"))),
            min_length_m=float(p("min_length_m")), max_length_m=float(p("max_length_m")),
            min_width_m=float(p("min_width_m")), max_width_m=float(p("max_width_m")),
            min_height_m=float(p("min_height_m")), max_height_m=float(p("max_height_m")),
            min_point_count=int(p("min_point_count")), min_hit_count=int(p("min_hit_count")),
            stale_timeout_sec=float(p("stale_timeout_sec")))
        self.tracks, self.start_map, self.end_map = [], None, None
        self.stationary_observations = {}
        self.ego_vel_base, self.ego_yaw_rate = np.zeros(3), 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.array_pub = self.create_publisher(BuoyDetectionArray, str(p("output_topic")), 10)
        self.point_pub = self.create_publisher(PointStamped, str(p("legacy_point_topic")), 10)
        self.marker_pub = None
        self.marker_ids = set()
        if bool(p("publish_detection_markers")):
            self.marker_pub = self.create_publisher(
                MarkerArray, str(p("detection_markers_topic")), 10)
        self.create_subscription(TrackedObjectArray, str(p("tracked_objects_topic")), self.tracks_callback, 10)
        self.create_subscription(Odometry, str(p("ego_odom_topic")), self.ego_callback, 10)
        self.create_subscription(PoseStamped, str(p("waypoint_start_topic")), lambda msg: self.endpoint_callback(msg, True), 10)
        self.create_subscription(PoseStamped, str(p("waypoint_end_topic")), lambda msg: self.endpoint_callback(msg, False), 10)
        self.timer = self.create_timer(1.0 / max(float(p("publish_rate_hz")), 1e-6), self.timer_callback)

    def tracks_callback(self, msg):
        self.tracks = [Track(
            object_id=int(o.object_id), position=np.array([o.state.pose.pose.position.x, o.state.pose.pose.position.y, o.state.pose.pose.position.z]),
            yaw=_yaw_from_quaternion(o.state.pose.pose.orientation), velocity_body=np.array([o.state.twist.twist.linear.x, o.state.twist.twist.linear.y, o.state.twist.twist.linear.z]),
            yaw_rate=float(o.state.twist.twist.angular.z), dimensions=np.array([o.dimensions.x, o.dimensions.y, o.dimensions.z]),
            point_count=int(o.point_count), track_state=int(o.track_state), stamp_sec=o.header.stamp.sec + o.header.stamp.nanosec * 1e-9,
            hit_count=int(o.hit_count), miss_count=int(o.miss_count)) for o in msg.objects]

    def ego_callback(self, msg):
        v = msg.twist.twist.linear
        self.ego_vel_base = np.array([v.x, v.y, v.z])
        self.ego_yaw_rate = float(msg.twist.twist.angular.z)

    def endpoint_callback(self, msg, is_start):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warning("Ignoring buoy-route waypoint outside map frame", throttle_duration_sec=2.0)
            return
        if is_start:
            self.start_map = np.array([msg.pose.position.x, msg.pose.position.y])
        else:
            self.end_map = np.array([msg.pose.position.x, msg.pose.position.y])

    def _marker_pair(self, detection, track):
        """Build a bounding-box and label for one buoy-selected cluster."""
        colour = {
            BuoyDetection.CLASS_GREEN: (0.1, 0.9, 0.1),
            BuoyDetection.CLASS_RED: (0.95, 0.1, 0.1),
        }.get(detection.class_id, (1.0, 0.8, 0.0))
        label = {
            BuoyDetection.CLASS_GREEN: "BUOY green",
            BuoyDetection.CLASS_RED: "BUOY red",
        }.get(detection.class_id, "BUOY unknown")
        marker_id = int(track.object_id) * 2
        box = Marker()
        box.header.frame_id = self.map_frame
        box.header.stamp = self.get_clock().now().to_msg()
        box.ns, box.id = "task2_buoy_clusters", marker_id
        box.type, box.action = Marker.CUBE, Marker.ADD
        box.pose.position = detection.position
        box.pose.orientation.w = 1.0
        box.scale.x = max(0.1, float(track.dimensions[0]))
        box.scale.y = max(0.1, float(track.dimensions[1]))
        box.scale.z = max(0.1, float(track.dimensions[2]))
        box.color.r, box.color.g, box.color.b = colour
        box.color.a = 0.8

        text = Marker()
        text.header = box.header
        text.ns, text.id = box.ns, marker_id + 1
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position = detection.position
        text.pose.position.z += box.scale.z * 0.5 + 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = 0.35
        text.color.r, text.color.g, text.color.b = colour
        text.color.a = 1.0
        text.text = f"{label}  id={track.object_id}"
        return box, text

    def timer_callback(self):
        now = self.get_clock().now()
        detections = BuoyDetectionArray()
        detections.header.stamp = now.to_msg()
        detections.header.frame_id = self.map_frame
        markers = MarkerArray()
        active_marker_ids = set()
        now_sec = now.nanoseconds * 1e-9
        eligible_tracks = tracking_glue.select_opponent(
            self.tracks, now_sec, params=self.selection_params)
        eligible_ids = {track.object_id for track in eligible_tracks}
        for track in eligible_tracks:
            if self.tracked_objects_in_map_frame:
                position_map, velocity_map = track.position, track.velocity_base
            else:
                try:
                    tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time(seconds=track.stamp_sec))
                except TransformException:
                    continue
                q, t = tf.transform.rotation, tf.transform.translation
                transform = cloud_ops.make_transform(cloud_ops.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
                absolute_base = tracking_glue.ego_compensate(track.velocity_base, self.ego_vel_base, self.ego_yaw_rate, track.position)
                position_map, velocity_map = tracking_glue.to_map_frame(track.position, absolute_base, transform)
            have_route = self.start_map is not None and self.end_map is not None
            colour = None
            if have_route:
                colour = buoy_glue.classify_route_side(
                    position_map, self.start_map, self.end_map,
                    self.expected_offset_m, self.lateral_tolerance_m,
                    self.start_margin_m, self.end_margin_m)
            is_stationary = buoy_glue.is_stationary(
                velocity_map, self.stationary_speed_max_mps)
            # Without map-frame GPS5/GPS6, neither the allowed area nor the
            # route-side colour is known. Do not publish a stale/unknown buoy.
            if not have_route or colour is None:
                continue
            observations = self.stationary_observations.setdefault(
                track.object_id, deque())
            while (observations and now_sec - observations[0]
                   > self.stationary_confirmation_window_sec):
                observations.popleft()
            if is_stationary:
                observations.append(now_sec)
            if (not is_stationary or len(observations)
                    < self.stationary_confirmation_min_observations):
                continue
            detection = BuoyDetection()
            if colour == buoy_glue.GREEN:
                detection.class_id = BuoyDetection.CLASS_GREEN
            elif colour == buoy_glue.RED:
                detection.class_id = BuoyDetection.CLASS_RED
            detection.confidence = min(1.0, float(track.hit_count) / max(1, int(self.selection_params.min_hit_count)))
            detection.position.x, detection.position.y, detection.position.z = map(float, position_map)
            detection.position_source = BuoyDetection.POSITION_LIDAR_FUSED
            detections.detections.append(detection)
            point = PointStamped()
            point.header = detections.header
            point.point = detection.position
            self.point_pub.publish(point)
            if self.marker_pub is not None:
                box, text = self._marker_pair(detection, track)
                markers.markers.extend((box, text))
                active_marker_ids.update((box.id, text.id))
        if self.marker_pub is not None:
            for marker_id in self.marker_ids - active_marker_ids:
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.ns, marker.id, marker.action = "task2_buoy_clusters", marker_id, Marker.DELETE
                markers.markers.append(marker)
            self.marker_ids = active_marker_ids
            self.marker_pub.publish(markers)
        self.stationary_observations = {
            track_id: observations
            for track_id, observations in self.stationary_observations.items()
            if track_id in eligible_ids
        }
        self.array_pub.publish(detections)


def main(args=None):
    rclpy.init(args=args)
    node = BuoySelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
