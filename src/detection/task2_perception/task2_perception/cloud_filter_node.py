#!/usr/bin/env python3
"""Task 2 cloud filter node (node name: task2_cloud_filter).

/livox/lidar (livox_frame) -> [optional emergency manual pre-rotation]
-> TF to base_link -> nonfinite / range / self crop-box / object height band
/ water removal (z band + guarded RANSAC) / optional voxel
-> /task2/points_filtered (frame = base_link, xyz + intensity).

The upside-down LiDAR correction lives ONLY in the URDF (lidar_joint,
roll = pi). This node just looks the transform up via TF. The
lidar_inverted / lidar_*_deg parameters are an emergency manual pre-rotation
applied BEFORE TF and default to a no-op; a startup warning fires when they
are active because the URDF TF is the authoritative inversion correction.

Voxel downsampling defaults OFF and accumulation to 1 frame because the
downstream pcl_preprocessing node (pcl_segmentation submodule) voxelizes and
accumulates itself.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

try:
    from tf2_ros import TransformException
except ImportError:  # older tf2_ros
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

from task2_perception import cloud_ops


def cloud_to_array(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N, 4) float array [x, y, z, intensity].

    Missing intensity fields are filled with zeros so the pipeline shape is
    stable.
    """
    field_names = [f.name for f in msg.fields]
    want = ["x", "y", "z"]
    has_intensity = "intensity" in field_names
    if has_intensity:
        want.append("intensity")
    data = point_cloud2.read_points(msg, field_names=want, skip_nans=False)
    arr = np.asarray(data)
    if arr.dtype.names:  # structured array (Humble and later)
        stacked = np.column_stack(
            [np.asarray(arr[n], dtype=np.float64) for n in want])
    else:  # generator fallback (older sensor_msgs_py)
        stacked = np.asarray([list(p) for p in data], dtype=np.float64)
        if stacked.size == 0:
            stacked = stacked.reshape(0, len(want))
    if not has_intensity:
        stacked = np.column_stack(
            [stacked, np.zeros(stacked.shape[0], dtype=np.float64)])
    return stacked


def array_to_cloud(header: Header, points: np.ndarray) -> PointCloud2:
    """(N, 4) [x, y, z, intensity] -> PointCloud2."""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12,
                   datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(
        header, fields, np.asarray(points, dtype=np.float32))


def transform_msg_to_matrix(tf_msg) -> np.ndarray:
    q = tf_msg.transform.rotation
    t = tf_msg.transform.translation
    rot = cloud_ops.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    return cloud_ops.make_transform(rot, [t.x, t.y, t.z])


class CloudFilterNode(Node):

    def __init__(self):
        super().__init__("task2_cloud_filter")

        self.declare_parameters("", [
            ("input_topic", "/livox/lidar"),
            ("output_topic", "/task2/points_filtered"),
            ("visual_output_topic", ""),
            ("publish_visual_z_mirror", False),
            ("output_frame", "base_link"),
            ("min_range_m", 0.5),
            ("max_range_m", 60.0),
            # A positive value makes this a fail-closed sensor-health gate:
            # no output is published when the raw cloud has too few valid
            # samples for a trustworthy obstacle observation.
            ("min_valid_input_points", 0),
            # Some consumers use an empty cloud as a valid "no obstacle"
            # observation rather than a sensor-fault signal.
            ("publish_empty_on_invalid_input", False),
            ("voxel_leaf_size_m", 0.0),
            ("accumulation_frames", 1),
            # 0 = process every frame.  A positive value processes at most
            # that rate and deliberately drops stale LiDAR frames.
            ("process_rate_hz", 0.0),
            ("lidar_inverted", False),
            ("lidar_roll_deg", 0.0),
            ("lidar_pitch_deg", 0.0),
            ("lidar_yaw_deg", 0.0),
            ("waterline_z_m", 0.0),
            ("water_remove_min_z_m", -0.3),
            ("water_remove_max_z_m", 0.15),
            ("water_plane_distance_threshold_m", 0.05),
            ("water_plane_normal_z_min", 0.9),
            ("water_plane_max_height_error_m", 0.3),
            ("use_water_plane_ransac", True),
            ("self_crop_min_x", -1.2),
            ("self_crop_max_x", 1.2),
            ("self_crop_min_y", -0.8),
            ("self_crop_max_y", 0.8),
            ("self_crop_min_z", -0.5),
            ("self_crop_max_z", 1.5),
            ("publish_self_marker", True),
            ("self_marker_topic", "/task2/self_vessel_marker"),
            ("self_marker_publish_rate_hz", 1.0),
            ("self_marker_yaw_deg", 0.0),
            ("object_min_z_m", -0.2),
            ("object_max_z_m", 4.0),
            ("publish_debug", False),
        ])
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        self.output_frame = str(gp("output_frame"))
        self.publish_visual_z_mirror = bool(gp("publish_visual_z_mirror"))
        self.publish_self_marker_enabled = bool(gp("publish_self_marker"))
        self.self_marker_yaw_rad = np.deg2rad(float(gp("self_marker_yaw_deg")))
        self.min_range = float(gp("min_range_m"))
        self.max_range = float(gp("max_range_m"))
        self.min_valid_input_points = max(0, int(gp("min_valid_input_points")))
        self.publish_empty_on_invalid_input = bool(gp("publish_empty_on_invalid_input"))
        self.voxel_leaf = float(gp("voxel_leaf_size_m"))
        self.process_period_ns = 0
        process_rate_hz = float(gp("process_rate_hz"))
        if process_rate_hz > 0.0:
            self.process_period_ns = int(1e9 / process_rate_hz)
        self.last_processed_stamp_ns = None
        self.lidar_inverted = bool(gp("lidar_inverted"))
        roll_deg = float(gp("lidar_roll_deg"))
        pitch_deg = float(gp("lidar_pitch_deg"))
        yaw_deg = float(gp("lidar_yaw_deg"))
        if self.lidar_inverted:
            roll_deg += 180.0
        self.manual_prerotation_active = (
            abs(roll_deg) > 1e-9 or abs(pitch_deg) > 1e-9 or abs(yaw_deg) > 1e-9)
        self.pre_transform = cloud_ops.make_transform(
            cloud_ops.rpy_to_rotation_matrix(
                np.deg2rad(roll_deg), np.deg2rad(pitch_deg), np.deg2rad(yaw_deg)),
            np.zeros(3))
        if self.manual_prerotation_active:
            self.get_logger().warning(
                "Manual LiDAR pre-rotation is ACTIVE "
                f"(inverted={self.lidar_inverted}, rpy_deg=({roll_deg}, "
                f"{pitch_deg}, {yaw_deg})). URDF TF is the authoritative "
                "inversion correction — do not double-apply.")

        self.waterline_z = float(gp("waterline_z_m"))
        self.water_band_min = float(gp("water_remove_min_z_m"))
        self.water_band_max = float(gp("water_remove_max_z_m"))
        self.water_dist_thresh = float(gp("water_plane_distance_threshold_m"))
        self.water_normal_z_min = float(gp("water_plane_normal_z_min"))
        self.water_max_height_err = float(gp("water_plane_max_height_error_m"))
        self.use_water_ransac = bool(gp("use_water_plane_ransac"))
        self.self_min = np.array([float(gp("self_crop_min_x")),
                                  float(gp("self_crop_min_y")),
                                  float(gp("self_crop_min_z"))])
        self.self_max = np.array([float(gp("self_crop_max_x")),
                                  float(gp("self_crop_max_y")),
                                  float(gp("self_crop_max_z"))])
        self.object_min_z = float(gp("object_min_z_m"))
        self.object_max_z = float(gp("object_max_z_m"))
        self.publish_debug = bool(gp("publish_debug"))

        self.accumulator = deque(maxlen=max(1, int(gp("accumulation_frames"))))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rng = np.random.default_rng(0)

        self.pub = self.create_publisher(PointCloud2, str(gp("output_topic")), 10)
        visual_output_topic = str(gp("visual_output_topic"))
        self.visual_pub = None
        if visual_output_topic and self.publish_visual_z_mirror:
            self.visual_pub = self.create_publisher(
                PointCloud2, visual_output_topic, 10)
        # A boat-shaped outline makes the base_link origin and +X heading
        # visible in Foxglove/RViz. It is visualization only and does not
        # affect the filtered cloud.
        self.self_marker_pub = None
        self.self_marker_timer = None
        if self.publish_self_marker_enabled:
            self.self_marker_pub = self.create_publisher(
                Marker, str(gp("self_marker_topic")), 1)
            marker_period = 1.0 / max(
                float(gp("self_marker_publish_rate_hz")), 1e-6)
            self.self_marker_timer = self.create_timer(
                marker_period, self.publish_self_marker)
        if self.publish_debug:
            self.pub_raw = self.create_publisher(
                PointCloud2, "/task2/debug/raw_transformed", 1)
            self.pub_self = self.create_publisher(
                PointCloud2, "/task2/debug/self_filtered", 1)
            self.pub_water = self.create_publisher(
                PointCloud2, "/task2/debug/water_removed", 1)
            self.pub_rejected = self.create_publisher(
                PointCloud2, "/task2/debug/rejected_water", 1)

        # Depth one prevents CPU-bound filtering from building an old-cloud
        # backlog.  For avoidance, a fresh observation is safer than a
        # complete sequence with growing latency.
        latest_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            PointCloud2, str(gp("input_topic")), self.cloud_callback,
            latest_sensor_qos)

        self.get_logger().info(
            f"task2_cloud_filter: {gp('input_topic')} -> {gp('output_topic')} "
            f"(frame={self.output_frame}, waterline_z={self.waterline_z} m, "
            f"process_rate={'unlimited' if self.process_period_ns == 0 else process_rate_hz} Hz, "
            f"self marker={'on' if self.publish_self_marker_enabled else 'off'})")

    # ------------------------------------------------------------------
    def publish_self_marker(self):
        """Publish a pentagonal boat footprint; its bow points along +X."""
        if self.self_marker_pub is None:
            return
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.output_frame
        marker.ns = "self_vessel"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08  # LINE_STRIP width [m]
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        min_x, min_y, _ = self.self_min
        max_x, max_y, _ = self.self_max
        # Bow: max_x,+/-0. The shoulders sit 65% of the way aft, followed
        # by a full-width transom at min_x. The closed outline is a clear
        # five-sided ship silhouette even when viewed from above.
        shoulder_x = min_x + 0.65 * (max_x - min_x)
        outline = [
            (max_x, 0.0),
            (shoulder_x, max_y),
            (min_x, max_y),
            (min_x, min_y),
            (shoulder_x, min_y),
            (max_x, 0.0),
        ]
        c, s = np.cos(self.self_marker_yaw_rad), np.sin(self.self_marker_yaw_rad)
        marker.points = [
            Point(x=float(c * x - s * y), y=float(s * x + c * y), z=0.0)
            for x, y in outline
        ]
        self.self_marker_pub.publish(marker)

    def _lookup(self, target: str, source: str, stamp) -> np.ndarray | None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(target, source, stamp)
        except TransformException:
            try:  # fall back to the latest available transform
                tf_msg = self.tf_buffer.lookup_transform(target, source, Time())
                self.get_logger().warning(
                    f"TF {source} -> {target} at cloud stamp unavailable; "
                    "using the latest transform.", throttle_duration_sec=5.0)
            except TransformException as e:
                self.get_logger().warning(
                    f"TF {source} -> {target} unavailable, dropping cloud: {e}",
                    throttle_duration_sec=2.0)
                return None
        return transform_msg_to_matrix(tf_msg)

    def cloud_callback(self, msg: PointCloud2):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        process, clock_rolled_back = cloud_ops.frame_timestamp_decision(
            stamp_ns, self.last_processed_stamp_ns, self.process_period_ns)
        if clock_rolled_back:
            # rosbag --loop (and a restarted sensor clock) moves the message
            # timestamp backwards.  Reset state so the new sequence is not
            # rejected forever as an apparently too-fast frame stream.
            self.accumulator.clear()
            self.get_logger().info(
                "LiDAR timestamp moved backwards; resetting cloud filter "
                "rate gate and accumulation history.")
        if not process:
            return
        self.last_processed_stamp_ns = stamp_ns
        transform = self._lookup(
            self.output_frame, msg.header.frame_id, msg.header.stamp)
        if transform is None:
            return

        points = cloud_to_array(msg)
        # NaN/Inf removal first so the transforms never propagate garbage.
        points = cloud_ops.remove_nonfinite(points)
        if points.shape[0] < self.min_valid_input_points:
            self.get_logger().warning(
                "Raw cloud has too few valid points for safety output "
                f"({points.shape[0]} < {self.min_valid_input_points}); withholding output",
                throttle_duration_sec=1.0)
            return
        if points.shape[0] == 0:
            if self.publish_empty_on_invalid_input:
                header = Header()
                header.stamp = msg.header.stamp
                header.frame_id = self.output_frame
                self.pub.publish(array_to_cloud(header, points))
            return

        # Emergency manual pre-rotation (BEFORE TF); no-op by default.
        if self.manual_prerotation_active:
            points = cloud_ops.apply_transform(points, self.pre_transform)
        points = cloud_ops.apply_transform(points, transform)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame

        if self.publish_debug:
            self.pub_raw.publish(array_to_cloud(header, points))

        # Range band (measured from base_link; the small lidar mount offset is
        # negligible against the 60 m working range).
        points = cloud_ops.range_filter(points, self.min_range, self.max_range)
        # Self crop: remove own hull / deck / mast returns.
        points = cloud_ops.crop_box_remove(points, self.self_min, self.self_max)
        if self.publish_debug:
            self.pub_self.publish(array_to_cloud(header, points))
        # Object height band keep.
        points = cloud_ops.crop_box_keep(
            points,
            [-np.inf, -np.inf, self.object_min_z],
            [np.inf, np.inf, self.object_max_z])
        # Water removal: z band around the waterline + guarded RANSAC plane.
        points, rejected_water = cloud_ops.water_removal(
            points,
            waterline_z_m=self.waterline_z,
            band_min_m=self.water_band_min,
            band_max_m=self.water_band_max,
            use_ransac=self.use_water_ransac,
            distance_threshold_m=self.water_dist_thresh,
            normal_z_min=self.water_normal_z_min,
            max_height_error_m=self.water_max_height_err,
            rng=self.rng)
        if self.publish_debug:
            self.pub_water.publish(array_to_cloud(header, points))
            self.pub_rejected.publish(array_to_cloud(header, rejected_water))

        # Optional voxel downsampling (default off: the downstream submodule
        # pcl_preprocessing voxelizes at 0.1 m itself).
        points = cloud_ops.voxel_downsample(points, self.voxel_leaf)

        # Optional short accumulation (default 1 = passthrough; the submodule
        # accumulates 3 frames downstream).
        self.accumulator.append(points)
        if self.accumulator.maxlen > 1:
            points = np.vstack(list(self.accumulator))

        self.pub.publish(array_to_cloud(header, points))
        if self.visual_pub is not None:
            # Visualization only: x/y (and therefore azimuth) are unchanged;
            # only height is mirrored for an intuitive upside-down-LiDAR view.
            visual_points = points.copy()
            visual_points[:, 2] *= -1.0
            visual_header = Header()
            visual_header.stamp = header.stamp
            visual_header.frame_id = self.output_frame
            self.visual_pub.publish(array_to_cloud(visual_header, visual_points))


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
