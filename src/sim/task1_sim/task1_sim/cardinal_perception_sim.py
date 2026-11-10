"""Sim stand-in for the ZED2i + Mid-360 intra-process cardinal-marker pipeline.

On the real vessel, `zed2i_driver`'s `sdk_node_zed` runs YOLO on the ZED2i
stereo pair and fuses the result with Livox Mid-360 returns to publish
`njord_interfaces/BuoyDetectionArray` on `/buoy_detections_3d`
(`src/driver/camera/zed2i_driver/src/sdk_node_zed.cpp`,
`src/driver/camera/zed2i_driver/config/zed2i_jetson_orin_nano.yaml`). The sim
has no camera/LiDAR, so nothing publishes that topic and
`cardinal_wall_publisher` never sees a cardinal marker to turn into a wall.

This node closes that gap without inventing new marker geometry: it reuses
`buoy_position_xy` from `task1_params.yaml` (the same source
`task1_orchestrator` uses) and the per-marker cardinal marks broadcast as a
JSON array on `/sim/cardinal_mark` (aligned index-wise with
`buoy_position_xy`, e.g. `["S", "N", "S"]`), and only emits a detection once
the simulated boat is within a configurable recognition radius. This keeps
the marker unknown at course start while avoiding camera-pose artifacts in a
simulation that has no rendered camera image.

Detection range/FOV defaults are taken directly from the real driver
configs:
  - camera_min/max_range_m: ZED2i `depth_min_m`/`depth_max_m`
    (zed2i_jetson_orin_nano.yaml -> 0.3 / 20.0)
  - camera_fov_deg: matches `camera_fov_deg` used by the real YOLO node
    (src/detection/yolo/yolo/main.py, default 90.0 for ZED2i HD720)
  - confidence_threshold: ZED2i `confidence_threshold` (0.25)
  - lidar_max_range_m / lidar_fov_deg: Livox Mid-360 non-repetitive scan,
    ~40 m usable range, 360 deg horizontal FOV
    (src/driver/lidar/livox_ros_driver2/config/MID360_config.json has no
    range field -- 40 m is the Mid-360 datasheet range at typical
    reflectivity, used here only as a non-binding upper bound since the
    camera FOV is the actual classification bottleneck).
"""

import json
import math

from njord_interfaces.msg import BuoyDetection, BuoyDetectionArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


CARDINAL_CLASS_BY_MARK = {
    "GREEN": BuoyDetection.CLASS_GREEN,
    "RED": BuoyDetection.CLASS_RED,
    "N": BuoyDetection.CLASS_NORTH,
    "E": BuoyDetection.CLASS_EAST,
    "S": BuoyDetection.CLASS_SOUTH,
    "W": BuoyDetection.CLASS_WEST,
}


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_xy_json(json_str: str) -> list:
    if not json_str:
        return []
    try:
        parsed = json.loads(json_str)
    except json.JSONDecodeError:
        return []
    points = []
    for point in parsed:
        if isinstance(point, list) and len(point) >= 2:
            points.append((float(point[0]), float(point[1])))
    return points


class CardinalPerceptionSim(Node):
    def __init__(self):
        super().__init__("cardinal_perception_sim")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cardinal_mark_topic", "/sim/cardinal_mark")
        self.declare_parameter("detection_topic", "/buoy_detections_3d")
        # buoy_position_xy is authored in the Task1 map frame.  Publishing
        # this simulator-only ground truth in map avoids combining /odom
        # coordinates with map->base_link TF a second time.
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("buoy_position_xy", "[[28.0, -25.0], [18.0, -25.0], [11.0, -25.0]]")
        self.declare_parameter("buoy_marks", "[]")
        self.declare_parameter("publish_rate_hz", 5.0)
        # Simulation-only proximity gate. It deliberately does not alter the
        # real ZED2i/Mid-360 perception pipeline.
        self.declare_parameter("recognition_range_m", 8.0)

        # ZED2i stereo depth envelope (zed2i_jetson_orin_nano.yaml: depth_min_m/depth_max_m).
        self.declare_parameter("camera_min_range_m", 0.3)
        self.declare_parameter("camera_max_range_m", 20.0)
        # Matches src/detection/yolo/yolo/main.py's camera_fov_deg default (ZED2i HD720).
        self.declare_parameter("camera_fov_deg", 90.0)
        # Matches zed2i_jetson_orin_nano.yaml confidence_threshold.
        self.declare_parameter("confidence_threshold", 0.25)

        # Livox Mid-360: 360 deg horizontal FOV, non-binding range ceiling.
        self.declare_parameter("lidar_max_range_m", 40.0)
        self.declare_parameter("lidar_fov_deg", 360.0)

        # Mirrors an on-boat "marker sighted -> start classification" trigger
        # so the sim doesn't require an external operator to call
        # /yolo/start_inference (nothing else in the graph calls it).
        self.declare_parameter("auto_trigger_inference", True)

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.cardinal_mark_topic = self.get_parameter("cardinal_mark_topic").get_parameter_value().string_value
        self.detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        self.output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        self.buoy_positions = parse_xy_json(
            self.get_parameter("buoy_position_xy").get_parameter_value().string_value
        )
        configured_marks = self._parse_marks(
            self.get_parameter("buoy_marks").get_parameter_value().string_value)
        self.publish_rate_hz = max(0.5, self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self.recognition_range_m = max(
            0.1, self.get_parameter("recognition_range_m").get_parameter_value().double_value)

        self.camera_min_range_m = self.get_parameter("camera_min_range_m").get_parameter_value().double_value
        self.camera_max_range_m = self.get_parameter("camera_max_range_m").get_parameter_value().double_value
        self.camera_fov_rad = math.radians(
            self.get_parameter("camera_fov_deg").get_parameter_value().double_value
        )
        self.confidence_threshold = self.get_parameter(
            "confidence_threshold"
        ).get_parameter_value().double_value

        self.lidar_max_range_m = self.get_parameter("lidar_max_range_m").get_parameter_value().double_value
        self.lidar_fov_rad = math.radians(
            self.get_parameter("lidar_fov_deg").get_parameter_value().double_value
        )

        self.auto_trigger_inference = self.get_parameter(
            "auto_trigger_inference"
        ).get_parameter_value().bool_value

        # Cardinal marks are unknown until /sim/cardinal_mark reports them.
        # task1_orchestrator publishes a JSON array aligned index-wise with
        # buoy_position_xy (e.g. '["S", "N", "S"]') so each marker keeps its
        # own orientation instead of sharing a single mark across the whole
        # course. Marks are not used for classification until have_mark is
        # True (mirrors the real pipeline needing YOLO inference to confirm
        # the marker type before it is trusted).
        self.current_marks = configured_marks
        self.have_mark = bool(configured_marks)
        self.latest_odom = None
        self.inference_requested = False

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.sub_mark = self.create_subscription(String, self.cardinal_mark_topic, self._on_mark, 10)
        self.pub_detections = self.create_publisher(BuoyDetectionArray, self.detection_topic, 10)
        self.inference_client = self.create_client(Trigger, "/yolo/start_inference")

        if not self.buoy_positions:
            self.get_logger().warn("cardinal_perception_sim: buoy_position_xy is empty; no markers to detect")

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self._on_timer)

    def _on_mark(self, msg: String):
        marks = self._parse_marks(msg.data)
        if marks:
            self.current_marks = marks
            self.have_mark = True

    @staticmethod
    def _parse_marks(data: str) -> list:
        """Parse the JSON mark array published on /sim/cardinal_mark.

        Also accepts a single legacy letter (e.g. "N") for backward
        compatibility, applying it to every marker.
        """
        text = data.strip()
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            parsed = None

        if isinstance(parsed, list):
            marks = [str(v).strip().upper() for v in parsed if isinstance(v, str)]
            return [m for m in marks if m in CARDINAL_CLASS_BY_MARK]

        legacy = text.strip().upper()
        if legacy in CARDINAL_CLASS_BY_MARK:
            return [legacy]
        return []

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _within_sensor_gate(self, distance: float, bearing: float) -> bool:
        camera_ok = (
            self.camera_min_range_m <= distance <= self.camera_max_range_m
            and abs(bearing) <= self.camera_fov_rad / 2.0
        )
        lidar_ok = distance <= self.lidar_max_range_m and abs(bearing) <= self.lidar_fov_rad / 2.0
        return camera_ok and lidar_ok

    def _mark_for_index(self, idx: int):
        """Return the cardinal mark for buoy_positions[idx], if known.

        Falls back to current_marks[0] when exactly one legacy mark was
        received for a multi-marker course (backward compatibility with the
        pre-per-marker /sim/cardinal_mark payload).
        """
        if idx < len(self.current_marks):
            return self.current_marks[idx]
        if len(self.current_marks) == 1:
            return self.current_marks[0]
        return None

    def _maybe_trigger_inference(self):
        if self.inference_requested or not self.auto_trigger_inference:
            return
        if not self.inference_client.service_is_ready():
            return
        self.inference_requested = True
        self.get_logger().info(
            "cardinal_perception_sim: marker within sensor range, triggering /yolo/start_inference"
        )
        self.inference_client.call_async(Trigger.Request())

    def _on_timer(self):
        array = BuoyDetectionArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.output_frame

        if self.latest_odom is not None and self.buoy_positions:
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y
            yaw = yaw_from_quaternion(self.latest_odom.pose.pose.orientation)

            any_in_range = False
            for idx, (bx, by) in enumerate(self.buoy_positions):
                dx = bx - x
                dy = by - y
                distance = math.hypot(dx, dy)
                # The simulator has no image plane, so proximity is the
                # recognition criterion. This is intentionally independent
                # of heading/FOV; static buoy classes are loaded from the
                # same scenario config as their positions.
                if distance > self.recognition_range_m:
                    continue
                any_in_range = True
                if not self.have_mark:
                    # Sensor sees a marker-shaped object but classification
                    # (cardinal type) has not been confirmed yet.
                    continue
                mark = self._mark_for_index(idx)
                if mark is None:
                    continue
                confidence = max(
                    self.confidence_threshold,
                    1.0 - 0.5 * (distance / self.camera_max_range_m),
                )
                detection = BuoyDetection()
                detection.class_id = CARDINAL_CLASS_BY_MARK[mark]
                detection.confidence = float(confidence)
                if self.output_frame == "map":
                    # The scenario's buoy coordinates are already map-frame
                    # ground truth.  Do not transform them through /odom and
                    # base_link: map->odom drift would create a new wall track
                    # for every observation of the same buoy.
                    detection.position.x = bx
                    detection.position.y = by
                else:
                    # Retain the real-driver-compatible base_link output for
                    # explicit non-map test configurations.
                    detection.position.x = dx * math.cos(yaw) + dy * math.sin(yaw)
                    detection.position.y = -dx * math.sin(yaw) + dy * math.cos(yaw)
                detection.position.z = 0.0
                detection.position_source = BuoyDetection.POSITION_LIDAR_FUSED
                array.detections.append(detection)

            if any_in_range:
                self._maybe_trigger_inference()

        self.pub_detections.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = CardinalPerceptionSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
