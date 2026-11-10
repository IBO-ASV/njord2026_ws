"""Publish the selected local waypoint YAML as geographic display markers."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from njord_interfaces.msg import MissionStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import yaml


_CONFIGS = {
    "task1": ("waypoint_publisher", "config/task1_waypoints.yaml", "task1_config"),
    "task1_skip_1_1": ("waypoint_publisher", "config/task1_skip_1_1_waypoints.yaml", "task1_skip_1_1_config"),
    "task1_follow": ("waypoint_publisher", "config/task1_follow_waypoints.yaml", "task1_follow_config"),
    "task2": ("waypoint_publisher", "config/task2_waypoints.yaml", "task2_config"),
    "task2_bak": ("waypoint_publisher", "config/task2_bak_waypoints.yaml", "task2_bak_config"),
    "task3_1": ("waypoint_publisher", "config/task3_waypoints.yaml", "task3_1_config"),
    "task3_2": ("waypoint_publisher", "config/task3_waypoints.yaml", "task3_2_config"),
    "task4": ("waypoint_publisher", "config/task4_waypoints.yaml", "task4_config"),
    "move_to_exam_field": (
        "mission_manager", "config/move_to_exam_field_waypoints.yaml", "move_to_exam_field_config"
    ),
    "back_to_birth_point": (
        "mission_manager", "config/move_to_exam_field_waypoints.yaml", "back_to_birth_point_config"
    ),
}

def _coordinate(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    match = re.fullmatch(r"\s*(\d+)°(\d+)[′'](\d+(?:\.\d+)?)[″\"]\s*([NSEW])\s*", str(value))
    if not match:
        raise ValueError(f"invalid coordinate {value!r}")
    degrees, minutes, seconds = int(match.group(1)), int(match.group(2)), float(match.group(3))
    if minutes >= 60 or not 0.0 <= seconds < 60.0:
        raise ValueError(f"invalid coordinate {value!r}")
    coordinate = degrees + minutes / 60.0 + seconds / 3600.0
    return -coordinate if match.group(4) in "SW" else coordinate


class GroundWaypointGeoPublisher(Node):
    """Ground-only WGS84 marker source selected by the active mission task."""

    def __init__(self):
        super().__init__("ground_waypoint_geo_publisher")
        self.declare_parameter("marker_topic", "/ground_waypoint_markers")
        self.declare_parameter("publish_rate_hz", 1.0)
        task_type = self.declare_parameter("task_type", "").value
        self.waypoints = []
        self.active_task = ""
        self.publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if task_type:
            self._select_task(str(task_type))
        else:
            status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_sub = self.create_subscription(
                MissionStatus, "/mission/status", self._on_status, status_qos)
        self.timer = self.create_timer(1.0 / rate, self._publish)

    def _on_status(self, message):
        """Load only the YAML selected by the task command sent from ground."""
        self._select_task(message.task_id.strip())

    def _select_task(self, task_type):
        if task_type == self.active_task:
            return
        self.active_task = task_type
        self.waypoints = []
        if task_type not in _CONFIGS:
            self.get_logger().info("No active task waypoint display")
            self._publish()
            return
        package, filename, key = _CONFIGS[task_type]
        config_path = Path(get_package_share_directory(package)) / filename
        with config_path.open() as stream:
            self.waypoints = yaml.safe_load(stream)[key].get("waypoints", [])
        self.get_logger().info(f"Showing local YAML waypoints for {task_type}")
        self._publish()

    def _publish(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        reset = Marker()
        reset.action = Marker.DELETEALL
        markers.markers.append(reset)
        for index, waypoint in enumerate(self.waypoints):
            if waypoint.get("display", True) is False:
                continue
            if "latitude" not in waypoint or "longitude" not in waypoint:
                continue
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = "wgs84", stamp
            marker.ns, marker.id = "ground_waypoint_wgs84", index
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            # The Foxglove geographic-map display uses x=longitude and y=latitude.
            marker.pose.position.x = _coordinate(waypoint["longitude"])
            marker.pose.position.y = _coordinate(waypoint["latitude"])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.9, 1.0, 1.0
            # GNSS Map Telemetry reads this field for the label beside the
            # numbered pin on the geographic map.
            marker.text = str(waypoint.get("name", "")).strip()
            markers.markers.append(marker)
            label = Marker()
            label.header.frame_id, label.header.stamp = "wgs84", stamp
            label.ns, label.id, label.type, label.action = (
                "ground_waypoint_order", index, Marker.TEXT_VIEW_FACING, Marker.ADD)
            label.pose.position.x = marker.pose.position.x
            label.pose.position.y = marker.pose.position.y
            # Repeated GPS8 points would otherwise place 03/05/06 exactly on
            # top of each other.  Offset only their label, not the waypoint.
            label.pose.position.z = 1.0 + 0.35 * index
            label.pose.orientation.w, label.scale.z = 1.0, 0.8
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            order = str(waypoint.get("competition_id", waypoint.get("id", index + 1)))
            name = str(waypoint.get("name", "")).strip()
            label.text = f"{order} {name}" if name else order
            markers.markers.append(label)
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = GroundWaypointGeoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
