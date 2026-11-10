#!/usr/bin/env python3
"""Estimate a U-shaped berth centre and yaw from LiDAR, conservatively."""

from __future__ import annotations

from math import atan2, cos, isfinite, pi, sin
import struct

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


def _yaw(quaternion) -> float:
    return atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                 1.0 - 2.0 * quaternion.z * quaternion.z)


def _median(values):
    values = sorted(values)
    if not values:
        return None
    middle = len(values) // 2
    return values[middle] if len(values) % 2 else (values[middle - 1] + values[middle]) / 2.0


def _wall_yaw_delta(walls, expected_angle):
    """Return the median signed wall-angle error, modulo a straight wall."""
    deltas = []
    for wall in walls:
        mean_u = sum(u for u, _ in wall) / len(wall)
        mean_v = sum(v for _, v in wall) / len(wall)
        uu = sum((u - mean_u) ** 2 for u, _ in wall)
        uv = sum((u - mean_u) * (v - mean_v) for u, v in wall)
        vv = sum((v - mean_v) ** 2 for _, v in wall)
        angle = 0.5 * atan2(2.0 * uv, uu - vv)
        delta = (angle - expected_angle + pi / 2.0) % pi - pi / 2.0
        deltas.append(delta)
    return _median(deltas)


class DockWallCorrection(Node):
    """Use three LiDAR walls around each berth to estimate its centre."""

    def __init__(self):
        super().__init__("dock_wall_correction")
        self.declare_parameters("", [
            ("cloud_topic", "/livox/lidar"),
            ("target_frame", "map"),
            ("min_wall_points", 12),
            ("min_wall_span_ratio", 0.55),
            ("wall_band_m", 0.25),
            ("search_margin_m", 1.2),
            ("max_correction_m", 1.0),
            ("max_yaw_correction_rad", 0.35),
            ("min_z_m", 0.35),
            ("max_z_m", 2.5),
        ])
        self._targets = {}
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._out = {
            "berth1": self.create_publisher(PoseStamped, "/docking/berth1_corrected", 10),
            "berth2": self.create_publisher(PoseStamped, "/docking/berth2_corrected", 10),
        }
        self.create_subscription(PoseStamped, "/docking/berth1_nominal",
                                 lambda msg: self._targets.__setitem__("berth1", msg), 1)
        self.create_subscription(PoseStamped, "/docking/berth2_nominal",
                                 lambda msg: self._targets.__setitem__("berth2", msg), 1)
        self.create_subscription(PointCloud2, str(self.get_parameter("cloud_topic").value),
                                 self._on_cloud, 5)

    @staticmethod
    def _points(message):
        fields = {field.name: field for field in message.fields}
        required = [fields.get(name) for name in ("x", "y", "z")]
        if any(field is None or field.datatype != PointField.FLOAT32 for field in required):
            return
        endian = ">" if message.is_bigendian else "<"
        for offset in range(0, len(message.data), message.point_step):
            x, y, z = (struct.unpack_from(endian + "f", message.data, offset + field.offset)[0]
                       for field in required)
            if isfinite(x) and isfinite(y) and isfinite(z):
                yield x, y, z

    def _on_cloud(self, cloud):
        for berth, target in tuple(self._targets.items()):
            if not cloud.header.frame_id or not target.header.frame_id:
                continue
            try:
                transform = self._tf.lookup_transform(
                    cloud.header.frame_id, target.header.frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            corrected = self._correct(berth, target, cloud, transform)
            if corrected is not None:
                self._out[berth].publish(corrected)

    def _correct(self, berth, target, cloud, transform):
        translation, rotation = transform.transform.translation, transform.transform.rotation
        transform_yaw = _yaw(rotation)
        target_yaw = _yaw(target.pose.orientation) + transform_yaw
        tx = translation.x + cos(transform_yaw) * target.pose.position.x - sin(transform_yaw) * target.pose.position.y
        ty = translation.y + sin(transform_yaw) * target.pose.position.x + cos(transform_yaw) * target.pose.position.y
        along, across, opening_along = (2.0, 2.0, True) if berth == "berth1" else (4.13, 2.0, False)
        margin = float(self.get_parameter("search_margin_m").value)
        band = float(self.get_parameter("wall_band_m").value) + float(self.get_parameter("max_correction_m").value)
        minimum, span_ratio = int(self.get_parameter("min_wall_points").value), float(self.get_parameter("min_wall_span_ratio").value)
        z_min, z_max = (float(self.get_parameter(name).value) for name in ("min_z_m", "max_z_m"))
        c, s = cos(target_yaw), sin(target_yaw)
        local = []
        for x, y, z in self._points(cloud):
            if z_min <= z <= z_max:
                u, v = c * (x - tx) + s * (y - ty), -s * (x - tx) + c * (y - ty)
                if abs(u) <= along / 2.0 + margin and abs(v) <= across / 2.0 + margin:
                    local.append((u, v))
        if opening_along:
            front = [(u, v) for u, v in local if abs(u - along / 2.0) <= band and abs(v) <= across / 2.0 + margin]
            left = [(u, v) for u, v in local if abs(v - across / 2.0) <= band and -along / 2.0 - margin <= u <= along / 2.0]
            right = [(u, v) for u, v in local if abs(v + across / 2.0) <= band and -along / 2.0 - margin <= u <= along / 2.0]
            if (min(len(front), len(left), len(right)) < minimum or max(v for _, v in front) - min(v for _, v in front) < across * span_ratio or max(u for u, _ in left) - min(u for u, _ in left) < along * span_ratio or max(u for u, _ in right) - min(u for u, _ in right) < along * span_ratio):
                return None
            centre_u, centre_v = _median([u for u, _ in front]) - along / 2.0, (_median([v for _, v in left]) + _median([v for _, v in right])) / 2.0
            yaw_delta = _wall_yaw_delta((left, right), 0.0)
        else:
            front = [(u, v) for u, v in local if abs(v - across / 2.0) <= band and abs(u) <= along / 2.0 + margin]
            left = [(u, v) for u, v in local if abs(u - along / 2.0) <= band and -across / 2.0 - margin <= v <= across / 2.0]
            right = [(u, v) for u, v in local if abs(u + along / 2.0) <= band and -across / 2.0 - margin <= v <= across / 2.0]
            if (min(len(front), len(left), len(right)) < minimum or max(u for u, _ in front) - min(u for u, _ in front) < along * span_ratio or max(v for _, v in left) - min(v for _, v in left) < across * span_ratio or max(v for _, v in right) - min(v for _, v in right) < across * span_ratio):
                return None
            centre_u, centre_v = (_median([u for u, _ in left]) + _median([u for u, _ in right])) / 2.0, _median([v for _, v in front]) - across / 2.0
            yaw_delta = _wall_yaw_delta((left, right), pi / 2.0)
        if yaw_delta is None or abs(yaw_delta) > float(self.get_parameter("max_yaw_correction_rad").value):
            return None
        map_yaw = _yaw(target.pose.orientation)
        dx, dy = cos(map_yaw) * centre_u - sin(map_yaw) * centre_v, sin(map_yaw) * centre_u + cos(map_yaw) * centre_v
        if dx * dx + dy * dy > float(self.get_parameter("max_correction_m").value) ** 2:
            return None
        result = PoseStamped()
        result.header = target.header
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose = target.pose
        result.pose.position.x += dx
        result.pose.position.y += dy
        corrected_yaw = map_yaw + yaw_delta
        result.pose.orientation.z, result.pose.orientation.w = sin(corrected_yaw / 2.0), cos(corrected_yaw / 2.0)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DockWallCorrection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
