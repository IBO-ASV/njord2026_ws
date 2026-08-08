#!/usr/bin/env python3
"""Publish mission runtime readiness from existing, read-only evidence.

This node never starts an executor, sends a Nav2 goal, or writes to the
perception/collision/TF producers.  It merely turns their live output into the
three typed Bool inputs consumed by Mission Manager.  Missing or stale input is
always reported as ``false``.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from nav2_msgs.msg import CollisionMonitorState
from njord_interfaces.msg import BuoyDetectionArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from .readiness_evidence import (
    buoy_perception_ready,
    collision_monitor_ready,
    dynamic_gate_tf_ready,
)


_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _stamp_seconds(stamp) -> Optional[float]:
    seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return seconds if seconds > 0.0 else None


class RuntimeReadinessAdapter(Node):
    """Fail-closed adapter for Task 2/3 external runtime interlocks."""

    def __init__(self) -> None:
        super().__init__("runtime_readiness_adapter")
        self.declare_parameter("collision_monitor_topic", "/collision_monitor_state")
        self.declare_parameter("buoy_detection_topic", "/buoy_detections_3d")
        self.declare_parameter("task2_perception_topic", "/task2/points_filtered")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter(
            "dynamic_gate_frames",
            ["b31_red_gate", "b31_green_gate", "b32_red_gate", "b32_green_gate"],
        )
        self.declare_parameter("collision_monitor_timeout_sec", 1.0)
        self.declare_parameter("buoy_perception_timeout_sec", 1.0)
        self.declare_parameter("dynamic_gate_tf_timeout_sec", 1.0)
        self.declare_parameter("publish_period_sec", 0.1)

        self._collision_seen_at: Optional[float] = None
        self._collision_action_type: Optional[int] = None
        self._buoy_seen_at: Optional[float] = None
        self._buoy_header_stamp: Optional[float] = None
        self._buoy_header_frame = ""
        self._task2_perception_seen_at: Optional[float] = None
        self._task2_perception_header_stamp: Optional[float] = None
        self._task2_perception_header_frame = ""
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._gate_frames = tuple(str(frame) for frame in self.get_parameter(
            "dynamic_gate_frames").value)
        if not self._target_frame or not self._gate_frames:
            raise ValueError("target_frame and dynamic_gate_frames must not be empty")
        if len(set(self._gate_frames)) != len(self._gate_frames):
            raise ValueError("dynamic_gate_frames must not contain duplicates")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._collision_sub = self.create_subscription(
            CollisionMonitorState,
            str(self.get_parameter("collision_monitor_topic").value),
            self._on_collision_monitor,
            10,
        )
        self._buoy_sub = self.create_subscription(
            BuoyDetectionArray,
            str(self.get_parameter("buoy_detection_topic").value),
            self._on_buoy_detections,
            10,
        )
        self._task2_perception_sub = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("task2_perception_topic").value),
            self._on_task2_perception,
            10,
        )
        # Do not use Node._publishers: it is rclpy's internal list of all
        # publishers and is needed by Node.destroy_node().
        self._readiness_publishers = {
            name: self.create_publisher(Bool, f"/mission/readiness/{name}", _STATUS_QOS)
            for name in ("collision_monitor", "buoy_perception", "dynamic_gate_tf")
        }
        period = float(self.get_parameter("publish_period_sec").value)
        if period <= 0.0:
            raise ValueError("publish_period_sec must be positive")
        self._timer = self.create_timer(period, self._publish_readiness)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_collision_monitor(self, message: CollisionMonitorState) -> None:
        self._collision_seen_at = self._now_seconds()
        self._collision_action_type = int(message.action_type)

    def _on_buoy_detections(self, message: BuoyDetectionArray) -> None:
        self._buoy_seen_at = self._now_seconds()
        self._buoy_header_stamp = _stamp_seconds(message.header.stamp)
        self._buoy_header_frame = message.header.frame_id

    def _on_task2_perception(self, message: PointCloud2) -> None:
        """Treat a fresh filtered Task 2 cloud as task-specific perception evidence."""
        self._task2_perception_seen_at = self._now_seconds()
        self._task2_perception_header_stamp = _stamp_seconds(message.header.stamp)
        self._task2_perception_header_frame = message.header.frame_id

    def _gate_transform_stamps(self) -> tuple[Optional[float], ...]:
        stamps = []
        for frame in self._gate_frames:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame, frame, rclpy.time.Time()
                )
            except Exception:
                stamps.append(None)
            else:
                stamps.append(_stamp_seconds(transform.header.stamp))
        return tuple(stamps)

    def _publish_readiness(self) -> None:
        now = self._now_seconds()
        collision = collision_monitor_ready(
            self._collision_seen_at,
            now,
            float(self.get_parameter("collision_monitor_timeout_sec").value),
            self._collision_action_type,
        )
        buoy_perception = buoy_perception_ready(
            self._buoy_seen_at,
            now,
            float(self.get_parameter("buoy_perception_timeout_sec").value),
            self._buoy_header_stamp,
            self._buoy_header_frame,
        )
        task2_perception = buoy_perception_ready(
            self._task2_perception_seen_at,
            now,
            float(self.get_parameter("buoy_perception_timeout_sec").value),
            self._task2_perception_header_stamp,
            self._task2_perception_header_frame,
        )
        perception = buoy_perception or task2_perception
        gate_tf = dynamic_gate_tf_ready(
            self._gate_transform_stamps(),
            now,
            float(self.get_parameter("dynamic_gate_tf_timeout_sec").value),
        )
        for name, ready in (
            ("collision_monitor", collision),
            ("buoy_perception", perception),
            ("dynamic_gate_tf", gate_tf),
        ):
            self._readiness_publishers[name].publish(Bool(data=ready))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuntimeReadinessAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
