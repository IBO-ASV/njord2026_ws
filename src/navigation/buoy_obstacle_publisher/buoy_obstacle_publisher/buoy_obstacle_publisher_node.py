#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
buoy_obstacle_publisher_node.py

Reads buoy TF frames by name (explicit list via parameter) and publishes a
nav_msgs/OccupancyGrid with lethal-cost circles around each buoy.

This OccupancyGrid is consumed by Nav2's StaticLayer via:
  map_topic: /buoy_costmap
in the local and/or global costmap config.

Design rationale:
  - Uses an explicit frame-name list (more reliable than scanning all_frames_as_yaml).
  - Works with real detection (buoy TFs from pcl_det/yolo) and simulation
    (buoy TFs from task3_orchestrator with b31_*/b32_* naming).
  - Grid is centered at map origin; large enough to cover the 40 m field.
  - Published at ~5 Hz so dynamic buoy motion is tracked.
"""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class BuoyObstaclePublisher(Node):
    def __init__(self):
        super().__init__('buoy_obstacle_publisher')

        # --- Parameters ---
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('grid_size_m', 60.0)
        self.declare_parameter('buoy_radius_m', 0.45)
        self.declare_parameter('inflation_radius_m', 1.2)
        self.declare_parameter('publish_rate_hz', 5.0)
        # Explicit list of buoy TF frame names to monitor
        self.declare_parameter('buoy_frame_names', [
            'b31_red_1', 'b31_red_2', 'b31_red_3', 'b31_red_gate',
            'b31_green_1', 'b31_green_2', 'b31_green_3', 'b31_green_gate',
            'b32_red_1', 'b32_red_2', 'b32_red_3', 'b32_red_gate',
            'b32_green_1', 'b32_green_2', 'b32_green_3', 'b32_green_gate',
        ])

        self.map_frame = self.get_parameter('map_frame').value
        self.resolution = self.get_parameter('resolution').value
        self.grid_size_m = self.get_parameter('grid_size_m').value
        self.buoy_radius = self.get_parameter('buoy_radius_m').value
        self.inflation_radius = self.get_parameter('inflation_radius_m').value
        self.buoy_frame_names = self.get_parameter('buoy_frame_names').value

        # Grid geometry (fixed, centered at map origin)
        self.grid_cells = int(math.ceil(self.grid_size_m / self.resolution))
        if self.grid_cells % 2 == 0:
            self.grid_cells += 1
        self.half_cells = self.grid_cells // 2
        self.origin_x = -self.half_cells * self.resolution
        self.origin_y = -self.half_cells * self.resolution

        # Pre-compute disk stamp (dx, dy) offsets for obstacle circles
        obstacle_r_m = self.buoy_radius + self.inflation_radius
        self.obstacle_r_cells = int(math.ceil(obstacle_r_m / self.resolution))
        r = self.obstacle_r_cells
        self._stamp = [
            (dx, dy)
            for dy in range(-r, r + 1)
            for dx in range(-r, r + 1)
            if dx * dx + dy * dy <= r * r
        ]

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(OccupancyGrid, '/buoy_costmap', 10)

        rate = max(0.1, self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'BuoyObstaclePublisher: monitoring {len(self.buoy_frame_names)} frames | '
            f'grid={self.grid_cells}x{self.grid_cells} cells | '
            f'obs_r={obstacle_r_m:.2f}m'
        )

    def _on_timer(self):
        buoy_positions = self._collect_buoy_positions()
        msg = self._build_grid(buoy_positions)
        self.pub.publish(msg)

    def _collect_buoy_positions(self):
        """Try to look up each known buoy frame. Silently skip frames not yet available."""
        positions = []
        for frame_id in self.buoy_frame_names:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, frame_id, rclpy.time.Time()
                )
                positions.append((t.transform.translation.x, t.transform.translation.y))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass  # Frame not yet available — normal during startup
        return positions

    def _build_grid(self, buoy_positions):
        """Build an OccupancyGrid centered at map origin with lethal circles per buoy."""
        n = self.grid_cells
        data = [0] * (n * n)

        for (bx, by) in buoy_positions:
            cx = int(round((bx - self.origin_x) / self.resolution))
            cy = int(round((by - self.origin_y) / self.resolution))
            for dx, dy in self._stamp:
                gx, gy = cx + dx, cy + dy
                if 0 <= gx < n and 0 <= gy < n:
                    data[gy * n + gx] = 100  # LETHAL

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = n
        msg.info.height = n
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BuoyObstaclePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
