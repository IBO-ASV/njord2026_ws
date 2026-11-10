#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
field_boundary_publisher_node.py

Publishes a static nav_msgs/OccupancyGrid that marks the outside of the
competition field and fixed dock structures as LETHAL (cost=100).

For Task 3 the field is a configurable rectangle centered in the map frame.
Everything outside is marked 100; everything inside is 0.
The two dock structures are also marked 100 so planning does not depend on
ideal dock point clouds being available.

The map is published LATCHED (transient_local QoS) so Nav2 StaticLayer
receives it even if it subscribes late.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import math


class FieldBoundaryPublisher(Node):
    def __init__(self):
        super().__init__('field_boundary_publisher')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.2)          # coarser for large static map
        self.declare_parameter('map_size_m', 80.0)         # total map side
        self.declare_parameter('field_size_x_m', 40.0)     # interior free-zone width
        self.declare_parameter('field_size_y_m', 40.0)     # interior free-zone height
        self.declare_parameter('field_center_x', 0.0)
        self.declare_parameter('field_center_y', 0.0)
        self.declare_parameter('boundary_cost', 100)
        self.declare_parameter('include_task3_docks', True)
        self.declare_parameter('dock_wall_thickness_m', 0.3)

        frame = self.get_parameter('map_frame').value
        res = self.get_parameter('resolution').value
        map_size = self.get_parameter('map_size_m').value
        field_size_x = self.get_parameter('field_size_x_m').value
        field_size_y = self.get_parameter('field_size_y_m').value
        cx = self.get_parameter('field_center_x').value
        cy = self.get_parameter('field_center_y').value
        boundary_cost = int(self.get_parameter('boundary_cost').value)
        include_task3_docks = self.get_parameter('include_task3_docks').value
        dock_wall_thickness = self.get_parameter('dock_wall_thickness_m').value

        half_field_x = field_size_x / 2.0
        half_field_y = field_size_y / 2.0
        n = int(math.ceil(map_size / res))
        if n % 2 == 0:
            n += 1
        origin_x = cx - (n // 2) * res
        origin_y = cy - (n // 2) * res

        data = []
        for row in range(n):
            wy = origin_y + row * res
            for col in range(n):
                wx = origin_x + col * res
                in_field = (
                    abs(wx - cx) <= half_field_x and
                    abs(wy - cy) <= half_field_y
                )
                in_task3_dock = (
                    include_task3_docks and
                    self._is_in_task3_dock_wall(wx, wy, dock_wall_thickness)
                )
                data.append(0 if in_field and not in_task3_dock else boundary_cost)

        # Build OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.info.resolution = res
        msg.info.width = n
        msg.info.height = n
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data

        # Latched publisher (transient_local)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/field_boundary_costmap', qos)
        self.pub.publish(msg)

        self.get_logger().info(
            f'FieldBoundaryPublisher: published {n}x{n} map | '
            f'field={field_size_x}m x {field_size_y}m | res={res}m | '
            f'origin=({origin_x:.1f},{origin_y:.1f}) | '
            f'task3_docks={include_task3_docks}'
        )

    def _is_in_task3_dock_wall(self, x: float, y: float, thickness: float) -> bool:
        half_t = thickness / 2.0

        def segment_rect(x_min, x_max, y_min, y_max):
            return x_min <= x <= x_max and y_min <= y <= y_max

        def horizontal(x1, x2, y0):
            return segment_rect(min(x1, x2), max(x1, x2), y0 - half_t, y0 + half_t)

        def vertical(x0, y1, y2):
            return segment_rect(x0 - half_t, x0 + half_t, min(y1, y2), max(y1, y2))

        # Task 3.1 normal dock at x=19..21. Three prongs and rear walls.
        normal_dock = (
            horizontal(19.0, 21.0, 2.13) or
            horizontal(19.0, 21.0, -2.13) or
            horizontal(19.0, 21.0, 0.0) or
            vertical(21.0, 0.0, 2.13) or
            vertical(21.0, -2.13, 0.0)
        )

        # Task 3.2 parallel dock at x=-19..-21. Two prongs and one rear wall.
        parallel_dock = (
            horizontal(-21.0, -19.0, 2.13) or
            horizontal(-21.0, -19.0, -2.13) or
            vertical(-21.0, -2.13, 2.13)
        )

        return normal_dock or parallel_dock


def main(args=None):
    rclpy.init(args=args)
    node = FieldBoundaryPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
