#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray


class Task2GpsWaypointPublisher(Node):
    """
    /sim/task2_gps_markers から task2_gps_points の marker.pose.position を取り出し，
    MPPI planner 用の /waypoint1_pose, /waypoint2_pose として publish するノード．
    """

    def __init__(self):
        super().__init__("task2_gps_waypoint_publisher")

        self.declare_parameter("gps_marker_topic", "/sim/task2_gps_markers")
        self.declare_parameter("waypoint1_topic", "/waypoint1_pose")
        self.declare_parameter("waypoint2_topic", "/waypoint2_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_frequency", 2.0)

        self.gps_marker_topic = self.get_parameter("gps_marker_topic").value
        self.waypoint1_topic = self.get_parameter("waypoint1_topic").value
        self.waypoint2_topic = self.get_parameter("waypoint2_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        publish_frequency = self.get_parameter("publish_frequency").value
        timer_period = 1.0 / max(publish_frequency, 1e-6)

        self.latest_markers = None

        self.marker_sub = self.create_subscription(
            MarkerArray,
            self.gps_marker_topic,
            self.marker_callback,
            10,
        )

        self.wp1_pub = self.create_publisher(
            PoseStamped,
            self.waypoint1_topic,
            10,
        )

        self.wp2_pub = self.create_publisher(
            PoseStamped,
            self.waypoint2_topic,
            10,
        )

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("task2_gps_waypoint_publisher started.")
        self.get_logger().info(f"Subscribe: {self.gps_marker_topic}")
        self.get_logger().info(f"Publish  : {self.waypoint1_topic}")
        self.get_logger().info(f"Publish  : {self.waypoint2_topic}")

    def marker_callback(self, msg: MarkerArray):
        self.latest_markers = msg

    def timer_callback(self):
        if self.latest_markers is None:
            self.get_logger().warn(
                "No /sim/task2_gps_markers received yet.",
                throttle_duration_sec=2.0,
            )
            return

        gps_points = []

        for marker in self.latest_markers.markers:
            if marker.ns != "task2_gps_points":
                continue

            # 今回のGPS markerは points ではなく pose.position に座標が入っている
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z

            gps_points.append((marker.id, x, y, z, marker.header.frame_id))

        if len(gps_points) < 2:
            self.get_logger().warn(
                f"Need at least 2 GPS point markers, but got {len(gps_points)}.",
                throttle_duration_sec=2.0,
            )
            return

        # id順に並べる
        gps_points.sort(key=lambda p: p[0])

        _, x1, y1, z1, frame1 = gps_points[0]
        _, x2, y2, z2, frame2 = gps_points[1]

        frame_id = frame1 if frame1 else self.frame_id

        now = self.get_clock().now().to_msg()

        wp1_msg = self.make_pose_stamped(
            x=x1,
            y=y1,
            z=z1,
            next_x=x2,
            next_y=y2,
            frame_id=frame_id,
            stamp=now,
        )

        wp2_msg = self.make_pose_stamped(
            x=x2,
            y=y2,
            z=z2,
            next_x=x1,
            next_y=y1,
            frame_id=frame_id,
            stamp=now,
        )

        self.wp1_pub.publish(wp1_msg)
        self.wp2_pub.publish(wp2_msg)

        self.get_logger().info(
            f"Published Task2 GPS waypoints: "
            f"wp1=({x1:.2f}, {y1:.2f}), "
            f"wp2=({x2:.2f}, {y2:.2f})",
            throttle_duration_sec=2.0,
        )

    def make_pose_stamped(self, x, y, z, next_x, next_y, frame_id, stamp):
        yaw = math.atan2(next_y - y, next_x - x)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = Task2GpsWaypointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()