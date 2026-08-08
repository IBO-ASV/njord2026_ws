import math

import rclpy
from geometry_msgs.msg import Point, Point32, PolygonStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def _get_xy(values, fallback):
    if len(values) >= 2:
        return float(values[0]), float(values[1])
    return fallback


def _parse_vertices(values):
    default_vertices = [
        (2.0, 0.0),
        (0.0, 1.0),
        (-2.0, 1.0),
        (-2.0, -1.0),
        (0.0, -1.0),
    ]
    if len(values) < 6 or len(values) % 2 != 0:
        return default_vertices
    return [(float(values[i]), float(values[i + 1])) for i in range(0, len(values), 2)]


class OpponentVesselNode(Node):
    def __init__(self):
        super().__init__("opponent_vessel_node")

        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.vessel_frame_id = self.declare_parameter("vessel_frame_id", "opponent_vessel").value
        footprint_topic = self.declare_parameter(
            "footprint_topic", "/sim/opponent_vessel_footprint"
        ).value
        marker_topic = self.declare_parameter("marker_topic", "/sim/opponent_vessel_outline").value
        gps_marker_topic = self.declare_parameter(
            "gps_marker_topic", "/sim/task2_gps_markers"
        ).value

        self.start_label = self.declare_parameter("start_label", "GPS point 6").value
        self.goal_label = self.declare_parameter("goal_label", "GPS point 5").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.start_delay_s = float(self.declare_parameter("start_delay_s", 3.0).value)
        self.speed_mps = float(self.declare_parameter("speed_mps", 1.0289).value)
        self.disturbance_amp_m = float(self.declare_parameter("disturbance_amp_m", 0.3).value)
        self.disturbance_freq_hz = float(self.declare_parameter("disturbance_freq_hz", 0.1).value)
        self.vessel_height_m = float(self.declare_parameter("vessel_height_m", 3.0).value)
        self.marker_line_width_m = float(self.declare_parameter("marker_line_width_m", 0.08).value)

        # GPS point 用の座標
        # start_xy / goal_xy は GPS marker の位置として使う
        self.gps_start_x, self.gps_start_y = _get_xy(
            self.declare_parameter("start_xy", [60.0, 0.0]).value, (60.0, 0.0)
        )
        self.gps_goal_x, self.gps_goal_y = _get_xy(
            self.declare_parameter("goal_xy", [0.0, 0.0]).value, (0.0, 0.0)
        )

        # 他船移動用の座標
        # opponent_start_xy / opponent_goal_xy があればそちらを使う
        # なければ従来互換として start_xy / goal_xy を使う
        self.start_x, self.start_y = _get_xy(
            self.declare_parameter(
                "opponent_start_xy",
                [self.gps_start_x, self.gps_start_y],
            ).value,
            (self.gps_start_x, self.gps_start_y),
        )
        self.goal_x, self.goal_y = _get_xy(
            self.declare_parameter(
                "opponent_goal_xy",
                [self.gps_goal_x, self.gps_goal_y],
            ).value,
            (self.gps_goal_x, self.gps_goal_y),
        )
        self.footprint_vertices = _parse_vertices(
            self.declare_parameter(
                "footprint_vertices",
                [2.0, 0.0, 0.0, 1.0, -2.0, 1.0, -2.0, -1.0, 0.0, -1.0],
            ).value
        )

        self.footprint_pub = self.create_publisher(PolygonStamped, footprint_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.gps_marker_pub = self.create_publisher(MarkerArray, gps_marker_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_time = self.get_clock().now()
        period_s = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period_s, self.on_timer)
        self.get_logger().info(
            "Opponent vessel: start=(%.2f, %.2f), goal=(%.2f, %.2f), speed=%.4f m/s, delay=%.2f s"
            % (
                self.start_x,
                self.start_y,
                self.goal_x,
                self.goal_y,
                self.speed_mps,
                self.start_delay_s,
            )
        )

    def compute_pose(self, elapsed_s):
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        distance = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if distance > 1e-9 else 0.0

        nominal_x = self.start_x
        nominal_y = self.start_y
        moving_elapsed_s = 0.0
        if distance > 1e-9 and elapsed_s > self.start_delay_s:
            moving_elapsed_s = elapsed_s - self.start_delay_s
            travel = min(distance, self.speed_mps * moving_elapsed_s)
            nominal_x += dx / distance * travel
            nominal_y += dy / distance * travel

        phase = 2.0 * math.pi * self.disturbance_freq_hz * moving_elapsed_s
        return (
            nominal_x + (self.disturbance_amp_m * math.sin(phase) if moving_elapsed_s > 0.0 else 0.0),
            nominal_y + (self.disturbance_amp_m * math.cos(phase) if moving_elapsed_s > 0.0 else 0.0),
            yaw,
        )

    def transformed_vertices(self, pose):
        x, y, yaw = pose
        c = math.cos(yaw)
        s = math.sin(yaw)
        return [(x + c * vx - s * vy, y + s * vx + c * vy) for vx, vy in self.footprint_vertices]

    def publish_tf(self, stamp, pose):
        x, y, yaw = pose
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.vessel_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_broadcaster.sendTransform(transform)

    def publish_footprint(self, stamp, vertices):
        msg = PolygonStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in vertices:
            point = Point32()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            msg.polygon.points.append(point)
        self.footprint_pub.publish(msg)

    def publish_marker(self, stamp, vertices):
        marker = Marker()
        marker.header.stamp = stamp.to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "task2_opponent_vessel"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_line_width_m
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.05
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.3).to_msg()

        for x, y in vertices + vertices[:1]:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            marker.points.append(point)
        self.marker_pub.publish(marker)

    def make_gps_sphere(self, stamp, marker_id, x, y, red, green, blue):
        marker = Marker()
        marker.header.stamp = stamp.to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "task2_gps_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.2
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.5).to_msg()
        return marker

    def make_gps_label(self, stamp, marker_id, x, y, text):
        marker = Marker()
        marker.header.stamp = stamp.to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "task2_gps_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.55
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        marker.lifetime = Duration(seconds=0.5).to_msg()
        return marker

    def publish_gps_markers(self, stamp):
        markers = MarkerArray()

        # GPS point 5
        markers.markers.append(
            self.make_gps_sphere(
                stamp,
                0,
                self.gps_goal_x,
                self.gps_goal_y,
                0.2,
                0.8,
                1.0,
            )
        )
        markers.markers.append(
            self.make_gps_label(
                stamp,
                1,
                self.gps_goal_x,
                self.gps_goal_y + 0.9,
                self.goal_label,
            )
        )

        # GPS point 6
        markers.markers.append(
            self.make_gps_sphere(
                stamp,
                2,
                self.gps_start_x,
                self.gps_start_y,
                1.0,
                0.8,
                0.1,
            )
        )
        markers.markers.append(
            self.make_gps_label(
                stamp,
                3,
                self.gps_start_x,
                self.gps_start_y + 0.9,
                self.start_label,
            )
        )

        self.gps_marker_pub.publish(markers)

    def on_timer(self):
        stamp = self.get_clock().now()
        elapsed_s = (stamp - self.start_time).nanoseconds * 1e-9
        pose = self.compute_pose(elapsed_s)
        vertices = self.transformed_vertices(pose)
        self.publish_tf(stamp, pose)
        self.publish_footprint(stamp, vertices)
        self.publish_marker(stamp, vertices)
        self.publish_gps_markers(stamp)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentVesselNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
