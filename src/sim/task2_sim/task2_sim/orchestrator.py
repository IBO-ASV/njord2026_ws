import math

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class Task2Orchestrator(Node):
    def __init__(self):
        super().__init__("task2_orchestrator")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("motion_amp", 0.7)
        self.declare_parameter("motion_freq_hz", 0.15)
        self.declare_parameter("goal_xy", [30.0, 0.0])
        self.declare_parameter("goal_radius", 2.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("projection_xywh", [15.0, 0.0, 8.0, 5.0])

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.motion_amp = self.get_parameter("motion_amp").get_parameter_value().double_value
        self.motion_freq_hz = self.get_parameter("motion_freq_hz").get_parameter_value().double_value
        self.goal_xy = list(self.get_parameter("goal_xy").get_parameter_value().double_array_value)
        self.goal_radius = self.get_parameter("goal_radius").get_parameter_value().double_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.projection_xywh = list(self.get_parameter("projection_xywh").get_parameter_value().double_array_value)

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_start = self.create_publisher(Bool, "/sim/start", transient_qos)
        self.pub_goal = self.create_publisher(Bool, "/sim/goal_reached", transient_qos)
        self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", transient_qos)
        self.pub_projection = self.create_publisher(PolygonStamped, "/sim/marker_vessel_projection", 10)

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        self.goal_announced = False
        self.publish_start_and_goal()

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

    def publish_start_and_goal(self):
        start_msg = Bool()
        start_msg.data = True
        self.pub_start.publish(start_msg)

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_xy[0]
        goal.pose.position.y = self.goal_xy[1]
        goal.pose.orientation.w = 1.0
        self.pub_goal_pose.publish(goal)

    def publish_projection(self):
        cx, cy, w, h = self.projection_xywh
        poly = PolygonStamped()
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.header.frame_id = self.frame_id
        corners = [
            (cx - w / 2.0, cy - h / 2.0),
            (cx + w / 2.0, cy - h / 2.0),
            (cx + w / 2.0, cy + h / 2.0),
            (cx - w / 2.0, cy + h / 2.0),
        ]
        for x, y in corners:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            poly.polygon.points.append(p)
        self.pub_projection.publish(poly)

    def on_timer(self):
        self.publish_projection()

    def on_odom(self, msg: Odometry):
        if self.goal_announced:
            return
        dx = msg.pose.pose.position.x - self.goal_xy[0]
        dy = msg.pose.pose.position.y - self.goal_xy[1]
        if math.hypot(dx, dy) <= self.goal_radius:
            done = Bool()
            done.data = True
            self.pub_goal.publish(done)
            self.goal_announced = True
            self.get_logger().info("Goal area reached")


def main(args=None):
    rclpy.init(args=args)
    node = Task2Orchestrator()
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
