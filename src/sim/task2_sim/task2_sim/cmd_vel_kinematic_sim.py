"""Minimal Task2 plant: integrate the smoothed velocity command directly."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .kinematics import integrate_pose


class CmdVelKinematicSim(Node):
    def __init__(self):
        super().__init__("cmd_vel_kinematic_sim")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.5)

        self.x = self.y = self.yaw = 0.0
        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_tick = self.get_clock().now()
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._on_cmd, 10
        )
        self._publish_static_map_to_odom()
        rate = float(self.get_parameter("update_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def _publish_static_map_to_odom(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(tf)

    def _on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        timeout = float(self.get_parameter("command_timeout_sec").value)
        cmd = self.cmd if (now - self.last_cmd_time).nanoseconds * 1e-9 <= timeout else Twist()

        self.x, self.y, self.yaw = integrate_pose(
            self.x,
            self.y,
            self.yaw,
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z,
            dt,
        )
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        stamp = now.to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_pub.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist = cmd
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelKinematicSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
