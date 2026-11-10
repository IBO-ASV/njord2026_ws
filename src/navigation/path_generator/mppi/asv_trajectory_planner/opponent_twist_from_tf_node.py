#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OpponentTwistFromTfNode(Node):
    """
    opponent_vessel TFから /other_ship/twist をTF差分で推定する。

    - map -> opponent_vessel TFが存在する場合のみpublish
    - TFが存在しない場合はpublishしない
    - planner側は /other_ship/twist がなければ他船なしとして扱う
    """

    def __init__(self):
        super().__init__("opponent_twist_from_tf_node")

        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "opponent_vessel")
        self.declare_parameter("twist_topic", "/other_ship/twist")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_dt_s", 1.0)
        self.declare_parameter("lowpass_alpha", 0.3)
        self.declare_parameter("max_speed_mps", 5.0)

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.twist_topic = self.get_parameter("twist_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.max_dt_s = float(self.get_parameter("max_dt_s").value)
        self.lowpass_alpha = float(self.get_parameter("lowpass_alpha").value)
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(TwistStamped, self.twist_topic, 10)

        self.prev_t = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None

        self.filt_vx = None
        self.filt_vy = None
        self.filt_wz = None

        timer_period = 1.0 / max(publish_rate_hz, 1e-6)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("opponent_twist_from_tf_node started.")
        self.get_logger().info(f"TF diff: {self.parent_frame} -> {self.child_frame}")
        self.get_logger().info(f"Publish: {self.twist_topic}")

    def timer_callback(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                Time(),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Opponent TF not available. Planning can continue without opponent. reason={e}",
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        tr = tf_msg.transform.translation
        rot = tf_msg.transform.rotation

        x = tr.x
        y = tr.y
        yaw = yaw_from_quaternion(rot)

        if self.prev_t is None:
            self.prev_t = t
            self.prev_x = x
            self.prev_y = y
            self.prev_yaw = yaw
            return

        dt = t - self.prev_t

        if dt <= 1e-6 or dt > self.max_dt_s:
            self.prev_t = t
            self.prev_x = x
            self.prev_y = y
            self.prev_yaw = yaw
            return

        raw_vx = (x - self.prev_x) / dt
        raw_vy = (y - self.prev_y) / dt
        raw_wz = normalize_angle(yaw - self.prev_yaw) / dt

        speed = math.hypot(raw_vx, raw_vy)
        if speed > self.max_speed_mps:
            self.get_logger().warn(
                f"Ignore opponent twist spike: speed={speed:.2f} m/s",
                throttle_duration_sec=2.0,
            )
            self.prev_t = t
            self.prev_x = x
            self.prev_y = y
            self.prev_yaw = yaw
            return

        alpha = min(max(self.lowpass_alpha, 0.0), 1.0)

        if self.filt_vx is None:
            self.filt_vx = raw_vx
            self.filt_vy = raw_vy
            self.filt_wz = raw_wz
        else:
            self.filt_vx = alpha * raw_vx + (1.0 - alpha) * self.filt_vx
            self.filt_vy = alpha * raw_vy + (1.0 - alpha) * self.filt_vy
            self.filt_wz = alpha * raw_wz + (1.0 - alpha) * self.filt_wz

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.parent_frame
        msg.twist.linear.x = float(self.filt_vx)
        msg.twist.linear.y = float(self.filt_vy)
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(self.filt_wz)

        self.pub.publish(msg)

        self.prev_t = t
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = OpponentTwistFromTfNode()

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
