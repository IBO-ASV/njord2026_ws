"""Convert GLIM IMU pose/world velocity into planar base-link odometry.

For the workspace's existing base->livox extrinsic. Angular velocity is
estimated from stamped base yaw because the installed GLIM publisher leaves
angular twist zero. Assumed fallback covariance is for offline checks only.
"""
import math
import numpy as np
from .cloud_ops import quaternion_to_rotation_matrix


def convert_state(position, quaternion, world_velocity, yaw_rate):
    q = np.asarray(quaternion, dtype=float)
    q /= np.linalg.norm(q)
    rotation = quaternion_to_rotation_matrix(*q)
    # base -> livox: translation (0.5, 0, 0.8), roll pi.
    sensor_to_base_rotation = np.diag([1.0, -1.0, -1.0])
    sensor_to_base_offset = -sensor_to_base_rotation @ np.array([0.5, 0.0, 0.8])
    base_rotation = rotation @ sensor_to_base_rotation
    lever_world = rotation @ sensor_to_base_offset
    base_position = np.asarray(position) + lever_world
    yaw = math.atan2(base_rotation[1, 0], base_rotation[0, 0])
    base_world_velocity = np.asarray(world_velocity) + np.cross([0, 0, yaw_rate], lever_world)
    c, s = math.cos(yaw), math.sin(yaw)
    velocity = np.array([c*base_world_velocity[0]+s*base_world_velocity[1],
                         -s*base_world_velocity[0]+c*base_world_velocity[1], 0.0])
    return base_position, yaw, velocity


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry

    class Adapter(Node):
        def __init__(self):
            super().__init__('glim_base_odom_adapter')
            self.last = None
            self.pub = self.create_publisher(Odometry, '/glim_base/odom', 20)
            self.sub = self.create_subscription(Odometry, '/glim_ros/odom', self.callback, qos_profile_sensor_data)

        def callback(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z]
            if not all(math.isfinite(x) for x in values) or sum(x*x for x in values[3:7]) < 1e-12:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            position, yaw, _ = convert_state(values[:3], values[3:7], values[7:], 0.0)
            rate = 0.0
            if self.last is not None:
                dt = stamp - self.last[0]
                if dt <= 0:
                    self.last = None
                    return
                if dt <= 0.5:
                    delta = math.atan2(math.sin(yaw-self.last[1]), math.cos(yaw-self.last[1]))
                    rate = delta/dt
            self.last = stamp, yaw
            position, yaw, velocity = convert_state(values[:3], values[3:7], values[7:], rate)
            out = Odometry()
            out.header = msg.header
            out.child_frame_id = 'base_link'
            out.pose.pose.position.x, out.pose.pose.position.y = float(position[0]),float(position[1])
            out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
            out.twist.twist.linear.x, out.twist.twist.linear.y = float(velocity[0]),float(velocity[1])
            out.twist.twist.angular.z = rate
            for i, variance in [(0,0.0025),(7,0.0025),(14,1e3),(21,1e3),(28,1e3),(35,0.0025)]:
                out.pose.covariance[i] = variance
            for i, variance in [(0,0.01),(7,0.01),(14,1e3),(21,1e3),(28,1e3),(35,0.01)]:
                out.twist.covariance[i] = variance
            self.pub.publish(out)

    rclpy.init(args=args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
