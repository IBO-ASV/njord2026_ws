#!/usr/bin/env python3
"""Valid UM982 ENU body heading for navsat_transform, orientation only."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
class HeadingImu(Node):
    def __init__(self):
        super().__init__('gnss_heading_imu')
        self.publisher = self.create_publisher(Imu, '/localization/gnss_heading_imu', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/sensor/vehicle_gnss/compass/raw', self.heading, 10)
    def heading(self, message):
        q = message.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1) > 0.01: return
        variance = message.pose.covariance[35]
        if not math.isfinite(variance) or variance < 0: return
        out = Imu()
        out.header.stamp = message.header.stamp
        out.header.frame_id = 'base_link'
        out.orientation = q
        out.orientation_covariance[0] = out.orientation_covariance[4] = 1e6
        out.orientation_covariance[8] = max(variance, 1e-6)
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(out)
def main():
    rclpy.init(); node = HeadingImu()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == '__main__': main()
