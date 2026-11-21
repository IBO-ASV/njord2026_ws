#!/usr/bin/env python3
"""Valid UM982 ENU body heading for navsat_transform, orientation only."""
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
class HeadingImu(Node):
    def __init__(self):
        super().__init__('gnss_heading_imu')
        self.yaw_variance = self.declare_parameter('yaw_variance', 0.0).value
        self.position_variances = self.declare_parameter('position_variances', [0.0, 0.0, 0.0]).value
        self.publisher = self.create_publisher(Imu, '/localization/gnss_heading_imu', 10)
        self.fix_publisher = self.create_publisher(NavSatFix, '/localization/gnss_fix_calibrated', 10)
        self.create_subscription(NavSatFix, '/sensor/vehicle_gnss/fix/raw', self.fix, 10)
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
        out.orientation_covariance[8] = max(variance, getattr(self, "yaw_variance", 0.0), 1e-6)
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(out)
    def fix(self, message):
        if message.status.status < 0:
            self.fix_publisher.publish(message)
            return
        variances = self.position_variances
        if len(variances) != 3 or any(not math.isfinite(v) or v <= 0 for v in variances):
            self.fix_publisher.publish(message)
            return
        # Preserve receiver uncertainty when it reports a less certain solution.
        covariance = message.position_covariance
        valid_cov = message.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
        diagonal = [max(variances[i], covariance[4*i])
                    if valid_cov and math.isfinite(covariance[4*i]) and covariance[4*i] >= 0
                    else variances[i] for i in range(3)]
        message.position_covariance = [diagonal[0], 0., 0., 0., diagonal[1], 0., 0., 0., diagonal[2]]
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_publisher.publish(message)
def main():
    rclpy.init(); node = HeadingImu()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
if __name__ == '__main__': main()
