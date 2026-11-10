import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from njord_interfaces.msg import MissionStatus
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException

from .voxel import cloud_points, voxelize


FIELDS = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(("x", "y", "z", "intensity"))]


def make_cloud(stamp, frame, points):
    message = PointCloud2()
    message.header.stamp, message.header.frame_id = stamp, frame
    message.height, message.width, message.fields = 1, len(points), FIELDS
    message.is_bigendian, message.point_step, message.row_step, message.is_dense = False, 16, 16 * len(points), True
    message.data = b"".join(struct.pack("<ffff", *point) for point in points)
    return message


def transform_point(point, transform):
    # Quaternion rotation plus translation; avoids a PCL dependency on the Jetson.
    x, y, z, intensity = point
    q = transform.transform.rotation
    tx, ty, tz = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
    ix = q.w*x + q.y*z - q.z*y; iy = q.w*y + q.z*x - q.x*z; iz = q.w*z + q.x*y - q.y*x; iw = -q.x*x - q.y*y - q.z*z
    return (ix*q.w + iw*-q.x + iy*-q.z - iz*-q.y + tx, iy*q.w + iw*-q.y + iz*-q.x - ix*-q.z + ty, iz*q.w + iw*-q.z + ix*-q.y - iy*-q.x + tz, intensity)


class Downsampler(Node):
    def __init__(self):
        super().__init__("livox_gui_downsampler")
        self.declare_parameter("voxel_size_m", .75); self.declare_parameter("max_points", 2000); self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("input_topic", "/livox/lidar"); self.declare_parameter("output_topic", "/gui/livox/points")
        self.voxel = self.get_parameter("voxel_size_m").value; self.limit = self.get_parameter("max_points").value
        self.period = 1.0 / self.get_parameter("publish_rate_hz").value; self.last = 0
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, 1)
        self.create_subscription(PointCloud2, self.get_parameter("input_topic").value, self.callback, qos_profile_sensor_data)

    def callback(self, message):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last < self.period: return
        try: transform = self.tf.lookup_transform("base_link", message.header.frame_id, Time.from_msg(message.header.stamp), timeout=Duration(seconds=.05))
        except TransformException as error:
            self.get_logger().warning("GUI Livox scan skipped: base_link TF unavailable (%s)" % error, throttle_duration_sec=5.0); return
        filtered = []
        for point in cloud_points(message):
            point = transform_point(point, transform); distance = math.sqrt(sum(v*v for v in point[:3]))
            # Hull exclusion is intentionally conservative: the vessel footprint is 2 x 1.2 m.
            if 1.0 <= distance <= 50.0 and not (-1.5 < point[0] < 1.5 and -0.9 < point[1] < .9 and -.8 < point[2] < 1.5): filtered.append(point)
        self.pub.publish(make_cloud(message.header.stamp, "base_link", voxelize(filtered, self.voxel, self.limit))); self.last = now


class Mapper(Node):
    def __init__(self):
        super().__init__("livox_splat_mapper")
        self.declare_parameter("voxel_size_m", .75); self.declare_parameter("max_points", 50000)
        self.declare_parameter("input_topic", "/gui/livox/points"); self.declare_parameter("output_topic", "/gui/livox/splat_map"); self.declare_parameter("mission_status_topic", "/mission/status")
        self.voxel = self.get_parameter("voxel_size_m").value; self.limit = self.get_parameter("max_points").value; self.map = {}; self.execution_id = ""
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, 1)
        self.create_subscription(PointCloud2, self.get_parameter("input_topic").value, self.callback, 10)
        self.create_subscription(MissionStatus, self.get_parameter("mission_status_topic").value, self.status, 10)
        self.create_timer(1.0, self.publish)

    def status(self, message):
        if message.execution_id and message.execution_id != self.execution_id:
            self.execution_id, self.map = message.execution_id, {}; self.get_logger().info("GUI splat map reset for execution %s" % self.execution_id)

    def callback(self, message):
        try: transform = self.tf.lookup_transform("odom", "base_link", Time.from_msg(message.header.stamp), timeout=Duration(seconds=.05))
        except TransformException as error:
            self.get_logger().warning("GUI Livox scan not mapped: odom TF unavailable (%s)" % error, throttle_duration_sec=5.0); return
        for point in cloud_points(message):
            point = transform_point(point, transform); self.map[tuple(math.floor(v / self.voxel) for v in point[:3])] = point
        while len(self.map) > self.limit:
            self.voxel *= 2; self.map = {tuple(math.floor(v / self.voxel) for v in p[:3]): p for p in self.map.values()}
            self.get_logger().warning("GUI splat map coarsened to %.2f m" % self.voxel)

    def publish(self): self.pub.publish(make_cloud(self.get_clock().now().to_msg(), "odom", list(self.map.values())))


def downsampler_main(): rclpy.init(); node = Downsampler(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
def mapper_main(): rclpy.init(); node = Mapper(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
