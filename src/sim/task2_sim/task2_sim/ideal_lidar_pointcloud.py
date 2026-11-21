import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PolygonStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class IdealLidarPointcloudNode(Node):
    def __init__(self):
        super().__init__("ideal_lidar_pointcloud_node")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.target_frame = self.declare_parameter("target_frame", "livox_frame").value
        input_topic = self.declare_parameter("input_topic", "/sim/opponent_vessel_footprint").value
        output_topic = self.declare_parameter("output_topic", "/pointcloud").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.point_z_m = float(self.declare_parameter("point_z_m", 0.0).value)
        self.min_range_m = float(self.declare_parameter("min_range_m", 0.2).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 80.0).value)
        self.min_angle_deg = float(self.declare_parameter("min_angle_deg", -180.0).value)
        self.max_angle_deg = float(self.declare_parameter("max_angle_deg", 180.0).value)
        self.angle_increment_deg = float(self.declare_parameter("angle_increment_deg", 0.1).value)
        self.tf_timeout_s = float(self.declare_parameter("tf_timeout_s", 0.02).value)

        self.latest_polygon = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub_footprint = self.create_subscription(
            PolygonStamped, input_topic, self.on_footprint, 10
        )
        self.pub_pointcloud = self.create_publisher(PointCloud2, output_topic, 10)

        period_s = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period_s, self.on_timer)
        self.get_logger().info(
            "Ideal LiDAR raycast pointcloud: rays from %s, angle=[%.1f, %.1f] deg, step=%.3f deg"
            % (
                self.target_frame,
                self.min_angle_deg,
                self.max_angle_deg,
                self.angle_increment_deg,
            )
        )

    def on_footprint(self, msg):
        self.latest_polygon = msg

    def polygon_vertices(self):
        if self.latest_polygon is None or len(self.latest_polygon.polygon.points) < 2:
            return []
        return [(float(point.x), float(point.y)) for point in self.latest_polygon.polygon.points]

    def transform_point(self, transform, x, y, z):
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        xx = q.x * q.x
        yy = q.y * q.y
        zz = q.z * q.z
        xy = q.x * q.y
        xz = q.x * q.z
        yz = q.y * q.z
        wx = q.w * q.x
        wy = q.w * q.y
        wz = q.w * q.z

        rx = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        ry = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        rz = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z
        return rx + tx, ry + ty, rz + tz

    @staticmethod
    def cross(ax, ay, bx, by):
        return ax * by - ay * bx

    def ray_segment_intersection(self, ray_angle, start, end):
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        sx = end[0] - start[0]
        sy = end[1] - start[1]
        denom = self.cross(dx, dy, sx, sy)
        if abs(denom) < 1e-9:
            return None

        # Ray is p = t * d. Segment is q = start + u * s.
        t = self.cross(start[0], start[1], sx, sy) / denom
        u = self.cross(start[0], start[1], dx, dy) / denom
        if t < self.min_range_m or t > self.max_range_m or u < 0.0 or u > 1.0:
            return None
        return t

    def raycast_polygon(self, vertices):
        if len(vertices) < 2:
            return []

        angle_step = max(1e-4, math.radians(abs(self.angle_increment_deg)))
        min_angle = math.radians(self.min_angle_deg)
        max_angle = math.radians(self.max_angle_deg)
        if max_angle < min_angle:
            min_angle, max_angle = max_angle, min_angle

        points = []
        ray_count = int(math.floor((max_angle - min_angle) / angle_step)) + 1
        for ray_index in range(ray_count):
            angle = min_angle + angle_step * ray_index
            closest_range = None
            for edge_index, start in enumerate(vertices):
                end = vertices[(edge_index + 1) % len(vertices)]
                hit_range = self.ray_segment_intersection(angle, start, end)
                if hit_range is not None and (
                    closest_range is None or hit_range < closest_range
                ):
                    closest_range = hit_range

            if closest_range is not None:
                points.append(
                    (
                        closest_range * math.cos(angle),
                        closest_range * math.sin(angle),
                        self.point_z_m,
                    )
                )
        return points

    def publish_cloud(self, stamp, points):
        header = Header()
        header.stamp = stamp.to_msg()
        header.frame_id = self.target_frame
        self.pub_pointcloud.publish(point_cloud2.create_cloud_xyz32(header, points))

    def on_timer(self):
        stamp = self.get_clock().now()
        if self.latest_polygon is None:
            self.publish_cloud(stamp, [])
            return

        source_frame = self.latest_polygon.header.frame_id or self.map_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as ex:
            self.get_logger().warn(
                "Could not transform opponent footprint to %s: %s" % (self.target_frame, ex),
                throttle_duration_sec=2.0,
            )
            return

        transformed_vertices = [
            self.transform_point(transform, x, y, 0.0)[:2] for x, y in self.polygon_vertices()
        ]
        self.publish_cloud(stamp, self.raycast_polygon(transformed_vertices))


def main(args=None):
    rclpy.init(args=args)
    node = IdealLidarPointcloudNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
