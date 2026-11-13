"""Deterministic CPU sensor/environment stand-ins; never talks to hardware."""

import math
import random
import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    Twist,
)
from lifecycle_msgs.srv import GetState
from mission_manager.geodesy import wgs84_to_enu
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, UInt8
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from njord_interfaces.msg import (
    BuoyDetection,
    BuoyDetectionArray,
    HealthSignal,
    HealthState,
)

from .scenario import dock_segments, scenario

QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def quaternion(yaw):
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class Environment(Node):
    def __init__(self):
        super().__init__("njord_sim_environment")
        self.s = scenario(self.declare_parameter("task", "task1").value)
        self.noise = self.declare_parameter("noise", False).value
        self.dropout_after = self.declare_parameter("dropout_after", 0.0).value
        self.seed = self.declare_parameter("seed", 2026).value
        self.model = self.declare_parameter("model", "kinematic").value
        self.encounter = self.declare_parameter("encounter", "head_on").value
        self.external_force = self.declare_parameter("disturbance", False).value
        self.rng = random.Random(self.seed)
        self.x, self.y, self.yaw = self.s.initial
        self.u = self.v = self.r = 0.0
        self.bias = [0.0, 0.0, 0.0]
        self.command = Twist()
        self.last_command = 0.0
        self.started = time.monotonic()
        self.last = self.started
        self.mission_start = None
        self.pose_pub = self.create_publisher(Odometry, "/sim/truth/odom", 10)
        self.estimate = self.create_publisher(Odometry, "/odometry/filtered/local", 10)
        self.global_est = self.create_publisher(
            Odometry, "/odometry/filtered/global", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.gnss = self.create_publisher(NavSatFix, "/sensor/vehicle_gnss/fix/raw", 10)
        self.heading = self.create_publisher(
            PoseWithCovarianceStamped, "/sensor/vehicle_gnss/compass/raw", 10
        )
        self.imu = self.create_publisher(Imu, "/wit/imu", 10)
        self.buoys = self.create_publisher(
            BuoyDetectionArray, "/buoy_detections_3d", 10
        )
        self.buoy_points = (
            self.create_publisher(PointStamped, "/buoy_detections", 10)
            if self.s.task.task_id == "task2" else None
        )
        self.cloud = self.create_publisher(PointCloud2, "/sim_obstacles", 10)
        self.lidar = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        self.ready = self.create_publisher(Bool, "/autonomy/ready", 10)
        self.perception_ready = self.create_publisher(
            Bool, "/mission/readiness/buoy_perception", QOS
        )
        self.estop = self.create_publisher(UInt8, "/safety/emergency_stop", 10)
        self.health = self.create_publisher(HealthState, "/health/state", QOS)
        self.battery = self.create_publisher(Float32, "/gui/battery_percent", 10)
        self.ground_speed = self.create_publisher(Float32, "/gui/ground_speed_mps", 10)
        self.actual_path = Path()
        self.actual_path.header.frame_id = "odom"
        self.path_pub = self.create_publisher(Path, "/sim/estimated_path", 10)
        self.geo_markers = self.create_publisher(
            MarkerArray, "/ground_waypoint_markers", QOS
        )
        self.preview_markers = self.create_publisher(
            MarkerArray, "/sim/waypoint_markers", QOS
        )
        self.publish_waypoint_previews()
        self.create_timer(1.0, self.publish_waypoint_previews)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        static = TransformStamped()
        static.header.frame_id = "map"
        static.child_frame_id = "odom"
        static.transform.rotation.w = 1.0
        static.header.stamp = self.get_clock().now().to_msg()
        self.static.sendTransform(static)
        self.nav = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.lifecycle = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in ("controller_server", "bt_navigator", "collision_monitor")
        }
        self.lifecycle_states = {}
        self.lifecycle_pending = {}
        self.create_timer(0.5, self.poll_lifecycle)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        if self.model == "mmg":
            self.create_subscription(Odometry, "/sim/physics/odom", self.on_physics, 10)
        self.create_service(FromLL, "/fromLL", self.from_ll)
        self.filtered_cloud_at = None
        self.planned_path_at = None
        if self.s.task.task_id == "task2":
            self.other_truth = self.create_publisher(Odometry, "/sim/truth/opponent", 10)
            self.create_subscription(Bool, "/mission/task2/enabled", self.on_enabled, QOS)
            self.create_subscription(Path, "/planned_path_pruned", self.on_planned_path, 10)
            self.create_subscription(
                PointCloud2, "/task2/points_filtered", self.on_filtered_cloud, 10
            )
        self.tick_count = 0
        self.create_timer(0.05, self.tick)

    def publish_waypoint_previews(self):
        """Display the selected sim route before, during and after a mission."""
        # Replace the previous route, including extra IDs from another sim
        # task that a viewer may have displayed before reconnecting.
        geographic = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        local = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        stamp = self.get_clock().now().to_msg()
        route = Marker()
        route.header = Header(stamp=stamp, frame_id="odom")
        route.ns, route.id = "sim_waypoint_route", 0
        route.type, route.action = Marker.LINE_STRIP, Marker.ADD
        route.pose.orientation.w = 1.0
        route.scale.x = 0.1
        route.color.g, route.color.b, route.color.a = 0.8, 1.0, 0.8
        for i, waypoint in enumerate(self.s.route.waypoints):
            marker = Marker()
            marker.header = Header(stamp=stamp, frame_id="wgs84")
            marker.ns, marker.id = f"ground_waypoint_wgs84/{self.s.task.task_id}", i
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = waypoint.longitude, waypoint.latitude
            marker.pose.orientation.w = 1.0
            marker.text = waypoint.name
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.g, marker.color.b, marker.color.a = 0.9, 1.0, 1.0
            geographic.markers.append(marker)
            point = deepcopy(marker)
            point.header.frame_id = "odom"
            point.ns = "sim_waypoints"
            point.pose.position.x, point.pose.position.y = waypoint.x, waypoint.y
            point.pose.position.z = 0.2
            point.scale.x = point.scale.y = point.scale.z = 0.45
            local.markers.append(point)
            route.points.append(deepcopy(point.pose.position))
            label = deepcopy(point)
            label.ns, label.type = "sim_waypoint_labels", Marker.TEXT_VIEW_FACING
            label.pose.position.z = 1.0 + (i % 3) * 0.35
            label.scale.z = 0.7
            label.text = f"{i + 1:02d} {waypoint.name}"
            local.markers.append(label)
        local.markers.append(route)
        self.geo_markers.publish(geographic)
        self.preview_markers.publish(local)

    def on_planned_path(self, message):
        if len(message.poses) >= 2:
            self.planned_path_at = time.monotonic()

    def on_filtered_cloud(self, message):
        if message.header.frame_id and (
            message.header.stamp.sec or message.header.stamp.nanosec
        ):
            self.filtered_cloud_at = time.monotonic()

    def poll_lifecycle(self):
        for name, client in self.lifecycle.items():
            pending = self.lifecycle_pending.get(name)
            if pending is not None and not pending.done():
                continue
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                self.lifecycle_pending[name] = future

                def done(result, name=name):
                    try:
                        self.lifecycle_states[name] = (
                            result.result().current_state.id,
                            time.monotonic(),
                        )
                    except Exception:
                        self.lifecycle_states.pop(name, None)

                future.add_done_callback(done)

    def on_enabled(self, message):
        if message.data and self.mission_start is None:
            self.mission_start = time.monotonic()

    def from_ll(self, request, response):
        response.map_point.x, response.map_point.y = wgs84_to_enu(
            request.ll_point.latitude, request.ll_point.longitude, self.s.datum
        )
        return response

    def on_command(self, msg):
        self.command = msg
        self.last_command = time.monotonic()

    def on_physics(self, msg):
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        self.u, self.v, self.r = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )

    def tick(self):
        now = time.monotonic()
        dt = min(0.2, now - self.last)
        self.last = now
        cmd = self.command if now - self.last_command < 0.5 else Twist()
        if self.model == "kinematic":
            # Velocity response includes inertia; this is explicitly not MMG.
            alpha = 1 - math.exp(-dt / 0.3)
            self.u += alpha * (cmd.linear.x - self.u)
            self.v += alpha * (cmd.linear.y - self.v)
            self.r += alpha * (cmd.angular.z - self.r)
            drift = 0.03 * math.sin(now - self.started) if self.external_force else 0.0
            self.x += (
                self.u * math.cos(self.yaw) - self.v * math.sin(self.yaw) + drift
            ) * dt
            self.y += (self.u * math.sin(self.yaw) + self.v * math.cos(self.yaw)) * dt
            self.yaw += self.r * dt
        stamp = self.get_clock().now().to_msg()
        truth = Odometry()
        truth.header = Header(stamp=stamp, frame_id="odom")
        truth.child_frame_id = "base_link"
        truth.pose.pose.position.x = self.x
        truth.pose.pose.position.y = self.y
        truth.pose.pose.orientation = quaternion(self.yaw)
        truth.twist.twist.linear.x = self.u
        truth.twist.twist.linear.y = self.v
        truth.twist.twist.angular.z = self.r
        self.pose_pub.publish(truth)
        if self.dropout_after and now - self.started >= self.dropout_after:
            health = HealthState(
                stamp=stamp, summary_state=4, message="SIMULATED sensor loss"
            )
            health.signals = [
                HealthSignal(name=n, state=4, message="SIMULATED loss", stamp=stamp)
                for n in ("ground_station_heartbeat", "local_odometry", "gnss")
            ]
            self.health.publish(health)
            self.perception_ready.publish(Bool(data=False))
            return
        estimate = deepcopy(truth)
        if self.noise:
            for i in range(3):
                self.bias[i] += self.rng.gauss(0, 0.001 * math.sqrt(dt))
            estimate.pose.pose.position.x += self.bias[0] + self.rng.gauss(0, 0.02)
            estimate.pose.pose.position.y += self.bias[1] + self.rng.gauss(0, 0.02)
            estimate.pose.pose.orientation = quaternion(
                self.yaw + self.bias[2] + self.rng.gauss(0, 0.005)
            )
        estimate.pose.covariance[0] = estimate.pose.covariance[7] = (
            0.0004 if self.noise else 1e-8
        )
        estimate.pose.covariance[35] = 0.000025 if self.noise else 1e-8
        self.estimate.publish(estimate)
        global_odom = deepcopy(estimate)
        global_odom.header.frame_id = "map"
        self.global_est.publish(global_odom)
        self.odom_pub.publish(estimate)
        tf = TransformStamped()
        tf.header = estimate.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = estimate.pose.pose.position.x
        tf.transform.translation.y = estimate.pose.pose.position.y
        tf.transform.rotation = estimate.pose.pose.orientation
        self.tf.sendTransform(tf)
        imu = Imu()
        imu.header = Header(stamp=stamp, frame_id="base_link")
        imu.orientation = quaternion(
            self.yaw + self.rng.gauss(0, 0.01) if self.noise else self.yaw
        )
        imu.angular_velocity.z = self.r + (
            self.rng.gauss(0, 0.003464) if self.noise else 0.0
        )
        imu.linear_acceleration.z = 9.80665
        imu.orientation_covariance[8] = 0.0001
        imu.angular_velocity_covariance[8] = 0.003464**2
        self.imu.publish(imu)
        self.tick_count += 1
        if self.tick_count % 10 == 0:
            pose = PoseStamped(header=estimate.header, pose=estimate.pose.pose)
            self.actual_path.poses.append(pose)
            self.actual_path.poses = self.actual_path.poses[-2400:]
            self.actual_path.header.stamp = stamp
            self.path_pub.publish(self.actual_path)
        if self.tick_count % 2:
            return
        self.publish_sensors(stamp, now - self.started)

    def publish_sensors(self, stamp, elapsed):
        fix = NavSatFix()
        fix.header = Header(stamp=stamp, frame_id="base_link")
        fix.status.status = 0
        nx = self.rng.gauss(0, 0.1) if self.noise else 0.0
        ny = self.rng.gauss(0, 0.1) if self.noise else 0.0
        fix.latitude = self.s.datum.latitude + math.degrees((self.y + ny) / 6378137.0)
        fix.longitude = self.s.datum.longitude + math.degrees(
            (self.x + nx) / (6378137.0 * math.cos(math.radians(self.s.datum.latitude)))
        )
        fix.position_covariance[0] = fix.position_covariance[4] = 0.01
        fix.position_covariance_type = 2
        self.gnss.publish(fix)
        compass = PoseWithCovarianceStamped()
        compass.header = Header(stamp=stamp, frame_id="base_link")
        compass.pose.pose.orientation = quaternion(
            self.yaw + (self.rng.gauss(0, 0.01745) if self.noise else 0.0)
        )
        compass.pose.covariance[35] = 0.01745**2
        self.heading.publish(compass)
        detections = BuoyDetectionArray()
        detections.header = Header(stamp=stamp, frame_id="map")
        points = []
        classes = {"RED": 1, "GREEN": 0, "N": 2, "E": 3, "S": 4, "W": 5}
        for x, y, mark in self.s.buoys:
            distance = math.hypot(x - self.x, y - self.y)
            bearing = math.atan2(y - self.y, x - self.x) - self.yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if distance < 30.0 and abs(bearing) < math.radians(75):
                d = BuoyDetection(
                    class_id=classes[mark], confidence=0.95, position_source=1
                )
                d.position.x = x + (self.rng.gauss(0, 0.04) if self.noise else 0.0)
                d.position.y = y + (self.rng.gauss(0, 0.04) if self.noise else 0.0)
                detections.detections.append(d)
                if self.buoy_points is not None:
                    point = PointStamped(header=detections.header, point=d.position)
                    self.buoy_points.publish(point)
            points.extend(
                (x + 0.175 * math.cos(a * 0.2), y + 0.175 * math.sin(a * 0.2), 0.3)
                for a in range(32)
            )
        self.buoys.publish(detections)
        for dock in self.s.docks:
            for a, b in dock_segments(dock):
                n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.1))
                points.extend(
                    (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, 0.3)
                    for i in range(n + 1)
                )
        if self.s.task.task_id == "task2" and self.encounter != "none":
            a, b = self.s.route.waypoints[0], self.s.route.waypoints[-1]
            course = math.atan2(b.y - a.y, b.x - a.x)
            theta = course + {
                "head_on": math.pi,
                "port": -math.pi / 2,
                "starboard": math.pi / 2,
            }.get(self.encounter, math.pi)
            t = (
                max(0.0, time.monotonic() - self.mission_start)
                if self.mission_start
                else 0.0
            )
            meeting = math.hypot(b.x - a.x, b.y - a.y) / 2 / 1.0288888889
            mx, my = (a.x + b.x) / 2, (a.y + b.y) / 2
            ox = mx + (t - meeting) * 1.2861111111 * math.cos(theta)
            oy = my + (t - meeting) * 1.2861111111 * math.sin(theta)
            other = Odometry()
            other.header = Header(stamp=stamp, frame_id="odom")
            other.child_frame_id = "sim_opponent"
            other.pose.pose.position.x = ox
            other.pose.pose.position.y = oy
            other.pose.pose.orientation = quaternion(theta)
            other.twist.twist.linear.x = 1.2861111111
            self.other_truth.publish(other)
            for u in range(11):
                for v in range(7):
                    for z in (0.25, 0.5, 0.75, 1.0):
                        if u not in (0, 10) and v not in (0, 6) and z != 1.0:
                            continue
                        du, dv = (u - 5) * 0.2, (v - 3) * 0.18
                        points.append(
                            (
                                ox + du * math.cos(theta) - dv * math.sin(theta),
                                oy + du * math.sin(theta) + dv * math.cos(theta),
                                z,
                            )
                        )
            # Water returns let the real RANSAC ground-remover estimate a plane.
            points.extend(
                (self.x + i * 2.0, self.y + j * 2.0, 0.0)
                for i in range(-12, 13)
                for j in range(-12, 13)
            )
        header = Header(stamp=stamp, frame_id="map")
        self.cloud.publish(point_cloud2.create_cloud_xyz32(header, points))
        # Simulated LiDAR uses the sensor/base frame and range gating.
        local = []
        for x, y, z in points:
            dx, dy = x - self.x, y - self.y
            if math.hypot(dx, dy) < 40.0:
                local.append(
                    (
                        dx * math.cos(self.yaw) + dy * math.sin(self.yaw),
                        -dx * math.sin(self.yaw) + dy * math.cos(self.yaw),
                        z,
                    )
                )
        self.lidar.publish(
            point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id="base_link"), local
            )
        )
        self.estop.publish(UInt8(data=0))
        self.ready.publish(
            Bool(
                data=self.nav.server_is_ready()
                and (
                    self.s.task.task_id != "task2"
                    or self.planned_path_at is not None
                    and time.monotonic() - self.planned_path_at < 5.0
                )
                and all(
                    self.lifecycle_states.get(n, (0, 0))[0] == 3
                    and time.monotonic() - self.lifecycle_states.get(n, (0, 0))[1] < 2.0
                    for n in self.lifecycle
                )
            )
        )
        processing = self.s.task.task_id != "task2" or (
            self.filtered_cloud_at is not None
            and time.monotonic() - self.filtered_cloud_at < 1.0
        )
        self.perception_ready.publish(Bool(data=processing))
        health = HealthState(stamp=stamp, summary_state=1, message="SIMULATED health")
        health.signals = [
            HealthSignal(name=n, state=1, message="SIMULATED", stamp=stamp)
            for n in ("ground_station_heartbeat", "local_odometry", "gnss")
        ]
        self.health.publish(health)
        self.battery.publish(Float32(data=80.0))
        self.ground_speed.publish(Float32(data=math.hypot(self.u, self.v)))


def main(args=None):
    rclpy.init(args=args)
    node = Environment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
