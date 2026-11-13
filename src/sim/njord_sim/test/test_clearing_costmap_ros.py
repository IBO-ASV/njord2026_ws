"""Exercise the real Nav2 layer: clear old hulls, retain live/occluded obstacles."""

import math
import os
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from njord_sim.clearing import (
    SCAN_ANGLE_INCREMENT,
    SCAN_ANGLE_MIN,
    SCAN_BEAMS,
    SCAN_MIN_RANGE,
    SCAN_RANGE,
    clearing_ranges,
)
from njord_sim.configuration import configurations
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def test_nav2_clears_visible_old_hulls_but_preserves_obstacles_and_shadows(tmp_path):
    nav, _, _ = configurations("task2", tmp_path)
    env = {
        **os.environ,
        "ROS_DOMAIN_ID": "128",
        "ROS_LOCALHOST_ONLY": "1",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    with (tmp_path / "planner.log").open("w") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "nav2_planner",
                "planner_server",
                "--ros-args",
                "--params-file",
                nav,
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init(domain_id=128)
        node = Node("costmap_clearing_fixture")
        broadcaster = TransformBroadcaster(node)
        cloud_pub = node.create_publisher(PointCloud2, "/sim_obstacles", 10)
        scan_pub = node.create_publisher(LaserScan, "/sim/costmap_clearing_scan", 10)
        state = {
            "opponent": (8.0, 0.0, 0.0, 2.0, 1.08),
            "buoys": [(4.0, 2.0), (12.0, 0.0)],
            "extra": [],
        }
        grid = {}

        def full(message):
            grid["info"] = message.info
            grid["data"] = (
                np.asarray(message.data, dtype=np.int16)
                .reshape(message.info.height, message.info.width)
                .copy()
            )

        def update(message):
            if "data" in grid:
                grid["data"][
                    message.y : message.y + message.height,
                    message.x : message.x + message.width,
                ] = np.asarray(message.data, dtype=np.int16).reshape(
                    message.height, message.width
                )

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            OccupancyGrid, "/global_costmap/obstacle_layer", full, qos
        )
        node.create_subscription(
            OccupancyGridUpdate, "/global_costmap/obstacle_layer_updates", update, 10
        )

        def publish():
            stamp = node.get_clock().now().to_msg()
            tf = TransformStamped(
                header=Header(stamp=stamp, frame_id="odom"), child_frame_id="base_link"
            )
            tf.transform.rotation.w = 1.0
            broadcaster.sendTransform(tf)
            points = [
                (x + 0.175 * math.cos(i * 0.2), y + 0.175 * math.sin(i * 0.2), 0.3)
                for x, y in state["buoys"]
                for i in range(32)
            ]
            if state["opponent"] is not None:
                x, y, _, _, _ = state["opponent"]
                points += [
                    (x + u * 0.2, y + v * 0.18, 0.3)
                    for u in range(-5, 6)
                    for v in range(-3, 4)
                ]
            points += [(*point, 0.3) for point in state["extra"]]
            cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(
                    Header(stamp=stamp, frame_id="odom"), points
                )
            )
            scan = LaserScan(
                header=Header(stamp=stamp, frame_id="base_link"),
                angle_min=SCAN_ANGLE_MIN,
                angle_max=SCAN_ANGLE_MIN + (SCAN_BEAMS - 1) * SCAN_ANGLE_INCREMENT,
                angle_increment=SCAN_ANGLE_INCREMENT,
                range_min=SCAN_MIN_RANGE,
                range_max=SCAN_RANGE,
            )
            scan.ranges = clearing_ranges(
                (0.0, 0.0, 0.0), state["buoys"], state["opponent"]
            ).tolist()
            scan_pub.publish(scan)

        node.create_timer(0.1, publish)

        def wait(predicate, seconds=12):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                assert process.poll() is None, (tmp_path / "planner.log").read_text()
                rclpy.spin_once(node, timeout_sec=0.03)
                if predicate():
                    return
            raise AssertionError(
                "Nav2 costmap condition timed out\n"
                + (tmp_path / "planner.log").read_text()[-3000:]
            )

        def occupied(x, y, rx=0.3, ry=0.3):
            if "data" not in grid:
                return 0
            info = grid["info"]
            iy, ix = np.where(grid["data"] >= 100)
            px = info.origin.position.x + (ix + 0.5) * info.resolution
            py = info.origin.position.y + (iy + 0.5) * info.resolution
            return int(np.count_nonzero((np.abs(px - x) < rx) & (np.abs(py - y) < ry)))

        try:
            client = node.create_client(ChangeState, "/planner_server/change_state")
            wait(lambda: client.service_is_ready(), 20)
            for transition in [
                Transition.TRANSITION_CONFIGURE,
                Transition.TRANSITION_ACTIVATE,
            ]:
                future = client.call_async(
                    ChangeState.Request(transition=Transition(id=transition))
                )
                wait(lambda: future.done(), 20)
                assert future.result().success
            wait(
                lambda: (
                    occupied(8.0, 0.0, 1.1, 0.7) > 20
                    and occupied(4.0, 2.0) > 0
                    and occupied(12.0, 0.0) > 0
                )
            )
            state["opponent"] = (8.0, 5.0, 0.0, 2.0, 1.08)
            wait(
                lambda: (
                    occupied(8.0, 0.0, 1.1, 0.7) == 0
                    and occupied(8.0, 5.0, 1.1, 0.7) > 20
                )
            )
            assert occupied(4.0, 2.0) > 0 and occupied(12.0, 0.0) > 0
            # The old target is then hidden by an opaque hull. Clearing must
            # preserve that unknown/occluded cell instead of globally wiping it.
            state.update(
                opponent=(4.0, 0.0, 0.0, 2.0, 1.08), buoys=[], extra=[(8.0, 0.0)]
            )
            wait(lambda: occupied(8.0, 0.0) > 0 and occupied(4.0, 0.0, 1.1, 0.7) > 20)
            state["extra"] = []
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.03)
            assert occupied(8.0, 0.0) > 0
            state["opponent"] = (4.0, 4.0, 0.0, 2.0, 1.08)
            wait(
                lambda: (
                    occupied(8.0, 0.0) == 0
                    and occupied(4.0, 0.0, 1.1, 0.7) == 0
                    and occupied(4.0, 4.0, 1.1, 0.7) > 20
                )
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
