"""Read-only post-mission map/transport evidence; never changes acceptance rules."""

import math
import time

import numpy as np
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class CostmapAudit:
    def __init__(self, node):
        self.node = node
        self.grids = {}
        self.rates = {}
        self.points = None
        self.scan = None
        self.scans = {}
        self.poses = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in ("global_costmap", "local_costmap"):
            base = f"/{name}/obstacle_layer"
            node.create_subscription(
                OccupancyGrid,
                base,
                lambda message, key=name: self.full(key, message),
                qos,
            )
            node.create_subscription(
                OccupancyGridUpdate,
                base + "_updates",
                lambda message, key=name: self.update(key, message),
                10,
            )
            full_topic = f"/{name}/costmap"
            node.create_subscription(
                OccupancyGrid,
                full_topic,
                lambda message, key=full_topic: self.measure(key, message),
                qos,
            )
            topic = f"/{name}/costmap_updates"
            node.create_subscription(
                OccupancyGridUpdate,
                topic,
                lambda message, key=topic: self.measure(key, message),
                10,
            )
        node.create_subscription(PointCloud2, "/sim_obstacles", self.cloud, 10)
        node.create_subscription(
            LaserScan, "/sim/costmap_clearing_scan", self.laser, 10
        )
        node.create_subscription(
            Odometry, "/odometry/filtered/local", self.odometry, 10
        )

    def measure(self, topic, message):
        now = time.monotonic()
        record = self.rates.setdefault(
            topic, {"samples": 0, "first": now, "last": now, "ages": []}
        )
        record["samples"] += 1
        record["last"] = now
        record["ages"].append(
            (self.node.get_clock().now().nanoseconds - stamp_ns(message.header.stamp))
            / 1e6
        )

    def full(self, name, message):
        self.measure(f"/{name}/obstacle_layer", message)
        self.grids[name] = {
            "info": message.info,
            "data": np.asarray(message.data, dtype=np.int16)
            .reshape(message.info.height, message.info.width)
            .copy(),
            "stamp": stamp_ns(message.header.stamp),
        }

    def update(self, name, message):
        self.measure(f"/{name}/obstacle_layer_updates", message)
        grid = self.grids.get(name)
        if grid is not None and stamp_ns(message.header.stamp) >= grid["stamp"]:
            data = grid["data"]
            if (
                message.y + message.height <= data.shape[0]
                and message.x + message.width <= data.shape[1]
            ):
                data[
                    message.y : message.y + message.height,
                    message.x : message.x + message.width,
                ] = np.asarray(message.data, dtype=np.int16).reshape(
                    message.height, message.width
                )
                grid["stamp"] = stamp_ns(message.header.stamp)

    def cloud(self, message):
        self.points = np.asarray(
            [
                (x, y)
                for x, y, z in point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )
                if z >= 0.2
            ],
            dtype=float,
        ).reshape(-1, 2)

    def laser(self, message):
        self.scan = message
        self.scans[stamp_ns(message.header.stamp)] = message
        while len(self.scans) > 100:
            self.scans.pop(next(iter(self.scans)))
        self.measure("/sim/costmap_clearing_scan", message)

    def odometry(self, message):
        q = message.pose.pose.orientation
        position = message.pose.pose.position
        self.poses[stamp_ns(message.header.stamp)] = (
            position.x,
            position.y,
            math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z),
        )
        while len(self.poses) > 100:
            self.poses.pop(next(iter(self.poses)))

    def snapshot(self):
        result = {"maps": {}, "transport": {}}
        for topic, r in self.rates.items():
            duration = r["last"] - r["first"]
            result["transport"][topic] = {
                "samples": r["samples"],
                "hz": (r["samples"] - 1) / duration if duration else 0.0,
                "mean_receive_age_ms": float(np.mean(r["ages"])),
            }
        result["display_streams"] = {}
        for name in ("global_costmap", "local_costmap"):
            for layer in ("costmap", "obstacle_layer"):
                topic = f"/{name}/{layer}"
                sources = [
                    self.rates[key]
                    for key in (topic, topic + "_updates")
                    if key in self.rates
                ]
                if sources:
                    count = sum(r["samples"] for r in sources)
                    duration = max(r["last"] for r in sources) - min(
                        r["first"] for r in sources
                    )
                    result["display_streams"][topic] = {
                        "samples": count,
                        "hz": (count - 1) / duration if duration else 0.0,
                        "full_samples": self.rates.get(topic, {}).get("samples", 0),
                        "incremental_samples": self.rates.get(
                            topic + "_updates", {}
                        ).get("samples", 0),
                    }
        scan = self.scan
        if scan is not None and stamp_ns(scan.header.stamp) not in self.poses:
            # Callback ordering can deliver the newest scan just before its
            # odometry. Use the newest exactly matched pair, never a mismatched
            # latest pose or an unavailable report despite earlier valid pairs.
            matched = self.scans.keys() & self.poses.keys()
            scan = self.scans[max(matched)] if matched else None
        pose = self.poses.get(stamp_ns(scan.header.stamp)) if scan is not None else None
        if self.points is None or scan is None or pose is None:
            result["ready"] = False
            return result
        result["scan_pose_stamp_ns"] = stamp_ns(scan.header.stamp)
        result["scan_pose_lag_ms"] = (
            stamp_ns(self.scan.header.stamp) - stamp_ns(scan.header.stamp)
        ) / 1e6
        sx, sy, yaw = pose
        ranges = np.asarray(scan.ranges)
        for name, grid in self.grids.items():
            info = grid["info"]
            iy, ix = np.where(grid["data"] >= 100)
            points = np.column_stack(
                (
                    info.origin.position.x + (ix + 0.5) * info.resolution,
                    info.origin.position.y + (iy + 0.5) * info.resolution,
                )
            )
            distance = (
                np.min(
                    np.linalg.norm(
                        points[:, None, :] - self.points[None, :, :], axis=2
                    ),
                    axis=1,
                )
                if len(points) and len(self.points)
                else np.full(len(points), np.inf)
            )
            residual = points[distance > 0.22]
            bearing = (
                np.arctan2(residual[:, 1] - sy, residual[:, 0] - sx)
                - yaw
                - scan.angle_min
            ) % math.tau
            indexes = np.rint(bearing / scan.angle_increment).astype(int) % len(ranges)
            lengths = np.linalg.norm(residual - [sx, sy], axis=1)
            # A cell must lie fully before the measured endpoint to count as
            # observed free space. A shadow/unknown cell must not be erased.
            visible = np.isfinite(ranges[indexes]) & (
                lengths + info.resolution * 0.71 < ranges[indexes]
            )
            result["maps"][name] = {
                "lethal_cells": len(points),
                "residual_cells": len(residual),
                "visible_residual_cells": int(np.count_nonzero(visible)),
                "occluded_or_unobserved_residual_cells": int(
                    np.count_nonzero(~visible)
                ),
                "residual_xy": residual.round(3).tolist(),
                "resolution_m": info.resolution,
                "size": [info.width, info.height],
            }
        result["ready"] = len(result["maps"]) == 2
        return result
