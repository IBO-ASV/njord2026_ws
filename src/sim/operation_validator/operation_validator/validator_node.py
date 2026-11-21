import csv
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def point_to_segment_distance(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    seg2 = vx * vx + vy * vy
    if seg2 <= 1.0e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg2))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


class OperationValidatorNode(Node):
    def __init__(self):
        super().__init__("operation_validator_node")

        self.declare_parameter("cte_threshold", 1.5)
        self.declare_parameter("output_dir", "/tmp/njord_sim_eval")
        self.declare_parameter("summary_csv", "summary.csv")
        self.declare_parameter("timeseries_csv", "timeseries.csv")

        self.cte_threshold = self.get_parameter("cte_threshold").get_parameter_value().double_value
        out_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        summary_name = self.get_parameter("summary_csv").get_parameter_value().string_value
        ts_name = self.get_parameter("timeseries_csv").get_parameter_value().string_value

        self.output_dir = Path(out_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.summary_csv = self.output_dir / summary_name
        self.timeseries_csv = self.output_dir / ts_name

        self.path_points = []
        self.samples = []
        self.start_time = None
        self.goal_time = None
        self.recovery_start = None
        self.recovery_durations = []
        self.max_cte = 0.0

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_path = self.create_subscription(PathMsg, "/plan", self.on_path, transient_qos)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 30)
        self.sub_start = self.create_subscription(Bool, "/sim/start", self.on_start, transient_qos)
        self.sub_goal = self.create_subscription(Bool, "/sim/goal_reached", self.on_goal, transient_qos)

        self.get_logger().info(f"Operation validator started. Output dir: {self.output_dir}")

    def on_start(self, msg: Bool):
        if msg.data and self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info("Validation timer started")

    def on_goal(self, msg: Bool):
        if msg.data and self.goal_time is None:
            self.goal_time = self.get_clock().now()
            self.get_logger().info("Goal event received. Writing CSV.")
            self.write_csv()

    def on_path(self, msg: PathMsg):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def compute_cte(self, x, y):
        if len(self.path_points) < 2:
            return float("nan")
        best = float("inf")
        for i in range(len(self.path_points) - 1):
            ax, ay = self.path_points[i]
            bx, by = self.path_points[i + 1]
            d = point_to_segment_distance(x, y, ax, ay, bx, by)
            if d < best:
                best = d
        return best

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = self.get_clock().now()

        if self.start_time is None:
            self.start_time = t

        cte = self.compute_cte(x, y)
        if not math.isnan(cte):
            self.max_cte = max(self.max_cte, cte)
            if cte > self.cte_threshold and self.recovery_start is None:
                self.recovery_start = t
            elif cte <= self.cte_threshold and self.recovery_start is not None:
                dt = (t - self.recovery_start).nanoseconds * 1.0e-9
                self.recovery_durations.append(dt)
                self.recovery_start = None

        elapsed = 0.0
        if self.start_time is not None:
            elapsed = (t - self.start_time).nanoseconds * 1.0e-9
        self.samples.append((elapsed, x, y, cte))

    def write_csv(self):
        with self.timeseries_csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["elapsed_sec", "x", "y", "cross_track_error"])
            writer.writerows(self.samples)

        duration = float("nan")
        if self.start_time is not None and self.goal_time is not None:
            duration = (self.goal_time - self.start_time).nanoseconds * 1.0e-9

        valid_cte = [s[3] for s in self.samples if not math.isnan(s[3])]
        mean_cte = float("nan")
        if valid_cte:
            mean_cte = sum(valid_cte) / len(valid_cte)

        if self.recovery_start is not None:
            now = self.get_clock().now()
            dt = (now - self.recovery_start).nanoseconds * 1.0e-9
            self.recovery_durations.append(dt)
            self.recovery_start = None

        mean_recovery = float("nan")
        if self.recovery_durations:
            mean_recovery = sum(self.recovery_durations) / len(self.recovery_durations)

        with self.summary_csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "start_to_goal_sec",
                "max_cross_track_error",
                "mean_cross_track_error",
                "recovery_count",
                "mean_recovery_sec",
            ])
            writer.writerow([
                duration,
                self.max_cte,
                mean_cte,
                len(self.recovery_durations),
                mean_recovery,
            ])

        self.get_logger().info(f"Wrote {self.summary_csv}")
        self.get_logger().info(f"Wrote {self.timeseries_csv}")

    def destroy_node(self):
        if self.samples and not self.summary_csv.exists():
            self.write_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OperationValidatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
