"""Bounded mission acceptance observer. Failure evidence is saved on every exit."""

import argparse
import csv
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from njord_interfaces.action import RunTask
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from njord_interfaces.msg import MissionStatus, ControlState
from njord_interfaces.srv import StopTask
from .scenario import (
    scenario,
    dock_segments,
    segment_distance,
    rectangle,
    segment_hits_polygon,
    point_rectangle_clearance,
)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Evaluation(Node):
    def __init__(self, args):
        super().__init__("njord_sim_evaluate")
        self.args = args
        self.s = scenario(args.task)
        self.samples = []
        self.events = []
        self.nav_ready = False
        self.state = None
        self.control = None
        self.received = 0.0
        self.execution = ""
        self.result = None
        self.action_result = None
        self.command = Twist()
        self.other = None
        self.min_ship_clearance = math.inf
        self.opponent_selected = False
        self.starboard_offset = 0.0
        self.cpa_astern = None
        self.gates = set()
        self.wall_events = []
        self.t0 = time.monotonic()
        self.wp = 0
        self.min_clearance = math.inf
        self.dwell = {}
        self.dwell_start = {}
        self.contacts = 0
        self.cardinal = {}
        self.previous = None
        self.stop_since = None
        self.stop_duration = 0.0
        self.start = ActionClient(self, RunTask, "/mission/run_task")
        self.stop = self.create_client(StopTask, "/mission/stop_task")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(MissionStatus, "/mission/status", self.on_status, qos)
        self.create_subscription(
            ControlState, "/control/state", lambda m: setattr(self, "control", m), qos
        )
        self.create_subscription(Odometry, "/sim/truth/odom", self.on_truth, 10)
        self.create_subscription(
            Twist, "/cmd_vel", lambda m: setattr(self, "command", m), 10
        )
        self.create_subscription(
            Odometry, "/sim/truth/opponent", lambda m: setattr(self, "other", m), 10
        )
        self.create_subscription(Bool, "/task2/opponent_detected", self.on_detected, 10)
        self.create_subscription(
            Bool,
            "/task4/buoy_wall_enable",
            lambda m: self.wall_events.append(m.data),
            qos,
        )
        self.create_subscription(
            Bool, "/autonomy/ready", lambda m: setattr(self, "nav_ready", m.data), 10
        )

    def on_detected(self, m):
        self.opponent_selected = self.opponent_selected or m.data

    def on_status(self, m):
        if self.state is None or (m.state, m.stage, m.message) != (
            self.state.state,
            self.state.stage,
            self.state.message,
        ):
            self.events.append(
                {
                    "t": time.monotonic() - self.t0,
                    "state": m.state,
                    "stage": m.stage,
                    "message": m.message,
                    "inhibit": list(m.inhibit_reasons),
                }
            )
            self.get_logger().info(
                f"{m.task_id}: state={m.state} {m.stage} {m.message}"
            )
        self.state = m
        if m.execution_id and m.task_id == self.args.task:
            self.execution = m.execution_id
        if m.task_id == self.args.task and m.state in (8, 9, 10):
            self.result = m.state

    def on_truth(self, m):
        t = time.monotonic() - self.t0
        self.received = time.monotonic()
        p = m.pose.pose.position
        heading = yaw(m.pose.pose.orientation)
        speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.samples.append((t, p.x, p.y, heading, speed, m.twist.twist.angular.z))
        route = self.s.route.waypoints
        if (
            self.wp < len(route)
            and math.hypot(p.x - route[self.wp].x, p.y - route[self.wp].y) < 1.5
        ):
            self.wp += 1
        for i, dock in enumerate(self.s.docks):
            x, y, axis, length, width = dock[:5]
            w = next(
                w
                for w in route
                if w.waypoint_type == "dock" and math.hypot(w.x - x, w.y - y) < 0.01
            )
            angle = abs(
                math.atan2(math.sin(heading - w.yaw), math.cos(heading - w.yaw))
            )
            inside = all(
                abs((hx - x) * math.cos(axis) + (hy - y) * math.sin(axis)) <= length / 2
                and abs(-(hx - x) * math.sin(axis) + (hy - y) * math.cos(axis))
                <= width / 2
                for hx, hy in rectangle(p.x, p.y, heading)
            )
            stationary = (
                inside
                and math.hypot(p.x - x, p.y - y) <= 0.8
                and angle <= math.radians(10)
                and speed <= 0.1
                and abs(m.twist.twist.angular.z) <= 0.05
            )
            if stationary:
                self.dwell_start.setdefault(i, t)
                self.dwell[i] = max(
                    self.dwell.get(i, 0.0), max(0.0, t - self.dwell_start[i] - 2.0)
                )
            else:
                self.dwell_start.pop(i, None)
        hull = rectangle(p.x, p.y, heading)
        for dock in self.s.docks:
            if any(segment_hits_polygon(a, b, hull) for a, b in dock_segments(dock)):
                self.contacts += 1
        for x, y, mark in self.s.buoys:
            self.min_clearance = min(
                self.min_clearance, point_rectangle_clearance(x, y, p.x, p.y, heading)
            )
        # Cardinal marks are scored at the closest approach, independently
        # of the oblique GPS3->GPS4 progress plane used for wall retirement.
        for i, (x, y, mark) in enumerate(self.s.buoys):
            distance = math.hypot(p.x - x, p.y - y)
            if (
                mark in ("E", "W", "N", "S")
                and distance < 10.0
                and distance < self.cardinal.get(i, {}).get("distance_m", math.inf)
            ):
                self.cardinal[i] = {
                    "mark": mark,
                    "distance_m": distance,
                    "correct_side": {
                        "E": p.x > x,
                        "W": p.x < x,
                        "N": p.y > y,
                        "S": p.y < y,
                    }[mark],
                    "x": p.x,
                    "y": p.y,
                }
        if self.args.task == "task2":
            a, b = route[0], route[-1]
            distance = math.hypot(b.x - a.x, b.y - a.y)
            fx, fy = (b.x - a.x) / distance, (b.y - a.y) / distance
            along = (p.x - a.x) * fx + (p.y - a.y) * fy
            lateral = -(p.x - a.x) * fy + (p.y - a.y) * fx
            self.starboard_offset = max(self.starboard_offset, -lateral)
            for gate, fraction in enumerate((0.15, 0.85)):
                if along >= distance * fraction and abs(lateral) < 1.6:
                    self.gates.add(gate)
            if self.other:
                op = self.other.pose.pose.position
                oh = yaw(self.other.pose.pose.orientation)
                other_hull = rectangle(op.x, op.y, oh, 2.0, 1.08)
                contact = any(
                    segment_hits_polygon(a, b, hull)
                    for a, b in zip(other_hull, other_hull[1:] + other_hull[:1])
                )
                clearance = (
                    0.0
                    if contact
                    else min(
                        segment_distance(v, a, b)
                        for poly, other in ((hull, other_hull), (other_hull, hull))
                        for v in poly
                        for a, b in zip(other, other[1:] + other[:1])
                    )
                )
                if clearance < self.min_ship_clearance:
                    self.min_ship_clearance = clearance
                    self.cpa_astern = (p.x - op.x) * math.cos(oh) + (
                        p.y - op.y
                    ) * math.sin(oh) < 0.0
        self.previous = (p.x, p.y)
        goal = route[-1]
        stationary = (
            math.hypot(p.x - goal.x, p.y - goal.y) <= 1.5
            and speed <= 0.1
            and abs(m.twist.twist.angular.z) <= 0.05
        )
        if stationary:
            self.stop_since = self.stop_since if self.stop_since is not None else t
            self.stop_duration = max(self.stop_duration, t - self.stop_since)
        else:
            self.stop_since = None

    def save(self, reason):
        out = Path(self.args.output)
        out.mkdir(parents=True, exist_ok=True)
        with (out / "trajectory.csv").open("w") as f:
            writer = csv.writer(f)
            writer.writerow(("time_s", "x", "y", "yaw", "speed", "yaw_rate"))
            writer.writerows(self.samples)
        required = [
            10.0 if i == 0 and self.args.task != "task3_2" else 5.0
            for i in range(len(self.s.docks))
        ]
        cardinal_ok = len(self.cardinal) == sum(
            mark in ("N", "E", "S", "W") for _, _, mark in self.s.buoys
        ) and all(v["correct_side"] for v in self.cardinal.values())
        encounter_ok = True
        if self.args.task == "task2":
            encounter_ok = len(self.gates) == 2
            if self.args.encounter != "none":
                encounter_ok = (
                    encounter_ok
                    and self.opponent_selected
                    and self.min_ship_clearance >= 0.5
                )
                if self.args.encounter == "head_on":
                    encounter_ok = encounter_ok and self.starboard_offset >= 0.5
                if self.args.encounter == "starboard":
                    encounter_ok = encounter_ok and self.cpa_astern is True
        passed = (
            encounter_ok
            and self.result == 8
            and self.action_result == 0
            and self.wp == len(self.s.route.waypoints)
            and self.min_clearance >= 0
            and self.contacts == 0
            and cardinal_ok
            and self.stop_duration >= 2.0
            and all(
                self.dwell.get(i, 0.0) >= seconds - 0.2
                for i, seconds in enumerate(required)
            )
        )
        fault_mode = (
            "sensor_loss"
            if self.args.dropout_after
            else ("cancel" if self.args.expected_cancel else "none")
        )
        if fault_mode != "none":
            stopped = (
                bool(self.samples)
                and self.samples[-1][4] <= 0.1
                and abs(self.samples[-1][5]) <= 0.05
            )
            zero = (
                math.hypot(self.command.linear.x, self.command.linear.y) < 1e-6
                and abs(self.command.angular.z) < 1e-6
            )
            passed = self.action_result == 1 and stopped and zero
            if fault_mode == "sensor_loss":
                passed = (
                    passed
                    and self.control is not None
                    and not self.control.auto_permitted
                )
        summary = {
            "test_mode": fault_mode,
            "task": self.args.task,
            "final_output": {
                "surge": self.command.linear.x,
                "sway": self.command.linear.y,
                "yaw": self.command.angular.z,
            },
            "final_inhibit_reasons": list(self.control.inhibit_reasons)
            if self.control
            else ["control unavailable"],
            "passed": passed,
            "reason": reason,
            "mission_result": self.result,
            "action_result_code": self.action_result,
            "waypoints_visited": self.wp,
            "waypoints_total": len(self.s.route.waypoints),
            "minimum_buoy_clearance_m": self.min_clearance
            if math.isfinite(self.min_clearance)
            else None,
            "stationary_dwell_s": self.dwell,
            "required_dwell_s": required,
            "other_ship_clearance_m": self.min_ship_clearance
            if math.isfinite(self.min_ship_clearance)
            else None,
            "opponent_selected": self.opponent_selected,
            "max_starboard_offset_m": self.starboard_offset,
            "cpa_astern": self.cpa_astern,
            "gates_passed": sorted(self.gates),
            "wall_enabled_seen": True in self.wall_events,
            "wall_disabled_seen": False in self.wall_events,
            "dock_contact_samples": self.contacts,
            "cardinal_passages": self.cardinal,
            "final_stationary_s": self.stop_duration,
            "samples": len(self.samples),
            "events": self.events,
            "evaluation_limits": [
                "synthetic perception; not image recognition accuracy",
                "MMG qualification is separate from kinematic runs",
            ],
        }
        (out / "evaluation.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False)
        )
        if self.args.plot:
            try:
                from .plotting import plot_run

                plot_run(self.s, self.samples, out)
            except Exception as error:
                (out / "plot_error.txt").write_text(str(error))
        return passed


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", default="task1")
    parser.add_argument("--timeout", type=float, default=900)
    parser.add_argument("--output", required=True)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--encounter", default="head_on")
    parser.add_argument("--cancel-after", type=float, default=0.0)
    parser.add_argument("--dropout-after", type=float, default=0.0)
    opts = parser.parse_args(args)
    opts.expected_cancel = opts.cancel_after > 0
    rclpy.init(args=[])
    node = Evaluation(opts)
    reason = "timeout"
    start_future = None
    result_future = None
    sent = False
    terminal_at = None
    try:
        while time.monotonic() - node.t0 < opts.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - node.t0
            if (
                not sent
                and node.start.server_is_ready()
                and node.received
                and node.control
                and node.nav_ready
                and elapsed > 10
            ):
                start_future = node.start.send_goal_async(
                    RunTask.Goal(
                        task_id=opts.task,
                        request_auto_mode=True,
                        request_id=f"sim-{time.time_ns()}",
                    )
                )
                sent = True
            if start_future and start_future.done():
                response = start_future.result()
                start_future = None
                if not response.accepted:
                    reason = "action goal rejected"
                    break
                result_future = response.get_result_async()
            if result_future and result_future.done():
                wrapped = result_future.result()
                result = wrapped.result
                node.action_result = (
                    result.result_code
                    if wrapped.status == 4 or result.result_code != 0
                    else 255
                )
                node.result = 8 if result.result_code == 0 else 9
                reason = result.message
                result_future = None
            should_cancel = opts.cancel_after and elapsed > opts.cancel_after
            if (
                opts.dropout_after
                and elapsed > opts.dropout_after + 6
                and node.result is None
            ):
                should_cancel = True
            if should_cancel and node.execution:
                node.stop.call_async(
                    StopTask.Request(execution_id=node.execution, return_to_manual=True)
                )
                opts.cancel_after = 0.0
            if node.result is not None:
                terminal_at = terminal_at or time.monotonic()
                if time.monotonic() - terminal_at > 4.0:
                    reason = "mission terminal"
                    break
            if opts.task == "task2" and node.execution and len(node.samples) > 2:
                recent = [row for row in node.samples if row[0] >= elapsed - 45.0]
                if (
                    recent
                    and elapsed - recent[0][0] >= 44.0
                    and max(row[1] for row in recent) - min(row[1] for row in recent)
                    < 0.2
                    and max(row[2] for row in recent) - min(row[2] for row in recent)
                    < 0.2
                ):
                    reason = "no physical progress for 45 seconds"
                    break
            if elapsed > 60 and not node.received:
                reason = "no truth odometry"
                break
    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        if node.execution and node.result is None and node.stop.service_is_ready():
            future = node.stop.call_async(
                StopTask.Request(execution_id=node.execution, return_to_manual=True)
            )
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        passed = node.save(reason)
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if passed else 1)
