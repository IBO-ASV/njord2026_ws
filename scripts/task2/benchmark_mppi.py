#!/usr/bin/env python3
"""No-ROS MPPI solve-step benchmark for the Task 2 planner.

Imports MPPIPlanner straight from the source tree (no colcon build, no ROS 2)
and times repeated plan() calls with synthetic state/waypoints that replicate
planner_node's calling convention:

    planner_node -> TrajectoryGenerator.generate() -> MPPIPlanner.plan(
        own=VesselState(x=0, y=0, yaw=0, u, v, r, vx, vy),   # base_link basis
        other=VesselState(... base_link relative ...) | None,
        waypoint1=(x, y),  # waypoints converted to base_link
        waypoint2=(x, y),
    )

The planner is constructed with ALL DEFAULTS (the current hardcoded values),
so the reported latency reflects the real Task 2 configuration
(horizon 225, 5000 samples) unless overridden via CLI.

Usage examples:
    ./benchmark_mppi.py                       # defaults, opponent present
    ./benchmark_mppi.py --device cpu -n 20
    ./benchmark_mppi.py --device cuda --tegrastats   # on Jetson
"""

import argparse
import contextlib
import io
import os
import statistics
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
MPPI_DIR = os.path.join(
    REPO_ROOT, "src", "navigation", "path_generator", "mppi"
)
sys.path.insert(0, MPPI_DIR)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "-n", "--num-steps", type=int, default=50,
        help="number of timed solve steps (default: 50)",
    )
    parser.add_argument(
        "--warmup", type=int, default=3,
        help="untimed warmup steps (default: 3)",
    )
    parser.add_argument(
        "--device", choices=["cpu", "cuda"], default=None,
        help="force torch device (default: cuda if available)",
    )
    parser.add_argument(
        "--horizon", type=int, default=None,
        help="override mppi horizon (default: planner default, 225)",
    )
    parser.add_argument(
        "--num-samples", type=int, default=None,
        help="override mppi sample count (default: planner default, 5000)",
    )
    parser.add_argument(
        "--no-opponent", action="store_true",
        help="benchmark without an opponent vessel (no CRM collision cost)",
    )
    parser.add_argument(
        "--tegrastats", action="store_true",
        help="sample tegrastats during the benchmark (Jetson only)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    import torch
    from asv_trajectory_planner.mppi_torch import MPPIPlanner
    from asv_trajectory_planner.vessel_state import VesselState

    if args.device == "cuda" and not torch.cuda.is_available():
        print("ERROR: --device cuda requested but torch.cuda.is_available() is False")
        return 1

    overrides = {}
    if args.horizon is not None:
        overrides["horizon"] = args.horizon
    if args.num_samples is not None:
        overrides["num_samples"] = args.num_samples

    planner = MPPIPlanner(**overrides)

    if args.device is not None:
        # MPPItorch auto-selects cuda when available; honor an explicit
        # override (tensors are moved with .to(_device) at use time).
        planner.mppi_controller._device = args.device

    device = planner.mppi_controller._device

    # Synthetic state matching TrajectoryGenerator's base_link convention:
    # own ship at origin, heading +x, cruising near target speed.
    own = VesselState(x=0.0, y=0.0, yaw=0.0, u=1.0, v=0.0, r=0.0, vx=1.0, vy=0.0)

    # Head-on opponent 15 m ahead, slightly to port (worst-case CRM branch).
    other = None
    if not args.no_opponent:
        other = VesselState(
            x=15.0, y=1.0, yaw=180.0, u=1.0, v=0.0, r=0.0, vx=-1.0, vy=0.0
        )

    # GPS line waypoints in base_link (course roughly matching
    # task2_waypoints.yaml: ~46 m diagonal).
    waypoint1 = (0.0, 0.0)
    waypoint2 = (45.0, -9.0)

    tegrastats_proc = None
    if args.tegrastats:
        try:
            tegrastats_proc = subprocess.Popen(
                ["tegrastats", "--interval", "1000"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
        except FileNotFoundError:
            print("WARN: tegrastats not found; continuing without it")

    print(f"device       : {device}")
    print(f"horizon      : {planner.mppi_controller.horizon}")
    print(f"num_samples  : {planner.mppi_controller.nb_sample}")
    print(f"opponent     : {'yes' if other is not None else 'no'}")
    print(f"steps        : {args.num_steps} (+{args.warmup} warmup)")
    print()

    def solve_once():
        # Suppress the planner's per-solve debug prints during timing.
        with contextlib.redirect_stdout(io.StringIO()):
            planner.plan(
                own=own,
                other=other,
                waypoint1=waypoint1,
                waypoint2=waypoint2,
            )

    for _ in range(max(args.warmup, 0)):
        solve_once()

    latencies_ms = []
    for i in range(args.num_steps):
        t0 = time.perf_counter()
        solve_once()
        if device == "cuda":
            torch.cuda.synchronize()
        latencies_ms.append((time.perf_counter() - t0) * 1000.0)

    if tegrastats_proc is not None:
        tegrastats_proc.terminate()
        try:
            tegra_out, _ = tegrastats_proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            tegrastats_proc.kill()
            tegra_out = ""
    else:
        tegra_out = ""

    latencies_ms.sort()
    mean_ms = statistics.mean(latencies_ms)
    median_ms = statistics.median(latencies_ms)
    p95_idx = min(len(latencies_ms) - 1, int(round(0.95 * len(latencies_ms))) )
    p95_ms = latencies_ms[p95_idx]
    achievable_hz = 1000.0 / mean_ms if mean_ms > 0 else float("inf")

    print("=== MPPI solve-step latency ===")
    print(f"mean         : {mean_ms:8.2f} ms")
    print(f"median       : {median_ms:8.2f} ms")
    print(f"p95          : {p95_ms:8.2f} ms")
    print(f"min / max    : {latencies_ms[0]:.2f} / {latencies_ms[-1]:.2f} ms")
    print(f"achievable   : {achievable_hz:8.2f} Hz (planner_node default planning_frequency is 2.0 Hz)")

    if achievable_hz < 2.0:
        print("WARN: below the 2 Hz planning_frequency target on this device")

    if tegra_out:
        print()
        print("=== tegrastats sample (last 5 lines) ===")
        for line in tegra_out.strip().splitlines()[-5:]:
            print(line)

    return 0


if __name__ == "__main__":
    sys.exit(main())
