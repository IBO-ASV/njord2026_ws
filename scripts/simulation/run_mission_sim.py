#!/usr/bin/env python3
"""Run only owned subprocess groups in a dedicated ROS domain; keep all evidence."""

import argparse
import hashlib, json, os, signal, subprocess, time
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--task", default="task1")
    p.add_argument("--noise", action="store_true")
    p.add_argument("--disturbance", action="store_true")
    p.add_argument("--seed", type=int, default=2026)
    p.add_argument("--model", choices=["kinematic", "mmg"], default="mmg")
    p.add_argument("--timeout", type=int, default=900)
    p.add_argument("--domain", type=int, default=89)
    p.add_argument("--output", required=True)
    p.add_argument("--plot", action="store_true")
    p.add_argument("--encounter", default="head_on")
    p.add_argument("--cancel-after", type=float, default=0.0)
    p.add_argument("--dropout-after", type=float, default=0.0)
    p.add_argument("--opponent-variable-speed", action="store_true")
    for name in ("initial-x-offset", "initial-y-offset", "initial-yaw-offset",
                 "detection-loss-period", "detection-loss-duration"):
        p.add_argument("--" + name, type=float, default=0.0)
    p.add_argument("--mmg-config", default="")
    p.add_argument("--velocity-control-config", default="")
    a = p.parse_args()
    if not 0 <= a.detection_loss_duration <= a.detection_loss_period:
        p.error("detection loss requires 0 <= duration <= period")
    out = Path(a.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / "run.json").exists():
        p.error("output already contains a run; choose a new directory")
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(a.domain),
        ROS_LOCALHOST_ONLY="1",
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
    )
    manifest = vars(a) | {
        "commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "started": time.time(),
        "source_sha256": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for root in (
                "src/sim/njord_sim",
                "src/mission/mission_manager/mission_manager",
                "src/navigation/path_generator/mppi/asv_trajectory_planner",
                "src/driver/micon/thruster_driver",
                "src/sim/dutyed_tf_pub_with_disturbance",
            )
            for path in sorted(Path(root).rglob("*"))
            if path.is_file() and path.suffix in (".py", ".cpp", ".hpp", ".yaml")
        },
        "working_tree_dirty": bool(
            subprocess.check_output(
                ["git", "status", "--porcelain", "--untracked-files=no"], text=True
            ).strip()
        ),
    }
    (out / "run.json").write_text(json.dumps(manifest, indent=2))
    children = []
    try:
        with (
            (out / "launch.log").open("w") as log,
            (out / "observer.log").open("w") as observer,
        ):
            launch = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "njord_sim",
                    "mission_sim.launch.py",
                    f"task:={a.task}",
                    "foxglove:=false",  # Parallel acceptance runs never own a GUI port.
                    f"noise:={str(a.noise).lower()}",
                    f"model:={a.model}",
                    f"seed:={a.seed}",
                    f"disturbance:={str(a.disturbance).lower()}",
                    f"encounter:={a.encounter}",
                    f"opponent_variable_speed:={str(a.opponent_variable_speed).lower()}",
                    *[f"{name}:={getattr(a, name)}" for name in (
                        "initial_x_offset", "initial_y_offset", "initial_yaw_offset",
                        "detection_loss_period", "detection_loss_duration",
                    )],
                    f"dropout_after:={a.dropout_after}",
                    f"config_dir:={out / 'config'}",
                    *[f"{name}:={getattr(a,name)}" for name in ("mmg_config","velocity_control_config") if getattr(a,name)],
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            children.append(launch)
            check = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "njord_sim",
                    "evaluate",
                    "--task",
                    a.task,
                    "--timeout",
                    str(a.timeout),
                    "--output",
                    str(out),
                    "--cancel-after",
                    str(a.cancel_after),
                    "--encounter",
                    a.encounter,
                    "--dropout-after",
                    str(a.dropout_after),
                    *(["--plot"] if a.plot else []),
                ],
                env=env,
                stdout=observer,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            children.append(check)
            try:
                return check.wait(timeout=a.timeout + 90)
            except subprocess.TimeoutExpired:
                if not (out / "evaluation.json").exists():
                    (out / "evaluation.json").write_text(
                        json.dumps(
                            {
                                "task": a.task,
                                "passed": False,
                                "reason": "observer watchdog expired including startup/artifact grace",
                            }
                        )
                    )
                return 124
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            for child in children:
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
            if sig != signal.SIGKILL:
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and any(
                    c.poll() is None for c in children
                ):
                    time.sleep(0.1)
        for child in children:
            child.wait()


if __name__ == "__main__":
    raise SystemExit(main())
