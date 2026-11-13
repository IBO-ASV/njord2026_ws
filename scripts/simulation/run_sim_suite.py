#!/usr/bin/env python3
"""Bounded acceptance matrix; separate ROS domain and artifacts for each worker."""

import argparse
import concurrent.futures
import json
import queue
import subprocess
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--tasks", default="task1,task1_2,task2,task3_1,task3_2,task4")
    p.add_argument("--seeds", default="2026,2027,2028")
    p.add_argument("--truth", action="store_true")
    p.add_argument("--encounters", default="head_on")
    p.add_argument("--workers", type=int, default=2)
    p.add_argument("--timeout", type=int, default=None)
    p.add_argument("--domain-base", type=int, default=100)
    p.add_argument("--output", required=True)
    p.add_argument("--model", choices=["kinematic", "mmg"], default="kinematic")
    p.add_argument("--disturbance", action="store_true")
    p.add_argument("--opponent-variable-speed", action="store_true")
    for name in ("initial-x-offset", "initial-y-offset", "initial-yaw-offset",
                 "detection-loss-period", "detection-loss-duration"):
        p.add_argument("--" + name, type=float, default=0.0)
    a = p.parse_args()
    if a.workers < 1 or a.domain_base < 0 or a.domain_base + a.workers > 233:
        p.error("workers must be positive and domains must be in 0..232")
    if not 0 <= a.detection_loss_duration <= a.detection_loss_period:
        p.error("detection loss requires 0 <= duration <= period")
    out = Path(a.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    domains = queue.Queue()
    for i in range(a.workers):
        domains.put(a.domain_base + i)
    jobs = []
    for task in a.tasks.split(","):
        encounters = a.encounters.split(",") if task == "task2" else ["head_on"]
        for encounter in encounters:
            if a.truth:
                jobs.append((task, None, encounter))
            jobs.extend(
                (task, int(seed), encounter) for seed in a.seeds.split(",") if seed
            )

    if not jobs:
        p.error("no cases selected; use --truth or at least one seed")

    def run(job):
        task, seed, encounter = job
        domain = domains.get()
        prefix = f"{task}-{encounter}" if task == "task2" else task
        name = prefix + "-" + ("truth" if seed is None else f"noise{seed}")
        command = [
            sys.executable,
            "scripts/simulation/run_mission_sim.py",
            "--task",
            task,
            "--domain",
            str(domain),
            "--timeout",
            str(a.timeout or (1800 if task == "task4" else 900)),
            "--output",
            str(out / name),
            "--encounter",
            encounter,
        ]
        command += ["--model", a.model]
        for flag in ("disturbance", "opponent_variable_speed"):
            if getattr(a, flag):
                command.append("--" + flag.replace("_", "-"))
        for option_name in ("initial_x_offset", "initial_y_offset", "initial_yaw_offset",
                     "detection_loss_period", "detection_loss_duration"):
            command += ["--" + option_name.replace("_", "-"), str(getattr(a, option_name))]
        if seed is not None:
            command += ["--noise", "--seed", str(seed)]
        try:
            code = subprocess.run(command, check=False).returncode
            result = (
                json.loads((out / name / "evaluation.json").read_text())
                if (out / name / "evaluation.json").exists()
                else {
                    "passed": False,
                    "reason": "observer failed before producing evaluation",
                }
            )
            if code != 0:
                result["passed"] = False
                result["reason"] = f"runner exit {code}: {result.get('reason', '')}"
            return {"run": name, "exit_code": code, **result}
        finally:
            domains.put(domain)

    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:
        for result in pool.map(run, jobs):
            results.append(result)
            (out / "suite.json").write_text(
                json.dumps(results, indent=2, ensure_ascii=False)
            )
            print(
                f"{result['run']}: {'PASS' if result['passed'] else 'FAIL'}", flush=True
            )
    return 0 if all(r["passed"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
