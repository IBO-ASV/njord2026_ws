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
    a = p.parse_args()
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

    def run(job):
        task, seed, encounter = job
        domain = domains.get()
        prefix = f"{task}-{encounter}" if task == "task2" else task
        name = prefix + "-" + ("truth" if seed is None else f"noise{seed}")
        command = [
            sys.executable,
            "scripts/run_mission_sim.py",
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
