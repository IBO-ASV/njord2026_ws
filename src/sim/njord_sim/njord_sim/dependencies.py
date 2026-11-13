"""Fail before bringing up a partial Task2 graph when MPPI cannot import."""

import os
import subprocess
import sys


def require_task2_python():
    try:
        result = subprocess.run(
            [
                sys.executable,
                "-c",
                "import torch; assert (torch.ones(1) + 1).item() == 2",
            ],
            capture_output=True,
            text=True,
            timeout=30,
            env={**os.environ, "OMP_NUM_THREADS": "2", "OPENBLAS_NUM_THREADS": "2"},
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(
            "Task2 CPU PyTorch startup check timed out after 30 seconds"
        ) from exc
    if result.returncode:
        raise RuntimeError(
            "Task2 MPPI requires working PyTorch in the ROS Python interpreter. "
            "Run: python3 scripts/setup/install_task2_dependencies.py "
            "(or --from-directory PATH for an existing CPU installation). "
            "The opponent-selector no-track warning is not this startup failure.\n"
            + result.stderr[-1500:]
        )
