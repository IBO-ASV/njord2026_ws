import subprocess

import pytest
from njord_sim.dependencies import require_task2_python


def test_missing_torch_reports_actionable_failure_before_launch(monkeypatch):
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(
            a[0], 1, "", "ModuleNotFoundError: No module named 'torch'"
        ),
    )
    with pytest.raises(RuntimeError, match="install_task2_dependencies.py"):
        require_task2_python()


def test_healthy_torch_passes_check(monkeypatch):
    monkeypatch.setattr(
        subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a[0], 0, "", "")
    )
    require_task2_python()


def test_stalled_torch_import_is_bounded(monkeypatch):
    def timeout(*a, **k):
        raise subprocess.TimeoutExpired(a[0], k["timeout"])

    monkeypatch.setattr(subprocess, "run", timeout)
    with pytest.raises(RuntimeError, match="timed out"):
        require_task2_python()
