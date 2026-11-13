"""A valid observer JSON must not hide a runner timeout or launch failure."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace


def test_runner_failure_cannot_be_reported_as_suite_success(tmp_path, monkeypatch):
    script = Path(__file__).resolve().parents[4] / "scripts/simulation/run_sim_suite.py"
    spec = importlib.util.spec_from_file_location("sim_suite", script)
    suite = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(suite)
    case = tmp_path / "task1-truth"
    case.mkdir()
    (case / "evaluation.json").write_text(
        json.dumps({"passed": True, "reason": "mission terminal"})
    )
    monkeypatch.setattr(
        suite.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=124)
    )
    monkeypatch.setattr(
        suite.sys,
        "argv",
        [
            "run_sim_suite.py",
            "--tasks",
            "task1",
            "--seeds",
            "",
            "--truth",
            "--workers",
            "1",
            "--output",
            str(tmp_path),
        ],
    )
    assert suite.main() == 1
    result = json.loads((tmp_path / "suite.json").read_text())[0]
    assert not result["passed"]
    assert "runner exit 124" in result["reason"]
