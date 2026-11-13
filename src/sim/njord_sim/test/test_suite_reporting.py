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


def test_suite_forwards_robustness_and_model_options(tmp_path, monkeypatch):
    script = Path(__file__).resolve().parents[4] / "scripts/simulation/run_sim_suite.py"
    spec = importlib.util.spec_from_file_location("sim_suite_options", script)
    suite = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(suite)
    commands = []

    def run(command, **kwargs):
        commands.append(command)
        case = Path(command[command.index("--output") + 1])
        case.mkdir()
        (case / "evaluation.json").write_text(json.dumps({"passed": True}))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(suite.subprocess, "run", run)
    monkeypatch.setattr(suite.sys, "argv", [
        "run_sim_suite.py", "--tasks", "task2", "--seeds", "2026",
        "--encounters", "head_on,port", "--model", "mmg", "--disturbance",
        "--opponent-variable-speed", "--initial-y-offset", "0.5",
        "--detection-loss-period", "15", "--detection-loss-duration", "2",
        "--output", str(tmp_path),
    ])
    assert suite.main() == 0
    assert len(commands) == 2
    for command in commands:
        assert command[command.index("--model") + 1] == "mmg"
        assert "--disturbance" in command
        assert "--opponent-variable-speed" in command
        assert command[command.index("--initial-y-offset") + 1] == "0.5"
        assert command[command.index("--detection-loss-duration") + 1] == "2.0"
