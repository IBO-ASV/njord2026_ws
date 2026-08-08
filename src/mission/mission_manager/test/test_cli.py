"""Contract tests for the operator-facing ``njord-task`` command surface."""

from mission_manager.njord_task import _parser, _stop_request


def test_manual_command_is_available():
    args = _parser().parse_args(["manual"])
    assert args.command == "manual"


def test_stop_defaults_to_current_execution():
    args = _parser().parse_args(["stop"])
    assert args.command == "stop"
    assert args.execution_id == ""


def test_stop_request_returns_to_manual_via_mode_manager():
    request = _stop_request("execution-42")
    assert request.execution_id == "execution-42"
    assert request.return_to_manual
