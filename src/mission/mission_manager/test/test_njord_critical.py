from argparse import Namespace

import pytest

from mission_manager.njord_critical import make_command
from njord_interfaces.msg import OperatorCommand


def test_start_is_always_auto_and_requires_task_id():
    command = make_command(Namespace(command="task", operation="start", task_id="task2"), 7)
    assert command.command == OperatorCommand.TASK_START
    assert command.task_id == "task2"
    assert command.requested_mode == 1
    with pytest.raises(ValueError):
        make_command(Namespace(command="task", operation="start", task_id=""), 7)


def test_restart_target_is_fixed_by_cli_grammar():
    command = make_command(Namespace(command="bringup", operation="restart", target="jetson"), 8)
    assert command.command == OperatorCommand.RESTART_BRINGUP
    assert command.target == OperatorCommand.JETSON
