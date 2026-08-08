"""ROS-independent stop contract checks using a small MissionManager harness."""

from mission_manager.mission_manager_node import MissionManager
from mission_manager.state_machine import MissionState, MissionStateMachine, StartRequest


class _Future:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class _ModeClient:
    def __init__(self):
        self.requests = []
        self.future = _Future()

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return self.future


class _Executor:
    def __init__(self):
        self.cancel_calls = []

    def cancel(self, execution_id):
        self.cancel_calls.append(execution_id)


def _manager_harness():
    manager = MissionManager.__new__(MissionManager)
    manager._machine = MissionStateMachine()
    decision = manager._machine.request_start(StartRequest("task1"), task_available=True)
    manager._machine.transition(MissionState.CONFIGURING, execution_id=decision.execution_id)
    manager._machine.transition(MissionState.RUNNING, execution_id=decision.execution_id)
    manager._active_executor = _Executor()
    manager._pending_start = (decision.execution_id, object())
    manager._manual_mode_pending = set()
    manager._manual_mode_requested = set()
    manager._mode_client = _ModeClient()
    manager._publish_task_readiness = lambda *_args: None
    manager._publish_status = lambda: None
    return manager, decision.execution_id


def test_stop_requests_manual_after_cancel_and_is_idempotent():
    manager, execution_id = _manager_harness()

    accepted, message = manager._stop(execution_id, return_to_manual=True)

    assert accepted
    assert "MANUAL request submitted" in message
    assert manager._machine.snapshot.state == MissionState.CANCELING
    assert manager._active_executor.cancel_calls == [execution_id]
    assert len(manager._mode_client.requests) == 1
    assert manager._mode_client.requests[0].requested_mode == 0  # MODE_MANUAL

    accepted, _ = manager._stop(execution_id, return_to_manual=True)
    assert accepted
    assert manager._active_executor.cancel_calls == [execution_id]
    assert len(manager._mode_client.requests) == 1
