from mission_manager.state_machine import MissionState, MissionStateMachine, ResultCode, StartRequest


def test_rejects_unavailable_without_leaving_idle():
    machine = MissionStateMachine()
    decision = machine.request_start(StartRequest("missing"), task_available=False, reason="unknown task")
    assert not decision.accepted
    assert machine.snapshot.state == MissionState.IDLE


def test_request_id_is_idempotent_and_other_active_request_is_rejected():
    machine = MissionStateMachine()
    first = machine.request_start(StartRequest("task1", request_id="operator-1"), task_available=True)
    duplicate = machine.request_start(StartRequest("task1", request_id="operator-1"), task_available=True)
    competing = machine.request_start(StartRequest("task1", request_id="operator-2"), task_available=True)
    assert first.accepted
    assert duplicate.accepted and duplicate.duplicate
    assert duplicate.execution_id == first.execution_id
    assert not competing.accepted


def test_late_callbacks_are_ignored_and_cancel_returns_idle():
    machine = MissionStateMachine()
    decision = machine.request_start(StartRequest("task1"), task_available=True)
    execution_id = decision.execution_id
    machine.transition(MissionState.CONFIGURING, execution_id=execution_id)
    machine.transition(MissionState.RUNNING, execution_id=execution_id)
    assert not machine.update_progress("old-id", "late", 0.5, "late callback")
    assert machine.begin_cancel(execution_id)
    assert machine.finish(execution_id, ResultCode.CANCELED, "canceled")
    assert machine.reset_to_idle(execution_id)
    assert machine.snapshot.state == MissionState.IDLE


def test_dry_run_can_complete_after_configuration():
    machine = MissionStateMachine()
    decision = machine.request_start(StartRequest("task1"), task_available=True)
    machine.transition(MissionState.CONFIGURING, execution_id=decision.execution_id)
    assert machine.update_progress(decision.execution_id, "configure", 1.0, "route validated")
    assert machine.finish(decision.execution_id, ResultCode.SUCCEEDED, "dry run complete")
