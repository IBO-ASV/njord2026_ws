"""Serialized, ROS-independent mission lifecycle state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Optional, Tuple
from uuid import uuid4


class MissionState(IntEnum):
    IDLE = 0
    VALIDATING = 1
    CONFIGURING = 2
    WAITING_FOR_READINESS = 3
    WAITING_FOR_AUTO_PERMISSION = 4
    RUNNING = 5
    PAUSED = 6
    CANCELING = 7
    SUCCEEDED = 8
    FAILED = 9
    REJECTED = 10


class ResultCode(IntEnum):
    SUCCEEDED = 0
    CANCELED = 1
    REJECTED = 2
    NOT_IMPLEMENTED = 3
    NOT_READY = 4
    SAFETY_INHIBITED = 5
    NAVIGATION_FAILED = 6
    CONFIGURATION_FAILED = 7
    INTERNAL_ERROR = 255


@dataclass(frozen=True)
class StartRequest:
    task_id: str
    request_auto_mode: bool = False
    dry_run: bool = False
    request_id: str = ""


@dataclass(frozen=True)
class StartDecision:
    accepted: bool
    result_code: ResultCode
    message: str
    execution_id: str = ""
    duplicate: bool = False


@dataclass(frozen=True)
class MissionSnapshot:
    state: MissionState
    task_id: str = ""
    execution_id: str = ""
    stage: str = ""
    progress: float = 0.0
    message: str = ""


_ACTIVE_STATES = {
    MissionState.VALIDATING,
    MissionState.CONFIGURING,
    MissionState.WAITING_FOR_READINESS,
    MissionState.WAITING_FOR_AUTO_PERMISSION,
    MissionState.RUNNING,
    MissionState.CANCELING,
}

_TRANSITIONS = {
    MissionState.IDLE: {MissionState.VALIDATING},
    MissionState.VALIDATING: {MissionState.CONFIGURING, MissionState.REJECTED, MissionState.FAILED},
    MissionState.CONFIGURING: {
        MissionState.WAITING_FOR_READINESS,
        MissionState.WAITING_FOR_AUTO_PERMISSION,
        MissionState.RUNNING,
        MissionState.SUCCEEDED,
        MissionState.FAILED,
        MissionState.CANCELING,
    },
    MissionState.WAITING_FOR_READINESS: {
        MissionState.WAITING_FOR_AUTO_PERMISSION,
        MissionState.RUNNING,
        MissionState.FAILED,
        MissionState.CANCELING,
    },
    MissionState.WAITING_FOR_AUTO_PERMISSION: {
        MissionState.RUNNING,
        MissionState.FAILED,
        MissionState.CANCELING,
    },
    MissionState.RUNNING: {
        MissionState.SUCCEEDED,
        MissionState.FAILED,
        MissionState.CANCELING,
    },
    MissionState.CANCELING: {MissionState.FAILED, MissionState.REJECTED},
    MissionState.SUCCEEDED: {MissionState.IDLE},
    MissionState.FAILED: {MissionState.IDLE},
    MissionState.REJECTED: {MissionState.IDLE},
}


class MissionStateMachine:
    """Owns all lifecycle changes for exactly one mission execution.

    This intentionally has no ROS dependencies.  ROS callbacks must call this
    object while holding the mission node's single lock; callbacks from an old
    execution are ignored by :meth:`is_current`.
    """

    def __init__(self) -> None:
        self._snapshot = MissionSnapshot(MissionState.IDLE)
        self._request_ids: Dict[str, str] = {}
        self._results: Dict[str, Tuple[ResultCode, str]] = {}

    @property
    def snapshot(self) -> MissionSnapshot:
        return self._snapshot

    @property
    def active(self) -> bool:
        return self._snapshot.state in _ACTIVE_STATES

    def request_start(
        self, request: StartRequest, *, task_available: bool, reason: str = ""
    ) -> StartDecision:
        """Validate admission without causing executor side effects."""
        request_id = request.request_id.strip()
        if request_id and request_id in self._request_ids:
            execution_id = self._request_ids[request_id]
            return StartDecision(
                True,
                ResultCode.SUCCEEDED,
                "duplicate request; returning existing execution",
                execution_id,
                duplicate=True,
            )
        if self.active:
            return StartDecision(False, ResultCode.REJECTED, "another task is active")
        if not task_available:
            return StartDecision(False, ResultCode.REJECTED, reason or "task is unavailable")
        if not request.task_id.strip():
            return StartDecision(False, ResultCode.REJECTED, "task_id must not be empty")

        execution_id = str(uuid4())
        if request_id:
            self._request_ids[request_id] = execution_id
        self._snapshot = MissionSnapshot(
            MissionState.VALIDATING,
            task_id=request.task_id,
            execution_id=execution_id,
            message="task accepted",
        )
        return StartDecision(True, ResultCode.SUCCEEDED, "task accepted", execution_id)

    def transition(
        self,
        state: MissionState,
        *,
        execution_id: str,
        stage: Optional[str] = None,
        progress: Optional[float] = None,
        message: Optional[str] = None,
    ) -> bool:
        """Apply a legal transition for the current execution only."""
        if not self.is_current(execution_id):
            return False
        current = self._snapshot.state
        if state not in _TRANSITIONS.get(current, set()):
            raise ValueError(f"illegal mission transition {current.name} -> {state.name}")
        self._snapshot = MissionSnapshot(
            state,
            task_id=self._snapshot.task_id,
            execution_id=execution_id,
            stage=self._snapshot.stage if stage is None else stage,
            progress=self._snapshot.progress if progress is None else max(0.0, min(1.0, progress)),
            message=self._snapshot.message if message is None else message,
        )
        return True

    def update_progress(
        self, execution_id: str, stage: str, progress: float, message: str
    ) -> bool:
        if not self.is_current(execution_id) or self._snapshot.state not in _ACTIVE_STATES:
            return False
        self._snapshot = MissionSnapshot(
            self._snapshot.state,
            task_id=self._snapshot.task_id,
            execution_id=execution_id,
            stage=stage,
            progress=max(0.0, min(1.0, progress)),
            message=message,
        )
        return True

    def begin_cancel(self, execution_id: str, message: str = "cancellation requested") -> bool:
        if not self.is_current(execution_id) or not self.active:
            return False
        if self._snapshot.state != MissionState.CANCELING:
            self.transition(MissionState.CANCELING, execution_id=execution_id, message=message)
        return True

    def finish(self, execution_id: str, code: ResultCode, message: str) -> bool:
        """Record a terminal result, retaining it for idempotent callers."""
        if not self.is_current(execution_id):
            return False
        if code == ResultCode.SUCCEEDED:
            terminal = MissionState.SUCCEEDED
        elif code == ResultCode.CANCELED:
            terminal = MissionState.REJECTED
        else:
            terminal = MissionState.FAILED
        if terminal != self._snapshot.state:
            self.transition(terminal, execution_id=execution_id, message=message, progress=1.0)
        self._results[execution_id] = (code, message)
        return True

    def reset_to_idle(self, execution_id: str) -> bool:
        if not self.is_current(execution_id):
            return False
        if self._snapshot.state not in {MissionState.SUCCEEDED, MissionState.FAILED, MissionState.REJECTED}:
            raise ValueError("cannot reset a non-terminal mission")
        self._snapshot = MissionSnapshot(MissionState.IDLE, message=self._snapshot.message)
        return True

    def is_current(self, execution_id: str) -> bool:
        return bool(execution_id) and execution_id == self._snapshot.execution_id

    def result_for(self, execution_id: str) -> Optional[Tuple[ResultCode, str]]:
        return self._results.get(execution_id)
