"""Task 4's profile-switching waypoint sequence."""

from __future__ import annotations

from typing import Callable, Optional

from .base import CompletionCallback, ExecutorStatus, FeedbackCallback, TaskExecutor
from ..waypoint_config import Route, Waypoint


ProfileSwitch = Callable[[str, Callable[[bool, str], None]], None]
StrictGoals = Callable[[tuple[Waypoint, ...], Callable[[bool, str], None]], None]
MotionPolicy = Callable[[str, Callable[[bool, str], None]], None]


class Task4CompositeExecutor(TaskExecutor):
    """Run Task 3 -> Task 1 -> Task 3 without restarting the mission."""

    _STAGES = (
        ("normal_gate", "13", False),
        ("normal_approach", "14", True),
        ("normal_dock", "normal_dock", True),
        ("normal_exit", "normal_exit", False),
        ("switch_to_task1", "", False),
        ("task1_gps15", "15", False),
        ("task1_gps16", "16", False),
        ("task1_gps17", "17", False),
        ("switch_to_task3", "", False),
        ("parallel_approach", "17", True),
        ("parallel_dock", "parallel_dock", True),
    )

    def __init__(self, navigation, switch_profile: ProfileSwitch, strict_goals: StrictGoals,
                 motion_policy: MotionPolicy = lambda _policy, complete: complete(True, "")) -> None:
        super().__init__(navigation)
        self._switch_profile = switch_profile
        self._strict_goals = strict_goals
        self._motion_policy = motion_policy
        self._route: Optional[Route] = None
        self._stage_index = 0
        self._switching = False

    def start(self, execution_id: str, route: Route, feedback: FeedbackCallback,
              complete: CompletionCallback) -> None:
        self._begin(execution_id, feedback, complete)
        self._route = route
        if set(waypoint.waypoint_id for waypoint in route.waypoints) < {
            waypoint_id for _, waypoint_id, _ in self._STAGES if waypoint_id
        }:
            self._finish(ExecutorStatus.INTERNAL_ERROR, "Task 4 route is missing a composite stage waypoint")
            return
        self._send_stage()

    def cancel(self, execution_id: str) -> None:
        if execution_id != self.execution_id or self._finished:
            return
        super().cancel(execution_id)
        if self._switching:
            self._finish(ExecutorStatus.CANCELED, "Task 4 canceled during profile switch")

    def _send_stage(self) -> None:
        if self._finished or self._route is None:
            return
        stage, waypoint_id, strict = self._STAGES[self._stage_index]
        expected_id = self.execution_id
        self._report(stage, self._stage_index / len(self._STAGES), f"starting {stage}")
        if stage.startswith("switch_to_"):
            profile = stage.removeprefix("switch_to_")
            self._switching = True
            self._switch_profile(profile, lambda ok, message: self._switched(expected_id, ok, message))
            return
        waypoint = next(item for item in self._route.waypoints if item.waypoint_id == waypoint_id)
        if waypoint.docking_motion and (stage.startswith("normal_") or stage.startswith("parallel_")):
            self._motion_policy(
                waypoint.docking_motion,
                lambda ok, message: self._motion_ready(expected_id, waypoint, strict, ok, message),
            )
            return
        if strict:
            self._strict_goals((waypoint,), lambda ok, message: self._strict_ready(
                expected_id, waypoint, ok, message
            ))
        else:
            self._send_waypoint(expected_id, waypoint)

    def _motion_ready(self, expected_id: str, waypoint: Waypoint, strict: bool,
                      ok: bool, message: str) -> None:
        if expected_id != self.execution_id or self._finished:
            return
        if not ok:
            self._finish(ExecutorStatus.INTERNAL_ERROR, message or "Task 4 docking motion setup failed")
        elif strict:
            self._strict_goals((waypoint,), lambda ready, detail: self._strict_ready(
                expected_id, waypoint, ready, detail
            ))
        else:
            self._send_waypoint(expected_id, waypoint)

    def _switched(self, expected_id: str, ok: bool, message: str) -> None:
        self._switching = False
        if expected_id != self.execution_id or self._finished:
            return
        if not ok:
            self._finish(ExecutorStatus.INTERNAL_ERROR, message or "Nav2 profile switch failed")
            return
        self._advance()

    def _strict_ready(self, expected_id: str, waypoint: Waypoint, ok: bool, message: str) -> None:
        if expected_id != self.execution_id or self._finished:
            return
        if not ok:
            self._finish(ExecutorStatus.INTERNAL_ERROR, message or "Task 3 strict-goal configuration failed")
            return
        self._send_waypoint(expected_id, waypoint)

    def _send_waypoint(self, expected_id: str, waypoint: Waypoint) -> None:
        def accepted(ok: bool) -> None:
            if expected_id != self.execution_id or self._finished:
                return
            if self._cancel_requested:
                self._finish(ExecutorStatus.CANCELED, "Task 4 navigation canceled")
            elif not ok:
                self._finish(ExecutorStatus.REJECTED, "Nav2 rejected Task 4 waypoint goal")

        def completed(status: ExecutorStatus, message: str) -> None:
            if expected_id != self.execution_id or self._finished:
                return
            if self._cancel_requested:
                self._finish(ExecutorStatus.CANCELED, "Task 4 navigation canceled")
            elif status == ExecutorStatus.SUCCEEDED:
                self._advance()
            else:
                self._finish(status, message or "Task 4 navigation failed")

        self._navigation.send((waypoint,), accepted, completed)

    def _advance(self) -> None:
        self._stage_index += 1
        if self._stage_index == len(self._STAGES):
            self._report("complete", 1.0, "Task 4 complete")
            self._finish(ExecutorStatus.SUCCEEDED, "Task 4 complete")
        else:
            self._send_stage()
