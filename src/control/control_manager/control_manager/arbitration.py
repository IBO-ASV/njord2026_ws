"""ROS-independent command selection policy."""

from __future__ import annotations

from enum import IntEnum


class Source(IntEnum):
    ZERO = 0
    MANUAL = 1
    AUTO = 2
    SBUS = 3


class State(IntEnum):
    """Control-state values kept in lockstep with njord_interfaces/ControlState."""

    BOOTING = 0
    MANUAL = 1
    AUTO_REQUESTED = 2
    AUTO_ARMED = 3
    AUTO_RUNNING = 4
    AUTO_INHIBITED = 5
    EMERGENCY_STOP = 6


def compatibility_control_status(*, emergency_stop: bool, requested_mode: int) -> str:
    """Return the legacy Foxglove status without making it a control input."""
    if emergency_stop:
        return "emergency_stop"
    return "auto" if requested_mode == 1 else "manual"


def derive_control_state(
    *, emergency_stop: bool, auto_requested: bool, auto_permitted: bool,
    manual_command_fresh: bool, nav_command_fresh: bool, sbus_command_fresh: bool = False,
) -> tuple[State, Source]:
    """Derive canonical state; stale commands can never select a live source."""
    if emergency_stop:
        return State.EMERGENCY_STOP, Source.ZERO
    if sbus_command_fresh:
        return State.MANUAL, Source.SBUS
    if not auto_requested:
        return State.MANUAL, Source.MANUAL if manual_command_fresh else Source.ZERO
    if not auto_permitted:
        return State.AUTO_INHIBITED, Source.ZERO
    if nav_command_fresh:
        return State.AUTO_RUNNING, Source.AUTO
    return State.AUTO_ARMED, Source.ZERO


def select_source(
    *, emergency_stop: bool, effective_source: int, auto_permitted: bool,
    manual_command_fresh: bool, nav_command_fresh: bool, sbus_command_fresh: bool = False,
) -> Source:
    """Return the only source Command Arbiter is permitted to forward."""
    if emergency_stop:
        return Source.ZERO
    if effective_source == Source.SBUS and sbus_command_fresh:
        return Source.SBUS
    if effective_source == Source.MANUAL and manual_command_fresh:
        return Source.MANUAL
    if effective_source == Source.AUTO and auto_permitted and nav_command_fresh:
        return Source.AUTO
    return Source.ZERO
