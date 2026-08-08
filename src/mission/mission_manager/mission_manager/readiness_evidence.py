"""Pure, fail-closed predicates used by the runtime readiness adapter.

These helpers intentionally contain no ROS imports so their safety policy can
be tested without a running graph.  An input is not ready until the adapter has
observed concrete evidence, and it becomes unready as soon as that evidence
expires.
"""

from __future__ import annotations

from typing import Iterable, Optional


COLLISION_ACTION_MIN = 0
COLLISION_ACTION_MAX = 4


def is_fresh(observed_at_sec: Optional[float], now_sec: float, timeout_sec: float) -> bool:
    """Return whether an observation is recent, rejecting missing/bad clocks."""
    if observed_at_sec is None or timeout_sec <= 0.0 or now_sec < observed_at_sec:
        return False
    return now_sec - observed_at_sec <= timeout_sec


def collision_monitor_ready(
    observed_at_sec: Optional[float], now_sec: float, timeout_sec: float, action_type: Optional[int]
) -> bool:
    """A fresh, well-formed CollisionMonitorState is evidence of a live monitor."""
    return (
        action_type is not None
        and COLLISION_ACTION_MIN <= action_type <= COLLISION_ACTION_MAX
        and is_fresh(observed_at_sec, now_sec, timeout_sec)
    )


def buoy_perception_ready(
    observed_at_sec: Optional[float], now_sec: float, timeout_sec: float,
    header_stamp_sec: Optional[float], header_frame_id: str,
) -> bool:
    """A fresh, stamped detection cycle is evidence even when it found no buoy."""
    return (
        header_stamp_sec is not None
        and header_stamp_sec > 0.0
        and bool(header_frame_id)
        and is_fresh(observed_at_sec, now_sec, timeout_sec)
    )


def dynamic_gate_tf_ready(
    transform_stamp_secs: Iterable[Optional[float]], now_sec: float, timeout_sec: float
) -> bool:
    """All configured gate-frame transforms must exist and be freshly stamped."""
    stamps = tuple(transform_stamp_secs)
    return bool(stamps) and all(
        stamp is not None and stamp > 0.0 and is_fresh(stamp, now_sec, timeout_sec)
        for stamp in stamps
    )
