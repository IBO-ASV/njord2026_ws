"""Pure fail-closed readiness predicates."""

from __future__ import annotations

from typing import Optional


def filtered_cloud_ready(
    observed_at_sec: Optional[float], now_sec: float, timeout_sec: float,
    header_stamp_sec: Optional[float], header_frame_id: str,
) -> bool:
    return (
        observed_at_sec is not None
        and timeout_sec > 0.0
        and now_sec >= observed_at_sec
        and now_sec - observed_at_sec <= timeout_sec
        and header_stamp_sec is not None
        and header_stamp_sec > 0.0
        and bool(header_frame_id)
    )
