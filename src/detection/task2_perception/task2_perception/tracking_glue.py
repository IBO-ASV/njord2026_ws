"""Pure-python glue between the ship_tracking submodule output and the MPPI planner.

No ROS dependency. Mirrors the fields of ship_perception_msgs/TrackedObject
that the opponent selector needs, and implements:

  * body-frame twist -> base_link axes rotation (the submodule tracker
    publishes state.twist in the OBJECT body frame per REP-103),
  * ego compensation (the tracker's own ego compensation is a no-op stub, so
    TrackedObject velocities are RELATIVE to the moving base_link),
  * base_link -> map frame conversion,
  * CPA / TCPA computation,
  * opponent selection with gating (confirmed / distance / size / point count
    / staleness) and ranking policies "nearest" | "min_tcpa" | "track_id".
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

TRACK_TENTATIVE = 0
TRACK_CONFIRMED = 1
TRACK_LOST = 2


@dataclass
class Track:
    """Plain-python mirror of the TrackedObject fields the selector needs.

    position is relative to base_link. velocity_body is the tracker's twist
    (linear), expressed in the OBJECT body frame (rotated by `yaw`); it is a
    RELATIVE velocity because the submodule tracker's ego compensation is a
    no-op. dimensions = (length, width, height).
    """

    object_id: int
    position: np.ndarray            # (3,) [m], base_link
    yaw: float                      # [rad], object heading in base_link
    velocity_body: np.ndarray       # (3,) [m/s], object body frame, RELATIVE
    yaw_rate: float                 # [rad/s]
    dimensions: np.ndarray          # (3,) length, width, height [m]
    point_count: int
    track_state: int                # 0 tentative / 1 confirmed / 2 lost
    stamp_sec: float                # track header stamp [s]
    object_class: str = "unknown"
    hit_count: int = 0
    miss_count: int = 0
    velocity_stddev_mps: float = 0.0  # largest horizontal velocity std-dev [m/s]

    @property
    def velocity_base(self) -> np.ndarray:
        """Relative velocity rotated from the object body frame into base_link axes."""
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        v = np.asarray(self.velocity_body, dtype=float)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

    @property
    def distance(self) -> float:
        """Horizontal (xy) distance from base_link."""
        p = np.asarray(self.position, dtype=float)
        return float(np.hypot(p[0], p[1]))


def ego_compensate(
    rel_vel_base: np.ndarray,
    ego_lin_vel_base: np.ndarray,
    ego_yaw_rate: float = 0.0,
    pos_base: np.ndarray = None,
) -> np.ndarray:
    """Relative velocity (base_link axes) -> absolute ground velocity (base_link axes).

    The upstream tracker estimates in the moving base_link frame with ego
    compensation stubbed out, so its velocity contains the full apparent
    motion: -v_ego - omega x r. Restoring ground velocity therefore needs
    both the ego linear velocity AND the rotation-induced term omega x r
    at the target position r (omitting it injects ~omega*range of phantom
    tangential speed while the own ship turns).
    """
    v = np.asarray(rel_vel_base, dtype=float) + np.asarray(ego_lin_vel_base, dtype=float)
    if ego_yaw_rate != 0.0 and pos_base is not None:
        p = np.asarray(pos_base, dtype=float)
        v = v + np.array([-ego_yaw_rate * p[1], ego_yaw_rate * p[0], 0.0])
    return v


def to_map_frame(pos_base: np.ndarray, vel_base: np.ndarray, t_map_base: np.ndarray):
    """Convert a base_link position and velocity into the map frame.

    t_map_base is the 4x4 homogeneous transform map <- base_link. Positions
    get the full transform; velocities are rotated only.
    """
    t = np.asarray(t_map_base, dtype=float)
    rot = t[:3, :3]
    pos_map = rot @ np.asarray(pos_base, dtype=float) + t[:3, 3]
    vel_map = rot @ np.asarray(vel_base, dtype=float)
    return pos_map, vel_map


def predict_straight_motion(position: np.ndarray, velocity: np.ndarray,
                            elapsed_sec: float) -> np.ndarray:
    """Propagate a map-frame position with a constant ground velocity.

    Task 2 uses this only for a short, bounded coast after a confirmed target
    is temporarily occluded.  The caller owns the timeout; this helper does
    not turn an unbounded prediction into a valid observation.
    """
    return (np.asarray(position, dtype=float) +
            np.asarray(velocity, dtype=float) * max(float(elapsed_sec), 0.0))


def compute_cpa_tcpa(own_pos, own_vel, tgt_pos, tgt_vel):
    """Closest Point of Approach between two constant-velocity objects.

    Returns (cpa_distance_m, tcpa_seconds). tcpa < 0 means the objects are
    already diverging; cpa is then the current distance (evaluated at t=0).
    For near-zero relative speed tcpa is 0 and cpa the current distance.
    Only the xy components are used (surface vessels).
    """
    p = (np.asarray(tgt_pos, dtype=float) - np.asarray(own_pos, dtype=float))[:2]
    v = (np.asarray(tgt_vel, dtype=float) - np.asarray(own_vel, dtype=float))[:2]
    v2 = float(v @ v)
    if v2 < 1e-9:
        return float(np.linalg.norm(p)), 0.0
    tcpa = -float(p @ v) / v2
    t_eval = max(tcpa, 0.0)
    cpa = float(np.linalg.norm(p + v * t_eval))
    return cpa, tcpa


@dataclass
class SelectionParams:
    """Gates and knobs for select_opponent (defaults match the node params)."""

    confirmed_only: bool = True
    max_distance_m: float = 60.0
    min_length_m: float = 0.5
    max_length_m: float = 30.0
    min_width_m: float = 0.0
    max_width_m: float = float("inf")
    min_point_count: int = 5
    min_hit_count: int = 0
    max_velocity_stddev_mps: float = float("inf")
    stale_timeout_sec: float = 2.0
    target_track_id: int = -1
    rejected: list = field(default_factory=list)  # (track, reason) diagnostics


def _passes_gates(track: Track, now_sec: float, params: SelectionParams) -> str | None:
    """Returns None when the track passes, otherwise the rejection reason."""
    if params.confirmed_only and track.track_state != TRACK_CONFIRMED:
        return "not confirmed"
    if now_sec - track.stamp_sec > params.stale_timeout_sec:
        return "stale"
    if track.distance > params.max_distance_m:
        return "too far"
    length = float(np.asarray(track.dimensions, dtype=float)[0])
    if length < params.min_length_m:
        return "too small"
    if length > params.max_length_m:
        return "too large"
    width = float(np.asarray(track.dimensions, dtype=float)[1])
    if width < params.min_width_m:
        return "too narrow"
    if width > params.max_width_m:
        return "too wide"
    if track.point_count < params.min_point_count:
        return "too few points"
    if track.hit_count < params.min_hit_count:
        return "insufficient observation history"
    if track.velocity_stddev_mps > params.max_velocity_stddev_mps:
        return "velocity estimate too uncertain"
    return None


def in_oriented_corridor(
    point_xy: np.ndarray,
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    start_offset_m: float,
    end_margin_m: float,
    half_width_m: float,
) -> bool:
    """Return whether a map-frame point is in the GPS5->GPS6 rectangle.

    The rectangle follows the start-to-goal vector. It starts after the
    requested offset from GPS5 and ends after the requested margin past GPS6;
    it is independent of vessel heading.
    """
    start = np.asarray(start_xy, dtype=float)[:2]
    end = np.asarray(end_xy, dtype=float)[:2]
    point = np.asarray(point_xy, dtype=float)[:2]
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length <= 1e-6:
        return False
    forward = direction / length
    relative = point - start
    longitudinal = float(relative @ forward)
    lateral = float(relative[0] * -forward[1] + relative[1] * forward[0])
    return (float(start_offset_m) <= longitudinal <=
            length + float(end_margin_m) and
            abs(lateral) <= float(half_width_m))


def is_left_of_oriented_line(
    point_xy: np.ndarray,
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    margin_m: float = 0.0,
) -> bool:
    """Return whether point is left of the directed start-to-end route line."""
    start = np.asarray(start_xy, dtype=float)[:2]
    end = np.asarray(end_xy, dtype=float)[:2]
    point = np.asarray(point_xy, dtype=float)[:2]
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length <= 1e-6:
        return False
    forward = direction / length
    relative = point - start
    lateral = float(relative[0] * -forward[1] + relative[1] * forward[0])
    return lateral > float(margin_m)


def clip_velocity_bearing(
    velocity_map: np.ndarray,
    reference_yaw_rad: float,
    min_bearing_rad: float,
    max_bearing_rad: float,
) -> np.ndarray:
    """Keep speed while limiting a map-frame velocity bearing to a route yaw.

    Bearings use the normal counter-clockwise-positive convention, so the
    right side of the GPS5->GPS6 route is negative.  ``atan2(sin, cos)``
    normalizes across the +/-pi seam before clipping, including near 0 deg.
    A zero-speed vector has no direction and is returned unchanged.
    """
    if min_bearing_rad > max_bearing_rad:
        raise ValueError("min_bearing_rad must be <= max_bearing_rad")
    velocity = np.asarray(velocity_map, dtype=float).copy()
    speed = float(np.hypot(velocity[0], velocity[1]))
    if speed <= 1e-6:
        return velocity
    bearing = np.arctan2(
        np.sin(np.arctan2(velocity[1], velocity[0]) - reference_yaw_rad),
        np.cos(np.arctan2(velocity[1], velocity[0]) - reference_yaw_rad))
    yaw = reference_yaw_rad + np.clip(bearing, min_bearing_rad, max_bearing_rad)
    velocity[:2] = speed * np.array([np.cos(yaw), np.sin(yaw)])
    return velocity


def select_opponent(
    tracks: list[Track],
    now_sec: float,
    policy: str = "nearest",
    params: SelectionParams | None = None,
) -> list[Track]:
    """Gate and rank candidate opponent tracks. Returns the ranked list
    (best first); the caller takes the top-1 for the single-opponent Task 2
    and publishes NOTHING when the list is empty (safe MPPI degradation).

    Policies:
      "nearest"  — ascending horizontal distance from base_link.
      "min_tcpa" — ascending time to CPA computed from the RELATIVE state
                   (position and relative velocity are both in base_link, so
                   own pos/vel are zero); diverging targets rank last by
                   distance.
      "track_id" — only the track with object_id == params.target_track_id.
    """
    if params is None:
        params = SelectionParams()
    params.rejected = []

    candidates = []
    for track in tracks:
        reason = _passes_gates(track, now_sec, params)
        if reason is None:
            candidates.append(track)
        else:
            params.rejected.append((track, reason))

    if policy == "track_id":
        return [t for t in candidates if t.object_id == params.target_track_id]

    if policy == "min_tcpa":
        def key(t: Track):
            cpa, tcpa = compute_cpa_tcpa(
                np.zeros(3), np.zeros(3), t.position, t.velocity_base)
            if tcpa < 0.0:  # diverging: rank after all approaching targets
                return (1, t.distance)
            return (0, tcpa)
        return sorted(candidates, key=key)

    # Default: "nearest"
    return sorted(candidates, key=lambda t: t.distance)
