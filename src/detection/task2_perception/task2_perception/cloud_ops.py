"""Pure-numpy point cloud operations for Task 2 perception.

This module MUST stay free of any ROS (rclpy / message) dependency so that it
can be unit-tested with plain pytest + numpy on any machine.

Point clouds are represented as (N, 3) [x, y, z] or (N, 4) [x, y, z,
intensity] float arrays. Geometry always uses the first three columns; extra
columns (intensity) are carried through untouched.
"""

from __future__ import annotations

import numpy as np


# ---------------------------------------------------------------------------
# Rigid transforms
# ---------------------------------------------------------------------------

def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert a (x, y, z, w) quaternion to a 3x3 rotation matrix."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """XYZ fixed-axis (extrinsic) roll-pitch-yaw to rotation matrix (URDF convention).

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def make_transform(rotation: np.ndarray, translation) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a 3x3 rotation and a translation."""
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = np.asarray(translation, dtype=float)
    return t


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to an (N, 3) or (N, 4) point array.

    Only the xyz columns are transformed; any extra columns (intensity) are
    passed through unchanged.
    """
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts.reshape(0, pts.shape[1] if pts.ndim == 2 else 3)
    out = pts.copy()
    out[:, :3] = pts[:, :3] @ transform[:3, :3].T + transform[:3, 3]
    return out


# ---------------------------------------------------------------------------
# Filters (each returns the filtered array; *_mask helpers return masks)
# ---------------------------------------------------------------------------

def finite_mask(points: np.ndarray) -> np.ndarray:
    """Mask of rows without NaN/Inf in any column."""
    return np.all(np.isfinite(points), axis=1)


def remove_nonfinite(points: np.ndarray) -> np.ndarray:
    """Drop rows containing NaN/Inf."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts
    return pts[finite_mask(pts)]


def range_filter(points: np.ndarray, min_range_m: float, max_range_m: float) -> np.ndarray:
    """Keep points whose 3D Euclidean range from the origin is within the band."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts
    r = np.linalg.norm(pts[:, :3], axis=1)
    return pts[(r >= min_range_m) & (r <= max_range_m)]


def _inside_mask(points: np.ndarray, min_xyz, max_xyz) -> np.ndarray:
    lo = np.asarray(min_xyz, dtype=float)
    hi = np.asarray(max_xyz, dtype=float)
    xyz = np.asarray(points, dtype=float)[:, :3]
    return np.all((xyz >= lo) & (xyz <= hi), axis=1)


def crop_box_remove(points: np.ndarray, min_xyz, max_xyz) -> np.ndarray:
    """Remove points INSIDE the axis-aligned box (self-hull removal)."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts
    return pts[~_inside_mask(pts, min_xyz, max_xyz)]


def crop_box_keep(points: np.ndarray, min_xyz, max_xyz) -> np.ndarray:
    """Keep only points INSIDE the axis-aligned box (e.g. object height band)."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts
    return pts[_inside_mask(pts, min_xyz, max_xyz)]


def voxel_downsample(points: np.ndarray, leaf_size_m: float) -> np.ndarray:
    """Hash-grid voxel downsampling: one mean point per occupied voxel.

    All columns (xyz + intensity) are averaged per voxel. leaf_size_m <= 0
    returns the input unchanged (downsampling disabled).
    """
    pts = np.asarray(points, dtype=float)
    if pts.size == 0 or leaf_size_m <= 0.0:
        return pts
    cells = np.floor(pts[:, :3] / leaf_size_m).astype(np.int64)
    _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    out = np.empty((counts.size, pts.shape[1]), dtype=float)
    for col in range(pts.shape[1]):
        out[:, col] = np.bincount(inverse, weights=pts[:, col]) / counts
    return out


def frame_timestamp_decision(
    stamp_ns: int,
    last_stamp_ns: int | None,
    min_period_ns: int,
) -> tuple[bool, bool]:
    """Decide whether a cloud may pass a timestamp-based rate gate.

    Returns ``(process, clock_rolled_back)``.  rosbag ``--loop`` restarts
    message timestamps at the beginning of the recording.  Such a backwards
    jump must reset the rate gate instead of being discarded forever as a
    negative time interval.
    """
    if last_stamp_ns is None or min_period_ns <= 0:
        return True, False
    if stamp_ns < last_stamp_ns:
        return True, True
    return stamp_ns - last_stamp_ns >= min_period_ns, False


# ---------------------------------------------------------------------------
# Water surface removal
# ---------------------------------------------------------------------------

def fit_horizontal_plane_ransac(
    points: np.ndarray,
    distance_threshold_m: float = 0.05,
    normal_z_min: float = 0.9,
    waterline_z_m: float = 0.0,
    max_height_error_m: float = 0.3,
    max_iterations: int = 100,
    min_inliers: int = 30,
    rng: np.random.Generator | None = None,
):
    """Minimal RANSAC that finds ONLY a near-horizontal plane near the waterline.

    A candidate plane is accepted only if BOTH guards hold:
      * |normal_z| >= normal_z_min  (near-horizontal plane; vertical quay walls
        are structurally excluded and can never be removed), and
      * its height (z at the origin, -d/nz for the unit normal) lies within
        waterline_z_m +/- max_height_error_m (a deck plane well above the
        waterline is never removed).

    Returns (inlier_mask, plane) where plane = (nx, ny, nz, d) with unit normal
    (nz forced positive) satisfying n.p + d = 0, or (all-False mask, None)
    when no acceptable plane is found. `points` may be (N, 3) or (N, 4).
    """
    pts = np.asarray(points, dtype=float)
    xyz = pts[:, :3] if pts.size else pts.reshape(0, 3)
    n_pts = xyz.shape[0]
    no_mask = np.zeros(n_pts, dtype=bool)
    if n_pts < 3:
        return no_mask, None
    if rng is None:
        rng = np.random.default_rng(0)

    best_mask = None
    best_count = 0
    best_plane = None

    for _ in range(max_iterations):
        idx = rng.choice(n_pts, size=3, replace=False)
        p0, p1, p2 = xyz[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue  # degenerate sample
        normal = normal / norm
        if normal[2] < 0:
            normal = -normal
        # Horizontality guard: never accept vertical planes (quay walls).
        if normal[2] < normal_z_min:
            continue
        d = -float(normal @ p0)
        # Height guard: only planes near the expected waterline.
        plane_z = -d / normal[2]
        if abs(plane_z - waterline_z_m) > max_height_error_m:
            continue
        dist = np.abs(xyz @ normal + d)
        mask = dist <= distance_threshold_m
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
            best_plane = (float(normal[0]), float(normal[1]), float(normal[2]), d)

    if best_mask is None or best_count < min_inliers:
        return no_mask, None

    # Least-squares refinement on the inliers (mirrors the line fitter): the
    # 3-point sample plane can be slightly tilted and clip edge inliers. The
    # refined plane must still pass BOTH guards and not lose inliers,
    # otherwise the sample plane is kept.
    inliers = xyz[best_mask]
    centroid = inliers.mean(axis=0)
    try:
        _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    except np.linalg.LinAlgError:
        return best_mask, best_plane
    normal = vt[-1]
    norm = np.linalg.norm(normal)
    if norm > 1e-12:
        normal = normal / norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] >= normal_z_min:
            d = -float(normal @ centroid)
            plane_z = -d / normal[2]
            if abs(plane_z - waterline_z_m) <= max_height_error_m:
                dist = np.abs(xyz @ normal + d)
                mask = dist <= distance_threshold_m
                if int(mask.sum()) >= best_count:
                    return mask, (float(normal[0]), float(normal[1]),
                                  float(normal[2]), d)
    return best_mask, best_plane


def water_removal(
    points: np.ndarray,
    waterline_z_m: float = 0.0,
    band_min_m: float = -0.3,
    band_max_m: float = 0.15,
    use_ransac: bool = True,
    distance_threshold_m: float = 0.05,
    normal_z_min: float = 0.9,
    max_height_error_m: float = 0.3,
    iterations: int = 100,
    min_inliers: int = 30,
    rng: np.random.Generator | None = None,
):
    """Remove water-surface returns. Returns (kept, rejected_water).

    Two mechanisms, both anchored at waterline_z_m:
      1. z-band removal: points with z in
         [waterline_z_m + band_min_m, waterline_z_m + band_max_m] are dropped.
      2. optional RANSAC horizontal-plane removal with strict guards (see
         fit_horizontal_plane_ransac). Vertical quay walls and any deck plane
         far above the waterline SURVIVE by construction.

    `points` may be (N, 3) or (N, 4); columns are preserved.
    """
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts, pts
    z = pts[:, 2]
    remove = (z >= waterline_z_m + band_min_m) & (z <= waterline_z_m + band_max_m)
    if use_ransac:
        mask, plane = fit_horizontal_plane_ransac(
            pts,
            distance_threshold_m=distance_threshold_m,
            normal_z_min=normal_z_min,
            waterline_z_m=waterline_z_m,
            max_height_error_m=max_height_error_m,
            max_iterations=iterations,
            min_inliers=min_inliers,
            rng=rng,
        )
        if plane is not None:
            remove |= mask
    return pts[~remove], pts[remove]
