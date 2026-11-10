"""Small, dependency-free WGS84 helpers for home-relative mission routes."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, radians, sin, sqrt
from pathlib import Path

import yaml


@dataclass(frozen=True)
class HomeDatum:
    latitude: float
    longitude: float
    yaw: float


def load_home_datum(path: Path) -> HomeDatum:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    home = data.get("home") if isinstance(data, dict) else None
    if not isinstance(home, dict):
        raise ValueError(f"{path} must contain a home mapping")
    try:
        latitude = float(home["latitude"])
        longitude = float(home["longitude"])
        yaw = float(home["yaw"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{path} home requires numeric latitude, longitude, and yaw") from exc
    if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
        raise ValueError(f"{path} home latitude/longitude is invalid")
    return HomeDatum(latitude, longitude, yaw)


def _ecef(latitude_deg: float, longitude_deg: float) -> tuple[float, float, float]:
    # WGS84 ellipsoid, altitude intentionally zero: competition routes are planar.
    semi_major = 6378137.0
    eccentricity_sq = 6.69437999014e-3
    lat = radians(latitude_deg)
    lon = radians(longitude_deg)
    radius = semi_major / sqrt(1.0 - eccentricity_sq * sin(lat) ** 2)
    return (
        radius * cos(lat) * cos(lon),
        radius * cos(lat) * sin(lon),
        radius * (1.0 - eccentricity_sq) * sin(lat),
    )


def wgs84_to_enu(latitude: float, longitude: float, datum: HomeDatum) -> tuple[float, float]:
    """Return East/North metres relative to ``datum`` (the map frame)."""
    x, y, z = _ecef(latitude, longitude)
    x0, y0, z0 = _ecef(datum.latitude, datum.longitude)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0 = radians(datum.latitude)
    lon0 = radians(datum.longitude)
    east = -sin(lon0) * dx + cos(lon0) * dy
    north = -sin(lat0) * cos(lon0) * dx - sin(lat0) * sin(lon0) * dy + cos(lat0) * dz
    return east, north


def yaw_from_quaternion(quaternion) -> float:
    """Extract planar ROS ENU yaw from a geometry_msgs Quaternion-like object."""
    return atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )
