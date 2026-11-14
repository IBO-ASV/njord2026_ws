"""Sanity checks on the sensor_build_support build-time manifest (platform.yaml).

sensor_build_support writes share/sensor_build_support/platform.yaml at configure time
(see src/build_tools/sensor_build_support/cmake/SensorBuildSupport.cmake,
sensor_write_build_manifest()). This test only verifies the manifest
produced by *this* build/workspace is well-formed; it is not a check on the
robot package's own config.
"""

import pytest

try:
    import yaml
except ImportError:
    yaml = None

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ImportError:  # pragma: no cover - ament_index_python should always be available
    get_package_share_directory = None
    PackageNotFoundError = Exception

import os

_REQUIRED_KEYS = [
    "ros_distro",
    "profile",
    "has_cuda",
    "cuda_version",
    "cuda_variant",
    "has_tensorrt",
    "has_zed_sdk",
    "has_glim",
    "has_livox",
]

_REQUIRED_BOOL_KEYS = [
    "has_cuda",
    "has_tensorrt",
    "has_zed_sdk",
    "has_glim",
    "has_livox",
]


def _load_platform_manifest():
    if yaml is None:
        pytest.skip("PyYAML is not available")
    if get_package_share_directory is None:
        pytest.skip("ament_index_python is not available")

    try:
        share_dir = get_package_share_directory("sensor_build_support")
    except PackageNotFoundError:
        pytest.skip("sensor_build_support package not found (not built/sourced in this environment)")

    manifest_path = os.path.join(share_dir, "platform.yaml")
    if not os.path.isfile(manifest_path):
        pytest.skip(f"platform.yaml not found at {manifest_path}")

    with open(manifest_path, "r") as f:
        return yaml.safe_load(f)


def test_platform_manifest_has_required_keys():
    manifest = _load_platform_manifest()
    assert isinstance(manifest, dict)
    for key in _REQUIRED_KEYS:
        assert key in manifest, f"platform.yaml is missing required key '{key}'"


def test_platform_manifest_profile_is_valid():
    manifest = _load_platform_manifest()
    assert manifest["profile"] in ("jetson", "minipc")


def test_platform_manifest_bool_keys_are_actual_booleans():
    manifest = _load_platform_manifest()
    for key in _REQUIRED_BOOL_KEYS:
        value = manifest[key]
        assert isinstance(value, bool), (
            f"platform.yaml key '{key}' should parse as a YAML bool, got "
            f"{value!r} ({type(value).__name__}); check that SensorBuildSupport.cmake "
            f"emits lowercase true/false, not ON/OFF"
        )
