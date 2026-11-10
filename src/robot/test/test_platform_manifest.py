"""Sanity checks on the njord_platform build-time manifest (platform.yaml).

njord_platform writes share/njord_platform/platform.yaml at configure time
(see src/njord_platform/cmake/NjordPlatform.cmake,
njord_write_platform_manifest()). This test only verifies the manifest
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
        share_dir = get_package_share_directory("njord_platform")
    except PackageNotFoundError:
        pytest.skip("njord_platform package not found (not built/sourced in this environment)")

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
            f"{value!r} ({type(value).__name__}); check that NjordPlatform.cmake "
            f"emits lowercase true/false, not ON/OFF"
        )
