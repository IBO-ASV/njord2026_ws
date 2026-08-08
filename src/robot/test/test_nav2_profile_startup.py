"""Regression tests for startup-only Nav2 profile selection."""

import importlib.util
import sys
from pathlib import Path

import pytest
import yaml


ROBOT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROBOT_ROOT.parent
MISSION_ROOT = SOURCE_ROOT / "mission" / "mission_manager"
CATALOG_FILE = MISSION_ROOT / "config" / "nav2_profiles" / "profiles.yaml"


def _load_nav2_launch_module():
    sys.path.insert(0, str(MISSION_ROOT))
    path = ROBOT_ROOT / "launch" / "nav2.launch.py"
    spec = importlib.util.spec_from_file_location("_nav2_profile_startup_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("profile", ["task1", "task2", "task3"])
@pytest.mark.parametrize("distro", ["humble", "jazzy"])
def test_nav2_launch_materializes_the_selected_catalog_profile(profile, distro):
    module = _load_nav2_launch_module()
    generated = Path(module._write_resolved_profile(CATALOG_FILE, profile, distro))
    try:
        actual = yaml.safe_load(generated.read_text(encoding="utf-8"))
        expected = module.Nav2ProfileCatalog.from_file(CATALOG_FILE).resolve(
            profile, distro
        )
        assert actual == expected
        assert f"nav2_{profile}_{distro}_" in generated.name
    finally:
        generated.unlink(missing_ok=True)


def test_nav2_launch_rejects_an_unknown_profile_before_writing_params():
    module = _load_nav2_launch_module()
    with pytest.raises(ValueError, match="unknown Nav2 profile"):
        module._write_resolved_profile(CATALOG_FILE, "task4", "jazzy")


def test_minipc_passes_one_profile_identity_to_nav2_and_mission_manager():
    source = (ROBOT_ROOT / "launch" / "minipc_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert source.count('"active_nav2_profile": active_nav2_profile') == 2
    assert 'default_value="task1"' in source
    assert 'choices=["task1", "task2", "task3"]' in source


def test_nav2_full_parameter_override_is_explicitly_compatibility_only():
    source = (ROBOT_ROOT / "launch" / "nav2.launch.py").read_text(encoding="utf-8")
    assert "default_value=''" in source
    assert "Compatibility only: explicit full Nav2 parameter file" in source
    assert "Nav2ProfileCatalog.from_file" in source
