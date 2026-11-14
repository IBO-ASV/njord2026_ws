"""Regression checks for the miniPC localization diagnostics profile."""

import importlib.util
import os


def _load_diagnostics_module():
    path = os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "launch", "diagnostics.launch.py")
    )
    spec = importlib.util.spec_from_file_location("_njord_diagnostics_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_localization_profile_only_requires_minipc_localization_topics():
    module = _load_diagnostics_module()
    topics = {monitor["topic"] for monitor in module.PROFILES["localization"]}

    assert topics == {
        "/sensor/vehicle_gnss/fix/raw",
        "/odometry/filtered/local",
        "/odometry/filtered/global",
    }


def test_nav2_profile_monitors_its_configured_local_odometry():
    module = _load_diagnostics_module()
    topics = {monitor["topic"] for monitor in module.PROFILES["nav2"]}

    assert "/odometry/filtered/local" in topics
    assert "/odometry/filtered/global" not in topics


def test_legacy_task3_profile_has_no_unvalidated_monitors():
    assert _load_diagnostics_module().PROFILES["task3"] == []
