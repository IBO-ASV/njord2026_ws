from pathlib import Path

import pytest

from control_manager.policy import (
    PolicyError,
    SafetyInputs,
    evaluate_auto_permission,
    load_policy,
)


POLICY = Path(__file__).parents[1] / "config" / "control_policy.yaml"
INHIBITS = {
    "emergency_stop": 1,
    "nav2_not_ready": 2,
    "task_not_ready": 3,
    "require_ground_station": 5,
    "require_driver_heartbeat": 6,
    "require_localization_heartbeat": 7,
    "require_rtk_fix": 8,
    "critical_health": 9,
    "task_requirement": 10,
}


def test_default_policy_is_complete_and_task_requirements_overlay_common():
    policy = load_policy(POLICY)
    requirements = policy.requirements_for("task2")
    assert requirements["require_ground_station"] is True
    assert requirements["require_collision_monitor"] is False
    assert requirements["require_buoy_perception"] is False
    assert policy.nav_command_timeout_sec == 2.0


def test_return_home_does_not_require_ground_station_heartbeat():
    requirements = load_policy(POLICY).requirements_for("return_home")
    assert requirements["require_ground_station"] is False


def test_duplicate_or_unknown_policy_content_is_rejected(tmp_path):
    duplicate = tmp_path / "duplicate.yaml"
    duplicate.write_text("control_policy: {}\ncontrol_policy: {}\n", encoding="utf-8")
    with pytest.raises(PolicyError, match="duplicate"):
        load_policy(duplicate)

    incomplete = tmp_path / "incomplete.yaml"
    incomplete.write_text("control_policy:\n  common: {}\n", encoding="utf-8")
    with pytest.raises(PolicyError, match="exactly"):
        load_policy(incomplete)


def test_auto_permission_lists_all_failed_interlocks():
    requirements = load_policy(POLICY).requirements_for("task1")
    decision = evaluate_auto_permission(
        requirements,
        SafetyInputs(
            emergency_stop=True,
            nav2_ready=False,
            task_ready=False,
            task_requirements_ready=False,
        ),
        health_ok_state=1,
        health_disabled_state=5,
        inhibit_codes=INHIBITS,
    )
    assert not decision.auto_permitted
    assert [code for code, _ in decision.reasons] == [1, 5, 10]


def test_no_task_policy_does_not_evaluate_task_specific_requirements():
    requirements = load_policy(POLICY).requirements_for("")
    decision = evaluate_auto_permission(
        requirements,
        SafetyInputs(emergency_stop=False, nav2_ready=True, task_ready=True),
        health_ok_state=1,
        health_disabled_state=5,
        inhibit_codes=INHIBITS,
    )
    assert decision.auto_permitted


def test_auto_permission_requires_ok_enabled_health_signal():
    policy_path = Path(__file__).parents[1] / "config" / "control_policy.yaml"
    policy = load_policy(policy_path)
    requirements = dict(policy.requirements_for("task1"))
    requirements["require_driver_heartbeat"] = True
    decision = evaluate_auto_permission(
        requirements,
        SafetyInputs(
            emergency_stop=False,
            nav2_ready=True,
            task_ready=True,
            task_requirements_ready=True,
            health_states={"ground_station_heartbeat": 1, "driver_heartbeat": 4},
        ),
        health_ok_state=1,
        health_disabled_state=5,
        inhibit_codes=INHIBITS,
    )
    assert not decision.auto_permitted
    assert "driver_heartbeat" in decision.reasons[0][1]
    assert decision.reasons[0][0] == 6


def test_auto_permission_does_not_wait_for_a_navigation_command():
    requirements = load_policy(POLICY).requirements_for("task1")
    decision = evaluate_auto_permission(
        requirements,
        SafetyInputs(
            emergency_stop=False,
            nav2_ready=True,
            task_ready=True,
            task_requirements_ready=True,
            health_states={"ground_station_heartbeat": 1},
        ),
        health_ok_state=1,
        health_disabled_state=5,
        inhibit_codes=INHIBITS,
    )
    assert decision.auto_permitted
    assert decision.reasons == ()
