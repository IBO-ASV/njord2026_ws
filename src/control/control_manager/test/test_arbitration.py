import pytest

from control_manager.arbitration import (
    State, Source, compatibility_control_status, derive_control_state, select_source,
)


def test_compatibility_control_status_uses_requested_mode_except_emergency_stop():
    assert compatibility_control_status(emergency_stop=False, requested_mode=0) == "manual"
    assert compatibility_control_status(emergency_stop=False, requested_mode=1) == "auto"
    assert compatibility_control_status(emergency_stop=True, requested_mode=1) == "emergency_stop"


@pytest.mark.parametrize(
    ("kwargs", "expected"),
    [
        ({"emergency_stop": True, "effective_source": Source.AUTO, "auto_permitted": True,
          "manual_command_fresh": True, "nav_command_fresh": True}, Source.ZERO),
        ({"emergency_stop": False, "effective_source": Source.MANUAL, "auto_permitted": False,
          "manual_command_fresh": True, "nav_command_fresh": False}, Source.MANUAL),
        ({"emergency_stop": False, "effective_source": Source.AUTO, "auto_permitted": True,
          "manual_command_fresh": False, "nav_command_fresh": True}, Source.AUTO),
        ({"emergency_stop": False, "effective_source": Source.AUTO, "auto_permitted": True,
          "manual_command_fresh": False, "nav_command_fresh": False}, Source.ZERO),
        ({"emergency_stop": False, "effective_source": Source.AUTO, "auto_permitted": False,
          "manual_command_fresh": False, "nav_command_fresh": True}, Source.ZERO),
    ],
)
def test_only_authorized_fresh_source_is_forwarded(kwargs, expected):
    assert select_source(**kwargs) == expected


def test_auto_armed_is_reachable_before_nav_produces_a_command():
    state, source = derive_control_state(
        emergency_stop=False,
        auto_requested=True,
        auto_permitted=True,
        manual_command_fresh=False,
        nav_command_fresh=False,
    )
    assert state == State.AUTO_ARMED
    assert source == Source.ZERO


def test_fresh_sbus_command_overrides_manual_and_auto_sources():
    state, source = derive_control_state(
        emergency_stop=False,
        auto_requested=True,
        auto_permitted=True,
        manual_command_fresh=True,
        nav_command_fresh=True,
        sbus_command_fresh=True,
    )
    assert state == State.MANUAL
    assert source == Source.SBUS
    assert select_source(
        emergency_stop=False,
        effective_source=source,
        auto_permitted=True,
        manual_command_fresh=True,
        nav_command_fresh=True,
        sbus_command_fresh=True,
    ) == Source.SBUS


def test_arbiter_outputs_zero_when_armed_nav_command_is_stale():
    _, effective_source = derive_control_state(
        emergency_stop=False,
        auto_requested=True,
        auto_permitted=True,
        manual_command_fresh=False,
        nav_command_fresh=True,
    )
    assert effective_source == Source.AUTO
    assert select_source(
        emergency_stop=False,
        effective_source=effective_source,
        auto_permitted=True,
        manual_command_fresh=False,
        nav_command_fresh=False,
    ) == Source.ZERO
