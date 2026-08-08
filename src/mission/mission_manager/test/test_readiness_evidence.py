from mission_manager.readiness_evidence import (
    buoy_perception_ready,
    collision_monitor_ready,
    dynamic_gate_tf_ready,
    is_fresh,
)


def test_missing_stale_and_clock_reversal_evidence_is_never_fresh():
    assert not is_fresh(None, 10.0, 1.0)
    assert not is_fresh(8.9, 10.0, 1.0)
    assert not is_fresh(11.0, 10.0, 1.0)


def test_collision_monitor_requires_fresh_valid_state():
    assert collision_monitor_ready(10.0, 10.5, 1.0, 0)
    assert not collision_monitor_ready(10.0, 11.1, 1.0, 0)
    assert not collision_monitor_ready(10.0, 10.5, 1.0, 5)
    assert not collision_monitor_ready(10.0, 10.5, 1.0, None)


def test_perception_accepts_empty_detection_cycles_but_requires_stamped_frame():
    # Detection count is intentionally not an input: an empty, fresh array is
    # proof that the live perception pipeline ran and found no buoy.
    assert buoy_perception_ready(10.0, 10.5, 1.0, 10.0, "base_link")
    assert not buoy_perception_ready(10.0, 10.5, 1.0, None, "base_link")
    assert not buoy_perception_ready(10.0, 10.5, 1.0, 10.0, "")
    assert not buoy_perception_ready(10.0, 11.1, 1.0, 10.0, "base_link")


def test_dynamic_gate_requires_every_configured_tf_to_be_fresh():
    assert dynamic_gate_tf_ready((10.0, 10.1, 10.2, 10.3), 10.5, 1.0)
    assert not dynamic_gate_tf_ready((10.0, None), 10.5, 1.0)
    assert not dynamic_gate_tf_ready((10.0, 8.0), 10.5, 1.0)
    assert not dynamic_gate_tf_ready((), 10.5, 1.0)
