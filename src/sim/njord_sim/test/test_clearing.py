import math

import numpy as np
import pytest
import yaml
from njord_sim.clearing import (
    SCAN_BEAMS,
    SCAN_RANGE,
    clearing_ranges,
)
from njord_sim.configuration import configurations


def test_free_beams_cover_full_circle_without_marking_water():
    result = clearing_ranges((0.0, 0.0, 0.0), [])
    assert len(result) == SCAN_BEAMS
    assert np.all((result > 99.9) & (result < SCAN_RANGE))


def test_beams_stop_before_nearest_surface_and_do_not_clear_behind_it():
    ahead = SCAN_BEAMS // 2
    buoy = clearing_ranges((0.0, 0.0, 0.0), [(5.0, 0.0, "RED")])
    assert buoy[ahead] == pytest.approx(5 - 0.175 - 0.03)
    blocked = clearing_ranges(
        (0.0, 0.0, 0.0), [(10.0, 0.0, "RED")], (5.0, 0.0, 0.0, 2.0, 1.08)
    )
    assert blocked[ahead] == pytest.approx(4 - 0.03)
    rotated = clearing_ranges((0.0, 0.0, 0.0), [], (5.0, 0.0, math.pi / 2, 2.0, 1.08))
    assert rotated[ahead] == pytest.approx(5 - 0.54 - 0.03)
    assert blocked[0] > 99.9


def test_origin_inside_obstacle_does_not_create_fictitious_free_rays():
    assert np.isnan(clearing_ranges((0.0, 0.0, 0.0), [(0.0, 0.0)])).all()
    assert np.isnan(
        clearing_ranges((0.0, 0.0, 0.0), [], (0.0, 0.0, 0.0, 2.0, 1.0))
    ).all()


def test_ranges_are_covariant_under_rotation_and_translation():
    before = clearing_ranges((0.0, 0.0, 0.0), [(7.0, 2.0)], (5.0, -1.0, 0.3, 2.0, 1.08))
    a = 0.8

    def point(x, y):
        return (
            10 + x * math.cos(a) - y * math.sin(a),
            -4 + x * math.sin(a) + y * math.cos(a),
        )

    after = clearing_ranges(
        (10.0, -4.0, a), [point(7.0, 2.0)], (*point(5.0, -1.0), 0.3 + a, 2.0, 1.08)
    )
    np.testing.assert_allclose(before, after, atol=1e-5)


def test_only_task2_costmaps_gain_the_clear_only_source(tmp_path):
    for task in ["task1", "task2"]:
        path, _, _ = configurations(task, tmp_path / task)
        nav = yaml.safe_load(open(path))
        for name in ["global_costmap", "local_costmap"]:
            params = nav[name][name]["ros__parameters"]
            layer = params["obstacle_layer"]
            if task == "task2":
                assert params["publish_frequency"] == 10.0
                assert params["update_frequency"] == 5.0
                assert not params["always_send_full_costmap"]
                assert layer["free_space"]["clearing"]
                assert not layer["free_space"]["marking"]
                assert layer["free_space"]["raytrace_max_range"] == SCAN_RANGE
            else:
                assert "free_space" not in layer
