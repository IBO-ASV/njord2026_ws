from task1_buoy_roi.geometry import nearest_roi, parse_positions


def test_nearest_roi_uses_the_closest_buoy_within_ten_metres():
    assert parse_positions("[[1, 2], [3, 4]]") == [(1.0, 2.0), (3.0, 4.0)]
    distance, bearing = nearest_roi(0.0, 0.0, 0.0, [(12.0, 0.0), (3.0, 4.0)], 10.0)
    assert distance == 5.0
    assert bearing > 0.0
    assert nearest_roi(0.0, 0.0, 0.0, [(10.1, 0.0)], 10.0) is None
