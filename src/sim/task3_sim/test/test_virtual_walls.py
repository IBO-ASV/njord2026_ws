import math

from task3_sim.virtual_walls import task3_virtual_wall_points


def test_task3_walls_keep_each_course_interior_open_and_connect_colours():
    buoys = [
        {"name": "b31_red_1", "x": 0.0, "y": 4.0},
        {"name": "b31_red_2", "x": 4.0, "y": 4.0},
        {"name": "b31_green_1", "x": 0.0, "y": -4.0},
        {"name": "b31_green_2", "x": 4.0, "y": -4.0},
        {"name": "b32_red_1", "x": 0.0, "y": -4.0},
        {"name": "b32_green_1", "x": 0.0, "y": 4.0},
    ]
    wall = task3_virtual_wall_points(buoys, radius=2.0, max_gap_m=10.0, point_spacing_m=1.0)
    assert len(wall) == 136
    # Task3.1 travels east: red north / green south are outside the channel.
    assert (0.0, 4.0, 0.0) in wall
    assert (2.0, 4.0, 0.0) in wall
    assert (2.0, -4.0, 0.0) in wall
    # Task3.2 travels west: red south / green north are outside the channel.
    assert any(math.isclose(point[1], -6.0) for point in wall)
    assert any(math.isclose(point[1], 6.0) for point in wall)
