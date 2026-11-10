import math

from buoy_obstacle_publisher.cardinal_wall_geometry import (
    is_behind_retirement_frontier,
    wall_points,
)


def test_north_marker_blocks_the_south_side():
    # With a GNSS course heading north, N means pass forward/north, so the
    # virtual wall blocks the south/back side.
    points = wall_points(
        [-5.0, 5.0, -4.0, 4.0], 0.2, 0.1, 1.0, 2.0, 2, math.pi / 2.0)
    assert points
    assert min(y for _, y, _ in points) == -4.0
    assert max(y for _, y, _ in points) == 2.0


def test_east_marker_blocks_the_west_side():
    points = wall_points(
        [-5.0, 5.0, -4.0, 4.0], 0.2, 0.1, 1.0, 2.0, 3, math.pi / 2.0)
    assert points
    assert min(x for x, _, _ in points) == -5.0
    assert max(x for x, _, _ in points) == 1.0


def test_true_north_rotation_is_applied_in_map_frame():
    # A map whose true-north axis points along +x must rotate a north-marker
    # south-wall onto the -x side.
    points = wall_points([-5.0, 5.0, -4.0, 4.0], 0.2, 0.1, 1.0, 2.0, 2, 0.0)
    assert min(x for x, _, _ in points) == -5.0
    assert max(x for x, _, _ in points) == 1.0


def test_thirty_degree_true_north_offset_rotates_cardinal_wall():
    # In the Task1 simulator true north is 30 degrees clockwise from map +Y,
    # i.e. pi/3 from map +X.  A north marker's forbidden south ray must not
    # remain aligned to map -Y.
    points = wall_points([-5.0, 5.0, -5.0, 5.0], 0.2, 0.1, 0.0, 0.0, 2, math.pi / 3.0)
    assert min(y for _, y, _ in points) <= -5.0
    assert min(x for x, _, _ in points) < -2.0


def test_red_and_green_block_opposite_sides_of_the_course():
    # Course heading west: port is south and starboard is north.
    red = wall_points([-5.0, 5.0, -4.0, 4.0], 0.2, 0.1, 1.0, 0.0, 1, math.pi)
    green = wall_points([-5.0, 5.0, -4.0, 4.0], 0.2, 0.1, 1.0, 0.0, 0, math.pi)
    # Closing red's port/south side makes the vessel pass north, leaving red
    # on port. Green closes north/starboard and makes the vessel pass south.
    assert min(y for _, y, _ in red) == -4.0
    assert max(y for _, y, _ in green) == 4.0


def test_wall_length_is_capped_at_thirteen_metres():
    points = wall_points([-100.0, 100.0, -100.0, 100.0], 0.2, 0.1, 0.0, 0.0, 2, 0.0)
    assert points
    assert min(x for x, _, _ in points) == -13.0
    assert max(x for x, _, _ in points) == 0.0


def test_reached_next_waypoint_retires_only_cardinal_marks_behind_it():
    # The Task1.2 main course heads west (WP3 -> WP4). Once the next
    # waypoint is reached at x=24, the marker at x=28 has been passed, while
    # a later marker at x=18 must retain its wall.
    assert is_behind_retirement_frontier(28.0, -25.0, 24.0, -35.0, math.pi, 0.5)
    assert not is_behind_retirement_frontier(18.0, -25.0, 24.0, -35.0, math.pi, 0.5)
