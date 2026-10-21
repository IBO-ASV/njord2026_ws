"""Regression checks for the Task 2 CPU-light perception profile."""

from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"


def _params():
    return yaml.safe_load((CONFIG_DIR / "task2_params.yaml").read_text())


def test_cloud_filter_profile_reduces_work_without_debug_changes():
    params = _params()
    cloud = params["task2_cloud_filter"]["ros__parameters"]
    assert cloud["max_range_m"] == 40.0
    assert cloud["process_rate_hz"] == 5.0
    assert cloud["use_water_plane_ransac"] is False
    assert cloud["publish_debug"] is False
    assert "visual_output_topic" not in cloud


def test_task2_lite_pipeline_uses_latest_compact_clouds():
    params = _params()
    preprocessing = params["preprocessing_node"]["ros__parameters"]
    ground = params["ground_remover_node"]["ros__parameters"]
    cluster = params["cluster_node"]["ros__parameters"]
    tracker = params["ship_tracker_node"]["ros__parameters"]
    assert preprocessing["voxel_leaf_x"] == 0.15
    assert preprocessing["accumulate_frames"] is False
    assert preprocessing["input_queue_depth"] == 1
    assert ground["input_queue_depth"] == 1
    assert ground["ransac_max_iterations"] == 80
    assert cluster["input_queue_depth"] == 1
    assert cluster["min_cluster_size"] == 20
    assert cluster["min_cluster_size_far"] == 3
    assert cluster["far_cluster_distance_m"] == 40.0
    assert tracker["publish_tentative"] is True
    assert tracker["ego_pose_compensation"] is True
    assert tracker["ego_odom_max_age_sec"] == 0.15
    assert tracker["min_marker_speed_knots"] == 0.0
    assert tracker["max_misses_tentative"] == 4
    assert tracker["straight_hits_to_confirm"] == 10
    assert tracker["max_misses_confirmed"] == 20
    selector = params["opponent_selector"]["ros__parameters"]
    assert selector["tracked_objects_in_map_frame"] is False
    assert selector["selected_marker_topic"] == "/task2/selected_opponent_marker"
    assert selector["velocity_marker_topic"] == "/task2/selected_opponent_velocity_marker"
    assert selector["straight_min_hit_count"] == 5
    assert selector["straight_max_velocity_stddev_mps"] == 0.60
    assert selector["straight_coast_timeout_sec"] == 3.0
    assert selector["stale_timeout_sec"] == 4.0
    assert selector["min_length_m"] == 0.5
    assert selector["max_length_m"] == 3.5
    assert selector["min_width_m"] == 0.3
    assert selector["max_width_m"] == 2.2
    assert selector["min_absolute_speed_knots"] == 1.5
    assert selector["max_absolute_speed_knots"] == 3.5
    assert selector["corridor_enabled"] is True
    assert selector["corridor_start_offset_m"] == 5.0
    assert selector["corridor_end_margin_m"] == 5.0
    assert selector["corridor_half_width_m"] == 20.0
    assert selector["corridor_ignore_left_side"] is True
    assert selector["corridor_left_side_margin_m"] == 5.0
    assert selector["clip_opponent_bearing_to_corridor"] is True
    assert selector["opponent_bearing_min_deg"] == 90.0
    assert selector["opponent_bearing_max_deg"] == 180.0
    relay = params["task2_waypoint_map_frame_relay"]["ros__parameters"]
    assert relay["waypoint1_input_topic"] == "/waypoint1_pose"
    assert relay["waypoint1_output_topic"] == "/task2/waypoint1_pose_map"
    assert relay["target_frame"] == "map"
    buoy = params["task2_buoy_selector"]["ros__parameters"]
    assert buoy["tracked_objects_in_map_frame"] is False
    assert selector["corridor_start_topic"] == relay["waypoint1_output_topic"]
    assert buoy["waypoint_start_topic"] == relay["waypoint1_output_topic"]
    assert buoy["waypoint_end_topic"] == relay["waypoint2_output_topic"]
    assert buoy["expected_lateral_offset_m"] == 2.5
    assert buoy["lateral_tolerance_m"] == 1.5
    assert buoy["start_margin_m"] == 0.0
    assert buoy["end_margin_m"] == 0.0
    assert buoy["stationary_speed_max_mps"] == 0.35
    assert buoy["stationary_confirmation_window_sec"] == 3.0
    assert buoy["stationary_confirmation_min_observations"] == 5
    assert buoy["min_point_count"] == 5
    assert buoy["max_length_m"] == 1.2
    assert buoy["max_width_m"] == 1.2
