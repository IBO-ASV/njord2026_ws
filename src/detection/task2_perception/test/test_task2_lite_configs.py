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
    assert cluster["min_cluster_size"] == 30
    assert tracker["straight_hits_to_confirm"] == 10
    selector = params["opponent_selector"]["ros__parameters"]
    assert selector["straight_min_hit_count"] == 10
    assert selector["straight_coast_timeout_sec"] == 2.0
