from pathlib import Path


def test_raw_um982_heading_is_logged_without_waiting_for_ekf_odometry():
    source = (Path(__file__).parents[1] / "src" / "foxglove_logger_node.cpp").read_text()

    assert '"heading_topic", "/sensor/vehicle_gnss/compass/raw"' in source
    assert "heading_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>" in source
    assert "heading_orientation_ ? *heading_orientation_" in source


def test_task2_target_uses_its_follow_path_plan():
    source = (Path(__file__).parents[1] / "src" / "foxglove_logger_node.cpp").read_text()

    assert '"task2_plan_topic", "/planned_path_pruned"' in source
    assert 'mission_status_->task_id == "task2"' in source
    assert 'return task2_plan_ ? &*task2_plan_ : nullptr;' in source
    assert 'out << "task2 ";' in source
