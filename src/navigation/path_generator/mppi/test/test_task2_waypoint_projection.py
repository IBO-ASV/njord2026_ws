"""Regression contracts for the real-vessel Task2 waypoint source."""

from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "asv_trajectory_planner" /
          "task2_waypoint_pose_publisher.py").read_text(encoding="utf-8")


def test_task2_projects_both_gps_waypoints_after_fromll_is_ready():
    assert "self.from_ll_client.service_is_ready()" in SOURCE
    assert "self.projection_futures = [self.from_ll_client.call_async" in SOURCE
    assert "all(future.done() for future in self.projection_futures)" in SOURCE


def test_task2_projection_order_is_start_then_goal():
    assert "points = [self.start_wp, self.goal_wp]" in SOURCE
    assert "Resolved Task2 latitude/longitude waypoints in map frame." in SOURCE


def test_task2_final_completion_requires_a_path_ending_at_gps6():
    source = (Path(__file__).parents[1] / "asv_trajectory_planner" /
              "follow_path_client_node.py").read_text(encoding="utf-8")
    assert 'self.declare_parameter("final_goal_pose_topic", "/waypoint2_pose")' in source
    assert "if not self._path_ends_at_final_goal():" in source
    assert "waiting for a path terminating at Task2 GPS6" in source


def test_task2_mission_start_sends_follow_path_without_buoy_hold():
    adapter = (Path(__file__).parents[4] / "robot" / "launch" /
               "task2_mission_adapter.launch.py").read_text(encoding="utf-8")
    assert '"startup_hold_sec"' not in adapter


def test_task2_mission_gate_receives_the_retained_enable_state():
    source = (Path(__file__).parents[1] / "asv_trajectory_planner" /
              "follow_path_client_node.py").read_text(encoding="utf-8")
    assert "Bool, self.enabled_topic, self.enabled_callback, status_qos" in source


def test_task2_autonomy_readiness_requires_path_and_follow_path_only():
    source = (Path(__file__).parents[1] / "asv_trajectory_planner" /
              "task2_autonomy_ready_node.py").read_text(encoding="utf-8")
    assert "path_fresh and self.follow_path_client.server_is_ready()" in source
    assert "collision_monitor_active" not in source
