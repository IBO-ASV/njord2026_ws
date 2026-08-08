"""Static checks for the self-contained Task 2 rosbag perception launch."""

import ast
import os


LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "launch",
    "task2_bag_perception.launch.py",
)


def _source():
    with open(LAUNCH_FILE, encoding="utf-8") as file:
        return file.read()


def test_launch_parses():
    ast.parse(_source())


def test_launches_bag_and_perception_chain():
    source = _source()
    for required in (
        '"ros2", "bag", "play"',
        '"task2_perception"',
        '"ship_perception_bringup"',
        '"classical_pipeline.launch.py"',
        '"use_sim_time": "true"',
        '"ego_odom_topic": "/odom"',
        '"task2_params.yaml"',
        '"map_frame": "odom"',
    ):
        assert required in source


def test_launch_is_perception_only():
    source = _source()
    for forbidden in (
        "thruster_driver",
        "serial_writer",
        "real_bringup.launch.py",
        "navigation_launch_task2.py",
    ):
        assert forbidden not in source


def test_bag_path_and_loop_are_configurable():
    source = _source()
    assert '"bag_path"' in source
    assert '"loop"' in source
    assert '"--loop"' in source


def test_bag_storage_is_auto_detected_by_default():
    source = _source()
    assert '"storage_id", default_value=""' in source
    assert 'if storage_id:' in source
    assert 'command.extend(["-s", storage_id])' in source


def test_bag_player_is_non_interactive():
    source = _source()
    assert '"--disable-keyboard-controls"' in source


def test_bag_uses_the_packaged_qos_override_by_default():
    source = _source()
    assert '"collision_avoidance_qos.yaml"' in source
