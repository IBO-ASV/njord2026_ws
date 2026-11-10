import json
import os

from ament_index_python.packages import get_package_share_directory


def _load_config(filename):
    share_dir = get_package_share_directory("robot")
    config_path = os.path.join(share_dir, "config", "livox", filename)
    with open(config_path, "r") as f:
        return json.load(f)


def test_mid360_config_is_valid_json():
    config = _load_config("MID360_config.json")
    assert isinstance(config, dict)


def test_mid360s_platform_configs_are_valid_json():
    for filename in ("MID360S_jetson_config.json", "MID360S_minipc_config.json"):
        assert isinstance(_load_config(filename), dict)


def test_mid360_net_info_key():
    config = _load_config("MID360_config.json")
    assert "MID360" in config


def test_mid360s_platform_configs_have_expected_host_ips():
    expected_host_ips = {
        "MID360S_jetson_config.json": "192.168.1.5",
        "MID360S_minipc_config.json": "192.168.1.2",
    }
    for filename, expected_ip in expected_host_ips.items():
        config = _load_config(filename)
        assert "Mid360s" in config
        assert "MID360" not in config
        host_net_info = config["Mid360s"]["host_net_info"]
        for field in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
            assert host_net_info[field] == expected_ip


def test_mid360_lidar_type_is_8():
    config = _load_config("MID360_config.json")
    assert config["lidar_summary_info"]["lidar_type"] == 8


def test_mid360s_platform_configs_lidar_type_is_8():
    for filename in ("MID360S_jetson_config.json", "MID360S_minipc_config.json"):
        assert _load_config(filename)["lidar_summary_info"]["lidar_type"] == 8


def test_mid360_lidar_configs_present():
    config = _load_config("MID360_config.json")
    lidar_configs = config["lidar_configs"]
    assert isinstance(lidar_configs, list)
    assert len(lidar_configs) > 0
    for entry in lidar_configs:
        assert "ip" in entry
    assert "host_net_info" in config["MID360"]


def test_mid360s_platform_lidar_configs_present():
    for filename in ("MID360S_jetson_config.json", "MID360S_minipc_config.json"):
        config = _load_config(filename)
        lidar_configs = config["lidar_configs"]
        assert isinstance(lidar_configs, list)
        assert len(lidar_configs) > 0
        for entry in lidar_configs:
            assert "ip" in entry
        assert "host_net_info" in config["Mid360s"]
def test_lidar_launch_routes_raw_imu_through_si_scaler():
    share_dir = get_package_share_directory("robot")
    launch_path = os.path.join(share_dir, "launch", "lidar.launch.py")
    with open(launch_path, "r") as stream:
        source = stream.read()

    assert source.count('remappings=[("/livox/imu", "/livox/imu_raw")]') == 2
    assert 'plugin="livox_ros::LivoxImuScaler"' in source
    assert '"input_topic": "/livox/imu_raw"' in source
    assert '"output_topic": "/livox/imu"' in source
    assert '"acceleration_scale": 9.80665' in source


def test_glim_uses_raw_livox_imu_without_the_scaler_hop():
    share_dir = get_package_share_directory("robot")
    paths = [
        os.path.join(share_dir, "config", "glim_config", "config_ros.json"),
        os.path.join(share_dir, "config", "glim_config", "config_ros_cpu.json"),
        os.path.join(share_dir, "config", "glim_config_headless", "config_ros.json"),
    ]

    for path in paths:
        with open(path, "r") as stream:
            source = stream.read()
        assert '"imu_topic": "/livox/imu_raw"' in source
        assert '"acc_scale": 9.80665' in source


def test_jetson_glim_configs_are_odom_only():
    share_dir = get_package_share_directory("robot")
    paths = [
        os.path.join(share_dir, "config", "glim_config", "config_ros.json"),
        os.path.join(share_dir, "config", "glim_config", "config_ros_cpu.json"),
        os.path.join(share_dir, "config", "glim_config_headless", "config_ros.json"),
    ]

    for path in paths:
        with open(path, "r") as stream:
            source = stream.read()
        assert '"enable_local_mapping": false' in source
        assert '"enable_global_mapping": false' in source
        assert '"map_frame_id": "odom"' in source


def test_glim_interprets_livox_point_timestamps_as_absolute_nanoseconds():
    share_dir = get_package_share_directory("robot")
    path = os.path.join(share_dir, "config", "glim_config", "config_sensors.json")
    with open(path, "r") as stream:
        source = stream.read()

    assert '"autoconf_perpoint_times": false' in source
    assert '"perpoint_relative_time": false' in source
    assert '"perpoint_time_scale": 1e-9' in source
