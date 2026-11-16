"""The runtime GLIM profile must leave the mounting transform to its adapter."""
import importlib.util
import json
from pathlib import Path
from launch import LaunchContext


def test_glim_runtime_profile_publishes_sensor_pose_once(tmp_path, monkeypatch):
    share = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('glim_profile', share/'launch/task2_glim_localization.launch.py')
    profile = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(profile)
    monkeypatch.setattr(profile, 'get_package_share_directory', lambda _: str(share))
    monkeypatch.setattr(profile.tempfile, 'mkdtemp', lambda **_: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(glim_imu_topic='/livox/imu', glim_points_topic='/livox/lidar', glim_acc_scale='1.0')
    actions = profile.setup(context)
    ros = json.loads((tmp_path/'config_ros.json').read_text())['glim_ros']
    assert ros['base_frame_id'] == ''
    assert ros['acc_scale'] == 1.0
    assert ros['imu_topic'] == '/livox/imu'
    assert not ros['enable_local_mapping'] and not ros['enable_global_mapping']
    assert len(actions) == 4
