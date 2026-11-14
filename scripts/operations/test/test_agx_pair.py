"""Check DDS interface selection and direct-input routing without hardware."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.skipif(not Path('/opt/ros/jazzy/setup.bash').exists(), reason='ROS Jazzy required')
@pytest.mark.parametrize('role,override', [('ground', False), ('ground', True), ('vessel', False)])
def test_helper_selects_interface_and_disables_critical_link(tmp_path, role, override):
    scripts = tmp_path / 'scripts/operations'
    scripts.mkdir(parents=True)
    helper = scripts / 'run_agx_pair.sh'
    shutil.copy(ROOT / 'scripts/operations/run_agx_pair.sh', helper)
    (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').write_text('export PATH="' + str(tmp_path / 'bin') + ':$PATH"\n')
    (tmp_path / 'bin').mkdir()
    ip = tmp_path / 'bin/ip'
    ip.write_text('#!/bin/sh\n[ "$4" = "192.168.0.71" ] || exit 1\necho "192.168.0.71 dev wlan0 src 192.168.0.72 uid 1000"\n')
    ip.chmod(0o755)
    ros2 = tmp_path / 'bin/ros2'
    ros2.write_text('#!/usr/bin/python3\nimport os,sys,json\nprint(json.dumps({"args":sys.argv[1:],"dds":os.environ["CYCLONEDDS_URI"],"domain":os.environ["ROS_DOMAIN_ID"]}))\n')
    ros2.chmod(0o755)
    env = dict(os.environ)
    env.pop('NJORD_GROUND_CYCLONEDDS_URI', None)
    env.pop('NJORD_AGX_ADDRESS', None)
    env['CYCLONEDDS_URI'] = '/old/incorrect/interface.xml'
    if override:
        env['NJORD_GROUND_CYCLONEDDS_URI'] = '<CycloneDDS/>'
    result = subprocess.run(['bash', str(helper), role], env=env, capture_output=True, text=True, check=True)
    data = json.loads(result.stdout)
    assert data['domain'] == '126'
    assert 'enable_critical_link:=false' in data['args']
    if role == 'ground':
        assert data['args'][2] == 'ground_pc.launch.py'
        if override:
            assert data['dds'] == '<CycloneDDS/>'
        else:
            assert 'address="192.168.0.72"' in data['dds']
    else:
        assert data['dds'] == str(tmp_path / 'src/bringup/robot/config/cyclonedds_agx.xml')


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_ground_topics_follow_transport(enabled):
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location('ground_launch', ROOT / 'src/bringup/robot/launch/ground_pc.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', return_value='/tmp/zed2i_driver'):
        description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['enable_critical_link'] = enabled
    nodes = {action._Node__node_name: action
             for action in description.entities if isinstance(action, Node)}
    joy = nodes['joy_node']
    remaps = [(perform_substitutions(context, src), perform_substitutions(context, dst))
              for src, dst in joy._Node__remappings]
    expected = '/critical_link/input/joy' if enabled == 'true' else '/joy'
    assert remaps == [('/joy', expected)]
    heartbeat = nodes['ground_station_heartbeat']
    params = evaluate_parameters(context, heartbeat._Node__parameters)
    assert params[0]['topic'] == ('/critical_link/input/heartbeat' if enabled == 'true' else '/heartbeat/ground_station')
