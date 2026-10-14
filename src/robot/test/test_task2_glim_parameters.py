"""Ensure the tracker and selector cannot silently choose different ego odom."""
import importlib.util
from pathlib import Path
import pytest
import yaml
pytest.importorskip('launch')
from launch import LaunchContext

@pytest.mark.parametrize('source,expected',[('glim','/task2/odometry/local'),('external','/custom/ego')])
def test_named_yaml_matches_selected_odom(source,expected):
 path=Path(__file__).parents[1]/'launch/task2_jetson_autonomy.launch.py'
 spec=importlib.util.spec_from_file_location('task2_launch',path)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 context=LaunchContext()
 context.launch_configurations.update(ego_odom_source=source,own_odom_topic='/custom/ego',perception_world_frame='odom',corridor_enabled='false')
 for action in module._task2_parameters(context):action.execute(context)
 generated=Path(context.launch_configurations['task2_params_file'])
 try:
  data=yaml.safe_load(generated.read_text())
  selector=data['opponent_selector']['ros__parameters']
  assert selector['ego_odom_topic']==expected
  assert selector['map_frame']=='odom'
  assert selector['corridor_enabled'] is False
  assert data['task2_waypoint_map_frame_relay']['ros__parameters']['target_frame']=='odom'
 finally:generated.unlink()
