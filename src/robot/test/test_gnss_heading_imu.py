"""Reject unusable orientation before it can initialize geographic projection."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
spec=importlib.util.spec_from_file_location('heading',Path(__file__).parents[1]/'scripts/gnss_heading_imu.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
@pytest.mark.parametrize('q,cov,valid',[(1.,.1,True),(0.,.1,False),(float('nan'),.1,False),(1.,-1.,False)])
def test_heading_guard(q,cov,valid):
 out=[];node=SimpleNamespace(publisher=SimpleNamespace(publish=out.append))
 msg=PoseWithCovarianceStamped();msg.pose.pose.orientation.w=q;msg.pose.covariance[35]=cov
 module.HeadingImu.heading(node,msg)
 assert bool(out)==valid
 if valid:
  assert out[0].header.frame_id=='base_link'
  assert out[0].angular_velocity_covariance[0]==-1
  assert out[0].orientation_covariance[8]==cov
