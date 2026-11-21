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

def test_heading_variance_floor():
 out=[]; node=SimpleNamespace(publisher=SimpleNamespace(publish=out.append), yaw_variance=0.04)
 msg=PoseWithCovarianceStamped(); msg.pose.pose.orientation.w=1.; msg.pose.covariance[35]=0.01
 module.HeadingImu.heading(node,msg)
 assert out[0].orientation_covariance[8] == 0.04
 msg.pose.covariance[35]=0.09
 module.HeadingImu.heading(node,msg)
 assert out[1].orientation_covariance[8] == 0.09

def test_fix_variance_floor_and_invalid_fix():
 from sensor_msgs.msg import NavSatFix
 out=[];node=SimpleNamespace(position_variances=[0.1,0.2,0.3],fix_publisher=SimpleNamespace(publish=out.append))
 msg=NavSatFix();msg.status.status=0;msg.position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
 msg.position_covariance=[0.5,0.01,0.0,0.01,0.1,0.0,0.0,0.0,0.4]
 module.HeadingImu.fix(node,msg)
 assert list(out[-1].position_covariance)==[0.5,0.,0.,0.,0.2,0.,0.,0.,0.4]
 nofix=NavSatFix();nofix.status.status=-1
 module.HeadingImu.fix(node,nofix)
 assert out[-1].position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
