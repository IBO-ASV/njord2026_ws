"""A PCA hull axis must not turn an approaching vessel into a receding one."""
import math
import pytest
pytest.importorskip('torch')
from geometry_msgs.msg import Transform, TwistStamped
from asv_trajectory_planner.trajectory_generator import TrajectoryGenerator
from asv_trajectory_planner.mppi_torch import VesselState


def test_absolute_opponent_velocity_is_rotated_once_and_defines_cog():
    generator=object.__new__(TrajectoryGenerator)
    generator.other_twist_is_relative=False
    generator.other_heading_from_velocity=True
    own=VesselState(x=0.,y=0.,yaw=0.,u=1.,v=0.,r=0.,vx=1.,vy=0.)
    transform=Transform();transform.translation.x=20.;transform.rotation.w=1.
    twist=TwistStamped();twist.twist.linear.y=-1.286
    # Own ENU heading north: a south-going target approaches along base -X.
    target=generator._other_tf_twist_to_base_state(own,transform,twist,math.pi/2)
    assert target.vx==pytest.approx(-1.286)
    assert target.vy==pytest.approx(0.,abs=1e-8)
    assert abs(target.yaw)==pytest.approx(180.)
    assert target.u==pytest.approx(1.286)
    assert target.v==pytest.approx(0.,abs=1e-8)


def test_starboard_cost_rotates_with_gps_route_and_preserves_safe_side():
    import torch
    from asv_trajectory_planner.mppi_torch import starboard_preference_cost
    x=torch.tensor([[0.,1.,2.],[0.,1.,2.]])
    y=torch.tensor([[1.,1.,1.],[-2.,-2.,-2.]])
    cost=starboard_preference_cost(x,y,(0.,0.),(10.,0.))
    assert cost[0,0]>cost[1,0]
    # Route rotated north: east is starboard, west is port.
    rotated=starboard_preference_cost(-y,x,(0.,0.),(0.,10.))
    assert torch.allclose(cost,rotated)
