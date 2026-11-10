import math
import numpy as np
from task2_perception.glim_odom_adapter import convert_state


def test_sensor_offset_is_removed():
    position,yaw,velocity=convert_state([.5,0,.8],[1,0,0,0],[1,0,0],0)
    np.testing.assert_allclose(position,[0,0,0],atol=1e-12)
    np.testing.assert_allclose(velocity,[1,0,0],atol=1e-12)
    assert abs(yaw)<1e-12


def test_world_velocity_rotates_into_base_axes():
    c=math.sqrt(.5)
    position,yaw,velocity=convert_state([0,.5,.8],[c,c,0,0],[0,1,0],0)
    np.testing.assert_allclose(position,[0,0,0],atol=1e-12)
    np.testing.assert_allclose(velocity,[1,0,0],atol=1e-12)
    assert abs(yaw-math.pi/2)<1e-12


def test_rotating_sensor_lever_arm_does_not_move_base_center():
    _,_,velocity=convert_state([.5,0,.8],[1,0,0,0],[0,.5,0],1)
    np.testing.assert_allclose(velocity,[0,0,0],atol=1e-12)
