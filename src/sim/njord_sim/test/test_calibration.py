"""Identification refuses unexcited data and checks independently observed response."""
import numpy as np
import pytest
from njord_sim.calibration import fit_balance, fit_mmg, LEVERS, load_csv


def test_balance_recovers_directional_ratios_and_electrical_gain_order():
    response=np.array([.9,1.1,1.,.85,1.2,.9,.8,1.05])
    def trials(levels):
        rows=[]
        for sign in (1.,-1.):
            for anchor,adjust in ((2,3),(3,1),(1,0)):
                for gain in levels:
                    force=np.zeros(4);force[anchor]=8*sign;force[adjust]=8*gain*sign
                    physical=np.where(force>=0,response[:4],response[4:])*force
                    rows.append((force,float(physical@LEVERS/3+.02)))
        rows.append((np.zeros(4),.02))
        return rows
    result=fit_balance(trials((.75,1.25)),[True,False,False,True],trials((1.,)))
    assert result['passed']
    assert result['relative_response']==pytest.approx(response)
    assert result['static_map.thrusters.forward_gain']==pytest.approx([.8/1.2,.8/1.1,.8/1.,.8/1.05])
    assert result['static_map.thrusters.reverse_gain']==pytest.approx([.8/.9,.8/.9,1.,.8/.85])
    assert not fit_balance(trials((.75,1.25)),[False]*4)['passed']


def test_balance_rejects_unexcited_pairs():
    with pytest.raises(ValueError,match='rank'):
        fit_balance([([0,0,8,8],.1)]*20,[False]*4)


def test_mmg_refuses_stationary_nonidentifiable_log():
    rows=np.zeros(100,dtype=[(k,float) for k in ('time_s','u','v','r','tau_x','tau_y','tau_n')])
    rows['time_s']=np.arange(100)*.02
    with pytest.raises(ValueError,match='excitation'):
        fit_mmg(rows,rows,10.,1.5)


def test_log_rejects_reordered_timestamps(tmp_path):
    path=tmp_path/'bad.csv';path.write_text('time_s,u\n0,0\n.02,1\n.01,2\n')
    with pytest.raises(ValueError,match='timestamps'):load_csv(path)


def test_terminal_observation_waits_for_actual_stationarity_and_is_bounded():
    from njord_sim.evaluate import terminal_observation_reason
    assert terminal_observation_reason(4.1,1.0) is None
    assert terminal_observation_reason(5.2,2.0)=='mission terminal'
    assert terminal_observation_reason(3.0,2.0) is None
    assert 'not reached' in terminal_observation_reason(10.,0.)
    assert terminal_observation_reason(4.1,0.,fault=True)=='mission terminal'
    assert terminal_observation_reason(3.9,0.,fault=True) is None


def test_final_stop_requires_current_continuous_stationarity():
    import time
    from types import SimpleNamespace
    import rclpy
    from nav_msgs.msg import Odometry
    from njord_sim.evaluate import Evaluation
    rclpy.init(args=[],domain_id=184)
    node=None
    try:
        node=Evaluation(SimpleNamespace(task='task3_1'))
        goal=node.s.route.waypoints[-1]
        msg=Odometry();msg.pose.pose.position.x=goal.x;msg.pose.pose.position.y=goal.y
        msg.pose.pose.orientation.w=1.
        node.t0=time.monotonic()-3.
        node.stop_since=0.
        node.on_truth(msg)
        assert node.stop_duration>=2.
        msg.twist.twist.linear.x=.2
        node.on_truth(msg)
        assert node.stop_duration==0.
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


def test_force_recording_uses_directional_response_and_absolute_anchor():
    from njord_sim.calibration import calibrated_forces
    result=calibrated_forces([1.,-2.,3.,-4.],[.9,1.1,1.,.85,1.2,.9,.8,1.05],2.)
    assert result==pytest.approx([1.8,-3.6,6.,-8.4])
    with pytest.raises(ValueError):calibrated_forces([0.]*4,[1.]*8,0.)
