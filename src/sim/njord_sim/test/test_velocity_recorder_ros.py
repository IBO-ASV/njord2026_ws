"""Exercise explicit excitation cutout and passive pair recording without actuators."""
import csv
import os
import subprocess
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


@pytest.mark.parametrize('excitation',[False,True])
def test_recorder_passive_or_latched_feedback_loss(tmp_path,excitation):
    domain=181
    rclpy.init(args=[],domain_id=domain)
    node=rclpy.create_node('fake_calibration_feedback')
    odom=node.create_publisher(Odometry,'/test_calibration/odom',10)
    forces=node.create_publisher(Float32MultiArray,'/test_calibration/force',10)
    received=[]
    node.create_subscription(Twist,'/test_calibration/cmd',lambda msg:received.append(msg.linear.x),10)
    feed=[True]
    def tick():
        if not feed[0]:return
        msg=Odometry();msg.child_frame_id='base_link';msg.header.stamp=node.get_clock().now().to_msg()
        odom.publish(msg);forces.publish(Float32MultiArray(data=[0.]*4))
    node.create_timer(.02,tick)
    output=tmp_path/'samples.csv'
    command=[sys.executable,'-m','njord_sim.velocity_experiment','--output',str(output),
             '--duration','2','--odom-topic','/test_calibration/odom','--force-topic','/test_calibration/force',
             '--command-topic','/test_calibration/cmd','--wiring-signs','1,1,1,1']
    if excitation:
        reference=tmp_path/'reference.csv'
        reference.write_text('time_s,ref_u,ref_v,ref_r\n0,0,0,0\n0.2,0.2,0,0\n4,0.2,0,0\n')
        command+=['--reference',str(reference),'--publish-topic','/test_calibration/cmd']
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    proc=None
    try:
        proc=subprocess.Popen(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and proc.poll() is None:
            rclpy.spin_once(node,timeout_sec=.02)
            if excitation and any(x>.1 for x in received):feed[0]=False
        assert proc.poll() is not None
        log=proc.communicate()[0]
        if excitation:
            assert proc.returncode != 0
            assert 'Feedback lost' in log
            assert any(x>.1 for x in received)
            # Drain final zero publications after shutdown/discovery propagation.
            until=time.monotonic()+.3
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            assert received[-1]==0.
        else:
            assert proc.returncode==0,log
            assert not received
        records=list(csv.DictReader(output.open()))
        assert len(records)>3
    finally:
        if proc is not None and proc.poll() is None:proc.terminate();proc.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
