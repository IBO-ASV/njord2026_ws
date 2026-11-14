"""Passive body-velocity/force recorder; excitation requires an explicit topic."""
import argparse
import csv
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .calibration import ANGLES, LEVERS, IDS, calibrated_forces


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--reference', help='Velocity CSV only; force/pair plans are not published by this node')
    p.add_argument('--publish-topic', help='Explicit opt-in, normally /cmd_vel_manual through Control Manager')
    p.add_argument('--command-topic', default='/cmd_vel')
    p.add_argument('--odom-topic', default='/odometry/filtered/local')
    p.add_argument('--force-topic', default='/thruster_command')
    p.add_argument('--wiring-signs', default='-1,1,1,-1', help='electrical-to-physical RR,RL,FR,FL')
    p.add_argument('--balance-report',help='validated directional response JSON from identify_vessel --balance')
    p.add_argument('--force-anchor-scale',type=float,help='measured actual FR+ Newtons / published command Newtons')
    p.add_argument('--duration', type=float, default=180.)
    a = p.parse_args()
    signs = np.array([float(s) for s in a.wiring_signs.split(',')])
    if len(signs)!=4 or not np.all(np.isin(signs,[-1,1])): p.error('four wiring signs required')
    if a.publish_topic and not a.reference: p.error('publishing requires --reference')
    response=np.ones(8);anchor=1.
    if a.balance_report:
        balance=json.load(open(a.balance_report))
        if not balance.get('passed') or a.force_anchor_scale is None:p.error('validated balance report and measured --force-anchor-scale required')
        response=np.asarray(balance['relative_response']);anchor=a.force_anchor_scale
        try:calibrated_forces(np.ones(4),response,anchor)
        except ValueError as error:p.error(str(error))
    elif a.force_anchor_scale is not None:p.error('--force-anchor-scale requires --balance-report')
    reference = None
    if a.reference:
        reference = np.atleast_1d(np.genfromtxt(a.reference,delimiter=',',names=True))
        if not set(('time_s','ref_u','ref_v','ref_r')) <= set(reference.dtype.names):
            p.error('expected velocity reference columns')
        if np.any(np.diff(reference['time_s'])<=0): p.error('reference timestamps must increase')
        if not all(np.isfinite(reference[k]).all() for k in reference.dtype.names): p.error('nonfinite reference')
        for k,limit in (('ref_u',1.1),('ref_v',.5),('ref_r',.5)):
            if np.max(np.abs(reference[k]))>limit: p.error('reference exceeds experiment velocity bounds')
        a.duration = float(reference['time_s'][-1])+2.
    # Exclusive create avoids overwriting an existing experiment.
    stream = open(a.output,'x')
    writer = csv.writer(stream)
    writer.writerow(('time_s','ref_u','ref_v','ref_r','u','v','r','tau_x','tau_y','tau_n',
                     *('command_'+i for i in IDS),'force_age_s','command_age_s','yaw'))
    rclpy.init()
    node = Node('velocity_experiment')
    pub = node.create_publisher(Twist,a.publish_topic,1) if a.publish_topic else None
    state = {'fault':False,'force':None,'force_at':0.,'cmd':Twist(),'cmd_at':0.,'start':None,'odom_at':0.,'stamp':None}
    def force(msg):
        if len(msg.data)==4 and np.isfinite(msg.data).all():
            state['force']=np.asarray(msg.data)*signs; state['force_at']=time.monotonic()
    def command(msg):
        state['cmd']=msg; state['cmd_at']=time.monotonic()
    def odom(msg):
        now=time.monotonic()
        if msg.child_frame_id!='base_link':
            node.get_logger().error('Expected base_link body-frame twist; sample rejected',throttle_duration_sec=5.)
            return
        if not all(np.isfinite(v) for v in (msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z)):return
        state['odom_at']=now
        if state['force'] is None or now-state['force_at']>.1 or (pub and now-state['cmd_at']>.1): return
        if state['start'] is None and pub is None:state['start']=now
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if state['stamp'] is None: state['stamp']=stamp
        physical_command=state['force']
        f=calibrated_forces(physical_command,response,anchor); cmd=state['cmd']; vel=msg.twist.twist
        q=msg.pose.pose.orientation
        yaw=float(np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        writer.writerow((stamp-state['stamp'],cmd.linear.x,cmd.linear.y,cmd.angular.z,
                         vel.linear.x,vel.linear.y,vel.angular.z,
                         np.dot(f,np.cos(ANGLES)),np.dot(f,np.sin(ANGLES)),np.dot(f,LEVERS),
                         *physical_command,now-state['force_at'],now-state['cmd_at'],yaw))
    node.create_subscription(Float32MultiArray,a.force_topic,force,10)
    node.create_subscription(Twist,a.command_topic,command,10)
    node.create_subscription(Odometry,a.odom_topic,odom,20)
    began=time.monotonic()
    def tick():
        now=time.monotonic()
        if pub:
            msg=Twist()
            healthy=now-state['odom_at']<.2 and now-state['force_at']<.2
            if state['start'] is not None and now-state['start']>.5 and now-state['cmd_at']>.2:healthy=False
            if not healthy and state['start'] is not None: state['fault']=True
            healthy=healthy and not state['fault']
            if healthy and state['start'] is None: state['start']=now
            if healthy and state['start'] is not None:
                elapsed=now-state['start']
                if elapsed <= reference['time_s'][-1]:
                    row=reference[max(0,np.searchsorted(reference['time_s'],elapsed,side='right')-1)]
                    msg.linear.x=float(row['ref_u']);msg.linear.y=float(row['ref_v']);msg.angular.z=float(row['ref_r'])
            pub.publish(msg)
    node.create_timer(.02,tick)
    try:
        while state['start'] is None or time.monotonic()-state['start']<a.duration:
            rclpy.spin_once(node,timeout_sec=.05)
            if state['fault']:raise RuntimeError('Feedback lost during excitation; zero output latched')
            if state['start'] is None and time.monotonic()-began>15:
                raise RuntimeError('No fresh odometry/force feedback; excitation aborted')
    finally:
        try:
            if pub and rclpy.ok():
                for _ in range(3):
                    pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
        finally:
            stream.close();node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__': main()
