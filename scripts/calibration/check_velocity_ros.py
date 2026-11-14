#!/usr/bin/env python3
"""Bounded real-time ROS loop check; uses isolated calibration topics, no serial node."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml
from ament_index_python.packages import get_package_share_directory as share
from njord_sim.calibration import load_csv, tracking_metrics, write_reference


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--domain',type=int,default=180)
    a=p.parse_args();out=Path(a.output).resolve()
    if out.exists() and any(out.iterdir()):p.error('choose an empty output directory')
    out.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    robot=(Path(share('robot'))/'urdf/robot.urdf_modified.urdf').read_text()
    driver_base=Path(share('thruster_controller'))/'config/config.yaml'
    selected=Path(share('njord_sim'))/'config/velocity_control.yaml'
    plant=Path(share('dutyed_tf_pub_with_disturbance'))/'config/node_config.yaml'
    overrides={'thruster_controller_node':{'ros__parameters':{'robot_description':robot,
        'thrusters.reverse':[False]*4,'topics.cmd_vel':'/calibration/cmd_vel',
        'topics.feedback_odometry':'/calibration/odom','topics.sim_thruster_command':'/calibration/forces'}},
        'dutyed_tf_pub_with_disturbance_node':{'ros__parameters':{'publish_tf':False,'thruster_force_sign':[1.]*4,
        'topic_odom':'/calibration/odom','topic_thruster_command':'/calibration/forces',
        'disturbance.sigma_u':0.,'disturbance.sigma_v':0.,'disturbance.sigma_r':0.}}}
    (out/'overrides.yaml').write_text(yaml.safe_dump(overrides))
    rows=[];t=0.
    for axis,value in enumerate((.5,.2,.2)):
        for duration,amplitude in ((2,0.),(7,value),(3,0.)):
            for _ in range(round(duration/.02)):
                ref=[0.,0.,0.];ref[axis]=amplitude;rows.append((t,*ref));t+=.02
    write_reference(out/'reference.csv',rows)
    commands=[['ros2','run','thruster_controller','thruster_controller_node','--ros-args','--params-file',str(driver_base),'--params-file',str(selected),'--params-file',str(out/'overrides.yaml')],
              ['ros2','run','dutyed_tf_pub_with_disturbance','dutyed_tf_pub_with_disturbance_node','--ros-args','--params-file',str(plant),'--params-file',str(out/'overrides.yaml')],
              [sys.executable,'-m','njord_sim.velocity_experiment','--output',str(out/'samples.csv'),
               '--reference',str(out/'reference.csv'),'--publish-topic','/calibration/cmd_vel','--command-topic','/calibration/cmd_vel',
               '--odom-topic','/calibration/odom','--force-topic','/calibration/forces','--wiring-signs','1,1,1,1']]
    children=[];logs=[]
    try:
        for i,command in enumerate(commands):
            stream=(out/f'process_{i}.log').open('w');logs.append(stream)
            children.append(subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        code=children[-1].wait(timeout=65)
        if code or any(child.poll() is not None for child in children[:-1]):
            raise RuntimeError('ROS loop failed; inspect process logs')
        result=tracking_metrics(load_csv(out/'samples.csv'))
        result['mode']='production ROS driver + MMG, isolated /calibration topics; not hardware'
        (out/'report.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2));return 0 if result['passed'] else 1
    finally:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
            for child in children:
                try:os.killpg(child.pid,sig)
                except ProcessLookupError:pass
            if sig!=signal.SIGKILL:
                end=time.monotonic()+2
                while time.monotonic()<end and any(c.poll() is None for c in children):time.sleep(.05)
        for child in children:child.wait()
        for stream in logs:stream.close()


if __name__=='__main__':raise SystemExit(main())
