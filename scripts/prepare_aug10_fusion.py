#!/usr/bin/env python3
"""Offline, bag-specific GNSS/GLIM comparison inputs. No production TF changes."""
import sys,json,math
from pathlib import Path
import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message,serialize_message

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def enu(g):
 lat=np.deg2rad(g[:,1]);lon=np.deg2rad(g[:,2]);h=g[:,3];a=6378137.;e2=6.69437999014e-3;n=a/np.sqrt(1-e2*np.sin(lat)**2)
 xyz=np.column_stack(((n+h)*np.cos(lat)*np.cos(lon),(n+h)*np.cos(lat)*np.sin(lon),(n*(1-e2)+h)*np.sin(lat)))
 s,c=np.sin(lon[0]),np.cos(lon[0]);sl,cl=np.sin(lat[0]),np.cos(lat[0]);rot=np.array([[-s,c,0],[-sl*c,-sl*s,cl]])
 return (xyz-xyz[0])@rot.T

def main(root,source):
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/odometry/filtered/local']))
 od=[];messages=[]
 while r.has_next():
  _,d,_=r.read_next();m=deserialize_message(d,Odometry);p=m.pose.pose.position;v=m.twist.twist.linear;t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  od.append([t,p.x,p.y,yaw(m.pose.pose.orientation),v.x,v.y,m.twist.twist.angular.z]);messages.append(m)
 od=np.array(od);idx=np.argsort(od[:,0]);od=od[idx];od[:,3]=np.unwrap(od[:,3]);g=np.load(source)['gnss'];xy=enu(g);start=1786301520.145669423
 # Calibration precedes evaluation window 180-270 s. Fit antenna trajectories,
 # using recorded base->gnss_link (-0.5,+0.4 m); no fitted scale.
 sel=np.where((g[:,0]>=od[0,0])&(g[:,0]<=start+150))[0][::20];tt=g[sel,0]
 p=np.column_stack([np.interp(tt,od[:,0],od[:,i]) for i in [1,2]]);th=np.interp(tt,od[:,0],od[:,3]);lever=np.column_stack((-.5*np.cos(th)-.4*np.sin(th),-.5*np.sin(th)+.4*np.cos(th)));local=p+lever;target=xy[sel]
 x=local-local.mean(axis=0);y=target-target.mean(axis=0);u,_,vt=np.linalg.svd(x.T@y);rot=vt.T@u.T
 if np.linalg.det(rot)<0:vt[-1]*=-1;rot=vt.T@u.T
 trans=target.mean(axis=0)-rot@local.mean(axis=0);angle=math.atan2(rot[1,0],rot[0,0]);res=np.linalg.norm(local@rot.T+trans-target,axis=1)
 aligned=od.copy();aligned[:,1:3]=od[:,1:3]@rot.T+trans;aligned[:,3]+=angle
 np.savez(root/'comparison_inputs.npz',odom=aligned,gnss=np.column_stack((g[:,0],xy)),rotation=rot,translation=trans)
 report={'calibration_interval_sec':[0,150],'evaluation_interval_sec':[180,270],'rotation_deg':angle*180/math.pi,'calibration_residual_median_m':float(np.median(res)),'calibration_residual_p95_m':float(np.percentile(res,95)),'gnss_antenna_base_xy':[-.5,.4],'gnss_position_sigma_floor_m':.5,'note':'Calibration residual is agreement with input GNSS, not ground-truth accuracy.'}
 (root/'fusion_calibration.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
 writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(root/'fusion_input_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 for topic in ['/comparison/glim_enu','/comparison/gnss_base','/comparison/gnss_base_conservative']:writer.create_topic(rosbag2_py.TopicMetadata(id=0,name=topic,type='nav_msgs/msg/Odometry',serialization_format='cdr'))
 events=[]
 for m in messages:
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;pos=rot@np.array([m.pose.pose.position.x,m.pose.pose.position.y])+trans;theta=yaw(m.pose.pose.orientation)+angle;m.header.frame_id='map';m.pose.pose.position.x,m.pose.pose.position.y=map(float,pos);m.pose.pose.orientation.z,m.pose.pose.orientation.w=math.sin(theta/2),math.cos(theta/2)
  events.append((int(t*1e9),'/comparison/glim_enu',serialize_message(m)))
 for row,pos in zip(g,xy):
  t=row[0]
  if t<od[0,0] or t>od[-1,0] or np.min(np.abs(od[:,0]-t))>.15:continue
  theta=np.interp(t,aligned[:,0],aligned[:,3]);offset=np.array([-.5*np.cos(theta)-.4*np.sin(theta),-.5*np.sin(theta)+.4*np.cos(theta)]);base=pos-offset;m=Odometry();ns=int(t*1e9);m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9;m.header.frame_id='map';m.child_frame_id='base_link';m.pose.pose.position.x,m.pose.pose.position.y=map(float,base);m.pose.pose.orientation.w=1.;m.pose.covariance[0]=m.pose.covariance[7]=.25
  events.append((ns,'/comparison/gnss_base',serialize_message(m)))
  m.pose.covariance[0]=m.pose.covariance[7]=4.0
  events.append((ns,'/comparison/gnss_base_conservative',serialize_message(m)))
 for stamp,topic,data in sorted(events,key=lambda x:x[0]):writer.write(topic,data,stamp)

if __name__=='__main__':main(Path(sys.argv[1]),sys.argv[2])
