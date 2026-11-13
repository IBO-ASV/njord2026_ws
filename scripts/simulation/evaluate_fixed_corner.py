#!/usr/bin/env python3
"""Offline geometry check of one manually selected wall-like corner.

ROIs are selected from scene_alignment.png, not by velocity. These fixed
coordinates apply only to the July 24 trial, not arbitrary bags.
"""
import math,json,sys
from pathlib import Path
import rosbag2_py,numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
root=Path(sys.argv[1])
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/odometry/filtered/local','/task2/points_filtered']))
od=[];clouds=[]
while r.has_next():
 t,b,_=r.read_next()
 if t=='/odometry/filtered/local':
  m=deserialize_message(b,Odometry);q=m.pose.pose.orientation;p=m.pose.pose.position;stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;v=m.twist.twist.linear
  od.append([stamp,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),math.hypot(v.x,v.y)])
 else:
  m=deserialize_message(b,PointCloud2);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;fields={f.name:f.offset for f in m.fields};dtype=np.dtype({'names':['x','y'],'formats':['<f4']*2,'offsets':[fields[k] for k in ['x','y']],'itemsize':m.point_step});a=np.frombuffer(m.data,dtype=dtype);points=np.column_stack([a[k] for k in ['x','y']]);clouds.append((stamp,points[np.isfinite(points).all(axis=1)]))
od=np.asarray(od);od[:,3]=np.unwrap(od[:,3]);features=[]
for stamp,points in clouds:
 x,y,yaw=[np.interp(stamp,od[:,0],od[:,i]) for i in range(1,4)];c,s=math.cos(yaw),math.sin(yaw);p=points@np.array([[c,s],[-s,c]])+[x,y]
 vertical=p[(p[:,0]>5.5)&(p[:,0]<8.5)&(p[:,1]>-8)&(p[:,1]<22)]
 horizontal=p[(p[:,1]>-13.5)&(p[:,1]<-10.5)&(p[:,0]>-14)&(p[:,0]<6.5)]
 if len(vertical)<12 or len(horizontal)<12:continue
 # Two line offsets form a repeatable wall-like corner landmark.
 features.append([stamp,float(np.median(vertical[:,0])),float(np.median(horizontal[:,1])),np.interp(stamp,od[:,0],od[:,4]),len(vertical),len(horizontal)])
a=np.asarray(features);samples=[]
if len(a):
 for row in a:
  window=a[np.abs(a[:,0]-row[0])<=2]
  if len(window)<8 or np.ptp(window[:,0])<2 or row[3]<.2:continue
  t=window[:,0]-row[0];design=np.column_stack([t,np.ones(len(t))]);v=np.linalg.lstsq(design,window[:,1:3],rcond=None)[0][0];samples.append([row[0],*v,np.linalg.norm(v)])
result={'cloud_count':len(clouds),'corner_observations':len(a),'moving_speed_samples':len(samples),'method':'fixed geometric ROIs + median offsets of two orthogonal wall-like lines; 4s least-squares velocity; only ego speed>=0.2m/s','semantic_limit':'corner is visually selected wall-like geometry; no independently labelled survey ground truth'}
if samples:
 b=np.asarray(samples);result.update(ground_speed_median_mps=float(np.median(b[:,3])),ground_speed_p95_mps=float(np.percentile(b[:,3],95)),corner_position_radius95_m=float(np.percentile(np.linalg.norm(a[:,1:3]-np.median(a[:,1:3],axis=0),axis=1),95)))
 np.savetxt(root/'corner_speed.csv',b,delimiter=',',header='stamp,vx,vy,speed',comments='')
(root/'corner_summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if samples:
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(2,1,figsize=(9,6),layout='constrained');axes[0].plot(od[:,0]-od[0,0],od[:,4],label='Own vessel speed');axes[0].scatter(b[:,0]-od[0,0],b[:,3],s=28,color='darkorange',label='Wall-like corner speed (valid samples, 4s window)');axes[0].axhline(.2,color='gray',linestyle='--');axes[0].set(xlabel='Bag time (s)',ylabel='Speed (m/s)');axes[0].legend();axes[1].scatter(a[:,0]-od[0,0],a[:,1]-np.median(a[:,1]),s=12,label='Observed corner x deviation');axes[1].scatter(a[:,0]-od[0,0],a[:,2]-np.median(a[:,2]),s=12,label='Observed corner y deviation');axes[1].set(xlabel='Bag time (s)',ylabel='Position deviation (m)');axes[1].legend();fig.savefig(root/'corner_check.png',dpi=160)
