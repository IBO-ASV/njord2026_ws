#!/usr/bin/env python3
"""Prepare latter-half GLIM local-odom compensated planar preview.
Input is the prior acquisition-ordered full-bag GLIM/EKF evaluation."""
import math,sys,json,yaml
from pathlib import Path
from collections import defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message,deserialize_message
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from ship_perception_msgs.msg import TrackedObjectArray
root=Path(sys.argv[1]);target=sys.argv[2];source=Path(sys.argv[3])
start=yaml.safe_load((source/'metadata.yaml').read_text())['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']*1e-9
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
rr=rosbag2_py.SequentialReader();rr.open(rosbag2_py.StorageOptions(uri=str(root/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));rr.set_filter(rosbag2_py.StorageFilter(topics=['/odometry/filtered/local']))
rows=[]
while rr.has_next():
 _,d,_=rr.read_next();m=deserialize_message(d,Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
 rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
assert rows,'No local odometry'
od=np.array(sorted(rows));od[:,3]=np.unwrap(od[:,3])
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/task2/points_filtered','/tracked_objects']))
w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=target,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
for topic,typ in [('/preview/points','sensor_msgs/msg/PointCloud2'),('/preview/detections','visualization_msgs/msg/MarkerArray'),('/preview/status','std_msgs/msg/String'),('/preview/ownship','visualization_msgs/msg/MarkerArray'),('/preview/reference_points','sensor_msgs/msg/PointCloud2')]:w.create_topic(rosbag2_py.TopicMetadata(id=0,name=topic,type=typ,serialization_format='cdr'))
events=[];counts=defaultdict(int);reference=None;last_reference=-1e30
def make(kind,id,stamp):
 m=Marker();m.header.frame_id='odom';m.header.stamp=stamp;m.ns='candidate_preview';m.id=id;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.a=.85;return m
while r.has_next():
 topic,data,_=r.read_next();m=deserialize_message(data,PointCloud2 if topic.endswith('filtered') else TrackedObjectArray);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if not 202.7143936565<=t-start<=405.428787313 or np.min(abs(od[:,0]-t))>.15:continue
 x,y,th=[np.interp(t,od[:,0],od[:,i]) for i in [1,2,3]];c,s=math.cos(th),math.sin(th)
 if topic.endswith('filtered'):
  offsets={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y'],'formats':['<f4']*2,'offsets':[offsets[k] for k in ['x','y']],'itemsize':m.point_step});buf=bytearray(m.data);p=np.ndarray((m.height,m.width),dtype=dt,buffer=buf,strides=(m.row_step,m.point_step));px=p['x'].copy();py=p['y'].copy();p['x']=x+c*px-s*py;p['y']=y+s*px+c*py;m.data=bytes(buf);m.header.frame_id='odom';events.append((int(t*1e9),'/preview/points',serialize_message(m)));counts['clouds']+=1
  if reference is None:reference=serialize_message(m)
  if t-last_reference>=5:
   events.append((int(t*1e9),'/preview/reference_points',reference));last_reference=t
 else:
  out=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;out.markers.append(clear)
  for o in m.objects:
   if o.track_state!=1 or o.miss_count:continue
   p=o.state.pose.pose.position;q=o.state.pose.pose.orientation;wx=x+c*p.x-s*p.y;wy=y+s*p.x+c*p.y;theta=th+math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));id=int(o.object_id);color=(.15,.65,1.)
   box=make(Marker.CUBE,id*3,m.header.stamp);box.pose.position.x=wx;box.pose.position.y=wy;box.pose.position.z=p.z;box.pose.orientation.z=math.sin(theta/2);box.pose.orientation.w=math.cos(theta/2);box.scale.x=max(o.dimensions.x,.1);box.scale.y=max(o.dimensions.y,.1);box.scale.z=max(o.dimensions.z,.2);box.color.r,box.color.g,box.color.b=color;box.color.a=.35;out.markers.append(box)
   label=make(Marker.TEXT_VIEW_FACING,id*3+1,m.header.stamp);label.pose.position.x=wx;label.pose.position.y=wy;label.pose.position.z=p.z+1.;label.scale.z=.5;label.color.r,label.color.g,label.color.b=color;label.text=f'Candidate {id} | {o.dimensions.x:.2f} x {o.dimensions.y:.2f} m';out.markers.append(label)
  ego=make(Marker.LINE_STRIP,100000,m.header.stamp);ego.pose.position.x=x;ego.pose.position.y=y;ego.pose.orientation.z=math.sin(th/2);ego.pose.orientation.w=math.cos(th/2);ego.scale.x=.08;ego.color.r=0.;ego.color.g=.9;ego.color.b=1.;ego.color.a=1.
  outline=[(11,3),(7,11),(7,30),(11,37),(15,30),(15,22),(25,22),(25,30),(29,37),(33,30),(33,11),(29,3),(25,11),(25,18),(15,18),(15,11),(11,3)]
  ego.points=[Point(x=(20-yy)*2.4/34,y=(20-xx)*1.6/26,z=.15) for xx,yy in outline]
  events.append((int(t*1e9),'/preview/ownship',serialize_message(MarkerArray(markers=[ego]))))
  events.append((int(t*1e9),'/preview/detections',serialize_message(out)));events.append((int(t*1e9),'/preview/status',serialize_message(String(data=f'GLIM LOCAL ODOM REPLAY | Aug 10 | source {t-start:.2f}s | planar motion compensation | ownship cyan'))));counts['track_frames']+=1
for stamp,topic,data in sorted(events,key=lambda a:a[0]):w.write(topic,data,stamp)
print(json.dumps(dict(counts)))
