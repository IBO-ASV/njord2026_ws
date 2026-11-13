#!/usr/bin/env python3
"""Make a camera + perception preview for the Aug 10 03:48:56 bag.
Camera and perception are scheduled by their acquisition timestamps."""
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
from sensor_msgs.msg import CompressedImage
rr=rosbag2_py.SequentialReader();rr.open(rosbag2_py.StorageOptions(uri=str(root/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));rr.set_filter(rosbag2_py.StorageFilter(topics=['/odometry/filtered/local']))
rows=[]
while rr.has_next():
 _,d,_=rr.read_next();m=deserialize_message(d,Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
 rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
assert rows,'No local odometry'
od=np.array(sorted(rows));od[:,3]=np.unwrap(od[:,3])
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/task2/points_filtered','/tracked_objects']))
w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=target,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
for topic,typ in [('/preview/points','sensor_msgs/msg/PointCloud2'),('/preview/tracks','visualization_msgs/msg/MarkerArray'),('/preview/status','std_msgs/msg/String'),('/preview/camera/back/compressed','sensor_msgs/msg/CompressedImage')]:w.create_topic(rosbag2_py.TopicMetadata(id=0,name=topic,type=typ,serialization_format='cdr'))
events=[];counts=defaultdict(int)
def make(kind,id,stamp):
 m=Marker();m.header.frame_id='odom';m.header.stamp=stamp;m.ns='candidate_preview';m.id=id;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.a=.85;return m
while r.has_next():
 topic,data,_=r.read_next();m=deserialize_message(data,PointCloud2 if topic.endswith('filtered') else TrackedObjectArray);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if not 0<=t-start<=32 or np.min(abs(od[:,0]-t))>.15:continue
 x,y,th=[np.interp(t,od[:,0],od[:,i]) for i in [1,2,3]];c,s=math.cos(th),math.sin(th)
 if topic.endswith('filtered'):
  offsets={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y'],'formats':['<f4']*2,'offsets':[offsets[k] for k in ['x','y']],'itemsize':m.point_step});buf=bytearray(m.data);p=np.ndarray((m.height,m.width),dtype=dt,buffer=buf,strides=(m.row_step,m.point_step));px=p['x'].copy();py=p['y'].copy();p['x']=x+c*px-s*py;p['y']=y+s*px+c*py;m.data=bytes(buf);m.header.frame_id='odom';events.append((int(t*1e9),'/preview/points',serialize_message(m)));counts['clouds']+=1
 else:
  out=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;out.markers.append(clear)
  for o in m.objects:
   if o.track_state!=1 or o.miss_count:continue
   p=o.state.pose.pose.position;q=o.state.pose.pose.orientation;wx=x+c*p.x-s*p.y;wy=y+s*p.x+c*p.y;theta=th+math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));id=int(o.object_id);color=(.15,.65,1.)
   box=make(Marker.CUBE,id*3,m.header.stamp);box.pose.position.x=wx;box.pose.position.y=wy;box.pose.position.z=p.z;box.pose.orientation.z=math.sin(theta/2);box.pose.orientation.w=math.cos(theta/2);box.scale.x=max(o.dimensions.x,.1);box.scale.y=max(o.dimensions.y,.1);box.scale.z=max(o.dimensions.z,.2);box.color.r,box.color.g,box.color.b=color;box.color.a=.35;out.markers.append(box)
   label=make(Marker.TEXT_VIEW_FACING,id*3+1,m.header.stamp);label.pose.position.x=wx;label.pose.position.y=wy;label.pose.position.z=p.z+1.;label.scale.z=.5;label.color.r,label.color.g,label.color.b=color;label.text=f'Candidate {id} | {o.dimensions.x:.2f} x {o.dimensions.y:.2f} m';out.markers.append(label)
  ego=make(Marker.ARROW,100000,m.header.stamp);ego.pose.position.x=x;ego.pose.position.y=y;ego.pose.orientation.z=math.sin(th/2);ego.pose.orientation.w=math.cos(th/2);ego.scale.x=1.5;ego.scale.y=.45;ego.scale.z=.3;ego.color.g=1.;out.markers.append(ego)
  events.append((int(t*1e9),'/preview/tracks',serialize_message(out)));events.append((int(t*1e9),'/preview/status',serialize_message(String(data=f'RECORDED REPLAY | Aug 10 | source {t-start:.2f}s | object candidates blue | ego green'))));counts['track_frames']+=1
camera=rosbag2_py.SequentialReader();camera.open(rosbag2_py.StorageOptions(uri=str(source),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));camera.set_filter(rosbag2_py.StorageFilter(topics=['/back_cam/image_raw/compressed']))
cam_times=[]
while camera.has_next():
 _,data,rec=camera.read_next();m=deserialize_message(data,CompressedImage);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 if stamp==0:raise ValueError('Camera has no acquisition timestamp')
 if not -1<=stamp*1e-9-start<=33:raise ValueError('Camera clock differs from sensors')
 events.append((stamp,'/preview/camera/back/compressed',data));cam_times.append(stamp);counts['camera_images']+=1
assert cam_times,'No camera images'
print('camera_offset_seconds', (min(cam_times)*1e-9-start,max(cam_times)*1e-9-start))
for stamp,topic,data in sorted(events,key=lambda a:a[0]):w.write(topic,data,stamp)
print(json.dumps(dict(counts)))
