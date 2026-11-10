import rosbag2_py,numpy as np,json,math
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri='/home/ibo/njord2026_ws/rosbag2_2026_08_10-03_52_00',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={x.name:get_message(x.type) for x in r.get_all_topics_and_types() if x.name in ['/sensor/vehicle_gnss/fix/raw','/odometry/feedback','/sensor/vehicle_gnss/compass/raw','/tf_static']};r.set_filter(rosbag2_py.StorageFilter(topics=list(types)));data=defaultdict(list);first=set()
while r.has_next():
 t,d,rec=r.read_next();m=deserialize_message(d,types[t])
 if t not in first:print(t,str(m)[:2200]);first.add(t)
 if t=='/tf_static':continue
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if t.endswith('fix/raw'):data['gnss'].append([stamp,m.latitude,m.longitude,m.altitude,*m.position_covariance,m.position_covariance_type])
 elif t=='/odometry/feedback':
  p=m.pose.pose.position;q=m.pose.pose.orientation;data['feedback'].append([stamp,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]])
 else:
  q=m.pose.pose.orientation;data['heading'].append([stamp,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),m.pose.covariance[35]])
np.savez('/home/ibo/rosbags/glim_aug10_motion_fusion/source.npz',**{k:np.array(v) for k,v in data.items()})
g=np.array(data['gnss']);xy=(g[:,1:3]-g[0,1:3])*[111320,111320*np.cos(np.deg2rad(g[0,1]))];ts=g[:,0]-g[0,0]
for start in range(0,400,30):
 a=xy[(ts>=start)&(ts<start+30)];print('window',start,'span_m',np.linalg.norm(np.ptp(a,axis=0)))
print('GNSS cov diagonal percentiles',np.percentile(g[:,[4,8,12]],[0,50,95,100],axis=0));print('feedback cov percentiles',np.percentile(np.array(data['feedback'])[:,4:],[0,50,95,100],axis=0))
