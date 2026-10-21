#!/usr/bin/env python3
"""Create an offline sensor bag ordered by acquisition; preserve payload bytes.
LiDAR becomes available at its last point timestamp +1 ms. A 20 s heap
covers the measured source recording delay; abort if this bound is violated.
"""
import sys,heapq,json
from pathlib import Path
from collections import Counter
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu,PointCloud2,NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped

def main(source,target):
 types={'/livox/imu':Imu,'/livox/lidar':PointCloud2,'/sensor/vehicle_gnss/fix/raw':NavSatFix,'/sensor/vehicle_gnss/compass/raw':PoseWithCovarianceStamped}
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=source,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
 w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=target,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 for info in r.get_all_topics_and_types():
  if info.name in types:w.create_topic(info)
 heap=[];counts=Counter();last=-1;index=0;maxdelay=0
 def flush(limit):
  nonlocal last
  while heap and heap[0][0]<=limit:
   stamp,_,topic,payload=heapq.heappop(heap)
   if stamp<last:raise RuntimeError('20 s reordering window insufficient; discard incomplete output')
   w.write(topic,payload,stamp);last=stamp;counts[topic]+=1
 while r.has_next():
  t,d,rec=r.read_next();m=deserialize_message(d,types[t]);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  if t=='/livox/lidar':
   field=next(f for f in m.fields if f.name=='timestamp');assert field.datatype==8
   a=np.ndarray((m.height,m.width),dtype='<f8',buffer=m.data,offset=field.offset,strides=(m.row_step,m.point_step));valid=a[np.isfinite(a)]
   end=int(np.max(valid));assert -10**7<end-stamp<3*10**8, 'Unexpected point timestamp units/span'
   stamp=max(stamp,end)+1000000
  maxdelay=max(maxdelay,(rec-stamp)*1e-9);heapq.heappush(heap,(stamp,index,t,d));index+=1;flush(rec-20*10**9)
 flush(float('inf'));report={'messages':dict(counts),'max_recording_delay_s':maxdelay,'payloads_unchanged':True,'lidar_release':'last point timestamp + 1 ms','heap_window_s':20};Path(target).with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main(sys.argv[1],sys.argv[2])
