#!/usr/bin/env python3
"""Stream recorded camera/LiDAR only; never publish recorded control topics.
Playback follows bag recording timestamps, not hardware synchronization.
Fresh display stamps prevent backwards time at repeat boundaries.
"""
import sys,time,yaml
from pathlib import Path
import rclpy,rosbag2_py
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,CompressedImage
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray

class Preview(Node):
 def __init__(self,path):
  super().__init__('recorded_sensor_preview')
  self.path=path
  info=yaml.safe_load((Path(path)/'metadata.yaml').read_text())['rosbag2_bagfile_information']
  self.zero=info['starting_time']['nanoseconds_since_epoch']
  self.duration=info['duration']['nanoseconds']*1e-9+.15
  self.types={'/livox/lidar':PointCloud2,'/back_cam/image_raw/compressed':CompressedImage}
  self.pubs={t:self.create_publisher(cls,out,3) for t,cls,out in [('/livox/lidar',PointCloud2,'/preview/points'),('/back_cam/image_raw/compressed',CompressedImage,'/preview/camera/back/compressed')]}
  self.status=self.create_publisher(String,'/preview/status',3)
  self.clear=self.create_publisher(MarkerArray,'/preview/tracks',1)
  self.cycle=0;self.last_status=-1;self.reset();self.create_timer(.005,self.tick)
 def reset(self):
  self.reader=rosbag2_py.SequentialReader()
  self.reader.open(rosbag2_py.StorageOptions(uri=self.path,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
  self.reader.set_filter(rosbag2_py.StorageFilter(topics=list(self.types)))
  self.pending=self.reader.read_next() if self.reader.has_next() else None
  if self.pending is None:raise RuntimeError('No camera or LiDAR messages')
  self.begin=time.monotonic();self.last_status=-1
  self.get_logger().info(f'Loop {self.cycle}: {self.duration:.2f} seconds')
 def tick(self):
  elapsed=time.monotonic()-self.begin
  if elapsed>=self.duration:self.cycle+=1;self.reset();elapsed=0.
  count=0
  while self.pending and (self.pending[2]-self.zero)*1e-9<=elapsed and count<12:
   topic,data,stamp=self.pending;m=deserialize_message(data,self.types[topic]);m.header.stamp=self.get_clock().now().to_msg()
   self.pubs[topic].publish(m);count+=1
   self.pending=self.reader.read_next() if self.reader.has_next() else None
  if int(elapsed)!=self.last_status:
   self.last_status=int(elapsed)
   self.status.publish(String(data=f'RECORDED Aug10 04:01:37 | source +{elapsed:.1f}s | camera + raw LiDAR | no GLIM/tracks | recording-time playback'))
   marker=Marker();marker.action=Marker.DELETEALL;marker.header.frame_id='livox_frame';marker.header.stamp=self.get_clock().now().to_msg();self.clear.publish(MarkerArray(markers=[marker]))

rclpy.init();node=Preview(sys.argv[1])
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
