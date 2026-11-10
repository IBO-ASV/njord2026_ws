#!/usr/bin/env python3
"""Loop recorded display data with fresh visualization timestamps, no controls."""
import sys,time
import rclpy,rosbag2_py
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
class Preview(Node):
 def __init__(self,path):
  super().__init__('recorded_candidate_preview');r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=path,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));self.types={t.name:get_message(t.type) for t in r.get_all_topics_and_types()};self.pubs={t:self.create_publisher(cls,t,10) for t,cls in self.types.items()};self.events=[]
  while r.has_next():
   topic,data,stamp=r.read_next();self.events.append((stamp,topic,data))
  assert self.events,'Empty preview';self.events.sort();self.zero=self.events[0][0];self.duration=(self.events[-1][0]-self.zero)*1e-9+.15;self.begin=time.monotonic();self.index=0;self.cycle=0;self.create_timer(.005,self.tick);self.get_logger().info(f'Looping {self.duration:.2f} s recorded clip, {len(self.events)} events')
 def tick(self):
  elapsed=time.monotonic()-self.begin
  if elapsed>=self.duration:self.begin=time.monotonic();elapsed=0.;self.index=0;self.cycle+=1;self.get_logger().info(f'Preview loop {self.cycle}')
  while self.index<len(self.events) and (self.events[self.index][0]-self.zero)*1e-9<=elapsed:
   _,topic,data=self.events[self.index];m=deserialize_message(data,self.types[topic]);now=self.get_clock().now().to_msg()
   if hasattr(m,'header'):m.header.stamp=now
   if hasattr(m,'markers'):
    for marker in m.markers:marker.header.stamp=now
   self.pubs[topic].publish(m);self.index+=1
rclpy.init();node=Preview(sys.argv[1])
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
