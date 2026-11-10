import rosbag2_py,numpy as np,json
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu,PointCloud2
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri='/home/ibo/njord2026_ws/rosbag2_2026_08_10-03_52_00',storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/livox/imu','/livox/lidar']));data={'/livox/imu':[],'/livox/lidar':[]}
while r.has_next():
 t,d,rec=r.read_next();m=deserialize_message(d,Imu if t.endswith('imu') else PointCloud2);st=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;data[t].append([st,rec*1e-9])
res={}
for t,rows in data.items():
 a=np.array(rows);dt=np.diff(a[:,0]);res[t]={'header_interval_p50_p99_max_s':np.percentile(dt,[50,99,100]).tolist(),'record_minus_header_p0_p50_p99_max_s':np.percentile(a[:,1]-a[:,0],[0,50,99,100]).tolist(),'backwards_headers':int(np.sum(dt<0))}
print(json.dumps(res,indent=2))
