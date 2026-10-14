#!/usr/bin/env python3
"""Evaluate world-position-stable track candidates; not semantic wall labels."""
import sys,json,csv,math
from pathlib import Path
from collections import defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from ship_perception_msgs.msg import TrackedObjectArray


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def evaluate(root, window=None):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/odometry/filtered/local','/tracked_objects']))
    odoms=[];arrays=[]
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic=='/odometry/filtered/local':
            m=deserialize_message(data,Odometry);p=m.pose.pose.position;v=m.twist.twist.linear
            odoms.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,yaw(m.pose.pose.orientation),v.x,v.y,m.twist.twist.angular.z])
        else:arrays.append(deserialize_message(data,TrackedObjectArray))
    if not odoms:raise RuntimeError('No local odometry')
    od=np.asarray(sorted(odoms,key=lambda row:row[0]));od[:,3]=np.unwrap(od[:,3]);rows=[];rejected=0
    for arr in arrays:
        for obj in arr.objects:
            if obj.track_state!=1 or obj.miss_count or obj.point_count<5:continue
            stamp=obj.header.stamp.sec+obj.header.stamp.nanosec*1e-9
            nearest=np.min(np.abs(od[:,0]-stamp))
            if nearest>0.15:rejected+=1;continue
            ex,ey,heading,vx,vy,w=[np.interp(stamp,od[:,0],od[:,i]) for i in range(1,7)]
            p=obj.state.pose.pose.position;v=obj.state.twist.twist.linear;theta=yaw(obj.state.pose.pose.orientation)
            cb,sb=math.cos(theta),math.sin(theta);rx,ry=cb*v.x-sb*v.y,sb*v.x+cb*v.y
            ax,ay=rx+vx-w*p.y,ry+vy+w*p.x
            c,s=math.cos(heading),math.sin(heading)
            rows.append([stamp,obj.object_id,ex+c*p.x-s*p.y,ey+s*p.x+c*p.y,
                         math.hypot(rx,ry),math.hypot(ax,ay),math.hypot(vx,vy),w,obj.dimensions.x,obj.dimensions.y])
    if window is not None:
        rows=[r for r in rows if window[0] <= r[0]-od[0,0] <= window[1]]
    groups=defaultdict(list)
    for row in rows:groups[row[1]].append(row)
    candidates=[]
    for identifier,values in groups.items():
        a=np.asarray(values);span=np.ptp(a[:,0]);center=np.median(a[:,2:4],axis=0);radius=float(np.percentile(np.linalg.norm(a[:,2:4]-center,axis=1),95));moving=a[a[:,6]>=0.2]
        if span<8 or radius>0.75 or len(moving)<10:continue
        candidates.append(dict(track_id=int(identifier),duration_sec=float(span),position_radius95_m=radius,
            center_x_m=float(center[0]),center_y_m=float(center[1]),moving_samples=len(moving),
            relative_speed_median_mps=float(np.median(moving[:,4])),ground_speed_median_mps=float(np.median(moving[:,5])),
            ground_speed_p95_mps=float(np.percentile(moving[:,5],95))))
    result={'odom_count':len(odoms),'track_arrays':len(arrays),'confirmed_track_samples':len(rows),'unmatched_stamp_samples':rejected,
        'ego_position_span_m':float(np.linalg.norm(np.ptp(od[:,1:3],axis=0))),'ego_speed_max_mps':float(np.max(np.linalg.norm(od[:,4:6],axis=1))),
        'ego_yaw_span_deg':float(np.ptp(od[:,3])*180/math.pi),'candidate_rule':'duration>=8s, world-position radius95<=0.75m, >=10 samples with ego speed>=0.2m/s; no speed-based selection',
        'acceptance_rule':'median ground speed <=0.2m/s and p95<=0.5m/s (provisional, not calibrated)',
        'stationary_candidates':candidates}
    if candidates:
        selected={x['track_id'] for x in candidates};a=np.asarray([r for r in rows if r[1] in selected and r[6]>=0.2]);result.update(candidate_speed_median_mps=float(np.median(a[:,5])),candidate_speed_p95_mps=float(np.percentile(a[:,5],95)),relative_speed_median_mps=float(np.median(a[:,4])))
    (root/'static_track_summary.json').write_text(json.dumps(result,indent=2))
    with (root/'track_samples.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['stamp','track_id','x_world','y_world','relative_speed','ground_speed','ego_speed','ego_yaw_rate','length','width']);writer.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    axes[0].plot(od[:,1],od[:,2],color='black',label='Own vessel')
    if not candidates:
        for identifier,values in sorted(groups.items(),key=lambda kv:len(kv[1]),reverse=True)[:8]:
            a=np.asarray(values);axes[0].plot(a[:,2],a[:,3],'.-',markersize=2,alpha=.4,label=f'Unverified track {identifier}')
    for item in candidates[:8]:
        a=np.asarray(groups[item['track_id']]);axes[0].plot(a[:,2],a[:,3],'.-',markersize=2,label=f"Candidate {item['track_id']}")
    axes[0].set(aspect='equal',xlabel='Local world x (m)',ylabel='Local world y (m)',title='Position-stable candidates (not verified wall labels)');axes[0].legend(fontsize=7)
    if candidates:
        axes[1].boxplot([np.asarray([r[4] for r in rows if r[1] in selected and r[6]>=0.2]),np.asarray([r[5] for r in rows if r[1] in selected and r[6]>=0.2])],labels=['Relative','Ground compensated'],showfliers=False)
    axes[1].axhline(.2,color='orange',linestyle='--',label='Provisional median limit 0.2 m/s');axes[1].set(ylabel='Horizontal speed (m/s)',title='Only samples while own-vessel speed >=0.2 m/s');axes[1].legend(fontsize=8)
    fig.savefig(root/'static_tracks.png',dpi=160);plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':evaluate(Path(sys.argv[1]),tuple(map(float,sys.argv[2:4])) if len(sys.argv)>3 else None)
