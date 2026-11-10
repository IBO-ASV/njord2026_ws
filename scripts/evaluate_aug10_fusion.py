#!/usr/bin/env python3
"""Offline scene consistency and moving-track candidates; no semantic labels."""
import sys,json,math
from pathlib import Path
from collections import defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from prepare_aug10_fusion import yaw

def reader(path,topics):
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=topics));return r

def world(points,t,od):
 x,y,th=[np.interp(t,od[:,0],od[:,i]) for i in [1,2,3]];c,s=np.cos(th),np.sin(th);return points@np.array([[c,s],[-s,c]])+np.array([x,y])

def main(root,source,topic='/comparison/fused'):
 arrays=np.load(root/'comparison_inputs.npz');od=arrays['odom'];g=arrays['gnss'];start=1786301520.145669423;fused=[]
 r=reader(root/'fusion_output_bag',[topic])
 while r.has_next():
  _,d,_=r.read_next();m=deserialize_message(d,Odometry);p=m.pose.pose.position;v=m.twist.twist.linear;fused.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,yaw(m.pose.pose.orientation),v.x,v.y,m.twist.twist.angular.z])
 fu=np.array(sorted(fused));fu[:,3]=np.unwrap(fu[:,3]);assert np.isfinite(fu).all();np.save(root/'fused_odom.npy',fu)
 clouds=[];r=reader(root/'output_bag',['/task2/points_filtered']);last=-np.inf
 while r.has_next():
  _,d,_=r.read_next();m=deserialize_message(d,PointCloud2);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if t-last<.45:continue
  offsets={f.name:f.offset for f in m.fields};dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ['x','y','z']],'itemsize':m.point_step});a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step));pts=np.column_stack([a[k].ravel() for k in ['x','y']]);pts=pts[np.isfinite(pts).all(axis=1)];rr=np.linalg.norm(pts,axis=1);pts=pts[(rr>3)&(rr<30)][::2]
  if len(pts)<30 or max(np.min(abs(od[:,0]-t)),np.min(abs(fu[:,0]-t)))>.15:continue
  clouds.append((t,pts));last=t
 scores=[]
 for i,(t,p) in enumerate(clouds):
  if not 180<=t-start<=270:continue
  j=min(range(len(clouds)),key=lambda j:abs(clouds[j][0]-t-5));t2,p2=clouds[j]
  if abs(t2-t-5)>.6:continue
  if any(np.min(abs(o[:,0]-tt))>.15 for o in [od,fu] for tt in [t,t2]):continue
  row=[t-start]
  for o in [od,fu]:
   aa=world(p,t,o);bb=world(p2,t2,o);dist=cKDTree(bb).query(aa)[0];dist2=cKDTree(aa).query(bb)[0];both=np.r_[dist,dist2];row.extend([float(np.median(both)),float(np.percentile(both,80)),float(np.mean(both<.5))])
  scores.append(row)
 scores=np.array(scores);np.savetxt(root/'scene_consistency.csv',scores,delimiter=',',header='sec,glim_nn_median,glim_nn_p80,glim_overlap_05,fused_nn_median,fused_nn_p80,fused_overlap_05')
 # Split track IDs at observation gaps, because first-to-last duration can hide fragmentation.
 import csv
 rows=list(csv.DictReader((root/'track_samples.csv').open()));groups=defaultdict(list)
 for row in rows:
  r={k:float(v) for k,v in row.items()};groups[int(r['track_id'])].append(r)
 moving=[]
 for id,values in groups.items():
  values.sort(key=lambda r:r['stamp']);segments=[];seg=[]
  for v in values:
   if seg and v['stamp']-seg[-1]['stamp']>.75:segments.append(seg);seg=[]
   seg.append(v)
  segments.append(seg)
  for seg in segments:
   if len(seg)<10:continue
   tt=np.array([v['stamp'] for v in seg]);p=np.array([[v['x_world'],v['y_world']] for v in seg]);dur=tt[-1]-tt[0];net=np.linalg.norm(p[-1]-p[0]);dims=np.array([[v['length'],v['width']] for v in seg]);speed=np.median([v['ground_speed'] for v in seg])
   if dur>=4 and net>=2 and np.percentile(dims,90)<=8 and speed>=.3:
    moving.append({'track_id':id,'start_sec':float(tt[0]-start),'end_sec':float(tt[-1]-start),'duration_sec':float(dur),'net_displacement_m':float(net),'median_speed_mps':float(speed),'samples':len(seg)})
 moving.sort(key=lambda x:x['net_displacement_m'],reverse=True)
 result={'fused_topic':topic,'gnss_sigma_floor_m':2.0 if topic.endswith('conservative') else 0.5,'evaluation_window_sec':[180,270],'cloud_pair_interval_sec':5,'cloud_pairs':len(scores),'glim_scene_nn_median_m':float(np.median(scores[:,1])),'fused_scene_nn_median_m':float(np.median(scores[:,4])),'glim_scene_nn_p80_m':float(np.median(scores[:,2])),'fused_scene_nn_p80_m':float(np.median(scores[:,5])),'moving_candidates':moving,'limitations':'Scene nearest-neighbor residual includes changing visibility and moving objects; not surveyed accuracy. Moving candidates are GLIM-compensated tracks, not verified ships. GNSS covariance floor is an experimental assumption. No production adoption.'}
 (root/'fusion_comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
 fig,ax=plt.subplots(1,3,figsize=(15,4.8),layout='constrained');ax[0].plot(g[:,1],g[:,2],'.',ms=1,alpha=.4,label='GNSS antenna');ax[0].plot(od[:,1],od[:,2],label='GLIM base');ax[0].plot(fu[:,1],fu[:,2],label='Fused base');ax[0].set(aspect='equal',xlabel='East (m)',ylabel='North (m)',title='Full bag: positions');ax[0].legend(fontsize=8)
 ax[1].plot(od[:,0]-start,np.linalg.norm(od[:,4:6],axis=1),label='GLIM');ax[1].plot(fu[:,0]-start,np.linalg.norm(fu[:,4:6],axis=1),label='Fused',alpha=.7);ax[1].axvspan(180,270,color='gray',alpha=.15);ax[1].set(xlabel='Bag time (s)',ylabel='Own speed (m/s)',title='Moving interval');ax[1].legend()
 ax[2].plot(scores[:,0],scores[:,1],label='GLIM');ax[2].plot(scores[:,0],scores[:,4],label='Fused');ax[2].set(xlabel='Bag time (s)',ylabel='Median nearest point distance (m)',title='Scene consistency, 5 s pairs');ax[2].legend();ax[2].set_yscale('log');fig.savefig(root/'fusion_comparison.png',dpi=150);plt.close(fig)
 choices=[item for item in moving if item['start_sec']<270 and item['end_sec']>180][:3]
 if choices:
  fig,axes=plt.subplots(1,len(choices),figsize=(5*len(choices),5),squeeze=False,layout='constrained')
  for ax,item in zip(axes[0],choices):
   values=[v for v in groups[item['track_id']] if item['start_sec']-.01<=v['stamp']-start<=item['end_sec']+.01];path=np.array([[v['x_world'],v['y_world']] for v in values])@arrays['rotation'].T+arrays['translation'];mid=start+(item['start_sec']+item['end_sec'])/2;t,p=min(clouds,key=lambda pair:abs(pair[0]-mid));background=world(p,t,od);ax.scatter(background[:,0],background[:,1],s=2,color='gray',alpha=.45);ax.plot(path[:,0],path[:,1],'.-',color='darkorange');ax.scatter(path[0,0],path[0,1],marker='s',color='green',label='start');ax.scatter(path[-1,0],path[-1,1],marker='x',color='red',label='end');center=np.mean(path,axis=0);span=max(8,np.max(np.ptp(path,axis=0))*.65);ax.set(xlim=(center[0]-span,center[0]+span),ylim=(center[1]-span,center[1]+span),aspect='equal',xlabel='East (m)',ylabel='North (m)',title=f"Unverified track {item['track_id']}\n{item['start_sec']:.1f}-{item['end_sec']:.1f}s, {item['net_displacement_m']:.1f}m");ax.legend(fontsize=8)
  fig.savefig(root/'moving_candidates.png',dpi=140);plt.close(fig)
 selected=[(t,p) for t,p in clouds if 175<=t-start<=280][::2];fig,axes=plt.subplots(1,2,figsize=(10,5),layout='constrained');artists=[]
 for ax,label in zip(axes,['GLIM','GNSS + GLIM']):
  ax.set(aspect='equal',xlabel='East (m)',ylabel='North (m)',title=label);artists.append((ax.scatter([],[],s=2),ax.plot([],[],'ro',ms=5)[0]));ax.set_xlim(np.min(g[:,1])-15,np.max(g[:,1])+15);ax.set_ylim(np.min(g[:,2])-15,np.max(g[:,2])+15)
 def update(i):
  t,p=selected[i]
  for o,(sc,boat) in zip([od,fu],artists):sc.set_offsets(world(p,t,o));boat.set_data([np.interp(t,o[:,0],o[:,1])],[np.interp(t,o[:,0],o[:,2])])
  fig.suptitle(f'August 10 | {t-start:.1f} s | red: own vessel; points: observations')
  return []
 anim=FuncAnimation(fig,update,frames=len(selected));anim.save(root/'moving_scene.gif',writer=PillowWriter(fps=6),dpi=90);plt.close(fig)

if __name__=='__main__':main(Path(sys.argv[1]),sys.argv[2],sys.argv[3] if len(sys.argv)>3 else '/comparison/fused')
