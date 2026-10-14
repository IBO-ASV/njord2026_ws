#!/usr/bin/env python3
"""Inspect short moving segments separately; do not relax the main 4 s gate."""
import sys,csv,json
from pathlib import Path
from collections import defaultdict
import numpy as np
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message
from evaluate_aug10_fusion import reader,world
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);arrays=np.load(root/'comparison_inputs.npz');od=arrays['odom'];start=1786301520.145669423;groups=defaultdict(list)
for row in csv.DictReader((root/'track_samples.csv').open()):
 r={k:float(v) for k,v in row.items()}
 if 180<=r['stamp']-start<=270:groups[int(r['track_id'])].append(r)
candidates=[]
for id,rows in groups.items():
 a=np.array([[r[k] for k in ['stamp','x_world','y_world','ground_speed','length','width']] for r in rows])
 for s in np.split(a,np.where(np.diff(a[:,0])>.75)[0]+1):
  dur=s[-1,0]-s[0,0];net=np.linalg.norm(s[-1,1:3]-s[0,1:3])
  if len(s)>=5 and 2<=dur<4 and net>=2 and np.percentile(s[:,4:],90)<=8 and np.median(s[:,3])>=.3:candidates.append((net,id,s))
candidates.sort(key=lambda x:x[0],reverse=True);chosen=candidates[:2];wanted=[s[[0,len(s)//2,-1],0] for _,_,s in chosen];clouds=[];r=reader(root/'output_bag',['/task2/points_filtered'])
while r.has_next():
 _,d,_=r.read_next();m=deserialize_message(d,PointCloud2);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if min(abs(t-tt) for times in wanted for tt in times)>.35:continue
 offsets={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y'],'formats':['<f4']*2,'offsets':[offsets[k] for k in ['x','y']],'itemsize':m.point_step});a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step));p=np.column_stack([a[k].ravel() for k in ['x','y']]);p=p[np.isfinite(p).all(axis=1)];clouds.append((t,world(p,t,od)))
fig,axes=plt.subplots(len(chosen),3,figsize=(12,4*len(chosen)),squeeze=False,layout='constrained');summary=[]
for row,(net,id,s),times in zip(axes,chosen,wanted):
 path=s[:,1:3]@arrays['rotation'].T+arrays['translation'];center=path.mean(axis=0);span=max(7,np.max(np.ptp(path,axis=0))*.65)
 for ax,tt in zip(row,times):
  t,p=min(clouds,key=lambda v:abs(v[0]-tt));i=np.argmin(abs(s[:,0]-t));near=np.linalg.norm(p-path[i],axis=1)<2;ax.scatter(p[:,0],p[:,1],s=3,color='gray',alpha=.6);ax.scatter(p[near,0],p[near,1],s=8,color='teal');ax.plot(path[:,0],path[:,1],'--',color='orange',alpha=.5);ax.scatter(path[i,0],path[i,1],s=100,facecolors='none',edgecolors='red');ax.set(xlim=(center[0]-span,center[0]+span),ylim=(center[1]-span,center[1]+span),aspect='equal',xlabel='East (m)',ylabel='North (m)',title=f'Short track {id}, {t-start:.2f} s')
 summary.append({'track_id':id,'start_sec':float(s[0,0]-start),'end_sec':float(s[-1,0]-start),'duration_sec':float(s[-1,0]-s[0,0]),'net_displacement_m':float(net),'median_tracker_speed_mps':float(np.median(s[:,3])),'median_dimensions_m':np.median(s[:,4:],axis=0).tolist(),'status':'unverified short candidate; below main 4-second continuity gate'})
fig.suptitle('Exploratory short candidates: red = tracker, teal = nearby returns (not semantic labels)');fig.savefig(root/'short_moving_candidates.png',dpi=130);plt.close(fig);(root/'short_moving_candidates.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
