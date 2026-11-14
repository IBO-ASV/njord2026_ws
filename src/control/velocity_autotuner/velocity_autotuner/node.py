"""Run bounded surge gain trials while preserving the control-mode safety gate."""
from __future__ import annotations
import csv, math, time
from datetime import datetime, timezone
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger
from control_interfaces.msg import ControlState
from .search import BoundedSearch

class VelocityAutotuner(Node):
    def __init__(self):
        super().__init__('velocity_autotuner')
        self.target=float(self.declare_parameter('target_speed_mps',0.0).value)
        self.max_speed=float(self.declare_parameter('max_test_speed_mps',0.0).value)
        self.thrust_cap=float(self.declare_parameter('test_max_thruster_thrust_n',0.0).value)
        self.ff_bounds=tuple(self.declare_parameter('feedforward_bounds',[0.0,80.0]).value)
        self.p_bounds=tuple(self.declare_parameter('p_bounds',[0.0,80.0]).value)
        self.d_bounds=tuple(self.declare_parameter('d_bounds',[0.0,80.0]).value)
        self.rounds=int(self.declare_parameter('search_rounds',4).value)
        self.settle=float(self.declare_parameter('settle_time_sec',25.0).value)
        self.window=float(self.declare_parameter('steady_window_sec',8.0).value)
        self.step_duration=float(self.declare_parameter('step_duration_sec',18.0).value)
        self.pre_stop=float(self.declare_parameter('pre_step_stop_sec',3.0).value)
        self.d_filter=float(self.declare_parameter('derivative_filter_tau_sec',0.15).value)
        self.over_weight=float(self.declare_parameter('overshoot_weight',4.0).value)
        self.slew_weight=float(self.declare_parameter('thrust_slew_weight',0.01).value)
        self.output_dir=Path(str(self.declare_parameter('output_dir','~/njord_tuning').value)).expanduser()
        self.controller=str(self.declare_parameter('controller_node','/thruster_controller_node').value).rstrip('/')
        odom_topic=str(self.declare_parameter('odom_topic','/odometry/filtered/local').value)
        force_topic=str(self.declare_parameter('force_topic','/debug/current_force').value)
        self._validate()
        self.state=None; self.speed=None; self.odom_at=0.0; self.force=None; self.force_at=0.0
        self.prev_force=None; self.force_sample_at=0.0; self.reserved=False; self.status='IDLE'
        self.failure=''; self.operation=''; self.future=None; self.operation_at=0.0; self.old={}
        self.chosen={'ff':0.0,'p':0.0,'d':0.0}; self.stage=''; self.search=None; self.candidate=0.0
        self.trial_at=0.0; self.samples=[]; self.iae=0.0; self.overshoot=0.0; self.slew=0.0; self.slew_n=0
        self.last_sample=0.0; self.csv_stream=None; self.writer=None; self.run_id=''; self.trial_no=0
        self.cmd_pub=self.create_publisher(Twist,'/cmd_vel_tune',10)
        self.active_pub=self.create_publisher(Bool,'/control/tuning_active',10)
        self.status_pub=self.create_publisher(String,'/velocity_tuning/status',10)
        self.create_subscription(ControlState,'/control/state',self._control_state,10)
        self.create_subscription(Odometry,str(odom_topic),self._odom,20)
        self.create_subscription(Float32MultiArray,str(force_topic),self._thrust,10)
        self.create_service(Trigger,'/velocity_tuning/start',self._start)
        self.create_service(Trigger,'/velocity_tuning/pause',self._pause)
        self.create_service(Trigger,'/velocity_tuning/resume',self._resume)
        self.create_service(Trigger,'/velocity_tuning/stop',self._stop)
        self.get_client=self.create_client(GetParameters,self.controller+'/get_parameters')
        self.set_client=self.create_client(SetParameters,self.controller+'/set_parameters')
        self.create_timer(.05,self._tick)

    def _validate(self):
        if not (0 < self.target <= self.max_speed and self.thrust_cap > 0): raise ValueError('set positive target, max speed >= target, and test thrust cap')
        if self.rounds < 1 or min(self.settle,self.window,self.step_duration) <= 0 or self.pre_stop < 0: raise ValueError('trial durations and search_rounds are invalid')
        if self.d_filter < 0: raise ValueError('derivative filter time must be nonnegative')
        for bounds in (self.ff_bounds,self.p_bounds,self.d_bounds):
            if len(bounds)!=2 or not (0 <= bounds[0] < bounds[1]): raise ValueError('gain bounds must be increasing and nonnegative')

    def _control_state(self,msg):
        self.state=msg
        if not self.reserved: return
        if msg.emergency_stop: self._fault('Emergency stop asserted')
        elif self.status=='RUNNING' and (msg.requested_mode==ControlState.MODE_MANUAL or msg.effective_source in (ControlState.SOURCE_MANUAL,ControlState.SOURCE_SBUS)):
            self._pause_now('MANUAL selected; reposition, return to AUTO, then resume')
        elif self.status=='RUNNING' and not msg.auto_permitted:
            self._pause_now('AUTO safety gate closed; restore readiness, then resume')

    def _odom(self,msg):
        if msg.child_frame_id.strip('/')!='base_link': return
        value=float(msg.twist.twist.linear.x)
        if not math.isfinite(value): return
        now=time.monotonic(); self.speed=value; self.odom_at=now
        if self.status=='RUNNING' and self.trial_at:
            elapsed=now-self.trial_at-self.pre_stop
            if elapsed>=0:
                self.samples.append((elapsed,value))
                if self.stage!='ff':
                    dt=max(0.0,min(.2,now-(self.last_sample or now)))
                    self.iae+=abs(self.target-value)*dt
                    self.overshoot=max(self.overshoot,value-self.target)
                self.last_sample=now

    def _thrust(self,msg):
        vals=tuple(float(v) for v in msg.data)
        if len(vals)!=4 or not all(math.isfinite(v) for v in vals): return
        now=time.monotonic()
        if self.prev_force is not None and self.status=='RUNNING':
            dt=max(.001,min(.2,now-self.force_sample_at))
            self.slew+=sum((a-b)**2 for a,b in zip(vals,self.prev_force))/dt; self.slew_n+=1
        self.prev_force=vals; self.force=vals; self.force_at=now; self.force_sample_at=now
    def _param(self,n,v): return Parameter(n,value=v).to_parameter_msg()
    def _call_set(self,values,cb):
        q=SetParameters.Request(); q.parameters=[self._param(k,v) for k,v in values.items()]
        f=self.set_client.call_async(q); f.add_done_callback(cb)
    def _start(self,q,r):
        if self.reserved or self.state is None or self.state.requested_mode!=ControlState.MODE_AUTO or not self.state.auto_permitted or self.speed is None or time.monotonic()-self.odom_at>1:
            r.success=False; r.message='Need fresh odometry and ready AUTO, with no active session'; return r
        if not self.get_client.service_is_ready() or not self.set_client.service_is_ready(): r.success=False; r.message='Controller parameter services unavailable'; return r
        self.reserved=True; self.status='PREPARING'; self.operation='start'
        q=GetParameters.Request(); q.names=['safety.max_thruster_thrust_n','control.i.surge','control.ff.surge','control.p.surge','control.d.surge','input_scaling.max_linear_x']
        self.future=self.get_client.call_async(q); r.success=True; r.message='Preparing'; return r
    def _pause(self,q,r):
        if not self.reserved: r.success=False; r.message='No active session'; return r
        self._pause_now('Paused by operator'); r.success=True; r.message='Zero command; manual mode can take control'; return r
    def _pause_now(self,why):
        if self.stage and self.trial_at: self._write_row('INTERRUPTED',None,None,'INTERRUPTED')
        self.status='PAUSED'; self.failure=why; self.trial_at=0.
    def _resume(self,q,r):
        if not self.reserved or self.status!='PAUSED' or self.state is None or self.state.requested_mode!=ControlState.MODE_AUTO or not self.state.auto_permitted:
            r.success=False; r.message='Return to ready AUTO before resume'; return r
        self.status='RUNNING'; self.failure=''; self._begin_trial(); r.success=True; r.message='Resumed current candidate'; return r
    def _stop(self,q,r):
        if not self.reserved or self.state is None or self.state.requested_mode!=ControlState.MODE_MANUAL:
            r.success=False; r.message='Select MANUAL before stopping/restoring'; return r
        restore={'safety.max_thruster_thrust_n':self.old['safety.max_thruster_thrust_n'],'control.i.surge':self.old['control.i.surge']}
        if self.status!='COMPLETE': restore.update({k:self.old[k] for k in ('control.ff.surge','control.p.surge','control.d.surge')})
        self.status='RESTORING'; self._call_set(restore,self._restored); r.success=True; r.message='Restoring saved settings'; return r
    def _restored(self,f):
        self.reserved=False; self.status='IDLE'; self.trial_at=0.; self._close_csv()
    def _fault(self,s): self._pause_now(s)
    def _tick(self):
        now=time.monotonic(); self.active_pub.publish(Bool(data=self.reserved)); cmd=Twist()
        if self.status=='RUNNING' and self.trial_at:
            elapsed=now-self.trial_at
            if now-self.odom_at>1 or now-self.force_at>1: self._fault('Stale odometry/thrust; zero command')
            elif abs(self.speed)>self.max_speed: self._fault('Speed limit exceeded; zero command')
            elif elapsed>=self.pre_stop: cmd.linear.x=self.target
            limit=self.pre_stop+(self.settle+self.window if self.stage=='ff' else self.step_duration)
            if elapsed>=limit: self._finish_trial()
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(String(data=f'{self.status} {self.stage} {self.candidate:.3g} {self.failure}'))
        if self.future and self.future.done():
            f=self.future; self.future=None
            if self.operation=='start':
                try:
                    names=['safety.max_thruster_thrust_n','control.i.surge','control.ff.surge','control.p.surge','control.d.surge','input_scaling.max_linear_x']
                    self.old=dict(zip(names,[x.double_value for x in f.result().values]))
                    if self.target>self.old[names[-1]]: raise ValueError('target exceeds configured max_linear_x')
                    cap=self.thrust_cap if self.old[names[0]]<=0 else min(self.thrust_cap,self.old[names[0]])
                    self._call_set({names[0]:cap,names[1]:0.},self._prepared)
                except Exception as e: self._fault(str(e))
            elif self.operation=='candidate': self._candidate_applied(f)
    def _prepared(self,f):
        try:
            if not all(x.successful for x in f.result().results): raise RuntimeError('Controller rejected safety settings')
            self.output_dir.mkdir(parents=True,exist_ok=True); self.run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            self.csv_stream=(self.output_dir/f'velocity_tuning_{self.run_id}.csv').open('w',newline='')
            self.writer=csv.writer(self.csv_stream); self.writer.writerow(['stage','candidate','score','signed_error','overshoot_mps','iae','thrust_slew','result'])
            self.status='RUNNING'; self._next_stage('ff')
        except Exception as e: self._fault(str(e))
    def _next_stage(self,s):
        self.stage=s; b={'ff':self.ff_bounds,'p':self.p_bounds,'d':self.d_bounds}[s]
        self.search=BoundedSearch(b[0],b[1],self.rounds,'steady' if s=='ff' else 'loss'); self._next_candidate()
    def _next_candidate(self):
        c=self.search.next_candidate()
        if c is None:
            self.chosen[self.stage]=self.search.best_candidate()
            if self.stage=='ff': self._next_stage('p')
            elif self.stage=='p': self._next_stage('d')
            else: self._complete()
            return
        self.candidate=c; self.samples=[]; self.iae=0.; self.overshoot=0.; self.slew=0.; self.slew_n=0; self.last_sample=0.; self.trial_at=0.
        vals={'control.ff.surge':self.chosen['ff'],'control.p.surge':self.chosen['p'],'control.d.surge':self.chosen['d']}
        vals[{'ff':'control.ff.surge','p':'control.p.surge','d':'control.d.surge'}[self.stage]]=c
        self.operation='candidate'; self._call_set(vals,self._candidate_applied)
    def _candidate_applied(self,f):
        try:
            if not all(x.successful for x in f.result().results): raise RuntimeError('Controller rejected gain')
            self.operation=''; self._begin_trial()
        except Exception as e: self._fault(str(e))
    def _begin_trial(self): self.samples=[]; self.iae=0.; self.overshoot=0.; self.slew=0.; self.slew_n=0; self.last_sample=0.; self.trial_at=time.monotonic()
    def _finish_trial(self):
        vals=[v for t,v in self.samples if t>=self.pre_stop+(self.settle if self.stage=='ff' else 0)]
        if len(vals)<3: self._fault('Not enough fresh odometry data'); return
        if self.stage=='ff': mean=sum(vals)/len(vals); signed=mean-self.target; score=abs(signed)
        else: signed=0.; score=self.iae+self.over_weight*self.overshoot+self.slew_weight*math.sqrt(self.slew/max(1,self.slew_n))
        self._write_row(self.stage,score,signed,'OK'); self.search.record(self.candidate,score,signed); self.trial_at=0.; self._next_candidate()
    def _write_row(self,stage,score,signed,result):
        if self.writer:
            self.writer.writerow([stage,self.candidate,'' if score is None else score,'' if signed is None else signed,self.overshoot,self.iae,self.slew,result]); self.csv_stream.flush()
    def _complete(self):
        self.status='COMPLETE'; self.operation=''; self.trial_at=0.
        p=self.output_dir/f'velocity_gains_{self.run_id}.yaml'
        p.write_text(f'control:\n  ff:\n    surge: {self.chosen["ff"]:.6g}\n  p:\n    surge: {self.chosen["p"]:.6g}\n  d:\n    surge: {self.chosen["d"]:.6g}\n')
        self._close_csv()
    def _close_csv(self):
        if self.csv_stream: self.csv_stream.close(); self.csv_stream=None; self.writer=None

def main(args=None):
    rclpy.init(args=args); n=VelocityAutotuner()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()
