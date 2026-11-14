#!/usr/bin/env python3
"""Fast production-MMG speed tuning, plant sweep, identification and balance trials."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from njord_sim.calibration import (AXES, IDS, fit_balance, fit_mmg, force_profile,
                                  load_csv, tracking_metrics, velocity_profile, write_reference)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--plot',action='store_true')
    parser.add_argument('--mmg-config',help='identified MMG YAML; defaults to the installed model')
    args = parser.parse_args()
    out = Path(args.output).resolve()
    if out.exists() and any(out.iterdir()): parser.error('choose an empty output directory')
    out.mkdir(parents=True, exist_ok=True)
    executable = Path(get_package_prefix('dutyed_tf_pub_with_disturbance'))/'lib/dutyed_tf_pub_with_disturbance/velocity_benchmark'
    mmg_file = Path(args.mmg_config) if args.mmg_config else Path(get_package_share_directory('dutyed_tf_pub_with_disturbance'))/'config/node_config.yaml'
    base = yaml.safe_load(mmg_file.read_text())['dutyed_tf_pub_with_disturbance_node']['ros__parameters']['mmg']
    if base.get('x_G',0.)!=0.:parser.error('combined identification currently requires x_G=0')
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='182', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    write_reference(out/'velocity_reference.csv', velocity_profile())
    def run(name, reference, params, mode='velocity'):
        target = out/name
        target.mkdir()
        overrides = {**{'mmg.'+k:float(v) for k,v in base.items()}, **params,
                     'input_csv': str(reference), 'output_csv': str(target/'samples.csv'), 'mode':mode}
        (target/'parameters.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':overrides}}))
        with (target/'run.log').open('w') as log:
            result = subprocess.run([str(executable),'--ros-args','--params-file',str(target/'parameters.yaml')],
                                    env=env,stdout=log,stderr=subprocess.STDOUT,timeout=45)
        if result.returncode: raise RuntimeError(f'{name}: benchmark failed; inspect run.log')
        return load_csv(target/'samples.csv')
    baseline = tracking_metrics(run('baseline',out/'velocity_reference.csv',{}))
    candidates = []
    # Select on nominal and two calibration plants. Independently seeded plants below are holdouts.
    tuning_plants = [{}, {'mmg.mass_kg':base['mass_kg']*1.3,'mmg.I_zG':base['I_zG']*1.333333,'mmg.R_0':base['R_0']*1.4,'actuator_tau':.15},
                     {'mmg.mass_kg':base['mass_kg']*.8,'mmg.m_y':base['m_y']*.7,'mmg.Y_v':base['Y_v']*1.4,'actuator_scale':.85}]
    for i,(kp,ki,kpy,kiy,tau) in enumerate(((40,15,12,4.5,0.),(100,55,30,16.5,0.),
        (100,55,30,16.5,.15),(100,55,30,16.5,.25),(100,55,30,16.5,.35),
        (100,55,12,6.,0.),(100,55,10,6.,0.),(100,55,12,8.,.15),
        (80,45,20,12,.25))):
        params = {f'control.{kind}.{axis}':float(value) for kind,values in (
            ('p',(kp,kp,kpy)),('i',(ki,ki,kiy))) for axis,value in zip(('surge','sway','yaw'),values)}
        inertia_scale=((base['mass_kg']+base['m_x'])/11.,(base['mass_kg']+base['m_y'])/20.,(base['I_zG']+base['J_z'])/3.)
        for axis,factor in zip(('surge','sway','yaw'),inertia_scale):
            for kind in ('p','i'):params[f'control.{kind}.{axis}']*=factor
        params["control.reference_tau_sec"] = tau
        metrics = [tracking_metrics(run(f'tune_{i}_{j}',out/'velocity_reference.csv',params|plant))
                   for j,plant in enumerate(tuning_plants)]
        score = sum(sum(m['axes'][a]['rmse'] for a in AXES) for m in metrics)
        candidates.append({'parameters':params,'metrics':metrics,'score':score,'passed':all(m['passed'] for m in metrics)})
    eligible = [c for c in candidates if c['passed']]
    best = min(eligible or candidates,key=lambda c:c['score'])
    rng = np.random.default_rng(39)
    sweep = []
    for i in range(8):
        plant = {'mmg.'+k:float(base[k]*rng.uniform(.7,1.3))
                 for k in ('mass_kg','I_zG','m_x','m_y','J_z','R_0','Y_v','N_r')}
        plant.update(actuator_scale=float(rng.uniform(.85,1.15)), actuator_tau=float(rng.uniform(0,.15)))
        metric = tracking_metrics(run(f'holdout_{i}',out/'velocity_reference.csv',best['parameters']|plant))
        sweep.append({'plant':plant,'metrics':metric})
    loaded = {'mmg.mass_kg':max(24.,base['mass_kg']*1.3),'mmg.I_zG':max(3.,base['I_zG']*1.3),'mmg.m_x':max(2.,base['m_x']*1.3),'mmg.m_y':max(20.,base['m_y']*1.3),'mmg.J_z':max(3.,base['J_z']*1.3),'actuator_tau':.15,'actuator_scale':.8,'force_limit':50.}
    sweep.append({'plant':loaded,'metrics':tracking_metrics(run('holdout_loaded',out/'velocity_reference.csv',best['parameters']|loaded))})
    # Excitation and independent validation for the same actual C++ MMG equations.
    for name,seed in (('identification_train',7),('identification_holdout',91)):
        write_reference(out/(name+'.csv'),force_profile(seed))
        run(name,out/(name+'.csv'),{},mode='force')
    fitted = fit_mmg(load_csv(out/'identification_train/samples.csv'),load_csv(out/'identification_holdout/samples.csv'),
                     base['mass_kg'],base['I_zG'],base['Lpp'],base['D'])
    # Pair order requested by the operator, with signed physical forces and gain sweeps.
    # RR, RL, FR, FL. Third positive pair moves aft, not forwards.
    trials = []
    balance_holdout = []
    pair_truth = {'actuator_forward':[.9,1.1,1.,.85],'actuator_reverse':[1.2,.9,.8,1.05]}
    plans = []
    for sign in (1.,-1.):
        for pair,(anchor,adjust) in enumerate(((2,3),(3,1),(1,0))):
            for gain in (.75,1.,1.25):
                forces = np.zeros(4); forces[anchor]=sign*8; forces[adjust]=sign*8*gain
                reference = out/f'pair_{sign}_{pair}_{gain}.csv'
                write_reference(reference,[(i*.02,*forces) for i in range(16)], fields=('time_s','command_rr','command_rl','command_fr','command_fl'))
                rows = run(reference.stem,reference,pair_truth,mode='thruster')
                for i in range(4):
                    command=[float(rows['command_'+a][i]) for a in IDS]
                    acceleration=float((rows['r'][i+1]-rows['r'][i])/.02)
                    (balance_holdout if gain==1. else trials).append((command,acceleration))
                plans.append({'physical_force_N':forces.tolist(),'pair':pair+1,'sign':sign,'gain':gain})
    trials += [([0.,0.,0.,0.],0.)]
    balance = fit_balance(trials,[True,False,False,True],balance_holdout)
    # Re-run independent pair levels through the actual allocator/MMG using fitted corrections.
    wire=[True,False,False,True]
    ef=balance['static_map.thrusters.forward_gain'];er=balance['static_map.thrusters.reverse_gain']
    corrected={'actuator_forward':[pair_truth['actuator_forward'][i]*(er[i] if wire[i] else ef[i]) for i in range(4)],
               'actuator_reverse':[pair_truth['actuator_reverse'][i]*(ef[i] if wire[i] else er[i]) for i in range(4)]}
    pair_validation=[]
    for sign in (1.,-1.):
        for pair,(anchor,adjust) in enumerate(((2,3),(3,1),(1,0))):
            forces=np.zeros(4);forces[anchor]=forces[adjust]=sign*8
            reference=out/f'balanced_pair_{sign}_{pair}.csv'
            write_reference(reference,[(i*.02,*forces) for i in range(16)],fields=('time_s','command_rr','command_rl','command_fr','command_fl'))
            rows=run(reference.stem,reference,corrected,mode='thruster')
            first=rows[0]
            force=float(np.hypot(first['tau_x'],first['tau_y']))
            cross=abs(float(first['tau_x'] if pair==1 else first['tau_y']))/force
            moment=abs(float(first['tau_n']))/force
            pair_validation.append({'pair':pair+1,'sign':sign,'cross_force_fraction':cross,'yaw_lever_residual_m':moment,
                                    'passed':cross<.01 and moment<.005})
    noisy=load_csv(out/'identification_train/samples.csv').copy()
    noise_rng=np.random.default_rng(81)
    for name,sigma in (('u',.002),('v',.002),('r',.001)):
        noisy[name]+=noise_rng.normal(0,sigma,len(noisy))
    noisy_fit=fit_mmg(noisy,load_csv(out/'identification_holdout/samples.csv'),base['mass_kg'],base['I_zG'],base['Lpp'],base['D'])
    report = {'baseline':baseline,'candidates':candidates,'selected':best,'holdout_sweep':sweep,
              'identification':fitted,'pair_balance_synthetic':balance,'pair_truth':pair_truth,'pair_plan':plans,'pair_validation':pair_validation,'noisy_identification':noisy_fit,
              'passed':bool(eligible and all(s['metrics']['passed'] for s in sweep) and fitted['passed'] and balance['passed'] and noisy_fit['passed'] and all(p['passed'] for p in pair_validation)),
              'benchmark_sha256':hashlib.sha256(executable.read_bytes()).hexdigest(),
              'commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              'scope':'Synthetic calibration, not real-vessel parameter identification or mission acceptance'}
    report['source_sha256']={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (
        Path(__file__),Path('src/sim/njord_sim/njord_sim/calibration.py'),
        Path('src/driver/micon/thruster_driver/include/thruster_driver/velocity_control.hpp'),
        Path('src/sim/dutyed_tf_pub_with_disturbance/src/velocity_benchmark.cpp'),
        Path('src/sim/dutyed_tf_pub_with_disturbance/src/mmg_doyle_model.cpp'))}
    (out/'report.json').write_text(json.dumps(report,indent=2))
    (out/'identified_mmg.yaml').write_text(yaml.safe_dump({'dutyed_tf_pub_with_disturbance_node':{'ros__parameters':{'mmg':fitted['mmg']}}}))
    if balance['passed']:
        (out/'thruster_balance.yaml').write_text(yaml.safe_dump({'thruster_driver_node':{'ros__parameters':{key:balance[key] for key in ('static_map.thrusters.forward_gain','static_map.thrusters.reverse_gain')}}}))
    if report['passed']:
        (out/'velocity_control.yaml').write_text(yaml.safe_dump({'thruster_driver_node':{'ros__parameters':best['parameters']}}))
    if args.plot:
        from njord_sim.calibration import plot_tracking
        plot_tracking(load_csv(out/'baseline/samples.csv'),load_csv(out/f'tune_{candidates.index(best)}_0/samples.csv'),out/'velocity_tracking.png')
    print(json.dumps({'passed':report['passed'],'baseline':baseline,'selected':best['parameters'],
                      'holdouts_passed':sum(s['metrics']['passed'] for s in sweep),'identification':fitted},indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
