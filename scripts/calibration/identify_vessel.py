#!/usr/bin/env python3
"""Fit recorded, independently excited training/holdout runs; never actuates."""
import argparse
import json
import hashlib
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from njord_sim.calibration import IDS, fit_balance, fit_mmg, load_csv


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--training',required=True)
    p.add_argument('--holdout')
    p.add_argument('--mass',type=float)
    p.add_argument('--iz',type=float)
    p.add_argument('--length',type=float,default=1.)
    p.add_argument('--draft',type=float,default=.25)
    p.add_argument('--balance',action='store_true')
    p.add_argument('--output',required=True)
    p.add_argument('--driver-config',default=str(Path(get_package_share_directory('thruster_driver'))/'config/config.yaml'))
    a=p.parse_args()
    root=Path(a.output)
    if root.exists() and any(root.iterdir()):p.error('choose an empty output directory')
    root.mkdir(parents=True,exist_ok=True)
    if a.holdout and Path(a.training).read_bytes()==Path(a.holdout).read_bytes():p.error("training and holdout must be independent experiments")
    rows=load_csv(a.training)
    if a.balance:
        if not a.holdout:p.error('balance fit also requires an independent --holdout')
        def pulse_trials(data):
            result=[]
            for i in range(len(data)-1):
                if max(abs(float(data[k][i])) for k in ('u','v','r'))>.06:continue
                dt=data['time_s'][i+1]-data['time_s'][i]
                if not .005<=dt<=.1:continue
                result.append(([float(data['command_'+name][i]) for name in IDS],float((data['r'][i+1]-data['r'][i])/dt)))
            return result
        base=yaml.safe_load(Path(a.driver_config).read_text())['thruster_driver_node']['ros__parameters']
        def flatten(values,prefix=''):
            out={}
            for key,value in values.items():
                name=prefix+key
                if isinstance(value,dict):out.update(flatten(value,name+'.'))
                else:out[name]=value
            return out
        flat=flatten(base)
        if [i.lower() for i in flat['thrusters.ids']]!=list(IDS):p.error('expected RR,RL,FR,FL driver order')
        from njord_sim.calibration import ANGLES
        if not np.allclose(flat['thrusters.angle_rad'],ANGLES,atol=1e-5):p.error('different geometry needs a separate balance model')
        result=fit_balance(pulse_trials(rows),flat['thrusters.reverse'],pulse_trials(load_csv(a.holdout)))
        overlay={key:result[key] for key in ('static_map.thrusters.forward_gain','static_map.thrusters.reverse_gain')}
        document={'thruster_driver_node':{'ros__parameters':flat|overlay}}
    else:
        if not a.holdout or a.mass is None or a.iz is None:p.error('MMG fit needs --holdout, --mass and --iz')
        result=fit_mmg(rows,load_csv(a.holdout),a.mass,a.iz,a.length,a.draft)
        document={'dutyed_tf_pub_with_disturbance_node':{'ros__parameters':{'mmg':result['mmg']}}}
    result['data_sha256']={str(Path(path).resolve()):hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in (a.training,a.holdout) if path}
    (root/'report.json').write_text(json.dumps(result,indent=2))
    if not result.get('passed',True):raise SystemExit('Fit failed validation; no deployment overlay emitted')
    (root/'candidate.yaml').write_text(yaml.safe_dump(document))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
