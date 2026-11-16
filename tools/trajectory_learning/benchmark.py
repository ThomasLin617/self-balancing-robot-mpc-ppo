import argparse
import csv
import gzip
import hashlib
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from .environment import TrajectoryEnvironment
from .state import ROOT
from _controllers import MPC


def episode(job):
    case,directory,scenario_root,method,actor=job
    key=case.get("experiment",f"{method}_{case['family']}_{case['index']:02}")
    if "experiment" in case:key=method+"_"+key
    scenario=str((scenario_root/case['scenario']).resolve())
    if 'world_initial_yaw' in case:
        from .full_circle import FullCircleEnvironment
        env=FullCircleEnvironment(scenario,case['world_initial_yaw'])
    else:
        env=TrajectoryEnvironment('D',{'scenario':scenario})
    env.stage['velocity_range']=0
    env.reset(seed=518000,options={'pitch':case.get('initial_pitch',0.0)})
    values=np.array([env.trajectory.sample(float(t)) for t in np.arange(0,env.trajectory.duration+0.002,0.002)],dtype='<f8')
    if hashlib.sha256(values.tobytes()).hexdigest()!=case['sampled_reference_sha256']:
        raise ValueError('Paired reference hash mismatch: '+key)
    if method=='mpc':controller=MPC()
    else:
        from _trajectory_policy import TrajectoryPolicy
        controller=TrajectoryPolicy(str(actor))
    rows=[];timings=[];path_length=absolute=squared=saturation=0.0
    previous,_=env.truth();pitch_integral=0.0
    while True:
        state=env.state
        physical=[state.position,state.velocity,state.pitch,state.pitch_rate,state.yaw,state.yaw_rate]
        begin=time.perf_counter_ns()
        references=env.preview()
        result=controller.compute(physical,references)
        full_us=(time.perf_counter_ns()-begin)/1000
        torque=np.array(result[:2]);timings.append([result[2],full_us])
        if 'push' in case:
            push=case['push'];body=env.model.body('base_body').id
            env.data.xfrc_applied[body,:]=0
            if push['time']<=env.steps*.002<push['time']+push['duration']:
                env.data.xfrc_applied[body,:3]=env.data.xmat[body].reshape(3,3)@np.asarray(push['local_force'])
        _,_,terminated,truncated,info=env.step(torque/2)
        position,pitch=env.truth();point=env.trajectory.sample(env.steps*.002)
        path_length+=float(np.linalg.norm(position-previous));previous=position
        absolute+=float(np.abs(torque).sum())*.002;squared+=float(torque@torque)*.002
        saturation+=.002*(np.max(np.abs(torque))>=1.999999)
        pitch_integral+=pitch*pitch*.002
        state=env.state
        rows.append([env.steps*.002,state.position,state.velocity,state.pitch,state.pitch_rate,state.yaw,state.yaw_rate,
                     state.x,state.y,*position,pitch,*torque,*point])
        if terminated or truncated:break
    metrics=info['metrics'];elapsed=env.steps*.002
    if 'experiment' in case:metrics['experiment']=case['experiment'];metrics['initial_pitch']=case.get('initial_pitch',0.0);metrics['push']=case.get('push')
    if 'world_initial_yaw' in case:
        metrics.update(bearing=case['bearing'],world_initial_yaw=case['world_initial_yaw'],world_goal=case['world_goal'])
    metrics.update(method=method,family=case['family'],index=case['index'],goal=case['goal'],
                   reference_sha256=case['sampled_reference_sha256'],trajectory_duration=env.trajectory.duration,
                   path_length=path_length,planned_length=case['length'],path_ratio=case['length']/path_length if path_length>0 else None,
                   pitch_rms=float(np.sqrt(pitch_integral/elapsed)),absolute_torque=absolute,squared_torque=squared,
                   saturation_duration=saturation,compute_samples=len(timings))
    with gzip.open(directory/f'{key}.csv.gz','wt') as stream:
        writer=csv.writer(stream);writer.writerow(['time','p','v','pitch','pitch_rate','yaw','yaw_rate','estimated_x','estimated_y',
            'true_x','true_y','true_pitch','torque_left','torque_right','reference_x','reference_y','reference_p','reference_v','reference_pitch','reference_yaw','reference_yaw_rate']);writer.writerows(rows)
    np.save(directory/f'{key}.timing.npy',np.asarray(timings))
    (directory/f'{key}.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    env.close()
    return {'case':key,'success':metrics['success'],'fall':metrics['fall'],'collision':metrics['collision'],'timeout':metrics['timeout']}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scenarios',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--method',choices=['mpc','ppo'],required=True)
    parser.add_argument('--actor',type=Path);parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((args.scenarios/'manifest.json').read_text())
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    jobs=[(c,args.output,args.scenarios,args.method,args.actor) for c in manifest['cases']]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for result in pool.map(episode,jobs):print(json.dumps(result),flush=True)


if __name__=='__main__':main()
