import argparse
import csv
import json
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO
from .environment import TrajectoryEnvironment
from tools.learning.state import wrap


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    config=json.loads((args.run/'config.json').read_text())
    model=PPO.load(args.run/'best_validation.zip',device='cpu')
    summary=[]
    for index in range(4):
        seed=config['validation_seed']+index
        env=TrajectoryEnvironment(config['stage']);obs,_=env.reset(seed=seed)
        rows=[]
        while True:
            action,_=model.predict(obs,deterministic=True)
            obs,r,terminated,truncated,info=env.step(action)
            p=env.trajectory.sample(env.steps*.002);state=env.state
            position,pitch=env.truth()
            rows.append([env.steps*.002,state.position,state.velocity,state.pitch,pitch,state.yaw,
                         p[2],p[3],p[5],p[6],wrap(state.yaw-p[5]),*list(action*2),r])
            if terminated or truncated:break
        with (args.output/f'{seed}.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['time','position','velocity','pitch_est','pitch_true','yaw',
                'position_ref','velocity_ref','yaw_ref','yaw_rate_ref','yaw_error','torque_left','torque_right','reward'])
            writer.writerows(rows)
        summary.append({'seed':seed,**info['metrics'],
                        'yaw_tracking_rmse':float(np.sqrt(np.mean(np.square(np.asarray(rows)[:,10]))))})
        env.close()
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
