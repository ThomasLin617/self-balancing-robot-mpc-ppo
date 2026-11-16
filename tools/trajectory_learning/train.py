import argparse
import hashlib
import importlib.metadata
import json
import shutil
import time
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv,VecCheckNan
from stable_baselines3.common.logger import configure
from .environment import TrajectoryEnvironment
from tools.learning.state import ROOT


def environment(config):
    if config.get('curriculum',{}).get('full_circle',False):
        from .full_circle_training import FullCircleTrainingEnvironment
        return FullCircleTrainingEnvironment(config['curriculum'])
    return TrajectoryEnvironment(config['stage'],config.get('curriculum'))


def evaluate(model,config):
    env=environment(config);rows=[]
    for index in range(config['validation_episodes']):
        obs,_=env.reset(seed=config['validation_seed']+index)
        while True:
            action,_=model.predict(obs,deterministic=True)
            obs,_,terminated,truncated,info=env.step(action)
            if terminated or truncated:
                rows.append(info['metrics']);break
    env.close()
    result={'episodes':rows,'summary':{k:float(np.mean([r[k] for r in rows])) for k in rows[0]}}
    if config['stage'].startswith('C'):
        result['directions']={str(side):float(np.mean([r['success'] for r in rows if r['direction']==side])) for side in [-1,1]}
    return result


class Validation(BaseCallback):
    def __init__(self,config,directory):
        super().__init__();self.config=config;self.directory=directory
        self.next_check=config['evaluation_interval'];self.passed=False;self.best=(-1,-1,-float('inf'))
        self.started=time.perf_counter()

    def validate(self):
        result=evaluate(self.model,self.config);s=result['summary']
        result.update(steps=self.num_timesteps,wall_seconds=time.perf_counter()-self.started)
        with (self.directory/'validation.jsonl').open('a') as f:f.write(json.dumps(result,allow_nan=False)+'\n')
        score=(s['success'],-s['fall'],-s['final_distance'])
        if score>self.best:
            self.best=score;self.model.save(self.directory/'best_validation')
        self.model.save(self.directory/f'checkpoint_{self.num_timesteps}')
        self.passed=s['success']>=self.config['success_rate'] and s['fall']<=self.config['fall_rate']
        if self.config['stage'] in ['D','E']:
            self.passed=self.passed and s['collision']<=0.05 and s['boundary']==0
        if self.config['stage'].startswith('C'):
            self.passed=self.passed and min(result['directions'].values())>=self.config['success_rate']
        if self.passed:
            self.model.save(self.directory/'best_validation')
        print('VALIDATION',json.dumps({'steps':self.num_timesteps,**s}),flush=True)

    def _on_step(self):
        if self.num_timesteps>=self.next_check:
            self.next_check+=self.config['evaluation_interval'];self.validate()
        return not self.passed and self.num_timesteps<self.config['max_transitions']


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--resume')
    parser.add_argument('--config',default='tools/trajectory_learning/config.json')
    args=parser.parse_args();config=json.loads(Path(args.config).read_text())
    if config['stage']!='A':
        if not args.resume:raise ValueError('A validated preceding checkpoint is required.')
        previous=json.loads((Path(args.resume).parent/'result.json').read_text())
        continuing=config.get('continue_stage',False) and previous['stage']==config['stage']
        if not continuing and (not previous['validation_passed'] or previous['stage']!=config.get('previous_stage',{'B':'A','C':'B'}.get(config['stage']))):
            raise ValueError('Previous stage has not passed.')
    if config['max_transitions']>5000000:raise ValueError('Stage budget exceeds approved limit.')
    directory=Path(args.output);directory.mkdir(parents=True,exist_ok=False)
    (directory/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    versions={n:importlib.metadata.version(n) for n in ['numpy','torch','mujoco','gymnasium','stable-baselines3','pybind11']}
    (directory/'versions.json').write_text(json.dumps(versions,indent=2)+'\n')
    shutil.copytree(Path(__file__).parent,directory/'source',ignore=shutil.ignore_patterns('__pycache__'))
    (directory/'plant.sha256').write_text(hashlib.sha256((ROOT/'model/Robot1.xml').read_bytes()).hexdigest()+'\n')
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    env=VecCheckNan(DummyVecEnv([lambda:Monitor(environment(config)) for _ in range(config['environments'])]),raise_exception=True)
    env.seed(config['seed'])
    if args.resume:
        model=PPO.load(args.resume,env=env,device='cpu',seed=config['seed'],**config['ppo'])
        if 'exploration_std' in config:
            with torch.no_grad():
                model.policy.log_std.fill_(np.log(config['exploration_std']))
        (directory/'parent.json').write_text(json.dumps({'checkpoint':args.resume,
            'sha256':hashlib.sha256(Path(args.resume).read_bytes()).hexdigest()},indent=2)+'\n')
    else:
        model=PPO('MlpPolicy',env,**config['ppo'],seed=config['seed'],device='cpu',
                  policy_kwargs={'net_arch':dict(pi=config['network'],vf=config['network']),
                                 'activation_fn':torch.nn.Tanh,'log_std_init':-1.0})
    model.set_logger(configure(str(directory),['csv']))
    callback=Validation(config,directory);start=time.perf_counter()
    try:
        callback.init_callback(model)
        if args.resume:callback.validate()
        initial_pass=callback.passed
        if not initial_pass:
            model.learn(total_timesteps=config['max_transitions'],callback=callback)
        model.save(directory/'final')
        result=dict(stage=config['stage'],steps=0 if initial_pass else model.num_timesteps,validation_passed=callback.passed,
                    wall_seconds=time.perf_counter()-start)
        (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(result,flush=True)
    finally:env.close()


if __name__=='__main__':main()
