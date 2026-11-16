import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO
from .state import SCALES,observation
from tools.learning.state import EstimatedState,load_config
from _trajectory_policy import TrajectoryPolicy


def main():
    parser=argparse.ArgumentParser();parser.add_argument('checkpoint',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    result=json.loads((args.checkpoint.parent/'result.json').read_text())
    if result['stage']!='E' or not result['validation_passed']:raise ValueError('Select a validated Stage E actor.')
    model=PPO.load(args.checkpoint,device='cpu')
    layers=[layer for layer in model.policy.mlp_extractor.policy_net if hasattr(layer,'weight')]+[model.policy.action_net]
    if [tuple(layer.weight.shape) for layer in layers]!=[(128,1256),(128,128),(2,128)]:raise ValueError('Unexpected actor architecture.')
    actor=args.output/'actor.txt'
    with actor.open('w') as f:
        f.write('d18_mlp_v1\n1256 128 128\n'+' '.join(map(str,SCALES))+'\n5 2\n')
        for layer in layers:
            np.savetxt(f,layer.weight.detach().numpy(),fmt='%.17g')
            np.savetxt(f,layer.bias.detach().numpy()[None,:],fmt='%.17g')
    policy=TrajectoryPolicy(str(actor));rng=np.random.default_rng(618000)
    pre_error=action_error=0.0;cases=[]
    for i in range(100):
        state=EstimatedState(load_config())
        state.position=float(rng.uniform(-10,10));state.velocity=float(rng.uniform(-.5,.5))
        state.pitch=float(rng.uniform(-.2,.2));state.pitch_rate=float(rng.uniform(-1,1))
        state.yaw=float(rng.choice([np.pi-1e-8,-np.pi+1e-8]) if i<10 else rng.uniform(-np.pi,np.pi));state.yaw_rate=float(rng.uniform(-1,1))
        refs=np.zeros((250,5));refs[:,0]=state.position+np.linspace(0,rng.uniform(-.5,.5),250)
        refs[:,1]=rng.uniform(-.2,.2);refs[:,2]=rng.uniform(-.1,.1)
        refs[:,3]=(state.yaw+np.linspace(0,rng.uniform(-1,1),250)+np.pi)%(2*np.pi)-np.pi;refs[:,4]=rng.uniform(-.5,.5)
        values=[state.position,state.velocity,state.pitch,state.pitch_rate,state.yaw,state.yaw_rate]
        obs=observation(state,refs);cpp_obs=np.array(policy.observation(values,refs))
        expected,_=model.predict(obs,deterministic=True);actual=np.array(policy.normalized_action(cpp_obs.tolist()))
        pe=float(np.max(np.abs(obs-cpp_obs)));ae=float(np.max(np.abs(expected-actual)))
        pre_error=max(pre_error,pe);action_error=max(action_error,ae)
        cases.append({'case':i,'preprocessing_error':pe,'action_error':ae})
    if pre_error>1e-6 or action_error>1e-5:raise RuntimeError('Python/C++ actor parity failed.')
    manifest={'checkpoint':str(args.checkpoint),'checkpoint_sha256':hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
              'actor_sha256':hashlib.sha256(actor.read_bytes()).hexdigest(),'contract':'d18_mlp_v1','input_dimension':1256,
              'cases':cases,'preprocessing_max_error':pre_error,'action_max_error':action_error}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Parity passed',pre_error,action_error)


if __name__=='__main__':main()
