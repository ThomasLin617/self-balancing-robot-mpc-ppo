import argparse
import hashlib
import importlib.metadata
import json
import shutil
import sys
from pathlib import Path
import time

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecCheckNan

from .environment import GoalEnvironment
from .evaluate import evaluate
from .state import ROOT, load_config


class Validation(BaseCallback):
    def __init__(self, config, stage, directory):
        super().__init__()
        self.config, self.stage, self.directory = config, stage, directory
        self.interval = config['training']['evaluation_interval']
        self.next_evaluation = self.interval
        self.best_score = (-1.0, -float('inf'), -float('inf'))
        self.started = time.perf_counter()
        self.passed = False

    def validate(self):
        settings = self.config['training']
        result = evaluate(self.model, self.config, self.stage, settings['validation_seed'], settings['validation_episodes'])
        summary = result['summary']
        result['steps'] = self.num_timesteps
        result['wall_seconds'] = time.perf_counter() - self.started
        with open(self.directory/'validation.jsonl', 'a') as stream:
            stream.write(json.dumps(result, allow_nan=False)+'\n')
        self.model.save(self.directory/f'checkpoint_{self.num_timesteps}')
        score = (summary['success'], -summary['fall'], -summary['final_distance'])
        if score > self.best_score:
            self.best_score = score
            self.model.save(self.directory/'best_validation')
        passed_now = (summary['success'] >= settings['progression_success_rate']
                      and summary['fall'] <= settings['progression_fall_rate'])
        self.passed = self.passed or passed_now
        print('VALIDATION', json.dumps({'steps': self.num_timesteps, **summary}), flush=True)
        return self.passed

    def _on_step(self):
        if (self.directory/'stop_requested').exists():
            return False
        if not np.isfinite(self.locals['rewards']).all():
            raise RuntimeError('Non-finite PPO rollout reward.')
        if self.num_timesteps >= self.next_evaluation:
            self.next_evaluation += self.interval
            if self.validate():
                return False
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', default='A')
    parser.add_argument('--config')
    parser.add_argument('--steps', type=int)
    parser.add_argument('--resume')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    if args.stage not in config['stages']:
        raise ValueError('Unknown curriculum stage.')
    settings = config['training']
    if args.stage != 'A':
        if not args.resume:
            raise ValueError('Later stages require a validated previous-stage checkpoint.')
        previous = json.loads((Path(args.resume).parent/'result.json').read_text())
        allowed = {'B0': ['A'], 'B1': ['B0'], 'B2a': ['B1'], 'B2b': ['B2a'], 'B2': ['B1', 'B2b'], 'B3': ['B2'], 'B': ['A', 'B1', 'B2', 'B2b', 'B3'], 'C0': ['B'], 'C05': ['C0'], 'Cside': ['A', 'C0'], 'Cturn0': ['C05', 'Cside'], 'Cturn': ['B', 'C0', 'Cturn0', 'Cside'], 'C1': ['Cturn', 'Cturn0', 'C05', 'C0', 'Cside'], 'C': ['B', 'C1'], 'D': ['C']}
        allowed.update({'D0': ['C'], 'D1': ['D0'], 'D': ['C', 'D1']})
        if previous['stage'] != args.stage and (not previous['validation_passed'] or previous['stage'] not in allowed[args.stage]):
            raise ValueError('Previous curriculum stage has not passed validation.')
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=False)
    (directory/'invocation.json').write_text(json.dumps(vars(args), indent=2)+'\n')
    (directory/'config.json').write_text(json.dumps(config, indent=2)+'\n')
    (directory/'plant.sha256').write_text(hashlib.sha256((ROOT/config['model']).read_bytes()).hexdigest()+'\n')
    versions = {name: importlib.metadata.version(name)
                for name in ['numpy', 'torch', 'mujoco', 'gymnasium', 'stable-baselines3']}
    versions['python'] = sys.version
    (directory/'versions.json').write_text(json.dumps(versions, indent=2)+'\n')
    source_directory = directory/'source'
    source_directory.mkdir()
    for source in Path(__file__).parent.glob('*.py'):
        shutil.copy2(source, source_directory/source.name)
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    env = VecCheckNan(DummyVecEnv([lambda: Monitor(GoalEnvironment(args.stage, config))
                                 for _ in range(settings['environments'])]), raise_exception=True)
    env.seed(settings['seed'])
    if args.resume:
        model = PPO.load(args.resume, env=env, device='cpu', **config['ppo'])
        restart_std = config['network'].get('resume_action_std')
        if restart_std is not None:
            if not np.isfinite(restart_std) or restart_std <= 0.0:
                raise ValueError('Invalid exploration restart standard deviation.')
            with torch.no_grad():
                model.policy.log_std.clamp_(min=float(np.log(restart_std)))
            model.policy.optimizer.state.pop(model.policy.log_std, None)
    else:
        model = PPO('MlpPolicy', env, **config['ppo'], seed=settings['seed'], device='cpu',
                    policy_kwargs={'net_arch': dict(pi=config['network']['hidden_sizes'], vf=config['network']['hidden_sizes']),
                                   'activation_fn': torch.nn.Tanh, 'log_std_init': config['network']['log_std_init']})
    model.set_logger(configure(str(directory), ['csv']))
    callback = Validation(config, args.stage, directory)
    started = time.perf_counter()
    try:
        callback.init_callback(model)
        initial_pass = callback.validate() if args.resume else False
        if not initial_pass:
            model.learn(total_timesteps=args.steps or settings['stage_steps'], callback=callback)
        model.save(directory/'final')
        if not callback.passed:
            callback.validate()
        result = {'stage': args.stage, 'steps': 0 if initial_pass else model.num_timesteps,
                  'wall_seconds': time.perf_counter()-started, 'validation_passed': callback.passed}
        (directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
    finally:
        env.close()


if __name__ == '__main__':
    main()
