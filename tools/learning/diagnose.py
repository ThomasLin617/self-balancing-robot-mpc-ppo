import argparse
import json

import numpy as np
from stable_baselines3 import PPO

from .environment import GoalEnvironment
from .state import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('checkpoint')
    parser.add_argument('--config', required=True)
    parser.add_argument('--stage', default='A')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    policy = PPO.load(args.checkpoint, device='cpu')
    env = GoalEnvironment(args.stage, config)
    results = []
    limits = config['success']
    for seed in range(config['training']['validation_seed'],
                      config['training']['validation_seed'] + config['training']['validation_episodes']):
        obs, _ = env.reset(seed=seed)
        counts = np.zeros(4)
        max_hold = steps = 0
        speeds, rates, pitch_errors = [], [], []
        reward_totals = {}
        while True:
            action, _ = policy.predict(obs, deterministic=True)
            obs, _, done, truncated, info = env.step(action)
            for name, value in info['reward_terms'].items():
                reward_totals[name] = reward_totals.get(name, 0.0) + float(value)
            distance = np.hypot(env.goal[0]-env.state.x, env.goal[1]-env.state.y)
            counts += [distance < limits['radius'], abs(env.state.velocity) < limits['velocity'],
                       abs(env.state.yaw_rate) < limits['yaw_rate'], abs(env.state.pitch) < limits['pitch']]
            max_hold = max(max_hold, env.hold_steps)
            speeds.append(abs(env.state.velocity))
            rates.append(abs(env.state.yaw_rate))
            pitch_errors.append(abs(env.state.pitch-env.truth()[1]))
            steps += 1
            if done or truncated:
                results.append({'seed': seed, 'condition_fractions': (counts/steps).tolist(),
                                'longest_settled_seconds': max_hold*env.dt,
                                'mean_absolute_velocity': float(np.mean(speeds)),
                                'mean_absolute_yaw_rate': float(np.mean(rates)),
                                'mean_absolute_pitch_estimation_error': float(np.mean(pitch_errors)),
                                'reward_totals': reward_totals,
                                'metrics': info['metrics']})
                break
    summary = {key: np.mean([row[key] for row in results], axis=0).tolist()
               for key in ['condition_fractions', 'longest_settled_seconds', 'mean_absolute_velocity',
                           'mean_absolute_yaw_rate', 'mean_absolute_pitch_estimation_error']}
    with open(args.output, 'w') as stream:
        json.dump({'summary': summary, 'episodes': results}, stream, indent=2, allow_nan=False)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
