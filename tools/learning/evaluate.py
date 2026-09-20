import argparse
import json
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from .environment import GoalEnvironment
from .state import load_config


def evaluate(policy, config, stage, seed, episodes, goals=None, pitch=None):
    env = GoalEnvironment(stage, config)
    rows = []
    for index in range(episodes):
        options = {}
        if goals is not None:
            options['goal'] = goals[index]
        if pitch is not None:
            options['pitch'] = pitch
        obs, _ = env.reset(seed=seed + index, options=options)
        while True:
            action, _ = policy.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                rows.append(info['metrics'])
                break
    summary = {}
    for name in rows[0]:
        if name == 'goal':
            continue
        values = [row[name] for row in rows if row[name] is not None]
        summary[name] = float(np.mean(values)) if values else None
    summary['worst_pitch'] = max(row['maximum_pitch'] for row in rows)
    summary['worst_torque'] = max(row['maximum_torque'] for row in rows)
    summary['episodes'] = episodes
    summary['seed'] = seed
    return {'summary': summary, 'episodes': rows}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('checkpoint')
    parser.add_argument('--config')
    parser.add_argument('--stage', default='A')
    parser.add_argument('--split', default='validation', choices=['validation', 'audit', 'test', 'representative', 'challenge'])
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    config = load_config(args.config or Path(args.checkpoint).parent/'config.json')
    if args.stage not in config['stages']:
        raise ValueError('Unknown curriculum stage.')
    settings = config['training']
    goals = None
    pitch = None
    if args.split in ['representative', 'challenge']:
        goals = config['evaluation']['representative_goals']
        seed, episodes = config['evaluation']['representative_seed'], len(goals)
        if args.split == 'challenge':
            pitch = config['evaluation']['challenge_pitch']
    else:
        prefix = {'validation': 'validation', 'audit': 'audit', 'test': 'final_test'}[args.split]
        seed, episodes = settings[prefix+'_seed'], settings[prefix+'_episodes']
    result = evaluate(PPO.load(args.checkpoint, device='cpu'), config, args.stage,
                      seed, episodes, goals, pitch)
    with open(args.output, 'w') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result['summary'], indent=2))


if __name__ == '__main__':
    main()
