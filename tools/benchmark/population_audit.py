"""Publish episode summaries and verify the paired random-goal population."""
import argparse
import hashlib
import json
from pathlib import Path


def audit(episodes, expected):
    result = {}
    goals = {}
    for method in ['mpc', 'ppo']:
        rows = [r for r in episodes if r['group'] == 'goal_reaching'
                and r['population'] == 'random' and r['method'] == method]
        goals[method] = {r['name']: r['goal'] for r in rows}
        if len(rows) != expected or len(goals[method]) != expected:
            raise ValueError(f'{method}: missing or duplicate random-goal episodes')
        if any(sum(bool(r[k]) for k in ['success', 'fall', 'timeout']) != 1 for r in rows):
            raise ValueError(f'{method}: outcomes must be mutually exclusive and exhaustive')
        result[method] = {'denominator': len(rows), **{
            k: sum(bool(r[k]) for r in rows) for k in ['success', 'fall', 'timeout']}}
    if goals['mpc'] != goals['ppo']:
        raise ValueError('MPC and PPO do not share identical named goals')
    result['paired_goals_identical'] = True
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=Path('artifacts/benchmark/d16'))
    args = parser.parse_args()
    source = args.input/'episodes.json'
    config = json.loads((args.input/'config.json').read_text())
    episodes = json.loads(source.read_text())
    result = audit(episodes, config['goal_count'])
    destination = Path('docs/benchmark_episodes.json')
    destination.write_bytes(source.read_bytes())
    result.update({
        'source': str(destination),
        'source_sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
        'all_trial_count': len(episodes),
        'notes': [
            'Headline denominator excludes eight fixed directional checks per method.',
            'Completion-time statistics use successful episodes only; see success counts above.',
            'Disturbance pulse-exposed denominators are 5 MPC and 8 PPO, not 100.',
            'Historical D15 final 87/100 is a separate evaluation population.']})
    Path('docs/benchmark_population_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
