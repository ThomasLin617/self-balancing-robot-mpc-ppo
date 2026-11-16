import json
from pathlib import Path
import subprocess
import sys


def main():
    root = Path('artifacts/benchmark/d18_full_circle_v1')
    subprocess.run([sys.executable, '-m', 'tools.trajectory_learning.benchmark_report',
                    '--root', str(root)], check=True)
    summary = json.loads((root/'summary.json').read_text())
    lines = ['# Goal Bearing Results', '',
             '| Absolute relative bearing | MPC success | PPO success |',
             '|---|---:|---:|']
    labels = {'front': '0°–45°', 'front_side': '45°–90°',
              'rear_side': '90°–135°', 'rear': '135°–180°'}
    for key, label in labels.items():
        values = [summary['methods'][method]['bearing_bins'][key] for method in ['mpc', 'ppo']]
        lines.append('| '+label+' | '+' | '.join(
            f"{value['success']}/{value['count']}" for value in values)+' |')
    (root/'bearing_summary.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
