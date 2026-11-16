import argparse
import json
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    stages = [('a','config.json'), ('b','config_b.json'), ('c0','config_c0.json'),
              ('c1','config_c1.json'), ('c2','config_c2_profiled.json'),
              ('c3','config_c3_slow.json'), ('d','config_d.json'), ('e','config_e.json')]
    checkpoint = None
    for stage, config in stages:
        directory = args.output/stage
        command = [sys.executable, '-m', 'tools.trajectory_learning.train',
                   '--config', str(Path(__file__).parent/config), '--output', str(directory)]
        if checkpoint is not None:
            command += ['--resume', str(checkpoint)]
        subprocess.run(command, check=True)
        result = json.loads((directory/'result.json').read_text())
        if not result['validation_passed']:
            raise RuntimeError(f'Stage {stage} did not pass; inspect saved validation before continuing')
        checkpoint = directory/'best_validation.zip'
    print('Validated checkpoint', checkpoint)


if __name__ == '__main__':
    main()
