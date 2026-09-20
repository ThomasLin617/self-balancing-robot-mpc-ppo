import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scenarios(config):
    jobs = []
    def add(group, name, methods, goal, force=0.0, duration=None, population='fixed'):
        for method in methods:
            jobs.append(dict(group=group, name=name, method=method, goal=goal,
                             force=force, duration=duration or config['timeout'], population=population))
    for i, ref in enumerate(config['sagittal']['references']):
        add('sagittal', f'step_{i}', ['lqr', 'mpc'], [ref, 0], duration=config['sagittal']['duration'])
    add('sagittal', 'push', ['lqr', 'mpc'], [0.1, 0], config['disturbance']['force'], config['sagittal']['duration'])
    rng = np.random.default_rng(config['goal_seed'])
    for i in range(config['goal_count']):
        radius = rng.uniform(*config['goal_radius'])
        bearing = rng.uniform(*config['goal_bearing'])
        add('goal_reaching', f'random_{i:03}', ['mpc', 'ppo'],
            [radius * math.cos(bearing), radius * math.sin(bearing)], population='random')
    for i, goal in enumerate(config['representative_goals']):
        add('goal_reaching', f'direction_{i}', ['mpc', 'ppo'], goal)
        add('disturbance', f'clean_{i}', ['mpc', 'ppo'], goal)
        add('disturbance', f'push_{i}', ['mpc', 'ppo'], goal, config['disturbance']['force'])
    duration = (config['timing']['warmup'] + config['timing']['samples']) * config['physics_dt'] * config['control_steps']
    add('compute', 'isolated', ['lqr', 'mpc', 'ppo'], [0.1, 0.0], duration=duration)
    return jobs


def execute(job, config, output):
    directory = output / job['group']
    directory.mkdir(exist_ok=True)
    name = job['method'] + '_' + job['name']
    path = directory / name
    s, d, tr = config['success'], config['disturbance'], config['trajectory']
    group = 'sagittal' if job['group'] == 'sagittal' else ('compute' if job['group'] == 'compute' else job['group'])
    values = [job['method'], group, *job['goal'], job['duration'], tr['max_velocity'], tr['acceleration'],
              job['force'], d['time'], d['duration'], s['distance'], s['velocity'], s['yaw_rate'],
              s['pitch'], s['hold'], config['fall_pitch'], config['physics_dt'], config['control_steps']]
    path.with_suffix('.txt').write_text(' '.join(map(str, values)) + '\n')
    start = time.perf_counter()
    command = [str(ROOT/'ros2_ws/build/mujoco_backend/controller_benchmark'), str(output/'nominal.xml'),
               str(ROOT/config['actor']), str(path.with_suffix('.txt')), str(path.with_suffix('.csv'))]
    result = subprocess.run(command, capture_output=True, text=True)
    metadata = {**job, 'exit_code': result.returncode, 'wall_seconds': time.perf_counter()-start,
                'stderr': result.stderr, 'stdout': result.stdout}
    if path.with_suffix('.csv').exists():
        with path.with_suffix('.csv').open('rb') as source, gzip.open(str(path)+'.csv.gz', 'wb') as target:
            shutil.copyfileobj(source, target)
        path.with_suffix('.csv').unlink()
    path.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='tools/benchmark/config.json')
    parser.add_argument('--output', default='artifacts/benchmark/d16')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    for package in ['controller', 'learning_policy', 'mujoco_backend']:
        cache = (ROOT/'ros2_ws/build'/package/'CMakeCache.txt').read_text()
        if 'CMAKE_BUILD_TYPE:STRING=Release' not in cache:
            raise RuntimeError('Timing requires Release builds for every controller.')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output/'config.json').write_text(json.dumps(config, indent=2)+'\n')
    frozen = {str(p.relative_to(ROOT)): digest(p) for directory in ['ros2_ws/src', 'model', 'tools/learning', 'tools/benchmark']
              for p in (ROOT/directory).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    frozen[config['actor']] = digest(ROOT/config['actor'])
    (output/'hashes.json').write_text(json.dumps(frozen, indent=2)+'\n')
    machine = {'platform': platform.platform(), 'cpu': subprocess.check_output(['sysctl','-n','machdep.cpu.brand_string'],text=True).strip(),
               'compiler': subprocess.check_output(['c++','--version'],text=True), 'build_type': 'Release',
               'pixi_lock_sha256': digest(ROOT/'pixi.lock')}
    (output/'machine.json').write_text(json.dumps(machine,indent=2)+'\n')
    xml = (ROOT/config['model']).read_text()
    if xml.count('euler="0 -20 0"') != 1: raise ValueError('Unexpected initial-pose XML.')
    (output/'nominal.xml').write_text(xml.replace('euler="0 -20 0"', 'euler="0 0 0"'))
    jobs = scenarios(config)
    (output/'scenarios.json').write_text(json.dumps(jobs,indent=2)+'\n')
    functional = [j for j in jobs if j['group'] != 'compute']
    with ThreadPoolExecutor(max_workers=config['workers']) as pool:
        for i, result in enumerate(pool.map(lambda j: execute(j, config, output), functional)):
            print(f"{i+1}/{len(functional)} {result['method']} {result['name']} exit={result['exit_code']}", flush=True)
    # Isolate computation trials from the parallel functional jobs.
    for job in jobs:
        if job['group'] == 'compute':
            print(execute(job, config, output), flush=True)
    changed = [name for name, value in frozen.items() if digest(ROOT/name) != value]
    (output/'freeze_check.json').write_text(json.dumps({'changed': changed},indent=2)+'\n')
    if changed: raise RuntimeError('Frozen inputs changed during benchmark.')


if __name__ == '__main__':
    main()
