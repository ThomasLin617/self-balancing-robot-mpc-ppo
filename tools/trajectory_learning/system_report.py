import hashlib
from importlib.metadata import version
import json
import platform
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from .benchmark_report import distribution


def replace_section(path, name, content):
    text = path.read_text()
    begin, end = f'<!-- {name}_BEGIN -->', f'<!-- {name}_END -->'
    if begin not in text:
        return
    path.write_text(text.split(begin)[0]+begin+'\n\n'+content+'\n\n'+end+text.split(end)[1])


def main():
    root = Path('artifacts/benchmark/d18')
    trace = np.genfromtxt(root/'compute.csv', delimiter=',', names=True)
    assert len(trace) == 5000
    timing = {}
    for key in trace.dtype.names[1:]:
        timing[key] = distribution(trace[key])
        timing[key]['p99'] = float(np.percentile(trace[key], 99))
        timing[key]['above_2000us'] = int(np.sum(trace[key]>2000))
    machine = {'platform': platform.platform(), 'cpu': subprocess.check_output(['sysctl','-n','machdep.cpu.brand_string'], text=True).strip(),
               'compiler': subprocess.check_output(['clang','--version'], text=True).splitlines()[0],
               'build_type': 'Release', 'clock': 'std::chrono::steady_clock', 'warmup_updates': 500, 'samples': 5000}
    machine['python_dependencies'] = {name: version(name) for name in ['numpy','mujoco','stable-baselines3','torch','pybind11']}
    cache = Path('build/trajectory_learning/CMakeCache.txt').read_text().splitlines()
    machine['cmake_compiler'] = [line for line in cache if line.startswith(('CMAKE_CXX_COMPILER:','CMAKE_BUILD_TYPE:'))]
    challenges = [json.loads(p.read_text()) for p in sorted((root/'challenges').glob('*.json')) if p.name!='config.json']
    assert len(challenges) == 24
    nominal = json.loads(Path('docs/d18_forward_only_results.json').read_text())['episodes']
    for row in challenges:
        baseline = next(r for r in nominal[row['method']] if r['family']==row['family'] and r['index']==row['index'])
        row['nominal_success'] = baseline['success']
        row['additional_path_length'] = row['path_length']-baseline['path_length']
        row['additional_squared_torque'] = row['squared_torque']-baseline['squared_torque']
        row['pulse_exposed'] = row.get('push') is not None and row['time']>row['push']['time']
    ros = json.loads(Path('artifacts/ros2/d18/trajectory_ppo/result.json').read_text())
    faults = json.loads(Path('artifacts/ros2/d18/faults/result.json').read_text())
    output = {'timing': timing, 'machine': machine, 'challenges': challenges, 'ros': ros, 'ros_faults': faults,
              'timing_sha256': hashlib.sha256((root/'compute.csv').read_bytes()).hexdigest()}
    Path('docs/d18_system_results.json').write_text(json.dumps(output, indent=2, allow_nan=False)+'\n')
    (root/'system_summary.json').write_text(json.dumps(output, indent=2, allow_nan=False)+'\n')
    names = {'actor_us':'PPO actor inference', 'ppo_update_us':'PPO full update',
             'mpc_compute_us':'MPC gradient + QP + extraction', 'mpc_update_us':'MPC full update'}
    lines = ['| Measured computation (µs) | Mean | Median | p95 | p99 | Maximum |', '|---|---:|---:|---:|---:|---:|']
    for key, values in timing.items():
        lines.append('| '+names[key]+' | '+' | '.join(f'{values[k]:.4g}' for k in ['mean','median','p95','p99','maximum'])+' |')
    lines += ['', 'Release, '+machine['cpu']+'. Each distribution has 5000 samples after 500 warm-up updates. Full updates include C++ horizon construction. Actor-only timing excludes preprocessing. MPC compute timing includes gradient update, OSQP solve and first-command extraction. Physics, ROS, Python and viewer work are outside these timers. This isolated synthetic-state workload provides platform-specific measurements.']
    table = '\n'.join(lines)
    (root/'compute.md').write_text(table+'\n')
    for document in [Path('README.md'), Path('docs/TRAJECTORY_POLICY.md')]:
        replace_section(document, 'D18_TIMING', table)
    lines = ['| Separate challenge | MPC successes | Trajectory PPO successes |', '|---|---:|---:|']
    for label, token in [('Forward body pulse','_forward'),('Lateral body pulse','_lateral')]:
        values=[]
        for method in ['mpc','ppo']:
            rows=[r for r in challenges if r['method']==method and r['experiment'].endswith(token)]
            exposed=sum(r['time']>=5.05 for r in rows)
            values.append(f"{sum(r['success'] for r in rows)}/{len(rows)} ({exposed} fully pulse-exposed)")
        lines.append('| '+label+' | '+' | '.join(values)+' |')
    for angle in [5,10,15,20]:
        values=[]
        for method in ['mpc','ppo']:
            row=next(r for r in challenges if r['method']==method and r['experiment']==f'pitch_{angle}')
            outcome='success' if row['success'] else 'fall' if row['fall'] else 'timeout' if row['timeout'] else 'failure'
            values.append(f"{outcome}, estimated final distance {row['final_distance']:.4f} m")
        lines.append(f'| Initial pitch −{angle}° (one fixed case each) | '+' | '.join(values)+' |')
    lines += ['', 'The body-frame pulse is −50 N for 0.05 s beginning at 5 s. These eight pushed cases and four initial-pitch cases per controller retain the earlier forward-facing reset. They are separate from the full-circle 100-case denominator; effort/path deltas use the historical forward-only nominal matches. All four lateral-pulse cases fell for each controller. The tilted-start failures above were timeouts with no falls. No weights, gains or success thresholds were adjusted after these evaluations.']
    replace_section(Path('docs/TRAJECTORY_POLICY.md'), 'D18_CHALLENGES', '\n'.join(lines))
    figure, axis=plt.subplots(figsize=(8,4), constrained_layout=True)
    axis.boxplot([trace[k] for k in names], tick_labels=['PPO actor','PPO full','MPC compute','MPC full'])
    axis.set_yscale('log');axis.set_ylabel('Measured update time (µs, log scale)')
    axis.axhline(2000, color='gray', linestyle='--', label='2 ms simulation period')
    axis.legend();axis.grid(axis='y',alpha=.2)
    figure.savefig('docs/figures/benchmark/d18_compute.png',dpi=180)
    plt.close(figure)
    print(json.dumps(timing,indent=2))


if __name__ == '__main__':
    main()
