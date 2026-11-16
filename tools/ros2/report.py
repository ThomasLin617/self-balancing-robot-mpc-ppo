import hashlib
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path('artifacts/ros2/d17')


def read(name):
    return json.loads((ROOT / name / 'result.json').read_text())


def replace(path, marker, body):
    text = path.read_text()
    begin, end = f'<!-- {marker}_BEGIN -->', f'<!-- {marker}_END -->'
    if begin not in text:
        raise ValueError(f'Missing report markers: {path}')
    before, tail = text.split(begin, 1)
    _, after = tail.split(end, 1)
    path.write_text(before + begin + '\n\n' + body + '\n\n' + end + after)


def main():
    results = {name: read(name) for name in ['regulation', 'ppo', 'known_map', 'encoder_500hz', 'faults', 'command_interfaces', 'launch_bag_verified']}
    before = json.loads((ROOT / 'before_hashes.json').read_text())
    changed = [p for p, digest in before.items() if not Path(p).exists() or hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest]
    if changed:
        raise ValueError(f'Frozen sources changed: {changed}')
    results['freeze'] = {'files_checked': len(before), 'changed': changed, 'sha256': before}
    for mode in ['regulation', 'ppo', 'known_map']:
        trace = np.genfromtxt(ROOT / mode / 'ros.csv', delimiter=',', names=True)
        assert not np.any(trace['fault'])
        assert np.isfinite(trace.tolist()).all()
        assert np.abs(trace['applied_l']).max() <= 2 and np.abs(trace['applied_r']).max() <= 2
        if mode == 'known_map':
            results[mode]['post_recovery_max_pitch'] = float(np.abs(trace['pitch'][trace['time'] >= 4]).max())
            results[mode]['final_forward_velocity'] = float(trace['v'][-1])
            assert results[mode]['post_recovery_max_pitch'] < .2
            assert results[mode]['estimated_final_distance'] < .15 and results[mode]['true_final_distance'] < .15
            assert abs(trace['v'][-1]) < .04 and not np.any(trace['contact'])
            log = (ROOT / mode / 'mpc_node.log').read_text()
            results[mode]['trajectory_duration'] = float(re.search(r'duration ([0-9.]+)', log).group(1))
    goal_log = (ROOT / 'ppo/ppo_node.log').read_text()
    goals = re.findall(r'PPO goal (\d+) success=(\d+) time=([\d.]+) distance=([\d.]+) pitch=([-\d.]+)', goal_log)
    results['ppo']['goal_results'] = [dict(index=int(i), success=bool(int(s)), time=float(t), distance=float(d), pitch=float(p)) for i,s,t,d,p in goals]
    assert len(goals) == 4 and all(int(g[1]) == 1 for g in goals)
    figures = Path('docs/figures/ros2')
    figures.mkdir(parents=True, exist_ok=True)
    ros = np.genfromtxt(ROOT / 'regulation/ros.csv', delimiter=',', names=True)
    direct = np.genfromtxt(ROOT / 'regulation/direct.csv', delimiter=',', names=True)
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for ax, key, label in zip(axes, ['p', 'pitch', 'applied_l'], ['Forward position (m)', 'Estimated pitch (rad)', 'Left applied torque (N m)']):
        ax.plot(direct['time'], direct[key], label='Direct call', color='#355f8d', lw=2)
        ax.plot(ros['time'][::20], ros[key][::20], label='ROS: three processes', color='#d67c25', ls='--', lw=1)
        ax.set_ylabel(label); ax.grid(alpha=.2)
    axes[0].legend(frameon=False)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.suptitle('Identical regulation experiment: fixed sample alignment')
    fig.tight_layout(); fig.savefig(figures / 'direct_vs_ros.png', dpi=180); plt.close(fig)
    r, p, k = results['regulation'], results['ppo'], results['known_map']
    comparison = r['comparison']
    tables = ['| Distributed experiment | Measured result |', '|---|---|',
        f'| MPC regulation, {r["duration"]:g} s | {r["state_count"]} states, {r["command_count"]} commands; no fallback |',
        f'| Direct/ROS maximum position difference | {comparison["p"]["maximum_error"]:.3e} m |',
        f'| Direct/ROS maximum pitch difference | {comparison["pitch"]["maximum_error"]:.3e} rad |',
        f'| Direct/ROS maximum requested torque difference | {max(comparison[x]["maximum_error"] for x in ["left","right"]):.3e} N·m |',
        f'| PPO presentation goals | 4/4 settled; fourth success at {p["goal_results"][-1]["time"]:.3f} s |',
        f'| PPO peak estimated pitch / applied torque | {p["max_pitch"]:.6f} rad / {p["max_applied_torque"]:.6f} N·m |',
        f'| Known-map estimated / true final error at {k["duration"]:g} s | {k["estimated_final_distance"]:.6f} / {k["true_final_distance"]:.6f} m |',
        f'| Known-map trajectory duration / post-recovery peak pitch | {k["trajectory_duration"]:.6f} s / {k["post_recovery_max_pitch"]:.6f} rad |',
        f'| Known-map obstacle contact samples | {k["contact_samples"]} |',
        f'| 1000 Hz IMU / 500 Hz encoder trial, 2 s | {results["encoder_500hz"]["imu_count"]} IMU, {results["encoder_500hz"]["encoder_count"]} encoder, {results["encoder_500hz"]["command_count"]} commands |',
        f'| ROS PPO torque parity, {results["faults"]["ros_policy_parity_cases"]} vectors | max error {results["faults"]["ros_policy_torque_maximum_error"]:.3e} N·m |',
        f'| Frozen file comparison | {len(before)} files checked; zero changes |']
    summary = '\n'.join(tables)
    timing = ['| Backend-observed wall pipeline interval | Mean (µs) | p95 (µs) | Maximum (µs) |', '|---|---:|---:|---:|']
    for key, label in [('state_wall_us', 'Sensor publication → state receipt'), ('command_wall_us', 'Sensor publication → command receipt (control steps)'), ('total_wall_us', 'Sensor publication → application (all physics steps)')]:
        values = r['timing'][key]
        timing.append(f'| {label} | {values["mean"]:.1f} | {values["p95"]:.1f} | {values["maximum"]:.1f} |')
    detail = summary + '\n\n' + '\n'.join(timing)
    detail += '\n\nThe regulation run has zero simulated state age and 0–1 ms command age. Wall intervals include DDS delivery, callback scheduling and processing. Command receipt has one sample per 500 Hz update; application includes held-command steps at 1000 Hz. The measured intervals describe this lockstep execution.'
    results['status'] = 'Measured runs and frozen-source checks passed'
    encoded = json.dumps(results, indent=2, allow_nan=False) + '\n'
    (ROOT / 'summary.json').write_text(encoded)
    Path('docs/ros2_integration_results.json').write_text(encoded)
    replace(Path('README.md'), 'ROS_RESULTS', summary)
    replace(Path('docs/ROS2_INTEGRATION.md'), 'ROS_RESULTS', detail)
    print('Generated D17 tables, source-hash snapshot and direct/ROS plot.')


if __name__ == '__main__':
    main()
