import argparse
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {'count': 0}
    return {
        'count': len(values), 'mean': float(values.mean()),
        'median': float(np.median(values)), 'std': float(values.std()),
        'p95': float(np.percentile(values, 95)), 'maximum': float(values.max()),
    }


def read_episode(path):
    result = json.loads(path.read_text())
    with gzip.open(path.with_suffix('.csv.gz'), 'rt') as stream:
        trace = np.genfromtxt(stream, delimiter=',', names=True)
    position_error = trace['p'] - trace['reference_p']
    yaw_error = (trace['yaw'] - trace['reference_yaw'] + np.pi) % (2*np.pi) - np.pi
    result['position_rmse'] = float(np.sqrt(np.mean(position_error**2)))
    result['yaw_rmse'] = float(np.sqrt(np.mean(yaw_error**2)))
    reference = np.column_stack((trace['reference_x'], trace['reference_y']))
    actual = np.column_stack((trace['true_x'], trace['true_y']))
    result['timed_planar_rmse'] = float(np.sqrt(np.mean(np.sum((actual-reference)**2, axis=1))))
    # This is a sampled geometric distance used only in offline evaluation.
    lateral_distance = cKDTree(reference).query(actual)[0]
    result['sampled_path_distance_rms'] = float(np.sqrt(np.mean(lateral_distance**2)))
    result['sampled_path_distance_maximum'] = float(lateral_distance.max())
    result['requested_torque_maximum'] = float(max(np.abs(trace['torque_left']).max(), np.abs(trace['torque_right']).max()))
    last=trace[-1]
    result['endpoint_diagnostics']={
        'position_error':float(position_error[-1]), 'yaw_error':float(yaw_error[-1]),
        'velocity':float(last['v']), 'yaw_rate':float(last['yaw_rate']), 'pitch':float(last['pitch']),
        'settled_except_distance':bool(abs(last['v'])<.05 and abs(last['yaw_rate'])<.1 and abs(last['pitch'])<.1),
    }
    result['source'] = str(path)
    result['source_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path('artifacts/benchmark/d18_full_circle_v1'))
    args = parser.parse_args()
    groups = {}
    for method, folder in [('mpc', 'mpc_v2'), ('ppo', 'ppo')]:
        paths = sorted((args.root/folder).glob(f'{method}_*.json'))
        if len(paths) != 100:
            raise ValueError(f'{method} has {len(paths)} episodes; require all 100 before reporting')
        groups[method] = [read_episode(path) for path in paths]
    paired = [{(row['family'], row['index']): row['reference_sha256'] for row in rows} for rows in groups.values()]
    if len(paired[0]) != 100 or paired[0] != paired[1]:
        raise ValueError('Paired case/reference mismatch')
    metrics = ['final_distance', 'true_final_distance', 'path_length', 'path_ratio',
               'pitch_rms', 'maximum_pitch', 'absolute_torque', 'squared_torque',
               'saturation_duration', 'requested_torque_maximum', 'position_rmse',
               'yaw_rmse', 'timed_planar_rmse', 'sampled_path_distance_rms']
    manifest=json.loads((args.root/'mpc_v2'/'manifest.json').read_text())
    summary = {'count_per_method': 100, 'identical_paired_reference_hashes': True,
               'population_version':manifest.get('version','d18_forward_only'),
               'source_root':str(args.root), 'methods': {}}
    for method, rows in groups.items():
        outcomes = {key: sum(bool(row[key]) for row in rows) for key in ['success', 'fall', 'collision', 'boundary', 'timeout']}
        record = {'outcomes': outcomes, 'all_episodes': {}, 'successful_episodes': {}, 'failed_episodes': {}}
        for name, subset in [('all_episodes', rows), ('successful_episodes', [r for r in rows if r['success']]),
                             ('failed_episodes', [r for r in rows if not r['success']])]:
            record[name] = {key: distribution([row[key] for row in subset]) for key in metrics}
        record['successful_completion_time'] = distribution([row['time'] for row in rows if row['success']])
        record['families'] = {family: {'count': sum(r['family']==family for r in rows),
                                      'success': sum(r['family']==family and r['success'] for r in rows)}
                              for family in sorted({r['family'] for r in rows})}
        if all('bearing' in row for row in rows):
            record['bearing_bins']={}
            for label,low,high in [('front',0,45),('front_side',45,90),('rear_side',90,135),('rear',135,180)]:
                subset=[row for row in rows if low<=abs(np.degrees(row['bearing']))<high]
                record['bearing_bins'][label]={'count':len(subset),'success':sum(row['success'] for row in subset),
                                               'fall':sum(row['fall'] for row in subset),'timeout':sum(row['timeout'] for row in subset)}
        summary['methods'][method] = record
    if all('bearing' in row for rows in groups.values() for row in rows):
        figure,axes=plt.subplots(1,2,figsize=(10,4),subplot_kw={'projection':'polar'},constrained_layout=True)
        for axis,(method,rows) in zip(axes,groups.items()):
            for success,color,label in [(True,'#3274a1','Success'),(False,'#c44e52','Failure')]:
                subset=[row for row in rows if row['success']==success]
                axis.scatter([r['bearing'] for r in subset],[np.linalg.norm(r['goal']) for r in subset],s=20,c=color,label=label)
            axis.set_title(method.upper()+' · goal radius (m)');axis.set_theta_zero_location('E')
            axis.set_thetagrids(np.arange(0,360,45),['0°','45°','90°','135°','±180°','−135°','−90°','−45°'])
            axis.legend(loc='upper center',bbox_to_anchor=(.5,-.12),ncol=2,fontsize=8)
        figure.savefig('docs/figures/benchmark/d18_goal_bearings.png',dpi=180);plt.close(figure)
    (args.root/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    Path('docs/d18_benchmark_results.json').write_text(json.dumps({'summary': summary, 'episodes': groups}, indent=2, allow_nan=False)+'\n')
    destination = Path('docs/figures/benchmark')
    destination.mkdir(parents=True, exist_ok=True)
    colors = ['#3274a1', '#dd8452']
    figure, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
    for index, (metric, label) in enumerate([
            ('final_distance', 'Estimated final distance (m)'),
            ('position_rmse', 'Longitudinal tracking RMSE (m)'),
            ('yaw_rmse', 'Yaw tracking RMSE (rad)'),
            ('pitch_rms', 'True pitch RMS (rad)'),
            ('squared_torque', 'Integrated squared torque (N² m² s)'),
            ('path_ratio', 'Planned / travelled path length')]):
        axis = axes.flat[index]
        boxes = axis.boxplot([[r[metric] for r in rows] for rows in groups.values()], tick_labels=['MPC', 'Trajectory PPO'], patch_artist=True)
        for box, color in zip(boxes['boxes'], colors):
            box.set_facecolor(color); box.set_alpha(.6)
        axis.set_ylabel(label); axis.grid(axis='y', alpha=.2)
    figure.savefig(destination/'d18_tracking.png', dpi=180)
    plt.close(figure)
    figure, axis = plt.subplots(figsize=(7, 3.5), constrained_layout=True)
    x = np.arange(4)
    for index, (method, record) in enumerate(summary['methods'].items()):
        values = [record['outcomes'][key] for key in ['success', 'collision', 'fall', 'timeout']]
        bars = axis.bar(x+(index-.5)*.35, values, .35, label=method.upper(), color=colors[index])
        axis.bar_label(bars)
    axis.set_xticks(x, ['Settled success', 'Collision', 'Fall', 'Timeout'])
    axis.set_ylabel('Episodes / 100'); axis.set_ylim(0, 112); axis.legend()
    figure.savefig(destination/'d18_outcomes.png', dpi=180)
    plt.close(figure)
    lines = ['| Metric | Shared planner + MPC | Shared planner + trajectory PPO |', '|---|---:|---:|']
    for key in ['success', 'collision', 'fall', 'timeout']:
        lines.append('| '+key+' | '+' | '.join(f"{summary['methods'][m]['outcomes'][key]}/100" for m in groups)+' |')
    labels = {'final_distance':'estimated final distance (m)', 'true_final_distance':'true final distance (m)',
              'path_length':'travelled path length (m)', 'path_ratio':'planned / travelled path length',
              'pitch_rms':'true pitch RMS (rad)', 'maximum_pitch':'peak true pitch (rad)',
              'absolute_torque':'integrated absolute torque (N m s)', 'squared_torque':'integrated squared torque (N² m² s)',
              'saturation_duration':'saturation duration (s)', 'requested_torque_maximum':'peak requested torque (N m)',
              'position_rmse':'longitudinal RMSE (m)', 'yaw_rmse':'yaw RMSE (rad)',
              'timed_planar_rmse':'time-aligned true planar RMSE (m)', 'sampled_path_distance_rms':'sampled-path distance RMS (m)'}
    for key in metrics:
        lines.append('| Mean '+labels[key]+' | '+' | '.join(f"{summary['methods'][m]['all_episodes'][key]['mean']:.6g}" for m in groups)+' |')
    lines.append('| Successful completion time (s), mean and count | '+' | '.join(
        f"{summary['methods'][m]['successful_completion_time'].get('mean', float('nan')):.4g} (n={summary['methods'][m]['successful_completion_time']['count']})" for m in groups)+' |')
    table='\n'.join(lines)
    (args.root/'summary.md').write_text(table+'\n')
    for document in [Path('README.md'),Path('docs/TRAJECTORY_POLICY.md')]:
        text=document.read_text()
        begin='<!-- D18_RESULTS_BEGIN -->';end='<!-- D18_RESULTS_END -->'
        if begin in text and end in text:
            document.write_text(text.split(begin)[0]+begin+'\n\n'+table+'\n\n'+end+text.split(end)[1])
    print(json.dumps({m: r['outcomes'] for m, r in summary['methods'].items()}, indent=2))


if __name__ == '__main__':
    main()
