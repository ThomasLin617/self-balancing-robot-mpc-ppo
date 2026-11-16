import argparse
import gzip
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import shutil
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from .benchmark_report import distribution

ROOT = Path('artifacts/benchmark/d18_full_circle_retrained_v2')
ACTOR = Path('artifacts/learning/d18_selected_full_circle_v2/actor.txt')
FIGURES = Path('docs/figures/comparison')
DATA = Path('docs/data/comparison')
COLORS = {'mpc': '#286b9c', 'ppo': '#bc6139'}
METRICS = [
    ('true_final_distance', 'True final goal distance (m)'),
    ('timed_planar_rmse', 'Time-aligned planar RMSE (m)'),
    ('position_rmse', 'Longitudinal tracking RMSE (m)'),
    ('yaw_rmse', 'Yaw tracking RMSE (rad)'),
    ('pitch_rms', 'Pitch RMS (rad)'),
    ('squared_torque', 'Integrated squared torque (N² m² s)'),
]


def trace(method, case):
    with gzip.open(DATA/f'{method}_{case[0]}_{case[1]:02}.csv.gz', 'rt') as stream:
        return np.genfromtxt(stream, delimiter=',', names=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--refresh-data',action='store_true')
    args=parser.parse_args()
    FIGURES.mkdir(parents=True, exist_ok=True)
    result = json.loads(Path('docs/d18_benchmark_results.json').read_text())
    assert result['summary']['source_root'] == str(ROOT)
    rows = result['episodes']
    indexed = {m: {(r['family'], r['index']): r for r in records} for m, records in rows.items()}
    assert len(indexed['mpc']) == len(indexed['ppo']) == 100
    assert indexed['mpc'].keys() == indexed['ppo'].keys()
    for key in indexed['mpc']:
        assert indexed['mpc'][key]['reference_sha256'] == indexed['ppo'][key]['reference_sha256']
    selected = min(indexed['mpc'], key=lambda k: (
        -abs(indexed['mpc'][k]['timed_planar_rmse']-indexed['ppo'][k]['timed_planar_rmse']), k))
    manifest = json.loads(Path('docs/d18_benchmark_manifest.json').read_text())
    case = next(c for c in manifest['cases'] if (c['family'], c['index']) == selected)
    if args.refresh_data:
        files=[]
        for method,folder in [('mpc','mpc_v2'),('ppo','ppo')]:
            source=ROOT/folder/f'{method}_{selected[0]}_{selected[1]:02}.csv.gz'
            shutil.copyfile(source,DATA/source.name);files.append(source.name)
        for source in [Path('artifacts/benchmark/final_release/compute.csv'),Path('artifacts/learning/d18_full_circle_v1')/case['scenario']]:
            shutil.copyfile(source,DATA/source.name);files.append(source.name)
        machine={'platform':platform.platform(),'cpu':subprocess.check_output(['sysctl','-n','machdep.cpu.brand_string'],text=True).strip(),
                 'compiler':subprocess.check_output(['clang','--version'],text=True).splitlines()[0],
                 'build_type':'Release','warmup':500,'samples':5000,'clock':'std::chrono::steady_clock'}
        machine['python_dependencies']={name:importlib.metadata.version(name) for name in ['mujoco','numpy','torch','stable-baselines3','pybind11']}
        data_manifest={'files':{name:hashlib.sha256((DATA/name).read_bytes()).hexdigest() for name in files},
                       'machine':machine,'source':str(ROOT),'actor_sha256':hashlib.sha256(ACTOR.read_bytes()).hexdigest()}
        (DATA/'manifest.json').write_text(json.dumps(data_manifest,indent=2)+'\n')
    data_manifest=json.loads((DATA/'manifest.json').read_text())
    if hashlib.sha256(ACTOR.read_bytes()).hexdigest()!=data_manifest['actor_sha256']:
        raise ValueError('Selected actor differs from the timing and recording dataset.')
    for name,digest in data_manifest['files'].items():
        if hashlib.sha256((DATA/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Comparison input changed: '+name)
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
    for ax, (metric, title) in zip(axes.flat, METRICS):
        boxes = ax.boxplot([[r[metric] for r in rows[m]] for m in COLORS],
                           tick_labels=['MPC', 'PPO'], patch_artist=True)
        for patch, color in zip(boxes['boxes'], COLORS.values()):
            patch.set_facecolor(color); patch.set_alpha(.55)
        ax.set_ylabel(title); ax.grid(axis='y', alpha=.2)
    fig.savefig(FIGURES/'navigation_distributions.png', dpi=180); plt.close(fig)

    traces = {m: trace(m, selected) for m in COLORS}
    yaw = case['world_initial_yaw']
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    scenario = DATA/case['scenario']
    for line in scenario.read_text().splitlines():
        fields = line.split()
        if fields[0] == 'obstacle':
            x0,y0,x1,y1 = map(float, fields[2:6])
            axes[0].add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='#c2c7cb'))
    reference = np.column_stack((traces['mpc']['reference_x'], traces['mpc']['reference_y']))@rotation.T
    axes[0].plot(*reference.T, '--', color='#636b73', label='Reference', linewidth=1.4)
    for method, data in traces.items():
        actual = np.column_stack((data['true_x'], data['true_y']))@rotation.T
        axes[0].plot(*actual.T, color=COLORS[method], label=method.upper(), linewidth=1.6)
        error = np.hypot(data['true_x']-data['reference_x'], data['true_y']-data['reference_y'])
        axes[1].plot(data['time'], error, color=COLORS[method], label=method.upper())
    axes[0].scatter(0,0,marker='o',color='#444444',s=30,label='Start')
    axes[0].scatter(*case['world_goal'],marker='*',color='#448054',s=120,label='Goal')
    axes[0].set(xlabel='World x (m)',ylabel='World y (m)',aspect='equal')
    axes[0].legend(fontsize=9,loc='lower center',bbox_to_anchor=(.5,1.02),ncol=3)
    axes[0].grid(alpha=.15)
    axes[1].set(xlabel='Simulation time (s)',ylabel='Time-aligned planar error (m)')
    axes[1].legend(); axes[1].grid(alpha=.2)
    fig.savefig(FIGURES/'largest_tracking_difference.png',dpi=180); plt.close(fig)

    fig, axes = plt.subplots(2,2,figsize=(11,6),constrained_layout=True)
    for method,data in traces.items():
        values = [(data['yaw']-data['reference_yaw']+np.pi)%(2*np.pi)-np.pi,
                  data['true_pitch'],data['torque_left'],data['torque_right']]
        for ax,value in zip(axes.flat,values):
            ax.plot(data['time'],value,color=COLORS[method],label=method.upper())
    for ax,title in zip(axes.flat,['Yaw tracking error (rad)','Pitch (rad)','Left torque (N m)','Right torque (N m)']):
        ax.set(xlabel='Simulation time (s)',ylabel=title);ax.grid(alpha=.2);ax.legend()
    fig.savefig(FIGURES/'tracking_response.png',dpi=180);plt.close(fig)

    timing_path = DATA/'compute.csv'
    samples = np.genfromtxt(timing_path,delimiter=',',names=True)
    assert len(samples)==5000
    names = {'actor_us':'PPO actor','ppo_update_us':'PPO full update',
             'mpc_compute_us':'MPC compute','mpc_update_us':'MPC full update'}
    timing = {key:{**distribution(samples[key]),'p99':float(np.percentile(samples[key],99))} for key in names}
    fig,ax=plt.subplots(figsize=(9,4),constrained_layout=True)
    ax.boxplot([samples[key] for key in names],tick_labels=list(names.values()),showfliers=True)
    ax.set_yscale('log');ax.set_ylabel('Wall time per update (µs, log scale)')
    ax.axhline(2000,color='#777777',linestyle='--',label='2 ms simulation control period')
    ax.legend();ax.grid(axis='y',alpha=.2)
    fig.savefig(FIGURES/'controller_timing.png',dpi=180);plt.close(fig)
    machine=data_manifest['machine']
    output={**result,'selected_case':case,'selection':'Largest absolute difference in time-aligned true planar RMSE; ties use family/index.',
            'timing':timing,'machine':machine,'actor_sha256':hashlib.sha256(ACTOR.read_bytes()).hexdigest(),
            'timing_sha256':hashlib.sha256(timing_path.read_bytes()).hexdigest()}
    Path('docs/final_comparison.json').write_text(json.dumps(output,indent=2)+'\n')
    methods=result['summary']['methods']
    table=['| Metric | MPC | PPO |','|---|---:|---:|',
           '| Settled successes | '+' | '.join(f"{methods[m]['outcomes']['success']}/{len(rows[m])}" for m in COLORS)+' |',
           '| Falls / collisions / timeouts | '+' | '.join(' / '.join(str(methods[m]['outcomes'][k]) for k in ['fall','collision','timeout']) for m in COLORS)+' |']
    for key,label in METRICS:
        table.append('| Mean '+label[0].lower()+label[1:]+' | '+' | '.join(
            f"{result['summary']['methods'][m]['all_episodes'][key]['mean']:.5g}" for m in COLORS)+' |')
    table.append('| Mean successful completion time (s), n=100 each | '+' | '.join(
        f"{result['summary']['methods'][m]['successful_completion_time']['mean']:.4g}" for m in COLORS)+' |')
    timing_table=['| Computation (µs) | Mean | Median | p95 | p99 | Maximum |','|---|---:|---:|---:|---:|---:|']
    for key,label in names.items():
        timing_table.append('| '+label+' | '+' | '.join(f'{timing[key][v]:.4g}' for v in ['mean','median','p95','p99','maximum'])+' |')
    distributions=['| Metric | Controller | Mean | Median | Std | p95 | Maximum |','|---|---|---:|---:|---:|---:|---:|']
    for key,label in METRICS:
        for method in COLORS:
            values=methods[method]['all_episodes'][key]
            distributions.append('| '+label+' | '+method.upper()+' | '+' | '.join(f'{values[k]:.5g}' for k in ['mean','median','std','p95','maximum'])+' |')
    for path in [Path('README.md'),Path('docs/CONTROLLER_COMPARISON.md')]:
        if not path.exists():continue
        text=path.read_text()
        for name,lines in [('NAVIGATION_RESULTS',table),('COMPUTE_RESULTS',timing_table),('NAVIGATION_DISTRIBUTIONS',distributions)]:
            begin,end=f'<!-- {name}_BEGIN -->',f'<!-- {name}_END -->'
            if begin in text and end in text:
                text=text.split(begin)[0]+begin+'\n\n'+'\n'.join(lines)+'\n\n'+end+text.split(end)[1]
        path.write_text(text)
    destination=Path('artifacts/benchmark/final_release')
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'tables.md').write_text('\n'.join(table+['']+timing_table)+'\n')
    (DATA/'tables.md').write_text('\n'.join(table+['']+timing_table)+'\n')
    print('Selected',selected,'actor',output['actor_sha256'])


if __name__=='__main__':main()
