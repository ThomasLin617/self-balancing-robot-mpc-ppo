import argparse
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .metrics import measure, statistics


def table(headers, rows):
    return '\n'.join(['| '+' | '.join(headers)+' |', '| '+' | '.join(['---']*len(headers))+' |'] +
                     ['| '+' | '.join(map(str,row))+' |' for row in rows])


def number(value):
    return '—' if value is None else f'{value:.4g}'


def plain_scalars(row):
    return {key: value.item() if isinstance(value,np.generic) else value for key,value in row.items()}


def trial_label(row):
    reference = f"{row['goal'][0]:+.2f} m"
    return reference+' + pulse' if row['force'] else reference


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',default='artifacts/benchmark/d16')
    parser.add_argument('--figures',default='docs/figures/benchmark')
    args=parser.parse_args()
    root=Path(args.input)
    config=json.loads((root/'config.json').read_text())
    jobs=json.loads((root/'scenarios.json').read_text())
    rows=[]
    traces={}
    timings={}
    for job in jobs:
        stem=root/job['group']/(job['method']+'_'+job['name'])
        metadata=json.loads(stem.with_suffix('.json').read_text())
        if metadata['exit_code']:
            raise RuntimeError(f'Benchmark executable failed: {stem}; inspect stderr before reporting.')
        with gzip.open(str(stem)+'.csv.gz','rt') as stream:
            data=np.atleast_1d(np.genfromtxt(stream,delimiter=',',names=True))
        row=plain_scalars({**job,**measure(data,job,config)})
        if job['group']!='sagittal':
            for key in ['position_rmse','position_final_error','position_settling_time','velocity_rmse']:
                row.pop(key)
        if job['group']=='disturbance':
            row['pulse_exposed']=bool(np.any(data['force']!=0))
            row['failure_before_pulse']=bool(job['force'] and not row['pulse_exposed'] and not row['success'])
        rows.append(row)
        if job['name'] in ['step_4','push','push_0','clean_0']:
            traces[(job['group'],job['name'],job['method'])]=data
        if job['group']=='compute':
            valid=data[data['full_us']>=0][config['timing']['warmup']:]
            if len(valid)!=config['timing']['samples']:
                raise RuntimeError('Timing run did not produce the frozen sample count.')
            timings[job['method']]={key:valid[key] for key in ['full_us','core_us']}
    for row in rows:
        if row['group']=='disturbance' and row['force']:
            clean=next(r for r in rows if r['group']=='disturbance' and r['method']==row['method'] and r['name']==row['name'].replace('push','clean'))
            for key in ['path_length','absolute_torque','squared_torque']:
                row['additional_'+key]=row[key]-clean[key]
    groups={}
    for group in ['sagittal','goal_reaching','disturbance']:
        selected=[r for r in rows if r['group']==group and
                  (group!='goal_reaching' or r['population']=='random') and
                  (group!='disturbance' or r['force']!=0)]
        groups[group]={}
        for method in sorted({r['method'] for r in selected}):
            data=[r for r in selected if r['method']==method]
            metrics=(set().union(*(r.keys() for r in data))-set(jobs[0])) | {'duration'}
            groups[group][method]={'episodes':len(data),'successes':sum(r['success'] for r in data),
                                  'falls':sum(r['fall'] for r in data),'timeouts':sum(r['timeout'] for r in data),
                                  'metrics':{key:statistics([r.get(key) for r in data]) for key in sorted(metrics)}}
    audit_timing=root/'audit/lqr_batched_timing.json'
    if audit_timing.exists():
        metadata=json.loads(audit_timing.read_text())
        batches=root/'audit/lqr_batches.csv'
        if hashlib.sha256(batches.read_bytes()).hexdigest()!=metadata['raw_sha256']:
            raise ValueError('LQR batch timing provenance mismatch.')
        data=np.atleast_1d(np.genfromtxt(batches,delimiter=',',names=True))
        timings['lqr']={'batch_mean_us':data['per_update_us']}
    summary={'groups':groups,'compute':{m:{k:statistics(v) for k,v in values.items()} for m,values in timings.items()},
             'config':config,'machine':json.loads((root/'machine.json').read_text())}
    if audit_timing.exists(): summary['lqr_batch_method']=metadata
    summary['analysis_sources']={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in [Path(__file__),Path(__file__).with_name('metrics.py')]}
    if (root/'known_map.json').exists():
        summary['known_map']=json.loads((root/'known_map.json').read_text())
    (root/'episodes.json').write_text(json.dumps(rows,indent=2,allow_nan=False)+'\n')
    (root/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    Path('docs/benchmark_results.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    output=[]
    a=[r for r in rows if r['group']=='sagittal']
    output.append('### Sagittal regulation: LQR and MPC\n')
    output.append(table(['Trial','Method','Position RMSE (m)','Final error (m)','Max pitch (rad)','∫τ²dt','Saturation (s)','Recovery (s)'],
                        [[trial_label(r),r['method'].upper(),*[number(r[k]) for k in ['position_rmse','position_final_error','maximum_pitch','squared_torque','saturation_duration']],number(r.get('disturbance_recovery_time'))] for r in a]))
    output.append('\n### Common obstacle-free goal reaching: 100 paired random goals\n')
    b=groups['goal_reaching']
    output.append(table(['Method','Success','Falls','Timeouts','Final true distance, mean (m)','Successful time, mean (s)','Path length, mean (m)','RMS pitch, mean (rad)','∫τ²dt, mean'],
        [[m.upper(),f"{v['successes']}/{v['episodes']}",v['falls'],v['timeouts'],*[number(v['metrics'][k]['mean']) for k in ['true_final_distance','time_to_success','path_length','pitch_rmse','squared_torque']]] for m,v in b.items()]))
    output.append(table(['Method','Estimated final distance, mean (m)','Efficiency ratio, mean','Pitch, worst (rad)','Torque, worst (N·m)','∫|τ|dt, mean','Saturation, mean (s)'],
        [[m.upper(),number(v['metrics']['estimated_final_distance']['mean']),number(v['metrics']['path_efficiency']['mean']),
          number(v['metrics']['maximum_pitch']['maximum']),number(v['metrics']['applied_torque']['maximum']),
          number(v['metrics']['absolute_torque']['mean']),number(v['metrics']['saturation_duration']['mean'])] for m,v in b.items()]))
    output.append('\n### Disturbed goal reaching: eight paired fixed directions\n')
    output.append(table(['Method','Task success','Falls','Pulse exposed','Failures before pulse','Peak post-push pitch, worst (rad)','Goal-distance increase, worst (m)','Successful recovery, mean (s)','Additional path, mean (m)','Additional ∫τ²dt, mean'],
        [[m.upper(),f"{v['successes']}/{v['episodes']}",v['falls'],int(round(v['metrics']['pulse_exposed']['mean']*v['episodes'])),
          int(round(v['metrics']['failure_before_pulse']['mean']*v['episodes'])),number(v['metrics']['post_push_peak_pitch']['maximum']),number(v['metrics']['post_push_distance_increase']['maximum']),number(v['metrics']['disturbance_recovery_time']['mean']),number(v['metrics']['additional_path_length']['mean']),number(v['metrics']['additional_squared_torque']['mean'])] for m,v in groups['disturbance'].items()]))
    output.append('\n### Isolated Release computation: microseconds per update\n')
    output.append(table(['Method / scope','Samples','Mean','Median','p95','p99','Maximum'],
        [[m.upper()+' / '+scope,s['count'],*[number(s[k]) for k in ['mean','median','p95','p99','maximum']]]
         for m,values in summary['compute'].items() for scope,s in values.items()]))
    if audit_timing.exists():
        output.append(f"LQR entries describe {metadata['batches']} batch averages, each over {metadata['updates_per_batch']} production calls, after {metadata['warmup_batches']} warm-up batches. They include indexed input loads and output stores. Their p99/maximum are batch-average statistics, not single-call latency bounds. MPC/PPO retain the original individual-update measurements.")
    compact='\n\n'.join(output)+'\n'
    (root/'benchmark_summary.md').write_text(compact)
    details=[compact,'\n### Full scalar distributions\n','Population standard deviation is used. Null values are excluded only for conditionally defined metrics; their count is shown. All other episode statistics retain failures.\n']
    for group,methods in groups.items():
        for method,value in methods.items():
            details.append(f'\n#### {group}: {method.upper()}\n')
            details.append(table(['Metric','Defined count','Mean','Median','Std','p95','Maximum'],
                [[k,s['count'],*[number(s[x]) for x in ['mean','median','std','p95','maximum']]] for k,s in value['metrics'].items()]))
    full='\n\n'.join(details)+'\n'
    (root/'benchmark_distributions.md').write_text(full)
    for filename,fragment in [('README.md',compact),('docs/OBSTACLE_FREE_BENCHMARK.md',full)]:
        path=Path(filename)
        if path.exists():
            text=path.read_text()
            start,end='<!-- BENCHMARK_RESULTS_BEGIN -->','<!-- BENCHMARK_RESULTS_END -->'
            if start in text and end in text:
                before=text.split(start)[0]
                after=text.split(end,1)[1]
                path.write_text(before+start+'\n\n'+fragment+'\n'+end+after)
    readme=Path('README.md')
    if readme.exists():
        text=readme.read_text()
        start,end='<!-- KEY_RESULTS_BEGIN -->','<!-- KEY_RESULTS_END -->'
        if start in text and end in text:
            lines=[]
            for m in ['mpc','ppo']:
                v=b[m]
                lines.append(f"- **{m.upper()}, common D16 random goals:** {v['successes']}/{v['episodes']} settled successes; {v['falls']} estimated-pitch falls; mean true final distance {v['metrics']['true_final_distance']['mean']:.4f} m.")
            for m in ['lqr','mpc','ppo']:
                batched=m=='lqr' and audit_timing.exists()
                s=summary['compute'][m]['batch_mean_us' if batched else 'full_us']
                label='batch-mean per call' if batched else 'full update'
                lines.append(f"- **{m.upper()} measured {label}:** p95 {number(s['p95'])} µs, maximum {number(s['maximum'])} µs on the recorded Release platform.")
            readme.write_text(text.split(start)[0]+start+'\n\n'+'\n'.join(lines)+'\n\n'+end+text.split(end,1)[1])
    if 'known_map' in summary:
        values=summary['known_map']['metrics']
        units={'time':'s','estimated goal error':'m','debug true goal error':'m','max torque':'N·m',
               'max pitch after recovery':'rad','max odometry error':'m'}
        fragment=table(['Known-map capability metric','Measured value'],[['Total simulated duration (including startup recovery)' if k=='time' else k,number(v)+' '+units.get(k,'')] for k,v in values.items()])
        for name in ['README.md','docs/OBSTACLE_FREE_BENCHMARK.md']:
            path=Path(name)
            text=path.read_text()
            start,end='<!-- KNOWN_MAP_RESULT_BEGIN -->','<!-- KNOWN_MAP_RESULT_END -->'
            if start in text and end in text:
                path.write_text(text.split(start)[0]+start+'\n\n'+fragment+'\n\n'+end+text.split(end,1)[1])
    figures=Path(args.figures)
    figures.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':140})
    colors=['#2878a5','#c86c36']
    fig,axes=plt.subplots(2,4,figsize=(15,7),constrained_layout=True)
    methods=['mpc','ppo']
    axes.flat[0].bar([m.upper() for m in methods],[100*b[m]['successes']/b[m]['episodes'] for m in methods],color=colors)
    axes.flat[0].set(ylabel='Settled success (%)',ylim=(0,100),title='100 paired random goals')
    fields=[('true_final_distance','True final distance (m)'),('time_to_success','Successful time only (s)'),('path_length','Traveled path length (m)'),
            ('path_efficiency','Goal distance / path length'),('pitch_rmse','Estimated RMS pitch (rad)'),('maximum_pitch','Estimated peak pitch (rad)'),('squared_torque','Integrated squared torque ((N m)² s)')]
    selected=[r for r in rows if r['group']=='goal_reaching' and r['population']=='random']
    for axis,(key,label) in zip(list(axes.flat)[1:],fields):
        data=[[r[key] for r in selected if r['method']==m and r[key] is not None] for m in methods]
        boxes=axis.boxplot(data,tick_labels=[m.upper() for m in methods],patch_artist=True)
        for patch,color in zip(boxes['boxes'],colors):patch.set_facecolor(color);patch.set_alpha(.55)
        axis.set_ylabel(label)
        axis.grid(axis='y',alpha=.2)
    fig.savefig(figures/'goal_reaching.png');plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4),constrained_layout=True)
    labels=[];values=[]
    for m,scopes in timings.items():
        for scope,value in scopes.items():labels.append(m.upper()+'\n'+scope);values.append(value)
    ax.boxplot(values,tick_labels=labels,showfliers=False)
    ax.set_yscale('symlog',linthresh=.05)
    ax.set(ylabel='Measured time (µs; linear below 0.05)',title='Release; LQR batch averages, MPC/PPO individual updates' if audit_timing.exists() else 'Release; individual update measurements')
    ax.axhline(2000,color='gray',linestyle='--',label='2 ms simulation control period')
    ax.legend();fig.savefig(figures/'compute.png');plt.close(fig)
    for group,name,methods,title in [('sagittal','step_4',['lqr','mpc'],'+0.1 m position step'),('sagittal','push',['lqr','mpc'],'Sagittal standardized push'),('disturbance','push_0',['mpc','ppo'],'Goal-reaching standardized push')]:
        fig,axes=plt.subplots(3,1,figsize=(8,7),sharex=True,constrained_layout=True)
        for m,color in zip(methods,colors):
            data=traces[(group,name,m)]
            axes[0].plot(data['time'],data['pitch'],label=m.upper(),color=color)
            axes[1].plot(data['time'],data['p']-.1 if group=='sagittal' else data['goal_distance'],color=color)
            axes[2].plot(data['time'],data['applied_l'],label=m.upper()+' L',color=color)
            axes[2].plot(data['time'],data['applied_r'],label=m.upper()+' R',color=color,linestyle='--')
        axes[0].set(title=title,ylabel='Estimated pitch (rad)');axes[0].legend()
        axes[1].set_ylabel('Position error (m)' if group=='sagittal' else 'Estimated goal distance (m)')
        axes[2].set(ylabel='Applied torque (N m)',xlabel='Simulation time (s)');axes[2].legend(ncol=2)
        for ax in axes:
            ax.grid(alpha=.2)
            if 'push' in name:ax.axvspan(config['disturbance']['time'],config['disturbance']['time']+config['disturbance']['duration'],color='gray',alpha=.3)
        fig.savefig(figures/(group+'_'+name+'.png'));plt.close(fig)
    print(compact)


if __name__=='__main__':main()
