import argparse
import gzip
import json
import math
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .metrics import measure, statistics
from .run_benchmark import digest


def read_trace(path):
    with gzip.open(path, 'rt') if str(path).endswith('.gz') else open(path) as stream:
        return np.atleast_1d(np.genfromtxt(stream, delimiter=',', names=True))


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def bearing_bin(bearing):
    value = abs(bearing)
    return next(name for edge, name in [(math.pi/4, 'front'), (math.pi/2, 'front-side'),
                                      (3*math.pi/4, 'rear-side'), (math.pi, 'rear')] if value <= edge)


def groups(rows, key):
    result = {}
    for name in dict.fromkeys(row[key] for row in rows):
        subset = [row for row in rows if row[key] == name]
        result[name] = {'count': len(subset), 'successes': sum(row['success'] for row in subset),
                        'falls': sum(row['fall'] for row in subset),
                        'radius': statistics([row['radius'] for row in subset]),
                        'failure_time': statistics([row['failure_time'] for row in subset]),
                        'maximum_reference_yaw_rate': statistics([row['maximum_reference_yaw_rate'] for row in subset])}
    return result


def analyze(root, output, figures):
    config = json.loads((root/'config.json').read_text())
    jobs = [job for job in json.loads((root/'scenarios.json').read_text())
            if job['group'] == 'goal_reaching' and job['population'] == 'random']
    previous = json.loads((root/'episodes.json').read_text())
    probe = Path('ros2_ws/build/mujoco_backend/benchmark_audit_probe').resolve()
    trajectory_dir = output/'trajectories'
    trajectory_dir.mkdir(exist_ok=True)
    rows, estimates, traces, hashes = [], [], {}, {}
    for job in jobs:
        path = root/'goal_reaching'/f"{job['method']}_{job['name']}.csv.gz"
        data = read_trace(path)
        hashes[str(path)] = digest(path)
        measured = measure(data, job, config)
        old = next(row for row in previous if row['group'] == job['group'] and row['name'] == job['name'] and row['method'] == job['method'])
        for key in ['success', 'fall', 'duration', 'estimated_final_distance', 'true_final_distance', 'saturation_duration']:
            if measured[key] != old[key]: raise ValueError(f'Existing metric changed: {path}, {key}')
        if job['method'] != 'mpc': continue
        gx, gy = job['goal']
        bearing = math.atan2(gy, gx)
        trajectory_path = trajectory_dir/(job['name']+'.csv')
        command = [str(probe), 'trajectory', str(gx), str(gy), str(config['trajectory']['max_velocity']),
                   str(config['trajectory']['acceleration']), str(trajectory_path)]
        subprocess.run(command, check=True)
        trajectory = read_trace(trajectory_path)
        radius = math.hypot(gx, gy)
        row = dict(name=job['name'], goal=job['goal'], radius=radius, bearing=bearing,
                   bearing_bin=bearing_bin(bearing), radius_bin=next(f'{low:.1f}–{low+.2:.1f}' for low in [.2,.4,.6,.8] if radius <= low+.2),
                   success=measured['success'], fall=measured['fall'], failure_time=None if measured['success'] else measured['duration'],
                   maximum_pitch=measured['maximum_pitch'], maximum_requested_torque=measured['requested_torque'],
                   maximum_applied_torque=measured['applied_torque'], saturation_duration=measured['saturation_duration'],
                   trajectory_duration=float(trajectory['duration'][0]), initial_reference_yaw=float(trajectory['yaw'][0]),
                   first_preview_end_yaw=float(trajectory['yaw'][np.searchsorted(trajectory['time'], .5)]),
                   maximum_reference_yaw_rate=float(np.abs(trajectory['yaw_rate']).max()),
                   first_horizon_maximum_yaw_rate=float(np.abs(trajectory['yaw_rate'][trajectory['time'] <= .5]).max()),
                   maximum_reference_curvature=float(np.abs(trajectory['curvature']).max()))
        row['absolute_bearing'] = abs(bearing)
        paired = next(r for r in previous if r['group']=='goal_reaching' and r['method']=='ppo' and r['name']==job['name'])
        row['paired_ppo_success'] = paired['success']
        row['paired_ppo_fall'] = paired['fall']
        row['paired_ppo_timeout'] = paired['timeout']
        row['reference_yaw_rate_peak_time'] = float(trajectory['time'][np.argmax(np.abs(trajectory['yaw_rate']))])
        row['maximum_visible_reference_yaw_rate'] = float(np.abs(trajectory['yaw_rate'][trajectory['time'] <= measured['duration']+.5]).max())
        saturated = np.flatnonzero((np.abs(data['applied_l']) >= 2-1e-6) | (np.abs(data['applied_r']) >= 2-1e-6))
        row['first_saturation_time'] = float(data['time'][saturated[0]]) if len(saturated) else None
        rows.append(row)
        traces[job['name']] = data
        estimated_distance = np.hypot(gx-data['x'], gy-data['y'])
        true_distance = np.hypot(gx-data['true_x'], gy-data['true_y'])
        distance_error = max(float(np.abs(estimated_distance-data['goal_distance']).max()),
                             float(np.abs(true_distance-data['true_distance']).max()))
        if distance_error > 1e-10: raise ValueError('Distance/frame reconstruction failed.')
        dt = config['physics_dt']
        yaw = np.cumsum(data['yaw_rate']*dt)
        previous_yaw = np.r_[0.0, yaw[:-1]]
        distance = .5*(np.r_[0.0, data['v'][:-1]]+data['v'])*dt
        x = np.cumsum(distance*np.cos(.5*(yaw+previous_yaw)))
        y = np.cumsum(distance*np.sin(.5*(yaw+previous_yaw)))
        odometry_error = float(max(np.abs(x-data['x']).max(), np.abs(y-data['y']).max()))
        if odometry_error > 1e-8: raise ValueError('Odometry reconstruction failed.')
        discrepancy = np.hypot(data['x']-data['true_x'], data['y']-data['true_y'])
        snapshots = {}
        for name, mask in [('position_discrepancy_0.1m', discrepancy >= .1),
                           ('pitch_0.1rad', np.abs(data['pitch']) >= .1),
                           ('pitch_0.3rad', np.abs(data['pitch']) >= .3),
                           ('fall_0.7rad', np.abs(data['pitch']) > config['fall_pitch'])]:
            indices = np.flatnonzero(mask)
            i = int(indices[0]) if len(indices) else None
            snapshots[name] = None if i is None else {key: float(data[key][i]) for key in ['time','x','y','true_x','true_y','pitch','v','goal_distance','true_distance']}
            if i is not None: snapshots[name]['position_discrepancy'] = float(discrepancy[i])
        estimates.append(dict(name=job['name'], success=measured['success'], fall=measured['fall'],
                              estimated_final_distance=measured['estimated_final_distance'], true_final_distance=measured['true_final_distance'],
                              absolute_distance_discrepancy=abs(measured['estimated_final_distance']-measured['true_final_distance']),
                              final_position_discrepancy=float(discrepancy[-1]),
                              distance_reconstruction_max_error=distance_error, odometry_reconstruction_max_error=odometry_error,
                              snapshots=snapshots))
    distribution = {'source': 'Frozen 100 paired D16 random goals; no changed scenario or controller.',
                    'trajectory_sampling_dt': .002, 'initial_estimated_pose': [0,0,0],
                    'rows': rows, 'bearing_bins': groups(rows, 'bearing_bin'), 'radius_bins': groups(rows, 'radius_bin')}
    distribution['subsets'] = {}
    for name, success in [('success', True), ('fall', False)]:
        subset = [row for row in rows if row['success'] == success]
        distribution['subsets'][name] = {key: statistics([row[key] for row in subset]) for key in
                                       ['radius','absolute_bearing','failure_time','saturation_duration','maximum_requested_torque','maximum_reference_yaw_rate']}
    distribution['bearing_radius_counts'] = [dict(bearing=bearing, radius=radius,
        count=sum(r['bearing_bin']==bearing and r['radius_bin']==radius for r in rows),
        successes=sum(r['success'] and r['bearing_bin']==bearing and r['radius_bin']==radius for r in rows))
        for bearing in ['front','front-side','rear-side','rear'] for radius in ['0.2–0.4','0.4–0.6','0.6–0.8','0.8–1.0']]
    save(output/'mpc_failure_distribution.json', distribution)
    subsets = {}
    for name, success in [('success',True), ('fall',False)]:
        subset = [row for row in estimates if row['success'] == success]
        subsets[name] = {'count': len(subset), **{key: statistics([row[key] for row in subset]) for key in
            ['estimated_final_distance','true_final_distance','absolute_distance_discrepancy','final_position_discrepancy']}}
    fallen = sorted([row for row in estimates if row['fall']], key=lambda row: row['absolute_distance_discrepancy'])
    selected = [fallen[i]['name'] for i in [0,len(fallen)//2,len(fallen)-1]]
    save(output/'estimator_postfall_analysis.json', {'subsets':subsets,'episodes':estimates,'representatives':selected,
         'discrepancy_before_pitch_0.1_count':sum(row['snapshots']['position_discrepancy_0.1m']['time'] < row['snapshots']['pitch_0.1rad']['time'] for row in fallen),
         'selection':'Minimum, median and maximum absolute final-distance discrepancy among fallen episodes.',
         'limitation':'Traces terminate on estimated pitch > 0.7 rad; they contain no post-termination motion or true pitch/contact logs.',
         'maximum_distance_reconstruction_error':max(r['distance_reconstruction_max_error'] for r in estimates),
         'maximum_odometry_reconstruction_error':max(r['odometry_reconstruction_max_error'] for r in estimates)})
    save(output/'source_hashes.json',hashes)
    plot_outcomes(distribution, figures)
    print(json.dumps({'bearing':distribution['bearing_bins'],'radius':distribution['radius_bins'],'distance':subsets,'replay':selected},indent=2))


def plot_outcomes(distribution, figures):
    rows = distribution['rows']
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':150})
    fig = plt.figure(figsize=(13,4.2),constrained_layout=True)
    axes = [fig.add_subplot(131),fig.add_subplot(132,projection='polar'),fig.add_subplot(133)]
    bins = ['front','front-side','rear-side','rear']
    counts = [distribution['bearing_bins'][name] for name in bins]
    axes[0].bar(bins,[100*x['successes']/x['count'] for x in counts],color='#2878a5')
    for i,row in enumerate(counts): axes[0].text(i,100*row['successes']/row['count']+3,f"{row['successes']}/{row['count']}",ha='center')
    axes[0].set(ylabel='MPC pipeline success (%)',ylim=(0,115),title='Absolute bearing bins (45° each)')
    for success,color,label in [(True,'#2878a5','Settled success'),(False,'#c34e3a','Estimated-pitch fall')]:
        subset = [row for row in rows if row['success']==success]
        axes[1].scatter([r['bearing'] for r in subset],[r['radius'] for r in subset],c=color,s=18,label=label)
    axes[1].set_theta_zero_location('E'); axes[1].set_ylim(0,1);axes[1].set_title('Initial robot-relative goals (m)')
    axes[1].legend(loc='lower center',bbox_to_anchor=(.5,-.34),fontsize=8)
    failures = [row for row in rows if not row['success']]
    points = axes[2].scatter([math.degrees(abs(r['bearing'])) for r in failures],[r['failure_time'] for r in failures],
                            c=[r['radius'] for r in failures],vmin=.2,vmax=1,cmap='viridis',s=25)
    axes[2].set(xlabel='Absolute goal bearing (degrees)',ylabel='Failure simulation time (s)',title=f'{len(failures)} failed episodes',xlim=(90,180))
    fig.colorbar(points,ax=axes[2],label='Goal radius (m)')
    fig.savefig(figures/'mpc_goal_outcomes.png');plt.close(fig)


def replay(root, output, figures, execute=True):
    analysis = json.loads((output/'estimator_postfall_analysis.json').read_text())
    directory = output/'replays'
    directory.mkdir(exist_ok=True)
    config = json.loads((root/'config.json').read_text())
    results = []
    fig, axes = plt.subplots(3,3,figsize=(13,9),constrained_layout=True)
    for column,name in enumerate(analysis['representatives']):
        trace, diagnostics = directory/(name+'.csv'), directory/(name+'_diagnostics.csv')
        command = ['ros2_ws/build/mujoco_backend/controller_benchmark',str(root/'nominal.xml'),config['actor'],
                   str(root/'goal_reaching'/('mpc_'+name+'.txt')),str(trace),str(diagnostics)]
        if execute: subprocess.run(command,check=True)
        data, extra = read_trace(trace), read_trace(diagnostics)
        original = read_trace(root/'goal_reaching'/('mpc_'+name+'.csv.gz'))
        if len(data)!=len(original): raise ValueError('Replay termination changed.')
        error = max(float(np.abs(data[key]-original[key]).max()) for key in data.dtype.names if key not in ['full_us','core_us'])
        if error > 1e-10: raise ValueError('Replay differs from frozen physical trace.')
        yaw_error = (extra['yaw']-extra['true_yaw']+np.pi)%(2*np.pi)-np.pi
        true_speed = np.hypot(np.diff(data['true_x']),np.diff(data['true_y']))/config['physics_dt']
        results.append({'name':name,'command':command,'non_timing_trace_max_error':error,
                        'final_yaw_error':float(yaw_error[-1]),'maximum_yaw_error':float(np.abs(yaw_error).max()),
                        'maximum_wheel_speed':float(max(np.abs(extra['left_wheel_velocity']).max(),np.abs(extra['right_wheel_velocity']).max())),
                        'final_estimated_velocity':float(data['v'][-1]),'final_true_planar_speed':float(true_speed[-1]),
                        'maximum_logged_reference_yaw_rate':float(np.abs(extra['reference_yaw_rate']).max()),
                        'maximum_logged_preview_end_yaw_rate':float(np.abs(extra['preview_yaw_rate']).max())})
        axes[0,column].plot(data['time'],data['x'],label='Estimated x',color='#2878a5')
        axes[0,column].plot(data['time'],data['true_x'],label='True x',color='#2878a5',linestyle='--')
        axes[0,column].plot(data['time'],data['y'],label='Estimated y',color='#c86c36')
        axes[0,column].plot(data['time'],data['true_y'],label='True y',color='#c86c36',linestyle='--')
        axes[0,column].set(title=name,ylabel='Position (m)')
        axes[1,column].plot(data['time'],extra['yaw'],label='Estimated yaw')
        axes[1,column].plot(data['time'],extra['true_yaw'],label='True yaw',linestyle='--')
        axes[1,column].plot(data['time'],data['pitch'],label='Estimated pitch')
        axes[1,column].set_ylabel('Wrapped angle (rad)')
        axes[2,column].plot(data['time'],data['applied_l'],label='Left torque')
        axes[2,column].plot(data['time'],data['applied_r'],label='Right torque')
        axes[2,column].set(xlabel='Simulation time (s)',ylabel='Applied torque (N m)',ylim=(-2.2,2.2))
        for axis in axes[:,column]:
            axis.axvline(data['time'][-1],color='black',linestyle=':',label='Termination' if axis==axes[0,column] else None)
            axis.grid(alpha=.2)
    for axis in axes[:,0]: axis.legend(fontsize=8)
    fig.savefig(figures/'mpc_odometry_audit.png');plt.close(fig)
    save(output/'replay_validation.json',results)


def timing(output):
    for package in ['mujoco_backend','controller']:
        cache = (Path('ros2_ws/build')/package/'CMakeCache.txt').read_text()
        if 'CMAKE_BUILD_TYPE:STRING=Release' not in cache: raise RuntimeError('Release required.')
    executable = 'ros2_ws/build/mujoco_backend/benchmark_audit_probe'
    calibration = []
    for size in [1000,10000,100000]:
        path = output/f'lqr_calibration_{size}.csv'
        subprocess.run([executable,'lqr',str(size),'100','20','0',str(path)],check=True)
        data = read_trace(path)
        calibration.append({'updates':size,'batch_us':statistics(data['total_us']),'per_update_us':statistics(data['per_update_us'])})
        # A 20 microsecond batch is more than 400 of the previously observed 0.042 us clock steps.
        if np.median(data['total_us']) >= 20: break
    size = calibration[-1]['updates']
    if calibration[-1]['batch_us']['median'] < 20: raise RuntimeError('Batches remain too short.')
    path = output/'lqr_batches.csv'
    command = [executable,'lqr',str(size),'2000','100','0',str(path)]
    subprocess.run(command,check=True)
    data = read_trace(path)
    if np.min(data['total_us']) <= 0 or len(np.unique(data['checksum'])) < 2: raise RuntimeError('Invalid timing/checksum.')
    save(output/'lqr_batched_timing.json',{'command':command,'updates_per_batch':size,'batches':len(data),'warmup_batches':100,
        'calibration':calibration,'per_update_us':statistics(data['per_update_us']),'batch_us':statistics(data['total_us']),
        'minimum_batch_us':float(data['total_us'].min()),'raw_sha256':digest(path),
        'measurement':'Batch mean per production call, including indexed input loads and output stores; checksum and CSV I/O outside timer. Not individual-call latency percentiles.',
        'machine':{'compiler':subprocess.check_output(['c++','--version'],text=True),'cpu':subprocess.check_output(['sysctl','-n','machdep.cpu.brand_string'],text=True).strip(),
                   'build_type':'Release','pixi_lock_sha256':digest('pixi.lock')}})
    print((output/'lqr_batched_timing.json').read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode',choices=['analyze','replay','timing'])
    parser.add_argument('--input',default='artifacts/benchmark/d16')
    args = parser.parse_args()
    root = Path(args.input)
    output = root/'audit'
    output.mkdir(exist_ok=True)
    figures = Path('docs/figures/benchmark')
    figures.mkdir(parents=True,exist_ok=True)
    if args.mode == 'analyze': analyze(root,output,figures)
    elif args.mode == 'replay': replay(root,output,figures)
    else: timing(output)


if __name__ == '__main__': main()
