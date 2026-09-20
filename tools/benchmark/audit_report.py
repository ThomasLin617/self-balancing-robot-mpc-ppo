import argparse
import json
from pathlib import Path

from .report import number, table
from .run_benchmark import digest
from .audit import plot_outcomes, replay as plot_replays


def replace_block(path, fragment):
    start, end = '<!-- D16_AUDIT_BEGIN -->', '<!-- D16_AUDIT_END -->'
    text = path.read_text()
    if start not in text or end not in text: raise ValueError(f'Audit markers missing: {path}')
    path.write_text(text.split(start)[0]+start+'\n\n'+fragment+'\n\n'+end+text.split(end,1)[1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input',default='artifacts/benchmark/d16')
    args = parser.parse_args()
    root = Path(args.input)/'audit'
    names = ['mpc_failure_distribution','estimator_postfall_analysis','lqr_batched_timing','replay_validation']
    values = {name:json.loads((root/(name+'.json')).read_text()) for name in names}
    distribution, estimator, timing, replay = [values[name] for name in names]
    figures = Path('docs/figures/benchmark')
    plot_outcomes(distribution, figures)
    plot_replays(Path(args.input),root,figures,execute=False)
    values['sources'] = {str(root/(name+'.json')):digest(root/(name+'.json')) for name in names}
    values['analysis_sources'] = {str(p):digest(p) for p in [Path(__file__),Path(__file__).with_name('audit.py'),Path(__file__).with_name('audit_probe.cpp')]}
    Path('docs/benchmark_audit_results.json').write_text(json.dumps(values,indent=2,allow_nan=False)+'\n')
    bins = ['front','front-side','rear-side','rear']
    bounds = ['0–45°','>45–90°','>90–135°','>135–180°']
    bearing_table = table(['Absolute initial bearing','MPC settled successes','Estimated-pitch falls'],
        [[bound,f"{distribution['bearing_bins'][name]['successes']}/{distribution['bearing_bins'][name]['count']}",distribution['bearing_bins'][name]['falls']]
         for bound,name in zip(bounds,bins)])
    distance_table = table(['MPC subset','Episodes','Mean estimated distance (m)','Mean true distance (m)','Mean absolute distance discrepancy (m)'],
        [[name,row['count'],*[number(row[key]['mean']) for key in ['estimated_final_distance','true_final_distance','absolute_distance_discrepancy']]]
         for name,row in estimator['subsets'].items()])
    concise = '\n\n'.join([bearing_table,
        'Failures are concentrated in rear-side/rear goals. All failed trials reach the wheel clamp; successful trials do not. This is the measured behavior of the existing trajectory-plus-MPC pipeline on this distribution, not an isolated diagnosis of MPC feedback.',
        distance_table,
        'The distance formulas reproduce the raw logs exactly. Reintegrating recorded velocity/yaw rate reproduces odometry to '+number(estimator['maximum_odometry_reconstruction_error'])+' m. The large aggregate discrepancy is dominated by failed episodes: wheel/gyro odometry and true position separate **before termination**. '+str(estimator['discrepancy_before_pitch_0.1_count'])+' of 39 failed trials exceed 0.10 m pose discrepancy before estimated pitch first exceeds 0.10 rad. There is no post-termination trace, true-pitch trace or contact log establishing a particular physical fall/slip mechanism. These findings support breakdown of the low-slip/upright odometry assumptions during the failure transient, not a reporting or stale-state bug. Original outcomes and distances are retained.'])
    replace_block(Path('README.md'),concise+'\n\n![MPC goal outcome audit](docs/figures/benchmark/mpc_goal_outcomes.png)')
    radius_table = table(['Goal radius (m)','MPC settled successes','Falls'],
        [[name,f"{row['successes']}/{row['count']}",row['falls']] for name,row in sorted(distribution['radius_bins'].items())])
    cross_table = table(['Absolute bearing / radius','0.2–0.4 m','0.4–0.6 m','0.6–0.8 m','0.8–1.0 m'],
        [[name]+[f"{r['successes']}/{r['count']}" for r in distribution['bearing_radius_counts'] if r['bearing']==name] for name in bins])
    selected = estimator['representatives']
    trajectory_table = table(['Replay','Bearing (°)','Radius (m)','Failure (s)','Trajectory duration (s)','Initial yaw (rad)','First preview end yaw (rad)','Peak sampled yaw-rate reference (rad/s)','Saturated duration (s)'],
        [[r['name'],number(r['bearing']*180/3.141592653589793),number(r['radius']),number(r['failure_time']),number(r['trajectory_duration']),number(r['initial_reference_yaw']),number(r['first_preview_end_yaw']),number(r['maximum_reference_yaw_rate']),number(r['saturation_duration'])]
         for r in distribution['rows'] if r['name'] in selected])
    state_table = table(['Replay','Pose discrepancy reaches 0.1 m (s)','Pitch then (rad)','Final estimated v (m/s)','Final true planar speed (m/s)','Final wrapped yaw error (rad)','Max wheel speed (rad/s)','Replay max trace error'],
        [[r['name'],*[number(next(e for e in estimator['episodes'] if e['name']==r['name'])['snapshots']['position_discrepancy_0.1m'][key]) for key in ['time','pitch']],
          *[number(r[key]) for key in ['final_estimated_velocity','final_true_planar_speed','final_yaw_error','maximum_wheel_speed','non_timing_trace_max_error']]] for r in replay])
    calibration_table = table(['Calls per batch','Median batch duration (µs)','Median per-call batch average (µs)'],
        [[r['updates'],number(r['batch_us']['median']),number(r['per_update_us']['median'])] for r in timing['calibration']])
    timing_table = table(['Batch-mean per-call unit','Mean','Median','p95','p99','Maximum'],
        [['µs',*[number(timing['per_update_us'][key]) for key in ['mean','median','p95','p99','maximum']]]])
    long = '\n\n'.join([concise,'#### Radius and bearing together',radius_table,cross_table,
        'Radius-only counts are not monotonic and mix different bearings. The joint table retains that confounding instead of claiming radius is irrelevant or causal.',
        '#### Unchanged production trajectory and diagnostic replays',trajectory_table,
        'Every initial reference heading is zero because the Hermite start tangent follows initial estimated yaw. Large rearward goals do not produce an immediate initial yaw jump; the tight heading reversal occurs later. The obstacle-free D12/D13 generator uses a longitudinal trapezoid, unlike the separate D14 curvature-aware planner. The peak references above are from the complete trajectory sampled at 2 ms, not a continuous maximum or necessarily a value already applied by the failure time. Per-episode JSON also records the peak visible within the episode plus its 0.5 s preview. Preview sees the demand early, but these correlated signals do not isolate geometry, estimator, contact dynamics or feedback as the sole cause.',
        state_table,
        'Replay selection is deterministic: smallest, median and largest absolute final-distance discrepancy among the 39 failed trials. Original non-timing columns and termination times match the replays exactly. Additional yaw/reference/wheel logs are diagnostic only. Reference columns are sampled at the logged post-step timestamp and its +0.5 s horizon end; they are not the previous held action timestamp. True planar speed is the magnitude of a 1 ms finite difference, not signed forward velocity.',
        '![MPC bearing/radius outcomes](figures/benchmark/mpc_goal_outcomes.png)',
        '![Representative odometry and command traces](figures/benchmark/mpc_odometry_audit.png)',
        '#### LQR batch calibration and measurement',calibration_table,timing_table,
        f"Selected K={timing['updates_per_batch']} by requiring median calibration batch duration ≥20 µs; retained {timing['batches']} batches after {timing['warmup_batches']} warm-up batches. Minimum retained batch duration is {number(timing['minimum_batch_us'])} µs. Selection uses duration, not a favorable result. Each call reads a different prebuilt state/reference and stores its returned command. One state also changes between batches. Allocation/input generation and checksum/CSV writing are outside the timer. A recorded checksum consumes all torque outputs. Release disassembly shows a call to production LQR inside the timed loop, so there is no deleted or hoisted command evaluation.",
        'The batch contains sequential independent calls plus indexed loads/stores; cache residency, memory traffic, loop overhead and throughput differ from an isolated 500 Hz call. No empty-loop time is subtracted. Division by K amortizes two clock reads. p95/p99/max describe the distribution of batch means, not single-call tails, jitter or a hard real-time bound. MPC and PPO timings are the original frozen measurements. Original quantized LQR CSVs remain preserved; only the displayed LQR timing method is replaced.'])
    (root/'audit_summary.md').write_text(long+'\n')
    replace_block(Path('docs/D16_UNIFIED_BENCHMARK.md'),long)
    notebook = long.replace('](figures/','](../figures/')
    replace_block(Path('docs/learning/PPO_DEVELOPMENT_HISTORY.md'),notebook)
    print('Updated audit tables, figures links and traceable JSON snapshot.')


if __name__ == '__main__': main()
