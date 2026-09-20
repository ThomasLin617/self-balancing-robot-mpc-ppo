import numpy as np


def statistics(values):
    values = np.asarray([x for x in values if x is not None], dtype=float)
    if len(values) == 0:
        return dict(count=0, mean=None, median=None, std=None, p95=None, p99=None, maximum=None)
    if not np.isfinite(values).all():
        raise ValueError('Non-finite metric cannot enter the report.')
    return dict(count=len(values), mean=float(values.mean()), median=float(np.median(values)),
                std=float(values.std()), p95=float(np.percentile(values,95)),
                p99=float(np.percentile(values,99)), maximum=float(values.max()))


def settling_time(time, inside, hold, start=0.0, final=False):
    indices = np.flatnonzero(time >= start)
    if not len(indices): return None
    if final:
        outside = indices[~inside[indices]]
        first = int(outside[-1]+1) if len(outside) else int(indices[0])
        return float(time[first]-start) if first < len(time) and time[-1]-time[first] >= hold else None
    began = None
    for i in indices:
        if not inside[i]: began = None
        elif began is None: began = time[i]
        elif time[i]-began+1e-12 >= hold: return float(time[i]-start)
    return None


def measure(data, job, config):
    if len(data) == 0: raise ValueError('Empty trace.')
    t, dt = data['time'], config['physics_dt']
    if not np.allclose(np.diff(t),dt,atol=1e-9): raise ValueError('Unexpected trace cadence.')
    for name in data.dtype.names:
        if not np.isfinite(data[name]).all(): raise ValueError('Non-finite trace.')
    torque = np.column_stack([data['applied_l'],data['applied_r']])
    if np.abs(torque).max() > config['torque_limit']+1e-9: raise ValueError('Applied torque out of bounds.')
    pitch = data['pitch']
    path = np.column_stack([data['true_x'],data['true_y']])
    length = float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum() + np.linalg.norm(path[0]))
    success = bool(data['success'][-1]) and not bool(data['fall'][-1])
    if job['group'] == 'disturbance': success = success and t[-1] >= config['disturbance']['time']+config['disturbance']['duration']
    fallen = bool(data['fall'][-1])
    error = data['p']-job['goal'][0]
    hold = config['success']['hold']
    result = dict(success=success, fall=fallen, timeout=not success and not fallen,
                  duration=float(t[-1]), time_to_success=float(t[-1]) if success else None,
                  estimated_final_distance=float(data['goal_distance'][-1]), true_final_distance=float(data['true_distance'][-1]),
                  truth_inside_goal=bool(data['true_distance'][-1] < config['success']['distance']),
                  path_length=length, straight_line_distance=float(np.linalg.norm(job['goal'])),
                  path_efficiency=float(np.linalg.norm(job['goal'])/length) if length > 1e-9 else None,
                  pitch_rmse=float(np.sqrt(np.mean(pitch**2))), maximum_pitch=float(np.abs(pitch).max()),
                  position_rmse=float(np.sqrt(np.mean(error**2))), position_final_error=float(abs(error[-1])),
                  velocity_rmse=float(np.sqrt(np.mean(data['v']**2))),
                  requested_torque=float(max(np.abs(data['requested_l']).max(),np.abs(data['requested_r']).max())),
                  applied_torque=float(np.abs(torque).max()), absolute_torque=float(np.abs(torque).sum()*dt),
                  squared_torque=float((torque**2).sum()*dt),
                  saturation_duration=float(np.any(np.abs(torque)>=config['torque_limit']-1e-6,axis=1).sum()*dt),
                  position_settling_time=settling_time(t,np.abs(error)<config['sagittal']['position_band'],hold,final=True))
    end = config['disturbance']['time']+config['disturbance']['duration']
    post = t >= config['disturbance']['time']
    if job['force'] != 0:
        before = np.flatnonzero(t < config['disturbance']['time'])
        baseline = data['goal_distance'][before[-1]] if len(before) else np.linalg.norm(job['goal'])
        result.update(post_push_peak_pitch=float(np.abs(pitch[post]).max()) if post.any() else None,
                      post_push_distance_increase=float(max(0,np.max(data['goal_distance'][post])-baseline)) if post.any() else None,
                      pitch_recovery_time=settling_time(t,np.abs(pitch)<config['sagittal']['pitch_band'],hold,start=end),
                      disturbance_recovery_time=settling_time(t,(np.abs(error)<config['sagittal']['position_band']) &
                                                            (np.abs(pitch)<config['sagittal']['pitch_band']),hold,start=end)
                      if job['group']=='sagittal' else (float(t[-1]-end) if success else None))
    return result
