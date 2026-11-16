import sys
from pathlib import Path
import numpy as np
from tools.learning.state import ROOT, wrap

sys.path.insert(0, str(ROOT/'build/trajectory_learning'))
from _trajectory import Trajectory, KnownMap, World, StraightReference

SCALES = np.array([1.0, 1.0, 0.35, 2.0, np.pi, 2.0])
REFERENCE_SCALES = SCALES[[0, 1, 2, 4, 5]]


def observation(state, horizon):
    refs = np.array(horizon, dtype=np.float64, copy=True)
    if refs.shape != (250, 5) or not np.isfinite(refs).all():
        raise ValueError('Expected a finite 250 by 5 reference horizon.')
    current = np.array([0.0, state.velocity, state.pitch, state.pitch_rate, 0.0, state.yaw_rate])
    if not np.isfinite(current).all() or not np.isfinite([state.position, state.yaw]).all():
        raise ValueError('Non-finite estimated state.')
    refs[:, 0] -= state.position
    yaw = np.r_[state.yaw, refs[:, 3]]
    refs[:, 3] = np.cumsum(wrap(np.diff(yaw)))
    return np.clip(np.r_[current/SCALES, (refs/REFERENCE_SCALES).ravel()], -5, 5).astype(np.float32)


def reward(state, reference, origin, true_pitch, torque, success, failure, dt=0.002,
           tracking_scales=None):
    scales = np.asarray(tracking_scales if tracking_scales is not None else
                        [0.10,0.20,0.10,1.0,0.35,0.50])
    if scales.shape!=(6,) or not np.isfinite(scales).all() or np.any(scales<=0):
        raise ValueError('Expected six finite positive tracking reward scales.')
    errors = np.array([state.position-origin-reference[2],
                       state.velocity-reference[3],true_pitch,
                       state.pitch_rate,wrap(state.yaw-reference[5]),
                       state.yaw_rate-reference[6]])/scales
    cost = float(errors@errors + 0.02*np.dot(torque, torque))
    value = dt*(1.0-cost) + 30.0*success - 20.0*failure
    if not np.isfinite(value):
        raise ValueError('Non-finite trajectory reward.')
    return value
