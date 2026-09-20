import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load_config(path=None):
    with open(path or Path(__file__).with_name('config.json')) as stream:
        return json.load(stream)


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class EstimatedState:
    def __init__(self, config):
        self.radius = config['wheel_radius']
        self.time_constant = config['pitch_filter_time_constant']
        self.reset()

    def reset(self):
        self.initialized = False
        self.pitch = self.yaw = self.x = self.y = 0.0
        self.position = self.velocity = self.pitch_rate = self.yaw_rate = 0.0
        self.timestamp = self.origin = 0.0

    def update(self, timestamp, gyro, accel, left_position, right_position, left_velocity, right_velocity):
        values = [timestamp, *gyro, *accel, left_position, right_position, left_velocity, right_velocity]
        if not np.isfinite(values).all():
            raise ValueError('Non-finite sensor trace.')
        previous_yaw, previous_velocity = self.yaw, self.velocity
        if not self.initialized:
            self.origin = self.radius * (left_position + right_position) / 2.0
            self.initialized = True
            dt = 0.0
        else:
            dt = timestamp - self.timestamp
            if dt <= 0.0:
                raise ValueError('Estimator dt <= 0.')
            alpha = self.time_constant / (self.time_constant + dt)
            self.pitch = alpha * (self.pitch + gyro[1] * dt) + (1.0 - alpha) * math.atan2(-accel[0], accel[2])
            self.yaw = wrap(self.yaw + gyro[2] * dt)
        self.timestamp = timestamp
        self.pitch_rate, self.yaw_rate = gyro[1], gyro[2]
        self.position = self.radius * ((left_position + right_position) / 2.0 + self.pitch) - self.origin
        self.velocity = self.radius * ((left_velocity + right_velocity) / 2.0 + self.pitch_rate)
        midpoint = previous_yaw + 0.5 * wrap(self.yaw - previous_yaw)
        distance = 0.5 * (previous_velocity + self.velocity) * dt
        self.x += distance * math.cos(midpoint)
        self.y += distance * math.sin(midpoint)
        return self

    def vector(self):
        return np.array([self.position, self.velocity, self.pitch, self.pitch_rate,
                         self.yaw, self.yaw_rate, self.x, self.y])


def observation(state, goal, config):
    dx, dy = goal[0] - state.x, goal[1] - state.y
    c, s = math.cos(state.yaw), math.sin(state.yaw)
    raw = np.array([c * dx + s * dy, -s * dx + c * dy,
                    state.velocity, state.pitch, state.pitch_rate, state.yaw_rate])
    if not np.isfinite(raw).all():
        raise ValueError('Non-finite policy observation.')
    return np.clip(raw / config['observation_scales'],
                   -config['observation_clip'], config['observation_clip']).astype(np.float32)


def torques(action, config):
    action = np.asarray(action, dtype=np.float64)
    if action.shape != (2,) or not np.isfinite(action).all():
        raise ValueError('Expected two finite policy actions.')
    return config['torque_limit'] * np.clip(action, -1.0, 1.0)


def heading_potential(distance, bearing, config):
    radius = config['success']['radius']
    scale = config['reward']['heading_distance_scale']
    outside = max(0.0, distance - radius)
    fade = outside * outside / (outside * outside + scale * scale)
    return -config['reward']['heading_potential'] * fade * (1.0 - math.cos(bearing))


def reward_terms(previous_distance, distance, pitch, torque, dt, success, fallen, config, yaw_rate=0.0, heading_progress=0.0, turn_alignment=0.0):
    weights = config['reward']
    return {
        'progress': weights['progress'] * (previous_distance - distance),
        'upright': -weights['pitch'] * pitch * pitch * dt,
        'rotation': -weights.get('yaw_rate', 0.0) * yaw_rate * yaw_rate * dt,
        'heading': weights.get('heading', 0.0) * heading_progress,
        'turn_alignment': weights.get('turn_alignment', 0.0) * turn_alignment,
        'effort': -weights['torque'] * float(np.dot(torque, torque)) * dt,
        'success': weights['success'] if success else 0.0,
        'fall': weights['fall'] if fallen else 0.0,
    }
