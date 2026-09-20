import math

import gymnasium as gym
import mujoco
import numpy as np

from .state import ROOT, EstimatedState, heading_potential, load_config, observation, reward_terms, torques


class GoalEnvironment(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, stage='A', config=None):
        self.config = config or load_config()
        self.stage_name = stage
        self.stage = self.config['stages'][stage]
        if mujoco.__version__ != self.config['mujoco_version']:
            raise RuntimeError('Training MuJoCo version differs from the configured plant version.')
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / self.config['model']))
        self.data = mujoco.MjData(self.model)
        if abs(self.model.opt.timestep - self.config['physics_dt']) > 1e-12:
            raise ValueError('Unexpected plant timestep.')
        self.dt = self.config['physics_dt'] * self.config['action_steps']
        self.addresses = {name: int(self.model.sensor(name).adr[0]) for name in
                          ['imu_gyro', 'imu_accel', 'left_wheel_pos', 'right_wheel_pos',
                           'left_wheel_vel', 'right_wheel_vel']}
        self.motors = [self.model.actuator(name).id for name in ['left_wheel_motor', 'right_wheel_motor']]
        self.body = self.model.body('base_body').id
        self.state = EstimatedState(self.config)
        clip = self.config['observation_clip']
        self.observation_space = gym.spaces.Box(-clip, clip, (6,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
        self.done = True

    def read_estimate(self):
        a, sensors = self.addresses, self.data.sensordata
        self.state.update(self.data.time, sensors[a['imu_gyro']:a['imu_gyro']+3],
                          sensors[a['imu_accel']:a['imu_accel']+3],
                          sensors[a['left_wheel_pos']], sensors[a['right_wheel_pos']],
                          sensors[a['left_wheel_vel']], sensors[a['right_wheel_vel']])

    def truth(self):
        rotation = self.data.xmat[self.body].reshape(3, 3)
        pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
        return self.data.xpos[self.body, :2].copy(), pitch

    def truth_yaw(self):
        rotation = self.data.xmat[self.body].reshape(3, 3)
        return math.atan2(rotation[1, 0], rotation[0, 0])

    def goal_bearing(self, position):
        return math.atan2(self.goal[1] - position[1], self.goal[0] - position[0]) - self.truth_yaw()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        mujoco.mj_resetData(self.model, self.data)
        pitch = float(options.get('pitch', self.np_random.uniform(-self.stage['pitch_range'], self.stage['pitch_range'])))
        velocity = self.np_random.uniform(-self.stage['velocity_range'], self.stage['velocity_range'])
        self.data.qpos[3:7] = [math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0]
        self.data.qvel[0] = velocity
        for name in ['left_wheel_hinge', 'right_wheel_hinge']:
            self.data.qvel[self.model.joint(name).dofadr[0]] = velocity / self.config['wheel_radius']
        mujoco.mj_forward(self.model, self.data)
        self.state.reset()
        self.read_estimate()
        radius = self.np_random.uniform(*self.stage['goal_radius'])
        if self.stage.get('axis_only'):
            bearing = self.np_random.choice([0.0, math.pi])
        elif 'bearing_choices' in self.stage:
            bearing = self.np_random.choice(self.stage['bearing_choices'])
        else:
            bearing = self.np_random.uniform(*self.stage['bearing'])
        self.goal = np.asarray(options.get('goal', [radius * math.cos(bearing), radius * math.sin(bearing)]), dtype=float)
        if self.goal.shape != (2,) or not np.isfinite(self.goal).all():
            raise ValueError('Invalid episode goal.')
        position, _ = self.truth()
        self.previous_position = position
        self.previous_distance = float(np.linalg.norm(self.goal - position))
        self.previous_bearing = self.goal_bearing(position)
        self.initial_distance = self.previous_distance
        self.steps = self.hold_steps = 0
        self.episode_return = self.path_length = self.pitch_squared = 0.0
        self.absolute_torque = self.squared_torque = self.maximum_pitch = self.maximum_torque = 0.0
        self.done = False
        return observation(self.state, self.goal, self.config), {}

    def step(self, action):
        if self.done:
            raise RuntimeError('Reset is required before stepping a finished episode.')
        torque = torques(action, self.config)
        self.data.ctrl[self.motors] = torque
        for _ in range(self.config['action_steps']):
            mujoco.mj_step(self.model, self.data)
            self.read_estimate()
        self.steps += 1
        elapsed = self.steps * self.dt
        estimated_distance = math.hypot(self.goal[0]-self.state.x, self.goal[1]-self.state.y)
        limits = self.config['success']
        settled = (estimated_distance < limits['radius'] and abs(self.state.velocity) < limits['velocity']
                   and abs(self.state.yaw_rate) < limits['yaw_rate'] and abs(self.state.pitch) < limits['pitch'])
        self.hold_steps = self.hold_steps + 1 if settled else 0

        # Truth is used only for training reward, failure detection, and diagnostics.
        position, pitch = self.truth()
        distance = float(np.linalg.norm(self.goal - position))
        bearing = self.goal_bearing(position)
        fallen = abs(pitch) > self.config['fall_angle']
        success = (not fallen and self.hold_steps * self.dt >= limits['hold_seconds']
                   and elapsed >= self.stage['minimum_success_seconds'])
        heading_progress = math.cos(bearing) - math.cos(self.previous_bearing)
        turn_alignment = math.sin(bearing) * self.state.yaw_rate * self.dt
        terms = reward_terms(self.previous_distance, distance, pitch, torque, self.dt, success, fallen,
                             self.config, self.state.yaw_rate, heading_progress, turn_alignment)
        if self.config.get('reward_version') == 'potential_v1':
            previous = heading_potential(self.previous_distance, self.previous_bearing, self.config)
            current = 0.0 if success or fallen else heading_potential(distance, bearing, self.config)
            terms['heading_potential'] = self.config['ppo']['gamma'] * current - previous
        reward = sum(terms.values())
        obs = observation(self.state, self.goal, self.config)
        if not np.isfinite(reward) or not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError('Non-finite simulation or reward.')
        self.path_length += float(np.linalg.norm(position - self.previous_position))
        self.previous_position, self.previous_distance, self.previous_bearing = position, distance, bearing
        self.episode_return += reward
        self.pitch_squared += pitch * pitch * self.dt
        self.maximum_pitch = max(self.maximum_pitch, abs(pitch))
        self.absolute_torque += float(np.abs(torque).sum()) * self.dt
        self.squared_torque += float(np.dot(torque, torque)) * self.dt
        self.maximum_torque = max(self.maximum_torque, float(np.abs(torque).max()))
        terminated = bool(success or fallen)
        truncated = bool(not terminated and elapsed + 1e-12 >= self.stage['episode_seconds'])
        self.done = terminated or truncated
        info = {'reward_terms': terms}
        if self.done:
            info['metrics'] = {
                'success': bool(success), 'fall': bool(fallen), 'time': elapsed,
                'time_to_success': elapsed if success else None,
                'final_distance': distance, 'estimated_final_distance': estimated_distance,
                'truth_inside_goal': bool(distance < limits['radius']),
                'return': self.episode_return, 'maximum_pitch': self.maximum_pitch,
                'rms_pitch': math.sqrt(self.pitch_squared / elapsed),
                'absolute_torque': self.absolute_torque, 'squared_torque': self.squared_torque,
                'maximum_torque': self.maximum_torque, 'path_length': self.path_length,
                'straight_line_distance': self.initial_distance,
                'path_efficiency': self.initial_distance / self.path_length if self.path_length > 1e-9 else None,
                'goal': self.goal.tolist(),
            }
        return obs, reward, terminated, truncated, info
