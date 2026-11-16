import math
import tempfile
from pathlib import Path
import numpy as np
import gymnasium as gym
import mujoco
from tools.learning.environment import GoalEnvironment
from tools.learning.state import load_config, torques
from .state import Trajectory, StraightReference, KnownMap, World, ROOT, observation, reward


class TrajectoryEnvironment(GoalEnvironment):
    def __init__(self, stage='A', curriculum_config=None):
        if stage not in ['A', 'B', 'C', 'C0', 'C1', 'C2', 'C_full', 'D', 'E']:
            raise ValueError('Obstacle curriculum is gated on validated A/B/C learning.')
        config = load_config()
        self.world = self.known_map = None
        if stage in ['D','E']:
            self.known_map = KnownMap.load(str(ROOT/(curriculum_config or {}).get('scenario','model/known_map.txt')))
            self.world = World(str(ROOT/config['model']),self.known_map)
            config['model'] = self.world.path
        super().__init__('A', config)
        self.obstacle_ids = {i for i in range(self.model.ngeom) if (self.model.geom(i).name or '').startswith('known_')}
        self.curriculum = stage
        self.curriculum_config = curriculum_config or {}
        self.observation_space = gym.spaces.Box(-5.0, 5.0, (1256,), np.float32)

    def preview(self):
        return self.trajectory.horizon(self.steps*self.dt, self.position_origin)

    def reset(self, *, seed=None, options=None):
        if self.curriculum == 'E':
            from .scenarios import layout
            rng=np.random.default_rng(seed) if seed is not None else self.np_random
            family=['wall','staggered','elbow','corridor'][int(rng.integers(4))]
            for _ in range(2000):
                text,goal=layout(rng,family)
                with tempfile.TemporaryDirectory() as directory:
                    path=Path(directory)/'scenario.txt';path.write_text(text)
                    known=KnownMap.load(str(path))
                try:
                    trajectory=known.plan(0,0,0,*goal)
                    if known.direct_free(0,0,*goal) or not 1<=trajectory.sample(trajectory.duration)[2]<=4 or trajectory.duration>90:
                        continue
                except (RuntimeError,ValueError,IndexError):
                    continue
                self.known_map=known
                self.world=World(str(ROOT/'model/Robot1.xml'),known)
                self.model=mujoco.MjModel.from_xml_path(self.world.path)
                self.data=mujoco.MjData(self.model)
                self.obstacle_ids={i for i in range(self.model.ngeom) if (self.model.geom(i).name or '').startswith('known_')}
                break
            else:raise RuntimeError('No feasible training map within deterministic candidate budget.')
        super().reset(seed=seed, options=options)
        self.position_origin = self.state.position
        category = int(seed if seed is not None else self.np_random.integers(100000))
        self.direction = -1 if category % 2 else 1
        self.turn_only = False
        self.task_kind = 0
        mixed = self.curriculum in ['C0', 'C1', 'C2', 'D', 'E']
        if self.curriculum == 'C_full':
            low,high = self.curriculum_config['bearing']
            bearing = self.direction*self.np_random.uniform(low,high)
            self.turn_only = category % 4 < 2
            distance = 0.0 if self.turn_only else self.np_random.uniform(0.2,0.6)
            self.goal = distance*np.array([math.cos(bearing),math.sin(bearing)])
            self.trajectory = (Trajectory.turn(bearing) if self.turn_only else
                               Trajectory.turn_then_move(bearing,distance))
            self.task_kind = 3 if self.turn_only else 4
        elif self.curriculum in ['D','E']:
            self.goal = np.array(self.known_map.goal)
            self.trajectory = self.known_map.plan(self.state.x,self.state.y,self.state.yaw,*self.goal)
        elif self.curriculum == 'A' or (mixed and category % 10 < 2):
            self.goal = np.array([self.state.x, self.state.y])
            self.trajectory = Trajectory.hold(self.state.yaw)
        elif mixed and category % 10 < 4:
            distance = self.direction*self.np_random.uniform(0.2,0.6)
            self.goal = np.array([distance,0.0])
            self.trajectory = StraightReference(distance)
            self.task_kind = 1
        elif mixed:
            radius = self.np_random.uniform(0.2,0.6)
            low,high = self.curriculum_config['bearing']
            bearing = self.direction*self.np_random.uniform(low,high)
            self.goal = np.array([radius*math.cos(bearing),radius*math.sin(bearing)])
            curve = Trajectory.profiled_curve if self.curriculum_config.get('profiled',False) else Trajectory.point_to_point
            self.trajectory = curve(0,0,self.state.yaw,*self.goal)
            self.task_kind = 2
        else:
            radius = self.np_random.uniform(0.2, 0.6)
            side = -1 if (seed if seed is not None else self.np_random.integers(100000)) % 2 else 1
            turn_only = self.curriculum == 'C' and (seed if seed is not None else self.np_random.integers(100000)) % 4 < 2
            bearing = self.np_random.choice([0.0, math.pi]) if self.curriculum == 'B' else side*self.np_random.uniform(0.3,1.5)
            self.goal = np.array([self.state.x+radius*math.cos(bearing), self.state.y+radius*math.sin(bearing)])
            if turn_only:
                self.goal = np.array([self.state.x,self.state.y])
                self.trajectory = Trajectory.turn(bearing)
            else:
                curve = Trajectory.profiled_curve if self.curriculum_config.get('profiled',False) else Trajectory.point_to_point
                self.trajectory = (StraightReference(float(self.goal[0])) if self.curriculum == 'B' else
                                   curve(self.state.x,self.state.y,self.state.yaw,*self.goal))
            self.direction = side
            self.turn_only = turn_only
        self.timeout = self.trajectory.duration+10.0
        self.maximum_pitch = self.maximum_torque = 0.0
        return observation(self.state,self.preview()), {}

    def step(self, action):
        if self.done:
            raise RuntimeError('Reset a finished episode before stepping.')
        torque = torques(action,self.config)
        self.data.ctrl[self.motors] = torque
        collision = boundary = False
        for _ in range(2):
            mujoco.mj_step(self.model,self.data)
            self.read_estimate()
            if self.known_map is not None:
                collision = collision or any(int(c.geom1) in self.obstacle_ids or int(c.geom2) in self.obstacle_ids for c in self.data.contact)
                xy,_ = self.truth()
                try:
                    boundary = boundary or not self.known_map.inside(*xy)
                except IndexError:
                    boundary = True
        self.steps += 1
        elapsed = self.steps*self.dt
        position,pitch = self.truth()
        fallen = abs(pitch)>0.7
        failure = fallen or collision or boundary
        distance = float(np.linalg.norm(self.goal-[self.state.x,self.state.y]))
        settled = (elapsed>=self.trajectory.duration and distance<0.1 and abs(self.state.velocity)<0.05
                   and abs(self.state.yaw_rate)<0.1 and abs(self.state.pitch)<0.1)
        if self.curriculum == 'C_full':
            yaw_error = (self.state.yaw-self.trajectory.sample(elapsed)[5]+math.pi)%(2*math.pi)-math.pi
            settled = settled and distance<self.curriculum_config['settled_radius'] and abs(yaw_error)<0.05
        self.hold_steps = self.hold_steps+1 if settled else 0
        success = not failure and self.hold_steps*self.dt>=0.5
        value = reward(self.state,self.trajectory.sample(elapsed),self.position_origin,pitch,torque,success,failure,
                       tracking_scales=self.curriculum_config.get('tracking_scales'))
        truncated = not (success or failure) and elapsed>=self.timeout
        self.done = success or failure or truncated
        self.episode_return += value
        self.maximum_pitch = max(self.maximum_pitch,abs(pitch))
        self.maximum_torque = max(self.maximum_torque,float(np.max(np.abs(torque))))
        info = {}
        if self.done:
            info['metrics'] = dict(success=bool(success),fall=bool(fallen),timeout=bool(truncated),
                final_distance=distance,true_final_distance=float(np.linalg.norm(self.goal-position)),
                maximum_pitch=self.maximum_pitch,maximum_torque=self.maximum_torque,
                time=elapsed,episode_return=self.episode_return)
            if self.curriculum in ['D','E']:
                info['metrics'].update(collision=bool(collision),boundary=bool(boundary))
            if self.curriculum.startswith('C'):
                info['metrics'].update(direction=self.direction,turn_only=self.turn_only,task_kind=self.task_kind)
        return observation(self.state,self.preview()),value,bool(success or failure),bool(truncated),info
