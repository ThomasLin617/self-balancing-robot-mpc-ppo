import math

import mujoco
import numpy as np
from .environment import TrajectoryEnvironment
from .full_circle import LocalMap, LocalTrajectory, rotate
from .state import observation


class FullCircleTrainingEnvironment(TrajectoryEnvironment):
    def __init__(self, curriculum_config):
        self.world_yaw = 0.0
        super().__init__('E', curriculum_config)

    def truth(self):
        position,pitch = super().truth()
        return rotate(position,-self.world_yaw),pitch

    def reset(self, *, seed=None, options=None):
        self.world_yaw = 0.0
        super().reset(seed=seed,options=options)
        known_map = self.known_map
        goal = np.array(known_map.goal)
        bearing = self.np_random.uniform(-math.pi,math.pi)
        self.world_yaw = math.atan2(goal[1],goal[0])-bearing
        _,pitch = super().truth()
        velocity = float(self.data.qvel[0])
        cy,sy = math.cos(self.world_yaw/2),math.sin(self.world_yaw/2)
        cp,sp = math.cos(pitch/2),math.sin(pitch/2)
        self.data.qpos[3:7] = [cy*cp,-sy*sp,cy*sp,sy*cp]
        self.data.qvel[:2] = rotate([velocity,0],self.world_yaw)
        mujoco.mj_forward(self.model,self.data)
        self.state.reset()
        self.read_estimate()
        self.goal = rotate(goal,-self.world_yaw)
        trajectory = known_map.plan(0,0,self.world_yaw,*goal)
        if trajectory.duration>90:
            raise RuntimeError('Training trajectory exceeds configured duration.')
        self.trajectory = LocalTrajectory(trajectory,self.world_yaw)
        self.known_map = LocalMap(known_map,self.world_yaw)
        self.position_origin = self.state.position
        self.timeout = self.trajectory.duration+10
        return observation(self.state,self.preview()),{}
