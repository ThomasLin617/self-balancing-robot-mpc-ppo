import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import mujoco
import numpy as np
from .environment import TrajectoryEnvironment
from .state import KnownMap, observation


def rotate(point, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c*point[0]-s*point[1], s*point[0]+c*point[1]])


class LocalTrajectory:
    def __init__(self, trajectory, world_yaw):
        self.trajectory = trajectory
        self.world_yaw = world_yaw
        self.duration = trajectory.duration

    def sample(self, elapsed):
        result = np.array(self.trajectory.sample(elapsed))
        result[:2] = rotate(result[:2], -self.world_yaw)
        result[5] = (result[5]-self.world_yaw+np.pi) % (2*np.pi)-np.pi
        return result

    def horizon(self, elapsed, origin):
        result = np.array(self.trajectory.horizon(elapsed, origin))
        result[:,3] = (result[:,3]-self.world_yaw+np.pi) % (2*np.pi)-np.pi
        return result


class LocalMap:
    def __init__(self, known_map, world_yaw):
        self.known_map = known_map
        self.world_yaw = world_yaw

    def inside(self, x, y):
        return self.known_map.inside(*rotate([x,y], self.world_yaw))


class FullCircleEnvironment(TrajectoryEnvironment):
    def __init__(self, scenario, world_yaw):
        self.world_yaw = 0.0
        self.reset_yaw = world_yaw
        super().__init__('D', {'scenario': scenario})
        self.world_map = self.known_map

    def truth(self):
        position, pitch = super().truth()
        return rotate(position, -self.world_yaw), pitch

    def reset(self, *, seed=None, options=None):
        self.world_yaw = 0.0
        self.known_map = self.world_map
        super().reset(seed=seed, options=options)
        self.world_yaw = self.reset_yaw
        pitch = float((options or {}).get('pitch', 0.0))
        cy, sy = math.cos(self.world_yaw/2), math.sin(self.world_yaw/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        self.data.qpos[3:7] = [cy*cp, -sy*sp, cy*sp, sy*cp]
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.state.reset()
        self.read_estimate()
        self.goal = rotate(self.world_map.goal, -self.world_yaw)
        world_trajectory = self.world_map.plan(0,0,self.world_yaw,*self.world_map.goal)
        self.trajectory = LocalTrajectory(world_trajectory, self.world_yaw)
        self.known_map = LocalMap(self.world_map, self.world_yaw)
        self.position_origin = self.state.position
        self.timeout = self.trajectory.duration+10
        return observation(self.state,self.preview()), {}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=Path('artifacts/learning/d18_final_scenarios_v2'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=518000)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((args.source/'manifest.json').read_text())
    bearings = np.random.default_rng(args.seed).uniform(-np.pi,np.pi,100)
    for case, bearing in zip(manifest['cases'], bearings):
        source = args.source/case['scenario']
        shutil.copyfile(source,args.output/case['scenario'])
        known = KnownMap.load(str(source))
        world_goal = list(known.goal)
        world_yaw = math.atan2(world_goal[1],world_goal[0])-float(bearing)
        world_yaw = (world_yaw+np.pi)%(2*np.pi)-np.pi
        trajectory = LocalTrajectory(known.plan(0,0,world_yaw,*world_goal),world_yaw)
        length = trajectory.sample(trajectory.duration)[2]
        if not 1<=length<=4 or trajectory.duration>90:
            raise ValueError('Fixed case violates geometry limits; retain bearing and diagnose')
        values=np.array([trajectory.sample(float(t)) for t in np.arange(0,trajectory.duration+.002,.002)],dtype='<f8')
        case.update(world_goal=world_goal,world_initial_yaw=world_yaw,bearing=float(bearing),
                    goal=rotate(world_goal,-world_yaw).tolist(),duration=trajectory.duration,
                    timeout=trajectory.duration+10,sampled_reference_sha256=hashlib.sha256(values.tobytes()).hexdigest())
    manifest.update(version='d18_full_circle_v1',bearing_seed=args.seed,
                    bearing_range=[-math.pi,math.pi],
                    coordinate_frame='Episode odometry; world geometry unchanged, configured initial world yaw varies.',
                    selection='Original 100 obstacle scenes retained. All 100 bearings drawn once before control; no angular rejection.',
                    source_manifest_sha256=hashlib.sha256((args.source/'manifest.json').read_bytes()).hexdigest())
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Frozen 100 full-circle cases; no case or bearing rejected')


if __name__ == '__main__':
    main()
