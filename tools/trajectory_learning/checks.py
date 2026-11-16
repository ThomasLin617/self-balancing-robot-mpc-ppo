import unittest
import numpy as np
from tools.learning.state import EstimatedState,load_config
from .state import Trajectory,StraightReference,KnownMap,World,ROOT,observation,reward
from .environment import TrajectoryEnvironment


class Checks(unittest.TestCase):
    def test_full_circle_turn_then_move(self):
        for yaw in [-3.1,-1.8,1.8,3.1]:
            trajectory=Trajectory.turn_then_move(yaw,0.4)
            samples=np.array([trajectory.sample(t) for t in np.linspace(0,trajectory.duration,300)])
            self.assertTrue(np.isfinite(samples).all())
            turning=np.abs(samples[:,6])>1e-5
            np.testing.assert_allclose(samples[turning,:4],0,atol=1e-12)
            end=trajectory.sample(trajectory.duration+1)
            np.testing.assert_allclose(end[:2],0.4*np.array([np.cos(yaw),np.sin(yaw)]),atol=1e-10)
            self.assertAlmostEqual(end[2],0.4)
            self.assertEqual(end[3],0)
            self.assertEqual(end[6],0)

    def test_signed_straight(self):
        for distance in [0.2,-0.2,0.6,-0.6]:
            trajectory=StraightReference(distance)
            end=trajectory.sample(trajectory.duration+1)
            self.assertAlmostEqual(end[2],distance)
            self.assertEqual(end[3],0)
            for time in np.linspace(0,trajectory.duration,100):
                point=trajectory.sample(time)
                self.assertEqual(point[5],0)
                self.assertGreaterEqual(point[3]*distance,0)
                p=trajectory.sample(time+0.002)
                np.testing.assert_allclose(trajectory.horizon(time,3)[0],[3+p[2],p[3],0,0,0])

    def test_preview_matches_samples(self):
        t=Trajectory.point_to_point(0,0,0,0.4,0.1)
        h=t.horizon(0.1,2.0)
        for i in range(250):
            p=t.sample(0.1+(i+1)*0.002)
            np.testing.assert_allclose(h[i],[2+p[2],p[3],0,p[5],p[6]],atol=1e-12)
        p=t.sample(t.duration+10)
        self.assertEqual(p[3],0);self.assertEqual(p[6],0)

    def test_local_coordinates(self):
        s=EstimatedState(load_config());s.position=4;s.yaw=np.pi-0.01
        h=np.tile([4.2,0.1,0,-np.pi+0.01,0.2],(250,1))
        o=observation(s,h)
        self.assertEqual(o.shape,(1256,))
        self.assertAlmostEqual(float(o[6]),0.2,places=6)
        self.assertAlmostEqual(float(o[9]),0.02/np.pi,places=6)
        h[0,0]=np.nan
        with self.assertRaises(ValueError):observation(s,h)

    def test_reward(self):
        s=EstimatedState(load_config());p=[0]*7
        self.assertAlmostEqual(reward(s,p,0,0,[0,0],False,False),0.002)
        self.assertAlmostEqual(reward(s,p,0,0,[0,0],True,False),30.002)
        self.assertAlmostEqual(reward(s,p,0,0,[0,0],False,True),-19.998)
        self.assertAlmostEqual(reward(s,p,0,0,[1,1],False,False,0.01),5*reward(s,p,0,0,[1,1],False,False))
        s.position=0.03
        scales=[0.03,0.2,0.1,1,0.1,0.5]
        self.assertAlmostEqual(reward(s,p,0,0,[0,0],False,False,tracking_scales=scales),0)
        scales[0]=0
        with self.assertRaises(ValueError):
            reward(s,p,0,0,[0,0],False,False,tracking_scales=scales)

    def test_reset_and_fall(self):
        env=TrajectoryEnvironment()
        def rollout():
            obs,_=env.reset(seed=318000)
            result=[obs]
            for _ in range(30):result.append(env.step([0.01,-0.01])[0])
            return result
        np.testing.assert_array_equal(rollout(),rollout())
        other=TrajectoryEnvironment();other.reset(seed=1)
        self.assertEqual(other.steps,0)
        env.reset(seed=1,options={'pitch':0.8})
        _,r,terminated,_,info=env.step([0,0])
        self.assertTrue(terminated);self.assertTrue(info['metrics']['fall'])
        self.assertTrue(np.isfinite(r))

    def test_world_matches_map(self):
        import mujoco
        m=KnownMap.load(str(ROOT/'model/known_map.txt'))
        world=World(str(ROOT/'model/Robot1.xml'),m)
        model=mujoco.MjModel.from_xml_path(world.path)
        self.assertGreaterEqual(model.geom('known_wall').id,0)
        self.assertFalse(m.footprint_free(1.1,0))
        self.assertFalse(m.footprint_free(-10,0))

    def test_no_early_success_and_timeout(self):
        from unittest.mock import patch
        env=TrajectoryEnvironment();env.reset(seed=1)
        env.state.reset();env.position_origin=0;env.goal=np.zeros(2)
        env.read_estimate=lambda:None
        env.truth=lambda:(np.zeros(2),0.0)
        with patch('mujoco.mj_step'):
            for _ in range(2999):
                _,_,terminated,truncated,_=env.step([0,0])
                self.assertFalse(terminated or truncated)
            for _ in range(251):
                _,_,terminated,_,info=env.step([0,0])
                if terminated:break
            self.assertTrue(terminated)
            self.assertTrue(info['metrics']['success'])
        env=TrajectoryEnvironment();env.reset(seed=1);env.timeout=0.002
        _,_,terminated,truncated,_=env.step([0,0])
        self.assertFalse(terminated);self.assertTrue(truncated)

    def test_stopped_turns_and_categories(self):
        for yaw in [-1.5,1.5]:
            t=Trajectory.turn(yaw)
            end=t.sample(t.duration+1)
            self.assertAlmostEqual(end[5],yaw)
            self.assertEqual(end[3],0);self.assertEqual(end[6],0)
            for time in np.linspace(0,t.duration,30):
                p=t.sample(time)
                self.assertEqual(p[3],0)
                self.assertGreaterEqual(p[6]*yaw,0)
        categories=[]
        for seed in range(420000,420004):
            env=TrajectoryEnvironment('C');obs,_=env.reset(seed=seed)
            categories.append((env.direction,env.turn_only))
            self.assertTrue(np.isfinite(obs).all())
        self.assertEqual(len(set(categories)),4)

    def test_profiled_curve_limits(self):
        for side in [-1,1]:
            for bearing in [0.2,0.55,1.0,1.5]:
                t=Trajectory.profiled_curve(0,0,0,0.3*np.cos(bearing),side*0.3*np.sin(bearing))
                for time in np.linspace(0,t.duration,1000):
                    p=t.sample(float(time))
                    self.assertLessEqual(abs(p[6]),0.450001)
                    self.assertLessEqual(abs(p[3]*p[6]),0.025001)

    def test_obstacle_failure_and_reset(self):
        import mujoco
        env=TrajectoryEnvironment('D')
        obs,_=env.reset(seed=425000)
        self.assertTrue(np.isfinite(obs).all())
        env.data.qpos[0]=1.1
        mujoco.mj_forward(env.model,env.data)
        _,_,terminated,_,info=env.step([0,0])
        self.assertTrue(terminated);self.assertTrue(info['metrics']['collision'])
        env.reset(seed=425000)
        self.assertEqual(env.steps,0)
        env.data.qpos[0]=-10
        mujoco.mj_forward(env.model,env.data)
        _,_,terminated,_,info=env.step([0,0])
        self.assertTrue(terminated);self.assertTrue(info['metrics']['boundary'])

    def test_known_map(self):
        m=KnownMap.load(str(ROOT/'model/known_map.txt'))
        t=m.plan(0,0,0,2.4,0)
        self.assertGreater(t.duration,1)
        for time in np.linspace(0,t.duration,1000):
            p=t.sample(time);self.assertTrue(m.footprint_free(p[0],p[1]))


if __name__=='__main__':unittest.main()
