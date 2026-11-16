import json
import unittest
from pathlib import Path

import numpy as np
from .environment import TrajectoryEnvironment
from .full_circle import FullCircleEnvironment, rotate


class FullCircleChecks(unittest.TestCase):
    def test_zero_heading_matches_existing_environment(self):
        path=str(Path('model/known_map.txt').resolve())
        original=TrajectoryEnvironment('D',{'scenario':path})
        rotated=FullCircleEnvironment(path,0)
        try:
            original.stage['velocity_range']=rotated.stage['velocity_range']=0
            first,_=original.reset(seed=518000,options={'pitch':0})
            second,_=rotated.reset(seed=518000,options={'pitch':0})
            np.testing.assert_allclose(first,second,atol=1e-7)
            for _ in range(20):
                a=original.step([.01,-.01]);b=rotated.step([.01,-.01])
                np.testing.assert_allclose(a[0],b[0],atol=1e-7)
                np.testing.assert_allclose(original.truth()[0],rotated.truth()[0],atol=1e-12)
        finally:
            original.close();rotated.close()

    def test_heading_frames_and_horizon(self):
        path=str(Path('model/known_map.txt').resolve())
        for heading in [-3.1,-1.5,1.5,3.1]:
            env=FullCircleEnvironment(path,heading)
            try:
                env.reset(seed=518000,options={'pitch':0})
                self.assertEqual(env.state.yaw,0)
                np.testing.assert_allclose(rotate(env.goal,heading),env.world_map.goal,atol=1e-12)
                np.testing.assert_allclose(env.truth()[0],[0,0],atol=1e-12)
                self.assertTrue(env.known_map.inside(0,0))
                self.assertTrue(np.isfinite(env.preview()).all())
                point=env.trajectory.sample(.002)
                np.testing.assert_allclose(env.preview()[0],[point[2],point[3],point[4],point[5],point[6]],atol=1e-12)
                end=env.trajectory.sample(env.trajectory.duration+10)
                np.testing.assert_allclose(end[:2],env.goal,atol=1e-12)
                self.assertEqual(end[3],0);self.assertEqual(end[6],0)
            finally:env.close()

    def test_frozen_population(self):
        source=json.loads(Path('docs/d18_forward_only_manifest.json').read_text())
        revised=json.loads(Path('docs/d18_benchmark_manifest.json').read_text())
        expected=np.random.default_rng(518000).uniform(-np.pi,np.pi,100)
        self.assertEqual(len(revised['cases']),100)
        for old,new,bearing in zip(source['cases'],revised['cases'],expected):
            self.assertEqual(old['scenario_sha256'],new['scenario_sha256'])
            self.assertEqual(old['goal'],new['world_goal'])
            self.assertEqual(new['bearing'],bearing)
            self.assertAlmostEqual(np.arctan2(new['goal'][1],new['goal'][0]),bearing)
        self.assertGreater(sum(abs(c['bearing'])>np.pi/2 for c in revised['cases']),30)


if __name__=='__main__':unittest.main()
