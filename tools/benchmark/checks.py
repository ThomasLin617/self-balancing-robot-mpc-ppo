import json
from pathlib import Path
import unittest

import numpy as np

from .metrics import measure, settling_time, statistics
from .run_benchmark import scenarios


class BenchmarkChecks(unittest.TestCase):
    def test_pairing(self):
        config = json.loads(Path('tools/benchmark/config.json').read_text())
        jobs = scenarios(config)
        self.assertEqual(jobs, scenarios(config))
        random = [j for j in jobs if j['population']=='random']
        self.assertEqual(len(random),200)
        for a,b in zip(random[::2],random[1::2]):
            self.assertEqual(a['goal'],b['goal'])
            self.assertEqual(a['duration'],b['duration'])
            self.assertEqual([a['method'],b['method']],['mpc','ppo'])
            self.assertGreaterEqual(np.linalg.norm(a['goal']),.2)
            self.assertLessEqual(np.linalg.norm(a['goal']),1.0)

    def test_settling(self):
        t = np.arange(1,1001)*.001
        inside = t >= .2
        self.assertAlmostEqual(settling_time(t,inside,.5),.7)
        self.assertAlmostEqual(settling_time(t,inside,.5,final=True),.2)
        inside[-1]=False
        self.assertIsNone(settling_time(t,inside,.5,final=True))
        self.assertIsNone(settling_time(t,t>.8,.5))

    def test_effort_and_failures(self):
        config=json.loads(Path('tools/benchmark/config.json').read_text())
        names='time p v pitch pitch_rate yaw_rate x y true_x true_y goal_distance true_distance requested_l requested_r applied_l applied_r force success fall full_us core_us'.split()
        data=np.zeros(1000,dtype=[(name,float) for name in names])
        data['time']=np.arange(1,1001)*.001
        data['applied_l']=data['requested_l']=2
        data['applied_r']=data['requested_r']=-1
        data['goal_distance']=data['true_distance']=.2
        job=dict(group='goal_reaching',goal=[.2,0],force=0)
        result=measure(data,job,config)
        self.assertAlmostEqual(result['absolute_torque'],3)
        self.assertAlmostEqual(result['squared_torque'],5)
        self.assertAlmostEqual(result['saturation_duration'],1)
        self.assertTrue(result['timeout'])
        self.assertIsNone(result['time_to_success'])
        self.assertIsNone(result['path_efficiency'])
        self.assertEqual(statistics([1,2,3,None])['count'],3)
        with self.assertRaises(ValueError):statistics([float('nan')])


if __name__=='__main__': unittest.main()
