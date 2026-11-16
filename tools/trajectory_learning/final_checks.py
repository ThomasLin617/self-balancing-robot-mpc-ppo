import hashlib
import json
from pathlib import Path
import re
from types import SimpleNamespace
import unittest

import numpy as np
from .state import observation
from .runtime_checks import fixture_references
from _trajectory_policy import TrajectoryPolicy


class FinalChecks(unittest.TestCase):
    def test_selected_actor(self):
        actor=Path('artifacts/learning/d18_selected_full_circle_v2/actor.txt')
        fixture=json.loads(Path('docs/final_policy_parity.json').read_text())
        self.assertEqual(hashlib.sha256(actor.read_bytes()).hexdigest(),fixture['actor_sha256'])
        policy=TrajectoryPolicy(str(actor))
        for case in fixture['cases']:
            values=case['state']
            state=SimpleNamespace(**dict(zip(['position','velocity','pitch','pitch_rate','yaw','yaw_rate'],values)))
            refs=fixture_references(case)
            actual=policy.observation(values,refs)
            np.testing.assert_allclose(actual,observation(state,refs),atol=1e-6,rtol=0)
            np.testing.assert_allclose(policy.normalized_action(actual),case['python_action'],atol=1e-5,rtol=0)
        installed=Path('ros2_ws/install/robot_ros/share/robot_ros/policy/trajectory_actor.txt')
        self.assertEqual(installed.read_bytes(),actor.read_bytes())

    def test_final_population_and_selection(self):
        report=json.loads(Path('docs/final_comparison.json').read_text())
        groups={m:{(r['family'],r['index']):r for r in rows} for m,rows in report['episodes'].items()}
        self.assertEqual(len(groups['mpc']),100)
        self.assertEqual(groups['mpc'].keys(),groups['ppo'].keys())
        for case in groups['mpc']:
            self.assertEqual(groups['mpc'][case]['reference_sha256'],groups['ppo'][case]['reference_sha256'])
        selected=min(groups['mpc'],key=lambda k:(-abs(groups['mpc'][k]['timed_planar_rmse']-groups['ppo'][k]['timed_planar_rmse']),k))
        self.assertEqual(selected,(report['selected_case']['family'],report['selected_case']['index']))
        frozen=json.loads(Path('docs/final_validation.json').read_text())['frozen_algorithm_hashes']
        for name,digest in frozen.items():
            self.assertEqual(hashlib.sha256(Path(name).read_bytes()).hexdigest(),digest,name)

    def test_documents_and_generated_tables(self):
        fragments=Path('docs/data/comparison/tables.md').read_text().strip().split('\n\n')
        for name in ['README.md','docs/CONTROLLER_COMPARISON.md']:
            path=Path(name);text=path.read_text()
            for marker,expected in zip(['NAVIGATION_RESULTS','COMPUTE_RESULTS'],fragments):
                actual=text.split(f'<!-- {marker}_BEGIN -->')[1].split(f'<!-- {marker}_END -->')[0].strip()
                self.assertEqual(actual,expected)
            for target in re.findall(r'\]\(([^)]+)\)',text):
                if not target.startswith(('https:','http:','#')):
                    self.assertTrue((path.parent/target.split('#')[0]).exists(),target)
        for heading in re.findall(r'^#+ .+$',Path('README.md').read_text(),re.M):
            self.assertNotRegex(heading,r'\bD\d+\b')

    def test_demo_files(self):
        root=Path('Document/demo')
        for name in ['controller_comparison', 'mpc_obstacle_navigation', 'ppo_obstacle_navigation']:
            data=(root/(name+'.mp4')).read_bytes()
            self.assertEqual(data[4:8],b'ftyp')
        self.assertEqual((root/'controller_comparison.gif').read_bytes()[:6],b'GIF89a')


if __name__=='__main__':unittest.main()
