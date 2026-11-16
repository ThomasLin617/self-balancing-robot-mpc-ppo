import hashlib
import json
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from .state import observation
from _trajectory_policy import TrajectoryPolicy


def fixture_references(case):
    state = case['state']
    references = np.zeros((250, 5))
    references[:, 0] = state[0] + np.linspace(0, case['distance'], 250)
    references[:, 1] = case['velocity_reference']
    references[:, 2] = case['pitch_reference']
    references[:, 3] = (state[4] + np.linspace(0, case['yaw_change'], 250) + np.pi) % (2*np.pi) - np.pi
    references[:, 4] = case['yaw_rate_reference']
    return references


class RuntimeChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = TrajectoryPolicy('artifacts/learning/d18_selected/actor.txt')

    def test_exported_actor_parity(self):
        fixture = json.loads(Path('docs/d18_runtime_parity.json').read_text())
        actor = Path('artifacts/learning/d18_selected/actor.txt')
        self.assertEqual(hashlib.sha256(actor.read_bytes()).hexdigest(), fixture['actor_sha256'])
        for case in fixture['cases']:
            values = case['state']
            state = SimpleNamespace(**dict(zip(['position','velocity','pitch','pitch_rate','yaw','yaw_rate'], values)))
            refs = fixture_references(case)
            expected = observation(state, refs)
            actual = self.policy.observation(values, refs)
            np.testing.assert_allclose(actual, expected, atol=1e-6, rtol=0)
            action = self.policy.normalized_action(actual)
            np.testing.assert_allclose(action, case['python_action'], atol=1e-5, rtol=0)
            torque = self.policy.compute(values, refs)
            np.testing.assert_allclose(torque[:2], np.asarray(action)*2, atol=1e-12)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            self.policy.normalized_action([0.0]*6)
        values = np.zeros(1256)
        values[20] = np.nan
        with self.assertRaises(ValueError):
            self.policy.normalized_action(values.tolist())
        with self.assertRaises(ValueError):
            self.policy.compute([0.0]*6, np.zeros((249, 5)))
        with self.assertRaises(RuntimeError):
            TrajectoryPolicy('/nonexistent/d18_actor.txt')

    def test_paired_benchmark_population(self):
        report = json.loads(Path('docs/d18_benchmark_results.json').read_text())
        pairs = []
        for method in ['mpc','ppo']:
            episodes = report['episodes'][method]
            self.assertEqual(len(episodes), 100)
            pairs.append({(row['family'], row['index']): row['reference_sha256'] for row in episodes})
            self.assertEqual(len(pairs[-1]), 100)
            for family in ['wall','staggered','elbow','corridor']:
                self.assertEqual(sum(row['family']==family for row in episodes), 25)
            self.assertTrue(all(row['maximum_torque']<=2 for row in episodes))
        self.assertEqual(pairs[0], pairs[1])


if __name__ == '__main__':
    unittest.main()
