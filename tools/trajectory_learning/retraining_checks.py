import unittest

import numpy as np
from .full_circle_training import FullCircleTrainingEnvironment
from .full_circle import rotate


class RetrainingChecks(unittest.TestCase):
    def test_independent_full_circle_reset(self):
        env=FullCircleTrainingEnvironment({'full_circle':True})
        first,_=env.reset(seed=428000)
        goal=env.goal.copy()
        horizon=env.preview().copy()
        self.assertTrue(np.isfinite(first).all())
        np.testing.assert_allclose(rotate(env.goal,env.world_yaw),env.known_map.known_map.goal)
        for _ in range(10):
            env.step([0,0])
        second,_=env.reset(seed=428000)
        np.testing.assert_array_equal(first,second)
        np.testing.assert_array_equal(goal,env.goal)
        np.testing.assert_array_equal(horizon,env.preview())
        env.reset(seed=428001)
        self.assertFalse(np.array_equal(horizon,env.preview()))
        env.close()


if __name__=='__main__':
    unittest.main()
