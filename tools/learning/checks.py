import io
import math
import subprocess
import tempfile
import unittest

import numpy as np

from .state import ROOT, EstimatedState, heading_potential, load_config, observation, reward_terms, torques
from .environment import GoalEnvironment


class LearningChecks(unittest.TestCase):
    def setUp(self):
        self.config = load_config()

    def test_goal_and_normalization(self):
        state = EstimatedState(self.config)
        state.x, state.y, state.yaw = 2.0, -1.0, math.pi / 2.0
        np.testing.assert_allclose(observation(state, [2.0, 0.0], self.config)[:2], [1, 0], atol=1e-7)
        for yaw in [math.pi - 1e-8, -math.pi + 1e-8]:
            state.yaw = yaw
            np.testing.assert_allclose(observation(state, [1.0, -1.0], self.config)[:2], [1, 0], atol=1e-7)
        state.pitch = 0.35
        self.assertAlmostEqual(float(observation(state, [2, -1], self.config)[3]), 1.0)
        self.assertEqual(float(np.abs(observation(state, [100, 100], self.config)).max()), 5.0)

    def test_actions_and_reward(self):
        np.testing.assert_array_equal(torques([-1, 1], self.config), [-2, 2])
        np.testing.assert_array_equal(torques([-9, 9], self.config), [-2, 2])
        with self.assertRaises(ValueError):
            torques([np.nan, 0], self.config)
        positive = reward_terms(1, .9, 0, np.zeros(2), .002, False, False, self.config)
        negative = reward_terms(.9, 1, 0, np.zeros(2), .002, False, False, self.config)
        self.assertGreater(sum(positive.values()), 0)
        self.assertLess(sum(negative.values()), 0)
        a = reward_terms(1, 1, .1, np.ones(2), .002, False, False, self.config)
        b = reward_terms(1, 1, .1, np.ones(2), .01, False, False, self.config)
        self.assertAlmostEqual(b['effort'], 5 * a['effort'])
        self.assertAlmostEqual(b['upright'], 5 * a['upright'])
        rotation = reward_terms(1, 1, 0, np.zeros(2), .002, False, False, self.config, 2.0, .1, .003)
        self.assertAlmostEqual(rotation['rotation'], -self.config['reward']['yaw_rate'] * 4.0 * .002)
        self.assertAlmostEqual(rotation['heading'], self.config['reward']['heading'] * .1)
        self.assertAlmostEqual(rotation['turn_alignment'], self.config['reward']['turn_alignment'] * .003)

    def test_reset_rollout_and_termination(self):
        env = GoalEnvironment(config=self.config)
        actions = np.random.default_rng(5).uniform(-.1, .1, (100, 2))
        def rollout():
            obs, _ = env.reset(seed=123)
            result = [obs]
            for action in actions:
                obs, reward, terminated, truncated, _ = env.step(action)
                self.assertTrue(np.isfinite(reward))
                result.append(obs)
                if terminated or truncated:
                    break
            return np.array(result)
        np.testing.assert_array_equal(rollout(), rollout())
        env.reset(seed=1, options={'pitch': .8})
        _, _, terminated, _, info = env.step([0, 0])
        self.assertTrue(terminated)
        self.assertTrue(info['metrics']['fall'])
        with self.assertRaises(RuntimeError):
            env.step([0, 0])
        config = load_config()
        config['stages']['A']['episode_seconds'] = .004
        env = GoalEnvironment(config=config)
        env.reset(seed=4)
        env.step([0, 0])
        _, _, terminated, truncated, info = env.step([0, 0])
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info['metrics']['time'], .004)

    def test_turn_goal_distribution(self):
        config = load_config()
        env = GoalEnvironment('Cturn', config)
        values = []
        for seed in range(10):
            env.reset(seed=seed)
            values.append(round(math.atan2(env.goal[1], env.goal[0]), 6))
        self.assertTrue(all(abs(abs(value) - math.pi / 2.0) < 1e-5 for value in values))

    def test_turn_reward_direction(self):
        self.config['reward']['turn_alignment'] = 5.0
        for side in [-1, 1]:
            env = GoalEnvironment('Cside', self.config)
            totals = []
            for direction in [side, -side]:
                env.reset(seed=1, options={'goal': [0.0, side * .3], 'pitch': 0.0})
                total = 0.0
                for _ in range(20):
                    _, _, done, truncated, info = env.step([-.1 * direction, .1 * direction])
                    self.assertFalse(done or truncated)
                    total += info['reward_terms']['turn_alignment']
                totals.append(total)
            self.assertGreater(totals[0], 0.0)
            self.assertLess(totals[1], 0.0)
            env.close()

    def test_heading_potential(self):
        c = self.config
        self.assertEqual(heading_potential(.05, math.pi, c), 0.0)
        self.assertGreater(heading_potential(.5, .5, c), heading_potential(.5, 1.0, c))
        gamma = c['ppo']['gamma']
        for turns in [1, 3]:
            phases = np.linspace(.7, .7 + turns * 2.0 * math.pi, turns * 500 + 1)
            potentials = [heading_potential(.4 + .1 * math.cos(t), t, c) for t in phases]
            discounted = sum(gamma**i * (gamma * b - a)
                             for i, (a, b) in enumerate(zip(potentials, potentials[1:])))
            self.assertAlmostEqual(discounted, -potentials[0] + gamma**(len(phases)-1) * potentials[-1], places=10)
        for fallen in [False, True]:
            c['stages']['A']['episode_seconds'] = .002
            env = GoalEnvironment('A', c)
            env.reset(seed=1, options={'goal': [0.0, .5], 'pitch': .8 if fallen else 0.0})
            previous = heading_potential(env.previous_distance, env.previous_bearing, c)
            _, _, terminal, truncated, info = env.step([0.0, 0.0])
            self.assertEqual(terminal, fallen)
            self.assertEqual(truncated, not fallen)
            current = 0.0 if fallen else heading_potential(env.previous_distance, env.previous_bearing, c)
            self.assertAlmostEqual(info['reward_terms']['heading_potential'], gamma * current - previous)
            env.close()

    def test_settled_hold(self):
        config = load_config()
        config['stages']['A']['velocity_range'] = 0.0
        config['stages']['A']['minimum_success_seconds'] = 0.0
        config['success']['hold_seconds'] = .004
        env = GoalEnvironment(config=config)
        env.reset(seed=4, options={'pitch': 0.0, 'goal': [0.0, 0.0]})
        self.assertFalse(env.step([0, 0])[2])
        _, _, done, _, info = env.step([0, 0])
        self.assertTrue(done)
        self.assertTrue(info['metrics']['success'])
        env.reset(seed=4, options={'pitch': 0.0, 'goal': [0.0, 0.0]})
        for _ in range(2):
            _, _, done, _, _ = env.step([1, 1])
        self.assertFalse(done)

    def test_vector_environment_parity(self):
        from stable_baselines3.common.vec_env import DummyVecEnv
        single = GoalEnvironment(config=self.config)
        vector = DummyVecEnv([lambda: GoalEnvironment(config=self.config) for _ in range(3)])
        obs, _ = single.reset(seed=71)
        vector.seed(71)
        np.testing.assert_array_equal(obs, vector.reset()[0])
        for _ in range(100):
            obs, reward, done, truncated, _ = single.step([.01, -.01])
            batch, rewards, dones, _ = vector.step(np.tile([.01, -.01], (3, 1)))
            self.assertFalse(done or truncated)
            np.testing.assert_array_equal(obs, batch[0])
            self.assertAlmostEqual(reward, float(rewards[0]), places=7)
        vector.close()

    def test_torque_signs(self):
        results = []
        for action in [[.1, .1], [-.1, -.1], [-.1, .1], [.1, -.1]]:
            env = GoalEnvironment(config=self.config)
            env.reset(seed=2, options={'pitch': 0.0})
            for _ in range(20):
                env.step(action)
            results.append((env.state.velocity, env.state.yaw_rate))
        self.assertGreater(results[0][0], results[1][0])
        self.assertGreater(results[2][1], 0)
        self.assertLess(results[3][1], 0)
        print('Torque sign checks (velocity, yaw rate):', results)

    def test_cpp_estimator_parity(self):
        trace = []
        for i in range(5001):
            t = i * .001
            trace.append([t, .1*math.sin(t), .3*math.cos(t), 1.5,
                          -1.2*math.sin(t), .1, 9.81, .2+t, -.3+1.2*t, 1, 1.2])
        # Include a real sensor trace as well as the synthetic yaw-wrap trace.
        env = GoalEnvironment(config=self.config)
        env.reset(seed=11)
        actual = []
        for i in range(150):
            a, s = env.addresses, env.data.sensordata
            actual.append([env.data.time, *s[a['imu_gyro']:a['imu_gyro']+3],
                           *s[a['imu_accel']:a['imu_accel']+3],
                           *[s[a[name]] for name in ['left_wheel_pos', 'right_wheel_pos', 'left_wheel_vel', 'right_wheel_vel']]])
            env.step([.01*math.sin(i*.01), -.01*math.sin(i*.01)])
        with tempfile.TemporaryDirectory() as directory:
            executable = directory + '/estimator_trace'
            source = ROOT / 'ros2_ws/src'
            subprocess.run(['c++', '-std=c++17', '-O2', str(ROOT/'tools/learning/estimator_trace.cpp'),
                            str(source/'state_estimator/src/state_estimator.cpp'),
                            str(source/'navigation/src/planar_odometry.cpp'),
                            *['-I'+str(source/package/'include') for package in ['state_estimator', 'navigation', 'robot_interface']],
                            '-o', executable], check=True)
            for rows in [trace, actual]:
                data = '\n'.join(' '.join(map(str, row)) for row in rows)
                output = subprocess.run([executable], input=data, text=True, capture_output=True, check=True)
                cpp = np.loadtxt(io.StringIO(output.stdout))
                state = EstimatedState(self.config)
                expected = []
                for row in rows:
                    state.update(row[0], row[1:4], row[4:7], *row[7:])
                    expected.append(state.vector())
                error = float(np.max(np.abs(cpp-np.array(expected))))
                print('Estimator parity rows:', len(rows), 'maximum error:', error)
                np.testing.assert_allclose(cpp, expected, atol=1e-10, rtol=1e-10)


if __name__ == '__main__':
    unittest.main()
