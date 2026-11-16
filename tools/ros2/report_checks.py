import hashlib
import json
from pathlib import Path
import re
import unittest


class IntegrationReportChecks(unittest.TestCase):
    def test_frozen_core(self):
        before = json.loads(Path('artifacts/ros2/d17/before_hashes.json').read_text())
        extensions = json.loads(Path('docs/d18_build_extensions.json').read_text())
        for name, record in extensions.items():
            self.assertTrue(name.endswith('/CMakeLists.txt'))
            self.assertEqual(hashlib.sha256(Path(name).read_bytes()).hexdigest(), record['d18_sha256'], name)
        validation = json.loads(Path('docs/d18_validation_extensions.json').read_text())
        self.assertEqual(set(validation), {'tools/benchmark/audit_checks.py', 'tools/benchmark/audit_report.py', 'tools/benchmark/report.py'})
        for name, record in validation.items():
            self.assertEqual(hashlib.sha256(Path(name).read_bytes()).hexdigest(), record['d18_sha256'], name)
        extensions.update(validation)
        for name, digest in before.items():
            if Path(name).name == ".DS_Store":
                continue
            expected = extensions[name]['d18_sha256'] if name in extensions else digest
            self.assertEqual(hashlib.sha256(Path(name).read_bytes()).hexdigest(), expected, name)
        for package in ['controller', 'state_estimator', 'navigation', 'learning_policy']:
            for source in Path('ros2_ws/src', package).rglob('*'):
                if source.suffix in ['.cpp', '.hpp', '.xml']:
                    self.assertNotIn('rclcpp', source.read_text(), str(source))

    def test_snapshot_and_comparison(self):
        snapshot = json.loads(Path('docs/ros2_integration_results.json').read_text())
        self.assertEqual(snapshot, json.loads(Path('artifacts/ros2/d17/summary.json').read_text()))
        run = snapshot['regulation']
        self.assertEqual(len(set(run['process_ids'])), 3)
        self.assertEqual(run['state_count'], 12000)
        self.assertEqual(run['command_count'], 6000)
        for field, result in run['comparison'].items():
            self.assertLess(result['maximum_error'], 1e-5, field)
        self.assertEqual(len(snapshot['ppo']['goal_results']), 4)
        self.assertTrue(all(g['success'] for g in snapshot['ppo']['goal_results']))
        self.assertEqual(snapshot['known_map']['contact_samples'], 0)
        self.assertLess(snapshot['known_map']['estimated_final_distance'], .15)
        self.assertLess(snapshot['faults']['ros_policy_torque_maximum_error'], 2e-6)
        self.assertTrue(snapshot['launch_bag_verified']['record_use_sim_time'])
        for topic in ['/imu', '/joint_states', '/robot_state', '/wheel_torque', '/goal_pose', '/clock']:
            self.assertGreater(snapshot['launch_bag_verified']['bag_counts'][topic], 0)

    def test_report_links(self):
        for document in ['README.md', 'docs/ROS2_INTEGRATION.md']:
            path = Path(document)
            text = path.read_text()
            for target in re.findall(r'\]\(([^)]+)\)', text):
                if target.startswith(('http', '#')):
                    continue
                name = target.split('#')[0]
                self.assertTrue((path.parent / name).exists(), f'{document}: {target}')
            self.assertIn('### ROS 2 Runtime' if document=='README.md' else '<!-- ROS_RESULTS_BEGIN -->', text)
            self.assertNotIn('runtime ROS nodes and transport integration are future work', text)

    def test_bringup_sim_time(self):
        launch = Path('ros2_ws/src/robot_ros/launch/simulation.launch.py').read_text()
        self.assertIn("'use_sim_time':True", launch)
        self.assertIn("'ppo_node' if mode=='ppo' else 'mpc_node'", launch)
        for name in ['model_based', 'ppo', 'known_map']:
            self.assertIn('simulation.launch.py', Path(f'ros2_ws/src/robot_ros/launch/{name}.launch.py').read_text())


if __name__ == '__main__':
    unittest.main()
