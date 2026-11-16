import json
from pathlib import Path
import re
import unittest

import numpy as np

from .audit import read_trace
from .metrics import statistics
from .run_benchmark import digest


ROOT = Path('artifacts/benchmark/d16')
AUDIT = ROOT/'audit'


def read(path):
    return json.loads(path.read_text())


def frozen_algorithm(name):
    return (name.startswith(('ros2_ws/src/','model/','tools/learning/'))
            and name not in ['ros2_ws/src/mujoco_backend/CMakeLists.txt',
                             'ros2_ws/src/mujoco_backend/src/controller_benchmark.cpp']) or name == 'artifacts/learning/selected_policy.txt'


class AuditChecks(unittest.TestCase):
    def test_original_algorithms_and_configuration_unchanged(self):
        extensions=read(Path('docs/d18_build_extensions.json'))
        for name,record in extensions.items():
            self.assertTrue(name.endswith('/CMakeLists.txt'))
            self.assertEqual(digest(name),record['d18_sha256'],name)
        for path in [ROOT/'hashes.json',AUDIT/'before_hashes.json']:
            for name,value in read(path).items():
                if frozen_algorithm(name) or name.endswith(('config.json','scenarios.json')):
                    expected=extensions[name]['d18_sha256'] if name in extensions else value
                    self.assertEqual(digest(name),expected,name)
        for name,value in read(AUDIT/'source_hashes.json').items():
            self.assertEqual(digest(name),value,name)

    def test_outcomes_and_subset_accounting(self):
        distribution=read(AUDIT/'mpc_failure_distribution.json')
        rows=distribution['rows']
        self.assertEqual(len(rows),100)
        self.assertEqual(sum(r['success'] for r in rows),61)
        self.assertEqual(sum(r['fall'] for r in rows),39)
        self.assertEqual(sum(r['paired_ppo_success'] for r in rows),83)
        self.assertEqual(sum(r['paired_ppo_fall'] for r in rows),0)
        self.assertEqual(sum(r['paired_ppo_timeout'] for r in rows),17)
        subset=read(AUDIT/'estimator_postfall_analysis.json')['subsets']
        metrics=read(ROOT/'summary.json')['groups']['goal_reaching']['mpc']['metrics']
        for key in ['estimated_final_distance','true_final_distance']:
            mean=sum(row['count']*row[key]['mean'] for row in subset.values())/100
            self.assertAlmostEqual(mean,metrics[key]['mean'],places=12)
        for row in rows:
            self.assertLessEqual(row['maximum_applied_torque'],2)
        for row in read(AUDIT/'replay_validation.json'):
            self.assertEqual(row['non_timing_trace_max_error'],0)

    def test_timing_sources_and_scope(self):
        summary=read(ROOT/'summary.json')
        timing=read(AUDIT/'lqr_batched_timing.json')
        batches=read_trace(AUDIT/'lqr_batches.csv')
        self.assertEqual(digest(AUDIT/'lqr_batches.csv'),timing['raw_sha256'])
        np.testing.assert_allclose(batches['total_us']/batches['updates'],batches['per_update_us'])
        self.assertEqual(len(batches),timing['batches'])
        self.assertGreater(batches['total_us'].min(),20)
        self.assertEqual(summary['compute']['lqr']['batch_mean_us'],statistics(batches['per_update_us']))
        for method in ['mpc','ppo']:
            data=read_trace(ROOT/'compute'/(method+'_isolated.csv.gz'))
            samples=data[data['full_us']>=0][500:]
            for key in ['full_us','core_us']:
                self.assertEqual(summary['compute'][method][key],statistics(samples[key]))

    def test_generated_documents_and_links(self):
        files=[Path('docs/OBSTACLE_FREE_BENCHMARK.md')]
        for path in files:
            text=path.read_text()
            self.assertEqual(text.count('<!-- D16_AUDIT_BEGIN -->'),1)
            self.assertEqual(text.count('<!-- D16_AUDIT_END -->'),1)
            for target in re.findall(r'\]\(([^)]+)\)',text):
                if target.startswith(('http:','https:','#')): continue
                destination=target.split('#')[0]
                self.assertTrue((path.parent/destination).exists(),f'{path}: {target}')
        for path,fragment in [(files[0],ROOT/'benchmark_distributions.md')]:
            text=path.read_text().split('<!-- BENCHMARK_RESULTS_BEGIN -->')[1].split('<!-- BENCHMARK_RESULTS_END -->')[0].strip()
            self.assertEqual(text,fragment.read_text().strip())
        audit=files[0].read_text().split('<!-- D16_AUDIT_BEGIN -->')[1].split('<!-- D16_AUDIT_END -->')[0].strip()
        self.assertEqual(audit,(AUDIT/'audit_summary.md').read_text().strip())
        snapshot=read(Path('docs/benchmark_audit_results.json'))
        for path,value in snapshot['sources'].items(): self.assertEqual(digest(path),value)
        for path,value in snapshot['analysis_sources'].items(): self.assertEqual(digest(path),value)


if __name__ == '__main__': unittest.main()
