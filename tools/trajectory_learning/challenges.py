import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from .benchmark import episode


def main():
    scenarios = Path('artifacts/learning/d18_final_scenarios_v2')
    output = Path('artifacts/benchmark/d18/challenges')
    output.mkdir(exist_ok=False)
    manifest = json.loads((scenarios/'manifest.json').read_text())
    cases = []
    for family in ['wall', 'staggered', 'elbow', 'corridor']:
        original = next(c for c in manifest['cases'] if c['family']==family and c['index']==0)
        for axis, force in [('forward', [-50,0,0]), ('lateral', [0,-50,0])]:
            case = copy.deepcopy(original)
            case['experiment'] = f'push_{family}_{axis}'
            case['push'] = {'time': 5.0, 'duration': .05, 'local_force': force, 'body': 'base_body'}
            cases.append(case)
    original = next(c for c in manifest['cases'] if c['family']=='wall' and c['index']==0)
    for angle in [-5,-10,-15,-20]:
        case = copy.deepcopy(original)
        case['experiment'] = f'pitch_{abs(angle)}'
        case['initial_pitch'] = math.radians(angle)
        cases.append(case)
    (output/'config.json').write_text(json.dumps({'cases': cases, 'purpose': 'Separate fixed perturbation and initial-condition challenges'}, indent=2)+'\n')
    actor = Path('artifacts/learning/d18_selected/actor.txt')
    jobs = [(case,output,scenarios,method,actor) for method in ['mpc','ppo'] for case in cases]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for result in pool.map(episode,jobs):
            print(json.dumps(result),flush=True)


if __name__ == '__main__':
    main()
