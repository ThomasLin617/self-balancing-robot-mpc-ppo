import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    figure, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    quantities = [('success', 'Validation success fraction'),
                  ('fall', 'Validation fall fraction'),
                  ('final_distance', 'Mean final truth distance (m)'),
                  ('return', 'Mean episode return')]
    for directory in args.runs:
        path = Path(directory)
        rows = [json.loads(line) for line in (path/'validation.jsonl').read_text().splitlines()]
        for axis, (key, label) in zip(axes.flat, quantities):
            axis.plot([row['steps']/1e6 for row in rows],
                      [row['summary'][key] for row in rows], marker='.', label=path.name)
            axis.set_xlabel('Additional transitions in run (millions)')
            axis.set_ylabel(label)
            axis.grid(alpha=.25)
    axes[0, 0].axhline(.9, color='black', linestyle=':', label='Success gate')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].set_ylim(-.02, 1.02)
    axes[0, 1].set_ylim(-.02, 1.02)
    figure.savefig(args.output, dpi=160)
    plt.close(figure)


if __name__ == '__main__':
    main()
