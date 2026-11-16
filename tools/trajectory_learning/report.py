import argparse
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('runs',nargs='+',type=Path)
    args=parser.parse_args()
    previous=Path('docs/d18_learning_results.json')
    output=json.loads(previous.read_text()) if previous.exists() else {}
    figure,axes=plt.subplots(1,3,figsize=(12,3.5),constrained_layout=True)
    for run in args.runs:
        rows=[json.loads(line) for line in (run/'validation.jsonl').read_text().splitlines()]
        record={'config':json.loads((run/'config.json').read_text()),'validation':rows,
                'source':str(run),'validation_sha256':hashlib.sha256((run/'validation.jsonl').read_bytes()).hexdigest()}
        record['result']=json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else ({'status':'interrupted',**json.loads((run/'interruption.json').read_text())} if (run/'interruption.json').exists() else {'status':'training'})
        output[run.name]=record
        for axis,key,label in zip(axes,['success','fall','final_distance'],['Validation success fraction','Validation fall fraction','Estimated final distance (m)']):
            axis.plot([r['steps']/1e6 for r in rows],[r['summary'][key] for r in rows],marker='.',label=run.name.replace('d18_',''))
            axis.set_xlabel('Additional transitions (millions)');axis.set_ylabel(label);axis.grid(alpha=.25)
    axes[0].axhline(.9,color='gray',linestyle='--');axes[0].legend(title='Run',fontsize=6)
    Path('docs/figures/learning').mkdir(parents=True,exist_ok=True)
    figure.savefig('docs/figures/learning/d18_validation.png',dpi=160)
    plt.close(figure)
    Path('docs/d18_learning_results.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
