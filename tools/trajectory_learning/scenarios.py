import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from .state import KnownMap


def layout(rng,family):
    goal=[float(rng.uniform(2.1,2.5)),float(rng.uniform(-0.15,0.15))]
    shift=float(rng.uniform(-0.08,0.08))
    if family=='wall':
        boxes=[('wall',0.95,-0.35+shift,1.15,0.35+shift)]
    elif family=='staggered':
        boxes=[('first',0.7,-0.35+shift,0.9,0.15+shift),('second',1.35,-0.15+shift,1.55,0.35+shift)]
    elif family=='elbow':
        boxes=[('vertical',0.95,-0.35+shift,1.15,0.35+shift),('horizontal',1.15,0.15+shift,1.5,0.35+shift)]
    else:
        boxes=[('divider',0.95,-0.35+shift,1.15,0.35+shift),
               ('lower',0.3,-1.7,1.8,-1.5),('upper',0.3,1.5,1.8,1.7)]
    text='grid 120 100 0.05 -1.5 -2.5\n'
    text+=f'goal {goal[0]:.12f} {goal[1]:.12f}\n'
    text+=''.join(f'obstacle {n} {x0:.12f} {y0:.12f} {x1:.12f} {y1:.12f} 0.6\n' for n,x0,y0,x1,y1 in boxes)
    return text,goal


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int,default=518000)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rng=np.random.default_rng(args.seed);accepted=[];rejected=[]
    for family in ['wall','staggered','elbow','corridor']:
        count=0
        for candidate in range(2000):
            text,goal=layout(rng,family)
            path=args.output/f'{family}_candidate.txt';path.write_text(text)
            try:
                known=KnownMap.load(str(path))
                goal=list(known.goal)
                if known.direct_free(0,0,*goal):raise ValueError('Direct inflated-map segment is free.')
                trajectory=known.plan(0,0,0,*goal)
                length=trajectory.sample(trajectory.duration)[2]
                if not 1<=length<=4 or trajectory.duration>90:raise ValueError('Outside frozen length/duration bounds.')
                values=np.array([trajectory.sample(float(t)) for t in np.arange(0,trajectory.duration+0.002,0.002)],dtype='<f8')
                record={'family':family,'index':count,'candidate':candidate,'goal':goal,'start':[0,0,0],
                        'length':length,'duration':trajectory.duration,'timeout':trajectory.duration+10,
                        'scenario_sha256':hashlib.sha256(text.encode()).hexdigest(),
                        'sampled_reference_sha256':hashlib.sha256(values.tobytes()).hexdigest(),
                        'reference_hash_format':'little-endian float64, seven sample columns, times arange(0,duration+0.002,0.002)'}
                destination=args.output/f'{family}_{count:02}.txt';path.rename(destination)
                record['scenario']=destination.name;accepted.append(record);count+=1
                if count==25:break
            except (ValueError,RuntimeError) as exc:
                rejected.append({'family':family,'candidate':candidate,'scenario':text,'reason':str(exc)})
        if count!=25:raise RuntimeError(f'Only {count} geometrically valid {family} cases; no controller results were inspected.')
    result={'seed':args.seed,'count':len(accepted),'cases':accepted,'rejected':rejected,
            'initial_pitch':0,'initial_velocity':0,'physics_dt':0.001,'control_dt':0.002,
            'torque_limit':2,'success':{'distance':0.1,'velocity':0.05,'yaw_rate':0.1,'pitch':0.1,'hold':0.5},
            'selection':'Geometry only, before controller evaluation; no controller-dependent filtering.'}
    (args.output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Frozen',len(accepted),'cases; rejected',len(rejected))


if __name__=='__main__':main()
