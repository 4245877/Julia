"""Independent corner-force LP certificate for exported desired foot wrenches.

This only reads CSV. Each foot is a physical 90x170mm rectangle, with four
unilateral point forces in a conservative eight-facet subset of mu=0.6 cone.
"""
from pathlib import Path
import argparse,json,csv
import numpy as np
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation

p=argparse.ArgumentParser();p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
A=np.zeros((6,12));C=np.zeros((32,12))
corners=[(-.045,0,-.085),(-.045,0,.085),(.045,0,-.085),(.045,0,.085)]
for i,r in enumerate(corners):
    x,y,z=r;A[:3,i*3:i*3+3]=np.eye(3);A[3:,i*3:i*3+3]=[[0,-z,y],[z,0,-x],[-y,x,0]]
    for k in range(8):
        theta=2*np.pi*k/8;C[8*i+k,i*3:i*3+3]=[np.cos(theta),-.6*np.cos(np.pi/8),np.sin(theta)]
bounds=[(None,None),(0,None),(None,None)]*4
summary={}
for name in ('nominal','pulse','translated-yaw','contact008','contact012'):
    with (a.suite/name/'stand-v1.csv').open() as f:control=list(csv.DictReader(f))
    with (a.suite/name/'measurements-v1.csv').open() as f:measured=list(csv.DictReader(f))
    residual=0;count=0
    with (a.output/(name+'-certificate.csv')).open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['tick','side','residual',*[f'corner{i}_{axis}_N' for i in range(4) for axis in 'xyz']])
        for c,m in zip(control,measured):
            if float(c['time_s'])<=2:continue
            for side in ('left','right'):
                q=[float(m[side+'_'+x]) for x in ('qw','qx','qy','qz')]
                r=Rotation.from_quat(q,scalar_first=True).as_matrix()
                force=np.array([float(c[side+'_pred_f'+axis]) for axis in 'xyz'])
                moment=np.array([float(c[side+'_pred_m'+axis]) for axis in 'xyz'])
                target=np.r_[r.T@force,r.T@moment]
                lp=linprog(np.zeros(12),A_ub=C,b_ub=np.zeros(32),A_eq=A,b_eq=target,bounds=bounds,method='highs')
                assert lp.success,(name,c['tick'],side,lp.message)
                err=float(abs(A@lp.x-target).max());assert err<1e-7
                residual=max(residual,err);count+=1
                writer.writerow([c['tick'],side,err,*lp.x])
    summary[name]=dict(feasible_wrenches=count,max_residual=residual)
(a.output/'certificate.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
