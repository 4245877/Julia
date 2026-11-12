"""Independent numerical reader. Imports neither Julia controller nor solver."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
os.environ['MKL_NUM_THREADS']='1'
from pathlib import Path
import argparse,hashlib,json,struct
import numpy as np

def main():
    a=argparse.ArgumentParser();a.add_argument('--corpus',required=True,type=Path);a.add_argument('--output',required=True,type=Path);args=a.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    data=args.corpus.read_bytes();assert data[:8]==b'J7IQP001';pos=8;count=0;active=set();scenarios={}
    maxima={k:0. for k in ('objective_relative','primal_difference','equality','root_dynamic_scaled_100','contact_acceleration','inequality','projected_stationarity','complementarity','dual_negative','full_stationarity_relative')}
    def take(n):
        nonlocal pos
        v=np.frombuffer(data,dtype='<f8',count=n,offset=pos).copy();pos+=n*8;return v
    with (args.output/'residuals.csv').open('w') as raw:
        raw.write('scenario,tick,solver,objective,equality,root_dynamic_scaled_100,contact_acceleration,inequality,projected_stationarity,complementarity,dual_negative,full_stationarity_relative\n')
        while pos<len(data):
            scenario,tick,n,e,m=struct.unpack_from('<iIiii',data,pos);pos+=20
            assert (n,e,m)==(71,18,124)
            h=take(71*71).reshape(71,71);g=take(71);E=take(18*71).reshape(18,71);d=take(18);A=take(128*71).reshape(128,71)[:m];b=take(128)[:m]
            kkt=np.block([[h,E.T],[E,np.zeros((e,e))]])
            results=[]
            for solver in ('reference','optimized'):
                x=take(71);mu=take(128)[:m];iterations,numactive=struct.unpack_from('<II',data,pos);pos+=8
                assert iterations<=128
                gradient=h@x-g-A.T@mu
                # Independent dense KKT solve: upper solution is P*gradient.
                projected=np.linalg.solve(kkt,np.r_[gradient,np.zeros(e)])[:n]
                # Equality multipliers reconstructed, not compared across implementations.
                lam=np.linalg.lstsq(E.T,-gradient,rcond=None)[0]
                full=np.linalg.norm(gradient+E.T@lam,np.inf)/max(1.,np.linalg.norm(h@x,np.inf),np.linalg.norm(g,np.inf),np.linalg.norm(A.T@mu,np.inf))
                eq=E@x-d;slack=A@x-b
                r=dict(equality=float(abs(eq).max()),root_dynamic_scaled_100=float(abs(eq[12:]).max()*100),contact_acceleration=float(abs(eq[:12]).max()),inequality=float(max(0.,-slack.min())),projected_stationarity=float(abs(projected).max()),complementarity=float(abs(mu*slack).max()),dual_negative=float(max(0.,-mu.min())),full_stationarity_relative=float(full))
                assert r['equality']<=1e-6 and r['root_dynamic_scaled_100']<1e-6 and r['contact_acceleration']<1e-6,r
                assert r['inequality']<=1e-7 and r['projected_stationarity']<=1e-5 and r['complementarity']<=1e-5 and r['dual_negative']<=1e-8,r
                objective=float(.5*x@h@x-g@x)
                raw.write(f'{scenario},{tick},{solver},{objective},'+','.join(str(r[k]) for k in ('equality','root_dynamic_scaled_100','contact_acceleration','inequality','projected_stationarity','complementarity','dual_negative','full_stationarity_relative'))+'\n')
                for k,v in r.items():maxima[k]=max(maxima[k],v)
                results.append((x,objective))
                active.update(np.flatnonzero(mu>1e-9).tolist())
            difference=float(abs(results[0][0]-results[1][0]).max());relative=abs(results[0][1]-results[1][1])/max(1.,abs(results[0][1]))
            maxima['objective_relative']=max(maxima['objective_relative'],relative);maxima['primal_difference']=max(maxima['primal_difference'],difference)
            assert difference<=1e-5 and relative<=1e-8
            count+=1;scenarios[scenario]=scenarios.get(scenario,0)+1
    assert pos==len(data) and count>=300 and len(active)>1 and len(scenarios)==2
    certificate=dict(pass_all=True,problems=count,solutions_checked=2*count,scenarios=scenarios,active_constraint_ids=sorted(active),maxima=maxima,corpus_sha256=hashlib.sha256(data).hexdigest(),method='Independent NumPy KKT solve; equality multipliers reconstructed by least squares; all inequalities and complementarity evaluated')
    (args.output/'certificate.json').write_text(json.dumps(certificate,indent=2));print(json.dumps(certificate,indent=2))
if __name__=='__main__':main()
