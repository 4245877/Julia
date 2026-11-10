"""Bounded offline stance preparation on exported, unmodified real BodyRig MJCF.

Only scratch Python mjData; output is an explicit reset fixture, never live control.
Uses the same calibrated FK as the existing leg IK, with double precision least
squares. Finite seed budget and an independent static contact/torque LP.
"""
from pathlib import Path
import sys, json, argparse, hashlib
sys.path.append(str(Path('build/diagnostics/stage7g/python-deps').resolve()))
import mujoco as mj
import numpy as np
from scipy.optimize import least_squares, linprog
from scipy.spatial.transform import Rotation

p=argparse.ArgumentParser()
p.add_argument('--output',type=Path,required=True)
p.add_argument('--model',type=Path,default=Path('build/diagnostics/stage7f/parity-final/drop-headless/model.xml'))
a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=False)
m=mj.MjModel.from_xml_path(str(a.model.resolve())); d=mj.MjData(m)
qadr=np.array([m.jnt_qposadr[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,f'j{k}')] for k in range(41)])
vadr=np.array([m.jnt_dofadr[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,f'j{k}')] for k in range(41)])
limits=np.array([m.jnt_range[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,f'j{k}')] for k in range(41)])
caps=m.actuator_ctrlrange[:,1].copy()
geoms=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,f'g{i}') for i in (16,19)]
legs=np.arange(29,41)
upper=np.zeros(41);upper[[15,22]]=-.12;upper[[17,24]]=-.12
mj.mj_forward(m,d)
foot_rotations=[Rotation.from_matrix(d.geom_xmat[g].reshape(3,3)).as_matrix() for g in geoms]
def state(q,height):
    mj.mj_resetData(m,d); d.qpos[1]=height; d.qpos[qadr]=q; mj.mj_forward(m,d)
def foot_residual(x,height,z):
    q=upper.copy(); q[legs]=x; state(q,height)
    res=[]
    for side,g in enumerate(geoms):
        res.extend(d.geom_xpos[g]-[(.12 if side==0 else -.12),.025,z+(-.06 if side==0 else .06)-.035])
        res.extend(.2*Rotation.from_matrix(foot_rotations[side]@d.geom_xmat[g].reshape(3,3).T).as_rotvec())
    return np.array(res)
def static_audit(q,height):
    state(q,height); jac=[]; points=[]
    for g in geoms:
        r=d.geom_xmat[g].reshape(3,3)
        # Conservative inner contact region: physical box is unchanged.
        for x,z in ((-.036,-.068),(-.036,.068),(.036,-.068),(.036,.068)):
            point=d.geom_xpos[g]+r@np.array([x,-.025,z]); points.append(point.copy())
            jp=np.zeros((3,m.nv)); mj.mj_jac(m,d,jp,None,point,m.geom_bodyid[g]); jac.append(jp[1].copy())
    j=np.array(jac).T; b=d.qfrc_bias.copy()
    # Variables eight unilateral corner normal forces and peak actuator utilization.
    aub=np.zeros((82,9)); bub=np.empty(82)
    for k,v in enumerate(vadr):
        aub[2*k,:8]=-j[v]; aub[2*k,8]=-caps[k]; bub[2*k]=-b[v]
        aub[2*k+1,:8]=j[v]; aub[2*k+1,8]=-caps[k]; bub[2*k+1]=b[v]
    eq=np.zeros((6,9)); eq[:,:8]=j[:6]
    lp=linprog([0]*8+[1],A_ub=aub,b_ub=bub,A_eq=eq,b_eq=b[:6],bounds=[(0,None)]*9,method='highs')
    if not lp.success:return dict(feasible=False,message=lp.message)
    f=lp.x[:8]; tau=b[vadr]-j[vadr]@f
    other=[]
    for c in d.contact:
        if c.geom1 not in geoms and c.geom2 not in geoms:other.append(dict(g1=int(c.geom1),g2=int(c.geom2),distance=float(c.dist)))
    return dict(feasible=bool(lp.x[8]<.9),peak_utilization=float(lp.x[8]),normal_corner_N=f.tolist(),
                torque_Nm=tau.tolist(),torque_caps_Nm=caps.tolist(),base_residual=(b[:6]-j[:6]@f).tolist(),
                com=d.subtree_com[1].tolist(),points=np.array(points).tolist(),other_contacts=other,
                feet=[dict(center=d.geom_xpos[g].tolist(),rotation=d.geom_xmat[g].reshape(3,3).tolist()) for g in geoms])
results=[]; best=None
for height in (.80,.83,.86):
    for z in (-.04,0,.04,.08):
        seed=np.zeros(12)
        sol=least_squares(foot_residual,seed,args=(height,z),bounds=(limits[legs,0]+.001,limits[legs,1]-.001),
                          max_nfev=400,ftol=1e-11,xtol=1e-11,gtol=1e-11)
        q=upper.copy();q[legs]=sol.x;err=float(np.linalg.norm(sol.fun))
        audit=static_audit(q,height)
        row=dict(height=height,z=z,nfev=sol.nfev,error=err,q=q.tolist(),audit=audit);results.append(row)
        print(height,z,sol.nfev,err,audit.get('peak_utilization'),flush=True)
        if err<1e-5 and audit.get('feasible') and not audit['other_contacts']:
            if best is None or audit['peak_utilization']<best['audit']['peak_utilization']:best=row
(a.output/'search.json').write_text(json.dumps(results,indent=2))
if best is None:raise RuntimeError('Bounded local search found no admissible fixture; not a global impossibility proof')
best['model_sha256']=hashlib.sha256(a.model.read_bytes()).hexdigest()
best['root']=[0,best['height'],0,1,0,0,0]
(a.output/'prepared.json').write_text(json.dumps(best,indent=2))
print('SELECTED',best['height'],best['z'],best['audit']['peak_utilization'])
