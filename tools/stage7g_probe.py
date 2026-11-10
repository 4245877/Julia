"""Scratch inverse-dynamics experiment; no Julia runtime state is touched."""
from pathlib import Path
import sys,json,argparse,time
sys.path.append(str(Path('build/diagnostics/stage7g/python-deps').resolve()))
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation as R

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=14);p.add_argument('--feedback',type=float,default=1);p.add_argument('--stride',type=int,default=4)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
fixture=json.loads(Path('build/diagnostics/stage7g/preparation-03/prepared.json').read_text())
m=mj.MjModel.from_xml_path('build/diagnostics/stage7f/parity-final/drop-headless/model.xml');d=mj.MjData(m)
d.qpos[:7]=fixture['root'];d.qpos[7:]=fixture['q'];mj.mj_forward(m,d)
q0=d.qpos.copy();com0=d.subtree_com[1].copy();caps=m.actuator_ctrlrange[:,1]
feet=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,f'g{i}') for i in (16,19)]
pos0=[d.geom_xpos[g].copy()+[0,-.025,0] for g in feet];rot0=[d.geom_xmat[g].reshape(3,3).copy() for g in feet]
lambda0=np.zeros(12)
for s in range(2):
    for i in range(4):
        f=np.array([0,fixture['audit']['normal_corner_N'][4*s+i],0])
        lambda0[6*s:6*s+3]+=f
        lambda0[6*s+3:6*s+6]+=np.cross(np.array(fixture['audit']['points'][4*s+i])-pos0[s],f)
def footjac(g):
    jp=np.zeros((3,47));jr=np.zeros((3,47));point=d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@np.array([0,-.025,0]);mj.mj_jac(m,d,jp,jr,point,m.geom_bodyid[g]);return point,np.vstack([jp,jr])
mass=np.zeros((47,47));prevj=None;prevjc=None;held=np.zeros(41);log=[]
for step in range(round(a.seconds/m.opt.timestep)):
    t=d.time
    if step%a.stride==0:
        jc=np.zeros((3,47));mj.mj_jacSubtreeCom(m,d,jc,1)
        points,jacs=zip(*(footjac(g) for g in feet));J=np.vstack(jacs)
        jdotv=np.zeros(12) if prevj is None else ((J-prevj)/(m.opt.timestep*a.stride))@d.qvel
        jcdotv=np.zeros(3) if prevjc is None else ((jc-prevjc)/(m.opt.timestep*a.stride))@d.qvel
        prevj=J.copy();prevjc=jc.copy()
        acc=np.zeros(47);acc[6:]=100*(q0[7:]-d.qpos[7:])-20*d.qvel[6:]
        orient=R.from_quat(d.qpos[3:7],scalar_first=True)
        acc[3:6]=a.feedback*(-100*orient.as_rotvec()-20*d.qvel[3:6])
        desired=a.feedback*(40*(com0-d.subtree_com[1])-12*(jc@d.qvel))
        desired[1]=100*(com0[1]-d.subtree_com[1,1])-20*(jc@d.qvel)[1]
        # Weighted acceleration task, strict local contact acceleration. Joint
        # posture regularizes all leg coordinates; root remains unactuated.
        # Sticking contact acceleration, without commanding foot repositioning.
        # Actual support displacement/rotation is monitored against a small envelope.
        rhs=-20*(J@d.qvel)
        for s in range(2):
            delta=pos0[s]-points[s];delta[1]=0
            rhs[6*s:6*s+3]+=100*delta
            rhs[6*s+3:6*s+6]+=100*R.from_matrix(rot0[s]@d.geom_xmat[feet[s]].reshape(3,3).T).as_rotvec()
        task=np.vstack([np.eye(47)[6:],30*jc,10*np.eye(47)[3:6]])
        goal=np.r_[acc[6:],30*(desired-jcdotv),10*acc[3:6]]
        Hacc=task.T@task+1e-8*np.eye(47)
        Kacc=np.block([[Hacc,J.T],[J,np.zeros((12,12))]])
        acc=np.linalg.solve(Kacc,np.r_[task.T@goal,rhs-jdotv])[:47]
        mj.mj_fullM(m,mass,d.qM);req=mass@acc+d.qfrc_bias-d.qfrc_passive
        B=J.T;T=B[6:]/caps[:,None];r=req[6:]/caps
        # Quadratic cost regularizes forces and foot moments and avoids high actuator utilization.
        weights=np.tile([1/10,1/200,1/10,1/10,1/.2,1/10],2)
        H=.001*T.T@T+np.diag(weights**2);v=.001*T.T@r+weights**2*lambda0
        K=np.block([[H,B[:6].T],[B[:6],np.zeros((6,6))]])
        lam=np.linalg.solve(K,np.r_[v,req[:6]])[:12]
        held=req[6:]-B[6:]@lam
        cop=[];valid=True
        for s,g in enumerate(feet):
            rf=d.geom_xmat[g].reshape(3,3).T@lam[6*s:6*s+3];rm=d.geom_xmat[g].reshape(3,3).T@lam[6*s+3:6*s+6]
            cop.extend([rf[1],rm[2]/rf[1],-rm[0]/rf[1]])
            valid &= rf[1]>0 and np.hypot(rf[0],rf[2])<.6*rf[1] and abs(rm[2])<.045*rf[1] and abs(rm[0])<.085*rf[1]
        footmove=max(np.linalg.norm((points[s]-pos0[s])[[0,2]]) for s in range(2))
        footangle=max(np.linalg.norm(R.from_matrix(rot0[s]@d.geom_xmat[feet[s]].reshape(3,3).T).as_rotvec()) for s in range(2))
        slip=max(np.linalg.norm((J@d.qvel).reshape(2,6)[:,[0,2]],axis=1))
        pen=max([0]+[-c.dist for c in d.contact]);other=sum(c.geom1==0 and c.geom2 not in feet for c in d.contact)
        row=[t,*d.subtree_com[1],*(jc@d.qvel),*d.qpos[:3],*orient.as_rotvec(),max(abs(held/caps)),int(valid),*cop,footmove,footangle,slip,pen,other]
        log.append(row)
        if step%2400==0:print(row,flush=True)
    d.ctrl[:]=np.clip(held,-caps,caps)
    d.xfrc_applied[:]=0
    if 5<=t<5.1:d.xfrc_applied[mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'b3'),0]=3
    if 9<=t<9.1:d.xfrc_applied[mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'b3'),2]=3
    mj.mj_step(m,d);mj.mj_forward(m,d)
    if np.linalg.norm(d.qvel)>100:print('DIVERGED',t);break
np.savetxt(a.output/'probe.csv',log,delimiter=',',header='t,comx,comy,comz,vx,vy,vz,rootx,rooty,rootz,rx,ry,rz,util,feasible,lfn,lcopx,lcopz,rfn,rcopx,rcopz,footmove,footangle,slip,penetration,other',comments='')
np.savetxt(a.output/'final.csv',np.c_[d.qpos[7:],d.qvel[6:],d.qacc[6:],acc[6:],held,caps],delimiter=',',header='q,v,actual_acc,desired_acc,tau,cap',comments='')
print('rootacc',d.qacc[:6],acc[:6]);print('max qerror',max(abs(d.qpos[7:]-q0[7:])))
print('desired wrench',lam)
actual=np.zeros(12)
for i,c in enumerate(d.contact):
    for s,g in enumerate(feet):
        if g not in c.geom:continue
        f=np.zeros(6);mj.mj_contactForce(m,d,i,f);w=c.frame.reshape(3,3).T@f[:3];mom=c.frame.reshape(3,3).T@f[3:]
        if c.geom1==g:w=-w;mom=-mom
        actual[6*s:6*s+3]+=w;actual[6*s+3:6*s+6]+=np.cross(c.pos-points[s],w)+mom
print('actual wrench',actual);print('desired feetacc',rhs);print('actual feetacc',J@d.qacc+jdotv)
