"""Independent CSV reader for actual stage7g measurements, using only NumPy.

No C++ controller import, simulation, or acceptance values from its summary.
"""
import argparse,csv,json,math
from pathlib import Path
import numpy as np

ACCEPTED=('nominal','pulse','offset-x-minus','offset-x-plus','offset-z','tilt','translated-yaw','substeps32','contact008','contact012')
def read(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))
def col(rows,name):return np.array([float(r[name]) for r in rows])
def vector(rows,names):return np.array([[float(r[n]) for n in names.split()] for r in rows])
def angle(q,r):return 2*np.arccos(np.clip(np.abs(np.sum(q*r,axis=1)),0,1))
def check_case(path):
    measured=read(path/'measurements-v1.csv');control=read(path/'stand-v1.csv')
    assert len(measured)==len(control)==1921,path
    t=col(measured,'time');window=t>2
    com=vector(measured,'com_x com_y com_z');goal=vector(control,'goal_com_x goal_com_y goal_com_z')
    error=np.linalg.norm((com-goal)[:,[0,2]],axis=1)
    speed=np.linalg.norm(vector(measured,'v_x v_y v_z')[:,[0,2]],axis=1)
    rootgoal=vector(control,'goal_qw goal_qx goal_qy goal_qz')
    tilt=angle(vector(measured,'qw qx qy qz'),rootgoal)
    thorax=angle(vector(measured,'thorax_qw thorax_qx thorax_qy thorax_qz'),rootgoal)
    height=np.abs(col(measured,'root_y')-col(control,'goal_root_y'))
    maxima=dict(com_error_m=float(error[window].max()),speed_m_s=float(speed[window].max()),
                tilt_rad=float(tilt[window].max()),thorax_tilt_rad=float(thorax[window].max()),height_error_m=float(height[window].max()))
    assert maxima['com_error_m']<=.035 and maxima['speed_m_s']<=.12
    assert maxima['tilt_rad']<=.10 and maxima['thorax_tilt_rad']<=.10 and maxima['height_error_m']<=.025
    for side in ('left','right'):
        foot=vector(measured,f'{side}_x {side}_y {side}_z')
        displacement=np.linalg.norm((foot-foot[0])[:,[0,2]],axis=1)
        rotation=vector(measured,f'{side}_qw {side}_qx {side}_qy {side}_qz')
        footangle=angle(rotation,np.tile(rotation[0],(len(rotation),1)))
        assert displacement[window].max()<=.010 and footangle[window].max()<=.05
        assert col(measured,side+'_fn')[window].min()>=30
        assert col(measured,side+'_loaded')[window].min()==1
        assert col(measured,side+'_slip')[window].max()<=.05
        maxima[side+'_foot_displacement_m']=float(displacement[window].max())
        maxima[side+'_foot_angle_rad']=float(footangle[window].max())
        maxima[side+'_slip_m_s']=float(col(measured,side+'_slip')[window].max())
        maxima[side+'_min_normal_N']=float(col(measured,side+'_fn')[window].min())
    assert col(measured,'nonfoot')[window].max()==0
    for key,bound in [('penetration',.004),('limit',.005),('actual_utilization',1.000000001)]:
        maxima[key]=float(col(measured,key)[window].max());assert maxima[key]<=bound
    assert col(measured,'saturated_coordinates')[window].mean()/41<=.01
    assert all(r['phase']=='holding' for r,use in zip(control,window) if use)
    assert float(control[-1]['hold_s'])>=10
    assert max(error[-240:])<=.006 and max(speed[-240:])<=.015
    maxima['hold_s']=float(control[-1]['hold_s'])
    maxima['rms_error_m']=float(np.sqrt(np.mean(error[window]**2)))
    maxima['pulses']=[]
    for start,axis in ((5,'x'),(9,'z')):
        force=vector(measured,'fx fy fz');dt=np.r_[0,np.diff(t)]
        pulse=(t>start+1e-5)&(t<start+.10001)
        if not np.any(np.abs(force[pulse])>1):continue
        impulse=np.sum(force[pulse]*dt[pulse,None],axis=0)
        assert abs(np.linalg.norm(impulse)-.3)<1e-9
        before=(t>start-.5)&(t<=start)
        center=com[before][:,[0,2]].mean(axis=0)
        displacement=np.linalg.norm(com[:,[0,2]]-center,axis=1)
        response=(t>=start)&(t<=start+1)
        peak=float(displacement[response].max());peak_speed=float(speed[response].max())
        assert peak>5e-5 and peak_speed>.001
        settled=None
        for i in np.flatnonzero((t>=start+.1)&(t<=start+3)):
            if i+120<len(t) and displacement[i:i+120].max()<=peak*.2+2e-5 and speed[i:i+120].max()<=.0005:
                settled=float(t[i]-(start+.1));break
        assert settled is not None,(path,start,peak)
        maxima['pulses'].append(dict(axis=axis,impulse_world_Ns=impulse.tolist(),peak_displacement_m=peak,peak_speed_m_s=peak_speed,settled_after_pulse_s=settled))
    return maxima
def main():
    p=argparse.ArgumentParser();p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    result={name:check_case(a.suite/name) for name in ACCEPTED}
    on=read(a.suite/'pulse/measurements-v1.csv');off=read(a.suite/'feedback-off/measurements-v1.csv')
    # Every actual measured field is identical up to the explicit ablation boundary.
    assert on[:481]==off[:481]
    goal=vector(read(a.suite/'pulse/stand-v1.csv'),'goal_com_x goal_com_y goal_com_z')
    oc=vector(on,'com_x com_y com_z');fc=vector(off,'com_x com_y com_z');n=len(off)
    eo=np.linalg.norm((oc[:n]-goal[:n])[:,[0,2]],axis=1);ef=np.linalg.norm((fc-goal[:n])[:,[0,2]],axis=1)
    causal=dict(common_ticks=n-1,rms_on_m=float(np.sqrt(np.mean(eo[481:]**2))),rms_off_m=float(np.sqrt(np.mean(ef[481:]**2))))
    assert causal['rms_on_m']<causal['rms_off_m']*.5
    result['causal']=causal
    for name in ('weak','outside-initial','support-loss','outside-force'):
        r=read(a.suite/name/'stand-v1.csv');m=read(a.suite/name/'measurements-v1.csv')
        lost=next(i for i,x in enumerate(r) if x['phase']=='lost')
        assert all(float(x['actual_utilization'])==0 for x in m[lost+1:])
        result[name]=dict(lost_time_s=float(r[lost]['time_s']),reason=r[lost]['reason'],physical_root_drop_m=float(m[0]['root_y'])-float(m[-1]['root_y']))
        if name=='support-loss':assert min(col(m,'left_fn')[lost:])<1
    timings=read(a.suite/'performance.csv')
    result['performance']={key:dict(mean=float(col(timings,key).mean()),p95=float(np.sort(col(timings,key))[len(timings)*95//100]),max=float(col(timings,key).max())) for key in ('tick_us','controller_us')}
    assert result['performance']['tick_us']['p95']<8333
    (a.output/'audit.json').write_text(json.dumps(result,indent=2))
    (a.output/'audit.txt').write_text('PASS: independent measured geometry, support loads, hold, pulse recovery, causal comparison, zero effort after loss and raw timings\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
