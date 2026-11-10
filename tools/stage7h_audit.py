"""Independent reader: measurements and MuJoCo dynamics, no controller/solver code imported."""
from pathlib import Path
import argparse,csv,json,sys
import xml.etree.ElementTree as ET
import numpy as np

def rows(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))
def vec(r,prefix,axes=('x','y','z')):return np.array([float(r[prefix+x]) for x in axes])
def measured(path):
    state=rows(path/'state.csv'); com=rows(path/'com-v1.csv'); stand=rows(path/'stand-v1.csv')
    assert len(state)==len(com)==len(stand)
    n=len(state); t=np.arange(n)/120
    actual=np.array([vec(r,'com_',('x_m','y_m','z_m')) for r in state])
    velocity=np.array([vec(r,'p',('x_kg_m_s','y_kg_m_s','z_kg_m_s'))/55 for r in state])
    goal=np.array([vec(r,'command_') for r in com]); ref=np.array([vec(r,'reference_') for r in com])
    error=np.linalg.norm((actual-goal)[:,[0,2]],axis=1); speed=np.linalg.norm(velocity[:,[0,2]],axis=1)
    support=rows(path/'support-v1.csv'); feet={side:[r for r in support if r['side']==name] for side,name in (('left','LeftFoot'),('right','RightFoot'))}
    assert all(len(v)==n for v in feet.values())
    loaded=np.ones(n,dtype=bool)
    report={}
    for side,rs in feet.items():
        loaded &= np.array([r['loaded']=='1' and int(r['active'])>0 and float(r['normal_N'])>=30 and r['other_active']=='0' for r in rs])
        report[side]=dict(min_normal_N=min(float(r['normal_N']) for r in rs[240:]),max_slip_m_s=max(float(r['slip_m_s']) for r in rs[240:]),max_penetration_m=max(float(r['penetration_m']) for r in rs))
        positions=np.array([vec(r,'',('ox_m','oy_m','oz_m')) for r in rs]);report[side]['sole_displacement_m']=float(np.linalg.norm((positions-positions[0])[:,[0,2]],axis=1).max())
    serial=np.array([int(r['command_serial']) for r in com]); commands=[]
    for serial_id in np.unique(serial):
        if serial_id==0:continue
        ids=np.flatnonzero(serial==serial_id); good=(error<=.0005)&(speed<=.001)&loaded
        continuous=0.; best=0.; achieved=None
        for i in ids:
            continuous=continuous+1/120 if good[i] else 0.; best=max(best,continuous)
            if continuous>=2-1e-9 and achieved is None:achieved=float(t[i])
        final=ids[-1]; commands.append(dict(serial=int(serial_id),goal_world_m=goal[final].tolist(),start_s=float(t[ids[0]]),end_s=float(t[final]),best_measured_hold_s=best,first_measured_reached_s=achieved,final_error_m=float(error[final]),final_speed_m_s=float(speed[final]),measured_displacement_m=(actual[final]-actual[0]).tolist()))
    rv=np.array([vec(r,'reference_v_') for r in com]); ra=np.array([vec(r,'reference_a_') for r in com])
    report.update(commands=commands,max_measured_displacement_m=float(np.linalg.norm((actual-actual[0])[:,[0,2]],axis=1).max()),max_reference_speed=float(np.linalg.norm(rv,axis=1).max()),max_reference_acceleration=float(np.linalg.norm(ra,axis=1).max()),lost=[dict(tick=r['tick'],reason=r['reason']) for r in stand if r['phase']=='lost'][:1],max_requested_utilization=max(float(r['requested_utilization']) for r in stand),max_active_caps=max(int(r['active_caps']) for r in com),max_active_friction=max(int(r['active_friction']) for r in com),max_qp_iterations=max(int(r['qp_iterations']) for r in com))
    assert report['max_reference_speed']<=.001000001 and report['max_reference_acceleration']<=.002000001
    # Independently verify both floor reactions and normal moments use the exported common origin.
    origin_error=0.
    for side in ('left','right'):
        for c,s,f in zip(com,stand,feet[side]):
            force=vec(f,'',('fx_N','fy_N','fz_N')); moment=vec(f,'',('mx_Nm','my_Nm','mz_Nm'))
            old=vec(f,'',('ox_m','oy_m','oz_m')); new=vec(s,side+'_pred_origin_')
            exported=vec(c,side+'_measured_at_pred_',('mx','my','mz'))
            origin_error=max(origin_error,float(np.max(np.abs(moment+np.cross(old-new,force)-exported))))
    assert origin_error<1e-9;report['wrench_origin_transport_error']=origin_error
    xml=ET.parse(path/'model.xml').getroot();caps={int(x.attrib['name'][5:]):float(x.attrib['ctrlrange'].split()[1]) for x in xml.find('actuator')}
    actual_util=0.;run=0.;limit=0.;saturated=np.zeros(41)
    with (path/'drive-v1.csv').open(newline='') as f:
        for r in csv.DictReader(f):
            k=int(r['coordinate']);actual_util=max(actual_util,abs(float(r['actual_Nm']))/caps[k]);run=max(run,float(r['saturation_run_s']));saturated[k]=float(r['saturated_s'])
    with (path/'joints.csv').open(newline='') as f:
        for r in csv.DictReader(f):limit=max(limit,float(r['peak_limit_rad']))
    report.update(max_actual_tau_cap=actual_util,max_saturation_run_s=run,saturated_coordinate_time=float(saturated.sum()/(41*t[-1])),max_joint_violation=limit,max_actual_speed=float(speed.max()),continuous_two_supports_after_2s=bool(loaded[241:].all()))
    pulses=[]
    force=np.array([vec(r,'external_f',('x_N','y_N','z_N')) for r in state])
    enabled=np.linalg.norm(force,axis=1)>1.;starts=np.flatnonzero(enabled&~np.r_[False,enabled[:-1]])
    for start in starts:
        end=start
        while end+1<n and enabled[end+1]:end+=1
        impulse=force[start:end+1].sum(axis=0)/120;assert abs(np.linalg.norm(impulse)-.3)<1e-9
        good=(error<=.0005)&(speed<=.001)&loaded;recovery=None
        for i in range(end+1,min(end+361,n-240)):
            if good[i:i+240].all():recovery=(i-end)/120;break
        assert recovery is not None
        pulses.append(dict(start_s=float(t[start]-1/120),impulse_Ns=impulse.tolist(),goal_recovery_s=recovery))
    report['pulses']=pulses
    return report

def dynamics(path):
    # Reuse read-only, pinned offline dependency. No install/global environment modification.
    sys.path.insert(0,str(Path('build/diagnostics/stage7g/python-deps').resolve()))
    import mujoco as mj
    assert mj.__version__=='3.3.7'
    m=mj.MjModel.from_xml_path(str((path/'model.xml').resolve())); d=mj.MjData(m)
    assert (m.nq,m.nv,m.nu,m.neq)==(48,47,41,0) and abs(m.body_mass.sum()-55)<1e-10
    joint=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,f'j{i}') for i in range(41)]
    qa=m.jnt_qposadr[joint];va=m.jnt_dofadr[joint]
    root=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'root');rq,rv=m.jnt_qposadr[root],m.jnt_dofadr[root]
    act=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_ACTUATOR,f'motor{i}') for i in range(41)];caps=m.actuator_ctrlrange[act,1]
    control={r['tick']:r for r in rows(path/'stand-v1.csv')};corners={r['tick']:r for r in rows(path/'com-v1.csv')}
    mass=np.zeros((47,47)); jp=np.zeros((3,47));jr=np.zeros((3,47));count=0
    peak=dict(root_residual_N_Nm=0.,torque_reconstruction_Nm=0.,contact_acceleration_residual=0.,wrench_corner_reconstruction=0.,corner_friction_violation_N=0.,corner_normal_violation_N=0.,max_tau_cap=0.,corner_geometry_error_m=0.)
    for r in rows(path/'com-solver-v1.csv'):
        if r['solver']!='feasible':continue
        s=control[r['tick']]; c=corners[r['tick']]
        d.qpos[rq:rq+3]=vec(r,'root_p');d.qpos[rq+3:rq+7]=[float(r['root_q'+x]) for x in ('w','x','y','z')]
        d.qvel[rv:rv+3]=vec(r,'root_v');d.qvel[rv+3:rv+6]=vec(r,'root_w')
        d.qpos[qa]=[float(r[f'q{i}']) for i in range(41)];d.qvel[va]=[float(r[f'v{i}']) for i in range(41)]
        mj.mj_forward(m,d);mj.mj_fullM(m,mass,d.qM)
        a=np.array([float(r[f'a{i}']) for i in range(47)]);required=mass@a+d.qfrc_bias-d.qfrc_passive
        generalized=np.zeros(47);contact_rows=[]
        for side,link in (('left',16),('right',19)):
            geom=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,f'g{link}');rotation=d.geom_xmat[geom].reshape(3,3)
            x=rotation[:,0].copy();x[1]=0.;x/=np.linalg.norm(x);force_frame=np.column_stack((x,np.array([0.,1.,0.]),np.cross(x,np.array([0.,1.,0.]))))
            origin=vec(s,side+'_pred_origin_');force=vec(s,side+'_pred_',('fx','fy','fz'));moment=vec(s,side+'_pred_',('mx','my','mz'))
            mj.mj_jac(m,d,jp,jr,origin,int(m.geom_bodyid[geom]));generalized+=jp.T@force+jr.T@moment;contact_rows.extend(np.vstack((jp,jr)).copy())
            fs=np.zeros(3);ms=np.zeros(3);normal=0.
            for corner in range(4):
                point=vec(c,f'{side}_corner{corner}_p');f=vec(c,f'{side}_corner{corner}_f');local=force_frame.T@f
                assert np.linalg.norm(f[[0,2]])<=.6*f[1]+1e-6 and f[1]>=-1e-6
                expected=d.geom_xpos[geom]+rotation@np.array([.045 if corner&1 else -.045,-.025,.085 if corner&2 else -.085])
                peak['corner_geometry_error_m']=max(peak['corner_geometry_error_m'],float(np.max(np.abs(point-expected))))
                fs+=f;ms+=np.cross(point-origin,f);normal+=local[1]
                peak['corner_friction_violation_N']=max(peak['corner_friction_violation_N'],float(abs(local[0])+abs(local[2])-.6*local[1]))
                peak['corner_normal_violation_N']=max(peak['corner_normal_violation_N'],float(-local[1]))
            assert normal>=30-1e-6
            peak['wrench_corner_reconstruction']=max(peak['wrench_corner_reconstruction'],float(np.max(np.abs(fs-force))),float(np.max(np.abs(ms-moment))))
        tau=np.array([float(r[f'tau{i}']) for i in range(41)])
        peak['root_residual_N_Nm']=max(peak['root_residual_N_Nm'],float(np.max(np.abs((required-generalized)[rv:rv+6]))))
        peak['torque_reconstruction_Nm']=max(peak['torque_reconstruction_Nm'],float(np.max(np.abs((required-generalized)[va]-tau))))
        peak['max_tau_cap']=max(peak['max_tau_cap'],float(np.max(np.abs(tau)/caps)))
        contact=np.array([float(r[f'contact_a{i}'])-float(r[f'jdot{i}']) for i in range(12)])
        peak['contact_acceleration_residual']=max(peak['contact_acceleration_residual'],float(np.max(np.abs(np.array(contact_rows)@a-contact))));count+=1
    assert count>0
    for key,value in peak.items():assert value<=(1.000001 if key=='max_tau_cap' else 1e-6),(key,value)
    return dict(samples=count,**peak)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--require-goals',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    result=measured(a.run)
    if list(csv.DictReader((a.run/'com-solver-v1.csv').open())):result['independent_dynamics']=dynamics(a.run)
    (a.output/'certificate.json').write_text(json.dumps(result,indent=2))
    if a.require_goals:
        assert not result['lost'] and all(x['first_measured_reached_s'] is not None for x in result['commands'])
        assert result['max_measured_displacement_m']>=.002
        assert result['continuous_two_supports_after_2s'] and result['max_actual_tau_cap']<=1.000000001
        assert result['max_joint_violation']<=.005 and result['max_saturation_run_s']<=.1 and result['saturated_coordinate_time']<=.01
        for side in ('left','right'):
            assert result[side]['min_normal_N']>=30 and result[side]['max_slip_m_s']<=.05 and result[side]['max_penetration_m']<=.004 and result[side]['sole_displacement_m']<=.01
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
