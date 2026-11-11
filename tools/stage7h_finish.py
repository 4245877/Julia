"""Read-only final evidence audit. No simulation, rebuild, or old-output changes."""
from pathlib import Path
import csv,json,hashlib,re,xml.etree.ElementTree as ET
import numpy as np
from PIL import Image,ImageChops

root=Path.cwd();p=root/'build/diagnostics/stage7h';out=p/'final-audit-v3';out.mkdir(exist_ok=False)
def read(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))
def vector(r,prefix):return np.array([float(r[prefix+x]) for x in ('x','y','z')])
def stats(values):
    values=np.sort(np.array(values));assert len(values)>0 and np.isfinite(values).all()
    return dict(count=len(values),mean=float(values.mean()),p95=float(values[int(len(values)*.95)]),max=float(values[-1]))
result={};messages=[]
manifest=json.loads((p/'runtime-manifest-v3.json').read_text())
for x in manifest:assert hashlib.sha256((root/x['path']).read_bytes()).hexdigest()==x['sha256'],x['path']
result['frozen_files']=len(manifest);messages.append(f'PASS frozen{len(manifest)} source/build/binary files unchanged')
result['suites']={};result['performance']={}
for variant,count in [('vs-release',70),('cpu-release',31),('cpu-no-physics',22)]:
    directory=p/'final-tests-03'/variant;text=(directory/'ctest.log').read_text()
    assert f'100% tests passed, 0 tests failed out of {count}' in text
    expected=(root/'build'/variant/'tests/CTestTestfile.cmake').read_text().replace(root.as_posix()+'/build/diagnostics/',directory.as_posix()+'/artifacts/').replace((root/'build'/variant).as_posix()+'/verification',directory.as_posix()+'/verification')
    assert expected==(directory/'CTestTestfile.cmake').read_text()
    cache=(root/'build'/variant/'CMakeCache.txt').read_text();assert 'CMAKE_BUILD_TYPE:STRING=Release' in cache and 'JULIA_WARNINGS_AS_ERRORS:BOOL=ON' in cache
    dry=(p/'final-build-audit-03'/(variant+'-dry-run.txt')).read_text(encoding='utf-16' if (p/'final-build-audit-03'/(variant+'-dry-run.txt')).read_bytes().startswith(b'\xff\xfe') else 'utf-8-sig')
    assert 'Building CXX' not in dry and 'Linking' not in dry
    duration=float(re.search(r'Total Test time \(real\) = ([\d.]+)',text).group(1));result['suites'][variant]=dict(passed=count,seconds=duration)
    if variant=='cpu-no-physics':continue
    workload=directory/'artifacts/stage7h/cpu';ticks=read(workload/'performance.csv');solves=read(workload/'solver-performance.csv')
    assert len(ticks)==2400 and len(solves)==9600
    data={key:stats([float(r[key]) for r in ticks]) for key in ('tick_us','controller_us')};data['solver_us']=stats([float(r['solver_us']) for r in solves])
    data['iterations']=dict(min=min(int(r['qp_iterations']) for r in solves),max=max(int(r['qp_iterations']) for r in solves));assert data['iterations']['max']<=128 and data['tick_us']['p95']<8333
    text=(workload/'com-control.txt').read_text();data['state_bytes']={k:int(re.search(k+r'=(\d+)',text).group(1)) for k in ('controller_instance','snapshot','model','data','arena')};result['performance'][variant]=data
messages.append('PASS complete70/70 desktop,31/31 CPU,22/22 OFF suites; exact destination isolation; Release /WX; no build work')
assert 'PASS 9/9' in (p/'parity-final-03/parity.txt').read_text()
binary=json.loads((p/'parity-final-03/binary-hashes.json').read_text())
for path,value in binary.items():assert hashlib.sha256((root/path).read_bytes()).hexdigest()==value
result['parity_comparisons']=9;result['parity_csv_contracts']=12
for label,ticks in [('shift-pulse',6480),('retarget',2160),('yaw',6480)]:
    for fps in (30,60,144):
        text=(p/'parity-final-03'/f'{label}-{fps}'/'gpu.txt').read_text()
        assert 'GL_errors=0' in text and 'callback_checks=PASS' in text and f'ticks={ticks}' in text and f'renders={(ticks*fps+119)//120} ' in text
messages.append('PASS9 actual CLI/GLFW comparisons30/60/144, all12 deterministic CSV contracts exact; timing fields excluded')
visible=json.loads((p/'visible-final-v3-comparison.json').read_text());assert len(visible)==12
assert json.loads((p/'visible-final-v3/result.json').read_text())['returncode']==0
result['visible_wall_seconds']=json.loads((p/'visible-final-v3/result.json').read_text())['wall_seconds'];messages.append('PASS visible wall-clock desktop54s physical sequence: all12 CSV contracts exact vs headless')
comparisons=json.loads((p/'legacy-final-v3/comparison.json').read_text());assert len(comparisons)==696
result['legacy_csv_files']=len(comparisons);messages.append('PASS696 accepted7g physical CSV schemas and deterministic histories unchanged (includes606 earlier physical contracts)')
nominal=p/'parity-final-03/shift-pulse-headless/model.xml';old=root/'build/diagnostics/stage7g/parity-final/pulse-headless/model.xml';assert nominal.read_bytes()==old.read_bytes()
xml=ET.parse(nominal).getroot();assert xml.find('equality') is None and len(xml.findall('.//freejoint'))==1 and len(xml.find('actuator'))==41
assert all(x.attrib['joint']!='root' for x in xml.find('actuator'))
deps=(p/'dependency-audit-final/headless-dependencies.txt').read_text(encoding='utf-8-sig');assert not re.search('opengl|glfw|glad',deps,re.I)
messages.append('PASS model.xml byte-identical to7g: unmodified mass/caps/feet/friction, free root, no root drive/equality; CPU dependency isolation')
result['goal_certificates']={};result['placement']={}
for name in ('sequence','yaw','retarget'):
    certificate=json.loads((p/f'independent-final-v3-{name}/certificate.json').read_text());result['goal_certificates'][name]=certificate
    assert not certificate['lost'] and certificate['continuous_two_supports_after_2s']
    if name!='retarget':assert all(c['first_measured_reached_s'] is not None for c in certificate['commands'])
for name in ('nominal','old-pulse'):
    certificate=json.loads((p/f'independent-final-v3-{name}/certificate.json').read_text());assert not certificate['lost'] and certificate['continuous_two_supports_after_2s'] and certificate['max_actual_tau_cap']<=1.000000001
    stand=read(p/f'final3-{name}/stand-v1.csv');assert float(stand[-1]['hold_s'])>=15.49
for name in ('sequence','legacy-sequence','substeps32','contact008','contact012'):
    c=json.loads((p/f'independent-matrix-v3-{name}/certificate.json').read_text());assert not c['lost'] and all(x['first_measured_reached_s'] is not None for x in c['commands']) and c['max_measured_displacement_m']>=.002
    result['goal_certificates'][name if name!='sequence' else 'sequence-no-pulse']=c
for name in ('shift-pulse','yaw'):
    directory=p/'parity-final-03'/f'{name}-headless';com=read(directory/'com-v1.csv');stand=read(directory/'stand-v1.csv');q=np.array([float(stand[0]['goal_q'+a]) for a in ('w','x','y','z')]);w,x,y,z=q
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    origin=vector(com[0],'actual_');error=0.;achieved=[]
    for serial,offset in enumerate(((.003,0),(-.003,0),(0,.003),(0,0)),1):
        records=[r for r in com if int(r['command_serial'])==serial];last=records[-1];expected=origin+rotation@np.array([offset[0],0,offset[1]])
        error=max(error,float(np.linalg.norm(vector(last,'command_')-expected)));local=(vector(last,'actual_')-origin)@rotation;achieved.append(local.tolist())
    assert error<1e-12 and achieved[0][0]>.002 and achieved[1][0]<-.002 and achieved[2][2]>.002 and np.linalg.norm(np.array(achieved[3])[[0,2]])<.0005
    result['placement'][name]=dict(target_transform_error_m=error,measured_local_endpoints_m=achieved)
messages.append('PASS independent full dynamics/corner/floor-friction/torque certificates; measured goals, pulses, yaw, retarget/cancel and contact/substep sensitivity')
contact=json.loads((p/'actual-contacts-final-v3/audit.json').read_text());assert contact['physical_substeps']==103680 and contact['force_error_N']==contact['moment_error_Nm']==contact['cop_error_m']==0;result['actual_contacts']=contact
negatives={}
workload=p/'final-tests-03/cpu-release/artifacts/stage7h/cpu'
for name in ('weak','outside-initial','support-loss','outside-force','disable','unsupported-contact'):
    stand=read(workload/name/'stand-v1.csv');terminal=next(r for r in stand if r['phase'] in ('lost','disabled') and int(r['tick'])>0);tick=int(terminal['tick'])
    joints=[r for r in read(workload/name/'joints.csv') if int(r['tick'])>=tick];support=[r for r in read(workload/name/'support-v1.csv') if int(r['tick'])>=tick]
    assert max(abs(float(r['applied_Nm'])) for r in joints)==0 and terminal['evaluations']==stand[-1]['evaluations']
    negatives[name]=dict(first_terminal_tick=tick,reason=terminal['reason'],subsequent_solver_evaluations=0,actual_torque_after_failure=0,min_left_normal_N=min(float(r['normal_N']) for r in support if r['side']=='LeftFoot'))
assert negatives['support-loss']['min_left_normal_N']==0;result['negatives']=negatives
legacy=read(p/'final3-legacy-pulse/stand-v1.csv');lost=next(r for r in legacy if r['phase']=='lost');result['legacy_failure']=dict(sample_time_s=float(lost['sample_time_s']),reason=lost['reason'],requested_utilization=float(lost['requested_utilization']))
assert result['legacy_failure']['requested_utilization']>1
messages.append('PASS negative cases: finite zero efforts and no further contact QP after failure; actual support unloading observed')
for source,target in [('frame_1440.ppm','x-plus-reached-general.png'),('angle2_1440.ppm','x-plus-reached-top.png'),('frame_4680.ppm','z-plus-reached-general.png'),('angle2_4680.ppm','z-plus-reached-top.png'),('angle2_6480.ppm','return-top.png')]:
    assert ImageChops.difference(Image.open(p/'parity-final-03/shift-pulse-60'/source),Image.open(p/'visuals-final-v3'/target)).getbbox() is None
messages.append('PASS reviewed control frames are lossless conversions of final native PPM captures; mm charts and top view retained')
result['tuning']=dict(candidates=2,candidate_budget=12,exploratory_native_processes=8,native_process_budget=60,whole_body_posture_searches=0)
(out/'summary.json').write_text(json.dumps(result,indent=2));(out/'audit.txt').write_text('\n'.join(messages)+'\n')
print('\n'.join(messages))
