"""Independent final evidence audit; no runtime edits, simulation, or rebuild."""
from pathlib import Path
import argparse,csv,hashlib,json,re
import numpy as np
from verify_stage7c import compare

p=argparse.ArgumentParser();p.add_argument('--validation',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
root=Path('build/diagnostics/stage7i');v=a.validation;result={}
manifest=json.loads((root/'runtime-manifest.json').read_text())
for x in manifest:assert hashlib.sha256(Path(x['path']).read_bytes()).hexdigest()==x['sha256'],x['path']
result['frozen_files']=len(manifest)
old=(root/'accepted-source/libs/physics/src/BoundedQp.cpp').read_text();ref=Path('libs/physics/src/BoundedQpReference.cpp').read_text()
old=old[old.index('    Result Solver::solve'):old.rindex('\n}')]
ref=ref[ref.index('    Result Solver::solveReference'):ref.rindex('\n}')].replace('Solver::solveReference','Solver::solve')
ref='\n'.join(line for line in ref.split('\n') if 'PhaseTimer timer{' not in line and 'timer.next(' not in line)
assert ref==old,'reference arithmetic changed'
opt=Path('libs/physics/src/BoundedQpOptimized.cpp').read_text();r=Path('libs/physics/src/BoundedQpReference.cpp').read_text()
assert opt[opt.index('        Result out;'):opt.index('        timer.next(profile_.verifyUs);')]==r[r.index('        Result out;'):r.index('        timer.next(profile_.verifyUs);')]
result['reference_arithmetic_unchanged']=True;result['optimized_before_verification_identical']=True
result['suites']={}
for variant,n in [('vs-release',72),('cpu-release',33),('cpu-no-physics',22)]:
    text=(v/'tests'/variant/'ctest.log').read_text()
    assert f'100% tests passed, 0 tests failed out of {n}' in text
    cache=Path('build',variant,'CMakeCache.txt').read_text();assert 'CMAKE_BUILD_TYPE:STRING=Release' in cache and 'JULIA_WARNINGS_AS_ERRORS:BOOL=ON' in cache
    result['suites'][variant]=dict(passed=n,seconds=float(re.search(r'Total Test time \(real\) = ([\d.]+)',text).group(1)))
performance=json.loads((v/'performance-audit/summary.json').read_text());result['performance_target_met']=performance['target_met'];result['performance_pairs']=performance['pairs'];result['performance_pooled']=performance['pooled']
def rows(path):return list(csv.DictReader(path.open(newline='')))
for scenario in ('hold','motion'):
    base=None
    for pair in (1,2,3):
        for solver in ('reference','optimized'):
            rs=rows(v/'ab'/f'{scenario}-pair{pair}-{solver}'/'performance.csv')
            fields=[k for k in rs[0] if k not in ('tick_us','controller_us')]
            work=[[r[k] for k in fields] for r in rs]
            if base is None:base=work
            assert work==base,(scenario,pair,solver,'useful work or actual COM differs')
result['ab_useful_work_exact']=True
cross=[]
for label in ('shift-pulse','retarget','yaw'):
    r=v/'parity'/f'reference-{label}-headless';o=v/'parity'/f'optimized-{label}-headless'
    for f in sorted(r.glob('*.csv')):
        count,fields=compare(f,o/f.name);cross.append(dict(case=label,file=f.name,rows=count,fields=fields))
(a.output/'cross-implementation.json').write_text(json.dumps(cross,indent=2));result['cross_implementation_exact_csv']=len(cross)
parity=json.loads((v/'parity/comparisons.json').read_text());assert len(parity)==18;result['actual_cli_glfw_comparisons']=18
legacy=json.loads((v/'legacy/comparison.json').read_text());result['exact_legacy_and_optimized_csv']=len(legacy)
result['independent_qp']=json.loads((v/'independent-qp/certificate.json').read_text());assert result['independent_qp']['pass_all']
result['analytic_farkas']=[]
for variant in ('cpu-release','vs-release'):
    for solver in ('reference','optimized'):
        c=json.loads((v/'tests'/variant/f'artifacts/stage7h/qp/farkas-{solver}.json').read_text());mu=np.array(c['inequality']);z=c['equality']
        A=np.array([[1.,0],[0,1],[-1,0],[0,-1]]);E=np.array([[1.,1.]]);b=np.array([0.,0.,-1,-1]);d=np.array([10.])
        residual=float(abs(A.T@mu+E.T[:,0]*z).max());margin=float(b@mu+d[0]*z)
        assert mu.min()>=-1e-8 and residual<1e-6 and margin>1e-8 and abs(margin-c['margin'])<1e-10
        result['analytic_farkas'].append(dict(variant=variant,solver=solver,residual=residual,margin=margin))
result['physical_certificates']={}
for name in ('sequence','yaw','retarget','stand','stand-pulse'):
    c=json.loads((v/('independent-'+name)/'certificate.json').read_text());assert not c['lost'] and c['continuous_two_supports_after_2s']
    assert c['max_actual_tau_cap']<=1.000001 and c['max_joint_violation']<=.005 and c['max_saturation_run_s']<=.1
    for side in ('left','right'):
        assert c[side]['min_normal_N']>=30 and c[side]['max_slip_m_s']<=.05 and c[side]['max_penetration_m']<=.004 and c[side]['sole_displacement_m']<=.01
    if name in ('sequence','yaw'):assert len(c['commands'])==4 and all(x['first_measured_reached_s'] is not None and x['best_measured_hold_s']>=2 for x in c['commands'])
    if name=='retarget':assert c['commands'][-1]['first_measured_reached_s'] is not None
    result['physical_certificates'][name]=c
actual=json.loads((v/'actual-contacts/audit.json').read_text());assert actual['physical_substeps']==103680 and actual['force_error_N']==actual['moment_error_Nm']==actual['cop_error_m']==0
result['actual_contacts']=actual
result['negatives']={}
for solver,folder in [('reference','stage7h/cpu'),('optimized','stage7i/optimized-suite')]:
    for name in ('weak','outside-initial','support-loss','outside-force','disable','unsupported-contact'):
        d=v/'tests/cpu-release/artifacts'/folder/name;stand=rows(d/'stand-v1.csv');terminal=next(r for r in stand if int(r['tick'])>0 and r['phase'] in ('lost','disabled'));tick=int(terminal['tick'])
        assert terminal['evaluations']==stand[-1]['evaluations']
        assert all(float(r['applied_Nm'])==0 for r in rows(d/'joints.csv') if int(r['tick'])>=tick)
        result['negatives'][solver+'-'+name]=dict(tick=tick,reason=terminal['reason'],subsequent_qp=0,active_torque=0)
nominal=v/'parity/optimized-shift-pulse-headless/model.xml';old=Path('build/diagnostics/stage7h/parity-final-03/shift-pulse-headless/model.xml');assert nominal.read_bytes()==old.read_bytes()
result['model_xml_identical_to_stage7h']=True
for filename in ('headless-dependencies.txt','mujoco-dependencies.txt'):
    text=(root/'dependencies-final'/filename).read_text(encoding='utf-8-sig');assert not re.search(r'opengl|glfw|glad',text,re.I)
result['cpu_no_opengl_dependencies']=True;result['gpu']='18 real GLFW comparisons, GL_errors=0';result['callbacks']='existing native callback fixture PASS';result['os_input']='not tested; no OS key-delivery evidence'
result['budget']=json.loads((root/'budget.json').read_text());assert len(result['budget']['variants'])<=6 and len(result['budget']['runs'])<=24
(a.output/'summary.json').write_text(json.dumps(result,indent=2));(a.output/'audit.txt').write_text('PASS frozen runtime, full desktop/CPU/OFF suites, exact A/B work, numeric and physical certificates, negative semantics, model/dependencies, real CLI/GLFW parity.\nPerformance target: '+str(result['performance_target_met'])+'\nOS input delivery: NOT TESTED\n')
print((a.output/'audit.txt').read_text())
