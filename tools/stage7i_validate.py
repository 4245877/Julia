"""Fixed post-freeze validation, strictly sequential, always fresh output paths."""
from pathlib import Path
import argparse,datetime,json,subprocess,sys,time
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
commands=[
    [sys.executable,'tools/stage7i_manifest.py','check','--path','build/diagnostics/stage7i/runtime-manifest.json'],
    [sys.executable,'tools/stage7i_ab.py','--output',str(a.output/'ab')],
    [sys.executable,'tools/stage7i_performance.py','--root',str(a.output/'ab'),'--output',str(a.output/'performance-audit'),'--require-target'],
    [sys.executable,'tools/stage7h_ctest.py','--output',str(a.output/'tests')],
    [sys.executable,'tools/verify_stage7i.py','--output',str(a.output/'parity')],
    [sys.executable,'tools/stage7i_regression.py','--tests',str(a.output/'tests'),'--output',str(a.output/'legacy')],
    [sys.executable,'tools/stage7i_qp_audit.py','--corpus',str(a.output/'tests/cpu-release/artifacts/stage7i/corpus-suite/corpus.bin'),'--output',str(a.output/'independent-qp')],
]
cpu=Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe')
for case in ('stand','stand-pulse'):
    commands.append([str(cpu),'--case',case,'--stand-controller','bounded','--bounded-solver','optimized','--output',str(a.output/case)])
for label,run,goals in [('sequence',a.output/'parity/optimized-shift-pulse-headless',True),('yaw',a.output/'parity/optimized-yaw-headless',True),('retarget',a.output/'parity/optimized-retarget-headless',False),('stand',a.output/'stand',False),('stand-pulse',a.output/'stand-pulse',False)]:
    commands.append([sys.executable,'tools/stage7h_audit.py','--run',str(run),'--output',str(a.output/('independent-'+label))]+(['--require-goals'] if goals else []))
commands.append([sys.executable,'tools/stage7h_contact_audit.py','--run',str(a.output/'parity/optimized-shift-pulse-headless'),'--output',str(a.output/'actual-contacts')])
commands.append([sys.executable,'tools/stage7i_manifest.py','check','--path','build/diagnostics/stage7i/runtime-manifest.json'])
(a.output/'plan.json').write_text(json.dumps(commands,indent=2))
results=[]
for i,command in enumerate(commands):
    entry=dict(index=i,command=command,utc=datetime.datetime.now(datetime.timezone.utc).isoformat());start=time.perf_counter()
    print('START',i,command,flush=True)
    with (a.output/f'process-{i:02}.log').open('w') as log:r=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    entry.update(returncode=r.returncode,wall_seconds=time.perf_counter()-start);results.append(entry)
    (a.output/'results.json').write_text(json.dumps(results,indent=2));print('END',i,r.returncode,flush=True)
    # A failed performance target is evidence to report; remaining verification still runs.
    if r.returncode and i!=2:raise SystemExit(r.returncode)
print('Fixed validation finished',flush=True)
