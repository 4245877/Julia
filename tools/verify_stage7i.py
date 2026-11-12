"""Real CLI/GLFW processes for each solver, exact fields at 30/60/144 Hz."""
from pathlib import Path
import argparse,hashlib,json
from verify_stage7h import run
from verify_stage7c import compare
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
cpu=Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe');gpu=Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe')
frozen={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in (cpu,gpu)};records=[]
with (a.output/'parity.txt').open('w') as log:
    for solver in ('reference','optimized'):
        for case,extra,label,ticks in [('com-shift',['--com-pulses','on'],'shift-pulse',6480),('com-retarget',[],'retarget',2160),('com-shift',['--yaw','.73','--com-pulses','on'],'yaw',6480)]:
            args=['--case',case,'--ticks',str(ticks),'--bounded-solver',solver,*extra];ref=a.output/f'{solver}-{label}-headless';run(cpu,[*args,'--output'],ref)
            for fps in (30,60,144):
                dest=a.output/f'{solver}-{label}-{fps}';run(gpu,[*args,'--fps',str(fps),'--verify'],dest)
                counts={f.name:compare(f,dest/f.name) for f in ref.glob('*.csv')}
                text=(dest/'gpu.txt').read_text();assert 'GL_errors=0' in text and 'callback_checks=PASS' in text
                assert f'renders={(ticks*fps+119)//120} ' in text and f'steps={ticks*16} ' in text
                records.append(dict(solver=solver,case=label,fps=fps,counts=counts));message=f'PASS {solver} {label} {fps}Hz {counts}'
                log.write(message+'\n');log.flush();print(message,flush=True)
    log.write('PASS 18/18 actual CLI/GLFW comparisons; OS delivery not tested\n')
assert frozen=={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in (cpu,gpu)}
(a.output/'binary-hashes.json').write_text(json.dumps(frozen,indent=2));(a.output/'comparisons.json').write_text(json.dumps(records,indent=2))
