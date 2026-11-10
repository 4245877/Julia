"""Run frozen native CLI/desktop processes and compare every deterministic CSV field."""
from pathlib import Path
import argparse,subprocess,json,hashlib
from verify_stage7c import compare

def run(exe,args,out):
    out.mkdir(parents=True,exist_ok=False);command=[str(exe.resolve()),*args,str(out)]
    (out/'command.json').write_text(json.dumps(command,indent=2))
    with (out/'process.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    cpu=Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe');gpu=Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe')
    frozen={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in (cpu,gpu)}
    with (a.output/'parity.txt').open('w') as log:
        for case,extra,label,ticks in [('com-shift',['--com-pulses','on'],'shift-pulse',6480),('com-retarget',[],'retarget',2160),('com-shift',['--yaw','.73','--com-pulses','on'],'yaw',6480)]:
            args=['--case',case,'--ticks',str(ticks),*extra];ref=a.output/(label+'-headless');run(cpu,[*args,'--output'],ref)
            for fps in (30,60,144):
                actual=a.output/f'{label}-{fps}';run(gpu,[*args,'--fps',str(fps),'--verify'],actual)
                counts={f.name:compare(f,actual/f.name) for f in ref.glob('*.csv')}
                text=(actual/'gpu.txt').read_text();assert 'GL_errors=0' in text and 'callback_checks=PASS' in text
                assert f'renders={(ticks*fps+119)//120} ' in text and f'steps={ticks*16} ' in text
                message=f'PASS {label} {fps}Hz {counts}';print(message,flush=True);log.write(message+'\n');log.flush()
        log.write('PASS 9/9 actual CLI/GLFW comparisons; timings excluded; OS input not tested\n')
    assert frozen=={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in (cpu,gpu)}
    (a.output/'binary-hashes.json').write_text(json.dumps(frozen,indent=2))
if __name__=='__main__':main()
