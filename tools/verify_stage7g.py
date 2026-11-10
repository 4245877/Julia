"""Real native CLI/OpenGL parity; no simulation in Python. New output required."""
from pathlib import Path
import argparse,subprocess,json,hashlib
from verify_stage7c import compare

def run(exe,args,directory):
    directory.mkdir(parents=True,exist_ok=False)
    command=[str(exe.resolve()),*args,str(directory)]
    (directory/'command.txt').write_text(subprocess.list2cmdline(command))
    with (directory/'process.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    cpu=Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe')
    gpu=Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe')
    frozen={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in (cpu,gpu)}
    with (a.output/'parity.txt').open('w') as report:
        count=0
        for case,extra,label in [('stand',[],'nominal'),('stand-pulse',[],'pulse'),('stand-pulse',['--yaw','.73'],'translated-yaw')]:
            args=['--case',case,'--ticks','1920',*extra];reference=a.output/(label+'-headless')
            run(cpu,[*args,'--output'],reference)
            for fps in (30,60,144):
                actual=a.output/f'{label}-{fps}';run(gpu,[*args,'--fps',str(fps),'--verify'],actual)
                fields={}
                for file in ('state.csv','joints.csv','contacts.csv','drive-v1.csv','support-v1.csv','dynamics-v1.csv','support-regions-v1.csv','support-points-v1.csv','dynamics-substeps-v1.csv','stand-v1.csv'):
                    fields[file]=compare(reference/file,actual/file)
                text=(actual/'gpu.txt').read_text();assert 'GL_errors=0' in text and 'callback_checks=PASS' in text
                assert f'renders={fps*16} ' in text and 'steps=30720 ' in text
                count+=1;message=f'PASS {label} {fps}Hz {fields}'
                print(message,flush=True);report.write(message+'\n');report.flush()
        report.write(f'PASS {count}/9 actual CLI/GLFW comparisons; deterministic fields exact; OS input not tested\n')
    assert frozen=={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in (cpu,gpu)}
    (a.output/'binary-hashes.json').write_text(json.dumps(frozen,indent=2))
if __name__=='__main__':main()
