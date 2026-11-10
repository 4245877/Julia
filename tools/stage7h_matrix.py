"""Fixed final native validation matrix, separate from exploratory tuning budget."""
from pathlib import Path
import subprocess,json,argparse
parser=argparse.ArgumentParser();parser.add_argument('--prefix',default='final3');options=parser.parse_args()
root=Path('build/diagnostics/stage7h')
cases=[('nominal',['--case','stand','--stand-controller','bounded']),('old-pulse',['--case','stand-pulse','--stand-controller','bounded']),
       ('sequence',['--case','com-shift']),('legacy-sequence',['--case','com-shift','--stand-controller','equality']),
       ('legacy-pulse',['--case','com-shift','--stand-controller','equality','--com-pulses','on']),
       ('substeps32',['--case','com-shift','--substeps','32','--ticks','1440','--com-pulses','on']),
       ('contact008',['--case','com-shift','--contact-time','.008','--ticks','1440','--com-pulses','on']),
       ('contact012',['--case','com-shift','--contact-time','.012','--ticks','1440','--com-pulses','on'])]
result={}
for label,args in cases:
    command=['python','tools/stage7h_run.py','--phase','validation','--candidate','2','--label',options.prefix+'-'+label,'--',*args]
    r=subprocess.run(command);result[label]=r.returncode
    assert r.returncode==(1 if label=='legacy-pulse' else 0),(label,r.returncode)
destination=root/(options.prefix+'-validation-matrix.json');assert not destination.exists();destination.write_text(json.dumps(result,indent=2))
