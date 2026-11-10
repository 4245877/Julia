"""Account every native exploratory process before starting it. Never overwrite output."""
from pathlib import Path
import argparse,subprocess,hashlib,json,time
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--phase',choices=['exploratory','validation'],default='exploratory');p.add_argument('--candidate',type=int,default=1);p.add_argument('--exe',type=Path,default=Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe'));p.add_argument('args',nargs=argparse.REMAINDER);a=p.parse_args()
root=Path('build/diagnostics/stage7h');out=root/a.label
assert out.resolve().is_relative_to(root.resolve()) and not out.exists()
ledger=root/'runs.jsonl';previous=[json.loads(x) for x in ledger.read_text().splitlines()] if ledger.exists() else []
if a.phase=='exploratory':
    assert sum(x['phase']=='exploratory' for x in previous)<60 and 1<=a.candidate<=12
out.mkdir(parents=True)
args=a.args[1:] if a.args[:1]==['--'] else a.args
command=[str(a.exe.resolve()),*args,'--output',str(out)]
record=dict(label=a.label,phase=a.phase,candidate=a.candidate,command=command,exe_sha256=hashlib.sha256(a.exe.read_bytes()).hexdigest())
with ledger.open('a') as f:f.write(json.dumps(record)+'\n')
(out/'command.json').write_text(json.dumps(record,indent=2))
start=time.perf_counter()
with (out/'process.log').open('w') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
(out/'result.json').write_text(json.dumps(dict(returncode=result.returncode,wall_seconds=time.perf_counter()-start),indent=2))
print((out/'process.log').read_text());raise SystemExit(result.returncode)
