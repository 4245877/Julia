"""Charge each exploratory native process before launch, including failures."""
from pathlib import Path
import argparse, datetime, hashlib, json, subprocess, time
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('command',nargs=argparse.REMAINDER);a=p.parse_args()
root=Path('build/diagnostics/stage7j');b=json.loads((root/'budget.json').read_text())
assert len(b['runs'])<40
d=root/a.label;d.mkdir(exist_ok=False)
c=a.command[1:] if a.command[0]=='--' else a.command
e=dict(number=len(b['runs'])+1,label=a.label,command=c,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),binary_sha256=hashlib.sha256(Path(c[0]).read_bytes()).hexdigest())
b['runs'].append(e);(root/'budget.json').write_text(json.dumps(b,indent=2))
t=time.perf_counter()
with (d/'process.log').open('w') as f:r=subprocess.run(c,stdout=f,stderr=subprocess.STDOUT)
e.update(returncode=r.returncode,wall_seconds=time.perf_counter()-t)
(d/'run.json').write_text(json.dumps(e,indent=2));(root/'budget.json').write_text(json.dumps(b,indent=2));print(e,flush=True)
raise SystemExit(r.returncode)
