"""Account for every exploratory native launch before starting it."""
from pathlib import Path
import argparse, datetime, hashlib, json, subprocess, time
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('command',nargs=argparse.REMAINDER);a=p.parse_args()
root=Path('build/diagnostics/stage7i');budget=json.loads((root/'budget.json').read_text())
assert len(budget['runs'])<budget['exploratory_native_process_limit']
dest=root/a.label;dest.mkdir(exist_ok=False)
command=a.command[1:] if a.command[0]=='--' else a.command
entry=dict(number=len(budget['runs'])+1,label=a.label,command=command,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),binary_sha256=hashlib.sha256(Path(command[0]).read_bytes()).hexdigest())
budget['runs'].append(entry);(root/'budget.json').write_text(json.dumps(budget,indent=2))
start=time.perf_counter()
with (dest/'process.log').open('w') as f:result=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
entry.update(returncode=result.returncode,wall_seconds=time.perf_counter()-start)
(dest/'run.json').write_text(json.dumps(entry,indent=2));(root/'budget.json').write_text(json.dumps(budget,indent=2))
print(entry,flush=True);raise SystemExit(result.returncode)
