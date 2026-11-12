"""Charge each stage7k exploratory native process before launch, including failures.

Writes start (before launch) and result records; the budget ledger is updated twice so that
an interrupted process remains charged. Output label must be new. The executable is resolved
to an absolute Windows path before CreateProcess (forward-slash relative paths fail to launch);
the given and resolved commands are both recorded. Launch failures stay charged.
"""
from pathlib import Path
import argparse, datetime, hashlib, json, subprocess, time
p = argparse.ArgumentParser(); p.add_argument('--label', required=True); p.add_argument('--variant', type=int, required=True)
p.add_argument('command', nargs=argparse.REMAINDER); a = p.parse_args()
root = Path('build/diagnostics/stage7k'); b = json.loads((root/'budget.json').read_text())
assert len(b['runs']) < b['exploratory_native_process_limit'], 'Exploratory native process budget exhausted'
assert any(v['number'] == a.variant for v in b['variants']), 'Register the controller variant first'
d = root/a.label; d.mkdir(exist_ok=False)
c = a.command[1:] if a.command and a.command[0] == '--' else a.command
resolved = [str(Path(c[0]).resolve()), *c[1:]]
e = dict(number=len(b['runs'])+1, label=a.label, variant=a.variant, command=c, resolved_command=resolved,
         utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
         binary_sha256=hashlib.sha256(Path(c[0]).read_bytes()).hexdigest(), status='started')
b['runs'].append(e); (root/'budget.json').write_text(json.dumps(b, indent=2))
(d/'started.json').write_text(json.dumps(e, indent=2))
t = time.perf_counter()
try:
    with (d/'process.log').open('w') as f:
        r = subprocess.run(resolved, stdout=f, stderr=subprocess.STDOUT)
    e.update(returncode=r.returncode, status='completed')
except OSError as error:
    e.update(returncode=None, status='launch_failed', error=repr(error))
e.update(wall_seconds=time.perf_counter()-t, completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
(d/'run.json').write_text(json.dumps(e, indent=2)); (root/'budget.json').write_text(json.dumps(b, indent=2)); print(e, flush=True)
raise SystemExit(e['returncode'] if e['returncode'] is not None else 2)
