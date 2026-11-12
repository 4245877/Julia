"""Launch the frozen visible load app for a separate targeted Windows input test.

Input is delivered only by the documented computer-use API, never by this script.
The fixed CPU measurement must finish before invoking this post-freeze check.
"""
from pathlib import Path
import argparse, datetime, hashlib, json, subprocess, time
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
exe=Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe')
command=[str(exe),'--case','load-retarget','--ticks','72000','--bounded-solver','optimized','--output',str(a.output/'raw')]
e=dict(command=command,binary_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),phase='post-freeze OS input check; separate from performance and internal callbacks')
(a.output/'run.json').write_text(json.dumps(e,indent=2));t=time.perf_counter()
with (a.output/'process.log').open('w') as f:
    child=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    e['pid']=child.pid;(a.output/'run.json').write_text(json.dumps(e,indent=2))
    e['returncode']=child.wait()
e['wall_seconds']=time.perf_counter()-t;(a.output/'run.json').write_text(json.dumps(e,indent=2));print(e,flush=True)
raise SystemExit(e['returncode'])
