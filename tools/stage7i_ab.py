"""Frozen sequential A/B matrix. No concurrent native workloads or rebuilds."""
from pathlib import Path
import argparse,datetime,hashlib,json,subprocess,time
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
exe=Path('build/cpu-release/tests/julia_test_stage7i.exe');digest=hashlib.sha256(exe.read_bytes()).hexdigest();runs=[]
for scenario in ('hold','motion'):
    for pair in (1,2,3):
        for solver in (('optimized','reference') if pair==2 else ('reference','optimized')):
            dest=a.output/f'{scenario}-pair{pair}-{solver}';dest.mkdir()
            command=[str(exe.resolve()),str(dest.resolve()),'perf',solver,scenario]
            entry=dict(command=command,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),binary_sha256=digest)
            (dest/'command.json').write_text(json.dumps(entry,indent=2));start=time.perf_counter()
            with (dest/'process.log').open('w') as log:r=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            entry.update(returncode=r.returncode,wall_seconds=time.perf_counter()-start);runs.append(entry)
            (a.output/'runs.json').write_text(json.dumps(runs,indent=2));print(dest,r.returncode,flush=True)
            assert r.returncode==0
assert hashlib.sha256(exe.read_bytes()).hexdigest()==digest
