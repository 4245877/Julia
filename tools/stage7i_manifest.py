"""Freeze/check exact final runtime and tools; diagnostics evidence remains immutable."""
from pathlib import Path
import argparse,hashlib,json
p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','check']);p.add_argument('--path',required=True,type=Path);a=p.parse_args()
if a.action=='freeze':
    assert not a.path.exists()
    paths={Path(x['path']) for x in json.loads(Path('build/diagnostics/stage7h/runtime-manifest-v3.json').read_text())}
    for folder in ('libs/physics','apps/julia_body_physics','tests','tools'):
        paths.update(x for x in Path(folder).rglob('*') if x.is_file() and x.suffix in ('.cpp','.hpp','.py','.ps1','.txt') and '__pycache__' not in x.parts)
    for variant in ('cpu-release','vs-release'):
        paths.add(Path('build')/variant/'tests/julia_test_stage7i.exe')
    data=[dict(path=x.as_posix(),sha256=hashlib.sha256(x.read_bytes()).hexdigest(),bytes=x.stat().st_size) for x in sorted(paths) if x.is_file()]
    a.path.write_text(json.dumps(data,indent=2))
else:
    data=json.loads(a.path.read_text());bad=[x['path'] for x in data if not Path(x['path']).is_file() or hashlib.sha256(Path(x['path']).read_bytes()).hexdigest()!=x['sha256']];assert not bad,bad
print(a.action,len(data),'files PASS')
