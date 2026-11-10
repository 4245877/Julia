from pathlib import Path
import argparse,json,hashlib
p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','check']);p.add_argument('--path',type=Path,default=Path('build/diagnostics/stage7h/runtime-manifest.json'));a=p.parse_args()
if a.action=='freeze':
    assert not a.path.exists()
    paths={Path(x['path']) for x in json.loads(Path('build/diagnostics/stage7g/runtime-manifest.json').read_text(encoding='utf-8-sig'))}
    for folder in ('libs/physics','apps/julia_body_physics','tests','tools'):
        paths.update(x for x in Path(folder).rglob('*') if x.is_file() and x.suffix in ('.cpp','.hpp','.txt','.ps1','.py') and '__pycache__' not in x.parts)
    for folder in ('cpu-release','vs-release'):
        paths.update(x for x in (Path('build')/folder).rglob('julia_test_*qp.exe'))
        paths.update(x for x in (Path('build')/folder).rglob('julia_test_com_control.exe'))
    manifest=[dict(path=x.as_posix(),sha256=hashlib.sha256(x.read_bytes()).hexdigest(),bytes=x.stat().st_size) for x in sorted(paths) if x.exists()]
    a.path.write_text(json.dumps(manifest,indent=2))
else:
    manifest=json.loads(a.path.read_text());bad=[x['path'] for x in manifest if not Path(x['path']).exists() or hashlib.sha256(Path(x['path']).read_bytes()).hexdigest()!=x['sha256']]
    assert not bad,bad
print(a.action,len(manifest),'files PASS')
