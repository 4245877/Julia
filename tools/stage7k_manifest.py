"""Freeze/check the stage7k runtime, then the final delivery evidence without self-hashing.

Use --exclude-docs only for the prevalidation controller freeze. The final runtime
manifest also includes the completed stage7k report and README. Generate evidence
last, without redirecting this command's output into the directory being hashed.
"""
from pathlib import Path
import argparse
import hashlib
import json


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def runtime_paths(include_docs):
    old = Path('build/diagnostics/stage7j/runtime-manifest.json')
    paths = {Path(item['path']) for item in json.loads(old.read_text())}
    missing = [str(path) for path in paths if not path.is_file()]
    assert not missing, ('Accepted runtime files missing', missing)
    for folder in ('libs/physics', 'apps/julia_body_physics', 'tests', 'tools'):
        paths.update(path for path in Path(folder).rglob('*')
                     if path.is_file() and '__pycache__' not in path.parts)
    paths.add(Path('apps/julia_arm_physics/BenchHud.hpp'))
    for variant in ('cpu-release', 'vs-release'):
        path = Path('build')/variant/'tests/julia_test_stage7k.exe'
        assert path.is_file(), path
        paths.add(path)
    if include_docs:
        for path in (Path('docs/body_core_stage7k.md'), Path('build/diagnostics/stage7k/README.md')):
            assert path.is_file(), path
            paths.add(path)
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'check'))
    parser.add_argument('--path', type=Path, required=True)
    parser.add_argument('--evidence-root', type=Path)
    parser.add_argument('--exclude-docs', action='store_true')
    args = parser.parse_args()
    if args.action == 'freeze':
        assert not args.path.exists(), args.path
        if args.evidence_root:
            assert args.evidence_root.is_dir()
            paths = {p for p in args.evidence_root.rglob('*') if p.is_file() and p.resolve() != args.path.resolve()}
        else:
            paths = runtime_paths(not args.exclude_docs)
        data = [dict(path=p.as_posix(), sha256=digest(p), bytes=p.stat().st_size) for p in sorted(paths)]
        args.path.parent.mkdir(parents=True, exist_ok=True)
        args.path.write_text(json.dumps(data, indent=2), encoding='utf-8')
        print('freeze', len(data), 'files PASS')
    else:
        data = json.loads(args.path.read_text(encoding='utf-8'))
        bad = [item['path'] for item in data if not Path(item['path']).is_file()
               or Path(item['path']).stat().st_size != item['bytes']
               or digest(Path(item['path'])) != item['sha256']]
        assert not bad, bad
        print('check', len(data), 'files PASS')


if __name__ == '__main__':
    main()
