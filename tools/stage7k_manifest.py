"""Freeze/check the stage7k runtime, then the final delivery evidence without self-hashing.

Use --exclude-docs only for the prevalidation controller freeze. The final runtime
manifest also includes the completed stage7k report and README. Generate evidence
last, without redirecting this command's output into the directory being hashed.
"""
from pathlib import Path
import argparse
from collections import Counter, defaultdict
from contextlib import ExitStack
import hashlib
import json
import re
import zipfile


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def check_manifest(data, diagnostic_archives, runtime_snapshots):
    """Resolve original paths, checking bytes against the acceptance manifest."""
    bad, storage, member_hashes = [], Counter(), {}
    with ExitStack() as stack:
        archived = defaultdict(list)
        for archives, prefix in ((diagnostic_archives, 'build/diagnostics/'),
                                  (runtime_snapshots, '')):
            for archive_path in archives:
                archive = stack.enter_context(zipfile.ZipFile(archive_path))
                for item in json.loads(archive.read('_manifest.json')):
                    archived[prefix + item['path']].append((archive, item))

        for item in data:
            path = Path(item['path'])
            expected = (item['bytes'], item['sha256'])
            if path.is_file() and path.stat().st_size == expected[0] and digest(path) == expected[1]:
                storage['loose'] += 1
                continue
            for archive, record in archived[path.as_posix()]:
                if (record['size'], record['sha256']) != expected:
                    continue
                member = record.get('stored_path', record['path'])
                key = archive, member
                if key not in member_hashes:
                    with archive.open(member) as stream:
                        member_hashes[key] = (archive.getinfo(member).file_size,
                                              hashlib.file_digest(stream, 'sha256').hexdigest())
                if member_hashes[key] == expected:
                    storage['archive'] += 1
                    break
            else:
                bad.append(item['path'])
    return bad, dict(storage)


def runtime_paths(include_docs):
    old = Path('build/diagnostics/stage7j/runtime-manifest.json')
    paths = {Path(item['path']) for item in json.loads(old.read_text())}
    missing = [str(path) for path in paths if not path.is_file()]
    assert not missing, ('Accepted runtime files missing', missing)
    for folder in ('libs/physics', 'apps/julia_body_physics', 'apps/julia_desktop', 'julia_desktop/assets',
                   'julia_desktop/shaders', 'tests', 'tools'):
        paths.update(path for path in Path(folder).rglob('*')
                     if path.is_file() and '__pycache__' not in path.parts)
    paths.add(Path('apps/julia_arm_physics/BenchHud.hpp'))
    paths.update(Path('build/diagnostics/stage7k')/name for name in ('budget.json', 'criteria.txt', 'parameters.json'))
    # Include every executable that CTest will actually launch, especially the general
    # desktop runtime writer, which was absent from the original physical-only freeze.
    for variant in ('cpu-no-physics', 'cpu-release', 'vs-release'):
        text = (Path('build')/variant/'tests/CTestTestfile.cmake').read_text(encoding='utf-8')
        for name in re.findall(r'"([A-Za-z]:/[^"\n]+\.exe)"', text):
            executable = Path(name)
            paths.add(executable.relative_to(Path.cwd()))
            paths.update(p.relative_to(Path.cwd()) for p in executable.parent.glob('*.dll'))
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
    parser.add_argument('--diagnostic-archive', action='append', type=Path, default=[],
                        help='ZIP indexed relative to build/diagnostics (check only)')
    parser.add_argument('--runtime-snapshot', action='append', type=Path, default=[],
                        help='ZIP indexed relative to the project root (check only)')
    args = parser.parse_args()
    if args.action != 'check' and (args.diagnostic_archive or args.runtime_snapshot):
        parser.error('Archives apply to check')
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
        bad, storage = check_manifest(data, args.diagnostic_archive, args.runtime_snapshot)
        if bad:
            print(json.dumps(dict(mismatches=bad, storage=storage), indent=2))
            raise SystemExit(1)
        print('check', len(data), 'files PASS', json.dumps(storage))


if __name__ == '__main__':
    main()
