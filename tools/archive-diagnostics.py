"""Lossless storage of historical telemetry; never archives source/runtime assets.

plan lists bulky historical files. pack verifies every ZIP member by SHA-256
before --prune removes its unchanged original. restore recreates missing files
without overwriting anything. Metadata, scripts, certificates and PNGs stay loose.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DIAGNOSTICS = ROOT / 'build/diagnostics'
ARCHIVES = ROOT / 'build/research-archive'
SUFFIXES = {'.csv', '.ppm', '.pam', '.bin'}
CHUNK = 1024 * 1024
# Stage 7K's accepted evidence manifest contains these archive hashes.
CERTIFIED_STAGES = {'stage7h', 'stage7i', 'stage7j', 'stage7k'}


def checked(path, parent):
    path = Path(os.path.abspath(path))
    if not path.is_relative_to(parent) or path == parent:
        raise ValueError(f'Outside allowed directory: {path}')
    # Win32 trims trailing spaces/dots and accepts alternate data streams;
    # reject aliases before lexical confinement can differ from actual I/O.
    if any(part.rstrip(' .') != part or ':' in part for part in path.relative_to(parent).parts):
        raise ValueError(f'Ambiguous Windows path: {path}')
    current = path
    while current != ROOT:
        try:
            attributes = current.lstat().st_file_attributes
        except FileNotFoundError:
            attributes = 0
        if attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT:
            raise ValueError(f'Reparse point: {current}')
        current = current.parent
    return path


def candidates(stage):
    folder = checked(DIAGNOSTICS / stage, DIAGNOSTICS)
    result = []
    if not folder.exists():
        return result
    for directory, dirs, files in os.walk(folder, followlinks=False):
        for name in dirs:
            checked(Path(directory) / name, DIAGNOSTICS)
        for name in files:
            path = checked(Path(directory) / name, DIAGNOSTICS)
            if path.suffix.lower() in SUFFIXES:
                info = path.stat()
                result.append(dict(path=path.relative_to(DIAGNOSTICS).as_posix(),
                                   size=info.st_size, mtime_ns=info.st_mtime_ns))
    return sorted(result, key=lambda item: item['path'])


def digest(stream):
    sha = hashlib.sha256()
    while block := stream.read(CHUNK):
        sha.update(block)
    return sha.hexdigest()


def archive_path(stage, part=None):
    name = f'{stage}-{part}-raw.zip' if part else f'{stage}-raw.zip'
    return checked(ARCHIVES / name, ARCHIVES)


def archive_member_path(name):
    path = PurePosixPath(name)
    if not path.parts or path.is_absolute() or path.as_posix() != name or any(
            part == '..' or part.rstrip(' .') != part or ':' in part or '\\' in part
            for part in path.parts):
        raise ValueError(f'Invalid archive path: {name}')


def verify(archive, paths=None):
    with zipfile.ZipFile(archive) as source:
        manifest = json.loads(source.read('_manifest.json'))
        originals, members = set(), {'_manifest.json'}
        for item in manifest:
            name = item['path']
            stored = item.get('stored_path', name)
            archive_member_path(name)
            archive_member_path(stored)
            if name in originals or '_manifest.json' in (name, stored):
                raise ValueError(f'Duplicate or reserved archive path: {name}')
            originals.add(name)
            members.add(stored)
        names = source.namelist()
        if len(names) != len(members) or set(names) != members:
            raise ValueError(f'Archive members do not match the index: {archive}')
        if paths is not None:
            missing = set(paths) - originals
            if missing:
                raise ValueError(f'Paths not in archive: {sorted(missing)}')
            manifest = [item for item in manifest if item['path'] in paths]
        verified = {}
        for item in manifest:
            checked(DIAGNOSTICS / item['path'], DIAGNOSTICS)
            stored = item.get('stored_path', item['path'])
            checked(DIAGNOSTICS / stored, DIAGNOSTICS)
            if source.getinfo(stored).file_size != item['size']:
                raise ValueError(f'Archive size mismatch: {item["path"]}')
            if stored not in verified:
                with source.open(stored) as stream:
                    verified[stored] = digest(stream)
            if verified[stored] != item['sha256']:
                raise ValueError(f'Archive checksum mismatch: {item["path"]}')
    return manifest


def pack(stage, prune, part=None):
    items = candidates(stage)
    if not items:
        return dict(stage=stage, files=0, original_bytes=0)
    ARCHIVES.mkdir(parents=True, exist_ok=True)
    archive = archive_path(stage, part)
    if archive.exists():
        raise FileExistsError(archive)
    partial = checked(archive.with_suffix('.zip.partial'), ARCHIVES)
    processed = 0
    with zipfile.ZipFile(partial, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=1) as target:
        for item in items:
            path = DIAGNOSTICS / item['path']
            sha = hashlib.sha256()
            with path.open('rb') as src, target.open(item['path'], 'w', force_zip64=True) as dst:
                while block := src.read(CHUNK):
                    dst.write(block)
                    sha.update(block)
            item['sha256'] = sha.hexdigest()
            info = path.stat()
            if info.st_size != item['size'] or info.st_mtime_ns != item['mtime_ns']:
                raise ValueError(f'File changed during archiving: {path}')
            processed += item['size']
        target.writestr('_manifest.json', json.dumps(items, indent=2))
    # Verification alone can read OS cache. Persist the archive before pruning
    # originals so a power outage cannot leave a zero-filled cached ZIP.
    with partial.open('r+b') as stream:
        stream.flush()
        os.fsync(stream.fileno())
    verify(partial)
    partial.rename(archive)
    with archive.open('r+b') as stream:
        os.fsync(stream.fileno())
    if prune:
        # Check the whole plan again before removing any original in this stage.
        for item in items:
            path = checked(DIAGNOSTICS / item['path'], DIAGNOSTICS)
            with path.open('rb') as stream:
                unchanged = digest(stream) == item['sha256']
            info = path.stat()
            if not unchanged or info.st_size != item['size'] or info.st_mtime_ns != item['mtime_ns']:
                raise ValueError(f'Original changed; retained: {path}')
        for item in items:
            checked(DIAGNOSTICS / item['path'], DIAGNOSTICS).unlink()
    with archive.open('rb') as stream:
        archive_hash = hashlib.file_digest(stream, 'sha256').hexdigest()
    result = dict(stage=stage, files=len(items), original_bytes=processed,
                  archive_bytes=archive.stat().st_size, pruned=prune, sha256=archive_hash)
    with archive.with_suffix('.json').open('w', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    print(json.dumps(result), flush=True)
    return result


def restore(stage, part=None, destination=None, paths=None):
    archive = archive_path(stage, part)
    manifest = verify(archive, paths)
    target = checked(destination, ROOT) if destination is not None else DIAGNOSTICS
    # Refuse partial restoration over a changed dataset.
    for item in manifest:
        path = checked(target / item['path'], target)
        if path.exists():
            with path.open('rb') as stream:
                if digest(stream) != item['sha256']:
                    raise FileExistsError(f'Existing file differs: {path}')
    with zipfile.ZipFile(archive) as source:
        for item in manifest:
            path = checked(target / item['path'], target)
            if path.exists():
                continue
            path.parent.mkdir(parents=True, exist_ok=True)
            partial = checked(path.with_name(path.name + '.restore-partial'), target)
            with source.open(item.get('stored_path', item['path'])) as src, partial.open('xb') as dst:
                while block := src.read(CHUNK):
                    dst.write(block)
                dst.flush()
                os.fsync(dst.fileno())
            with partial.open('rb') as stream:
                if digest(stream) != item['sha256']:
                    raise ValueError(f'Restored checksum mismatch: {path}')
            partial.rename(path)
            os.utime(path, ns=(item['mtime_ns'], item['mtime_ns']))
    print(f'Restored {stage}: {len(manifest)} files')


def compact(stage, part=None):
    """Store exact duplicates once within a self-contained stage archive."""
    if part is None and stage.lower() in CERTIFIED_STAGES:
        raise ValueError(f'The accepted evidence requires this original ZIP: {stage}-raw.zip')
    archive = archive_path(stage, part)
    metadata = json.loads(archive.with_suffix('.json').read_text(encoding='utf-8'))
    with archive.open('rb') as stream:
        if hashlib.file_digest(stream, 'sha256').hexdigest() != metadata['sha256']:
            raise ValueError(f'Original archive checksum mismatch: {archive}')
    partial = checked(archive.with_suffix('.compact.zip'), ARCHIVES)
    seen = {}
    with zipfile.ZipFile(archive) as source, zipfile.ZipFile(
            partial, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as target:
        manifest = json.loads(source.read('_manifest.json'))
        for item in manifest:
            checked(DIAGNOSTICS / item['path'], DIAGNOSTICS)
            key = (item['sha256'], item['size'])
            if key in seen:
                item['stored_path'] = seen[key]
                continue
            seen[key] = item['path']
            sha = hashlib.sha256()
            with source.open(item.get('stored_path', item['path'])) as src, target.open(
                    item['path'], 'w', force_zip64=True) as dst:
                while block := src.read(CHUNK):
                    sha.update(block)
                    dst.write(block)
            if sha.hexdigest() != item['sha256']:
                raise ValueError(f'Source member checksum mismatch: {item["path"]}')
            item.pop('stored_path', None)
        target.writestr('_manifest.json', json.dumps(manifest, indent=2))
    with partial.open('r+b') as stream:
        os.fsync(stream.fileno())
    verify(partial)
    before = archive.stat().st_size
    os.replace(partial, archive)
    with archive.open('r+b') as stream:
        os.fsync(stream.fileno())
    with archive.open('rb') as stream:
        metadata['sha256'] = hashlib.file_digest(stream, 'sha256').hexdigest()
    metadata.update(archive_bytes=archive.stat().st_size, unique_members=len(seen),
                    duplicate_members=len(manifest)-len(seen), format='manifest-aliases-v1')
    with archive.with_suffix('.json').open('w', encoding='utf-8') as stream:
        json.dump(metadata, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    print(json.dumps(dict(before=before, **metadata)), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['plan', 'pack', 'verify', 'restore', 'compact'])
    parser.add_argument('--stage', action='append', help='Single directory name, e.g. stage7k; repeatable')
    parser.add_argument('--part', help='Additional archive name, e.g. validation for stage7k-validation-raw.zip')
    parser.add_argument('--destination', type=Path, help='Restore to a separate workspace directory')
    parser.add_argument('--file', action='append', help='Restore only this indexed path relative to build/diagnostics')
    parser.add_argument('--prune', action='store_true', help='Remove originals only after full verification')
    parser.add_argument('--jobs', type=int, choices=range(1, 5), default=2)
    args = parser.parse_args()
    stages = args.stage or sorted(path.name for path in DIAGNOSTICS.iterdir() if path.is_dir())
    if any(not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', stage) for stage in stages):
        parser.error('stage must be a simple directory name (letters, digits, underscore, hyphen)')
    if args.part and (not args.stage or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.part)):
        parser.error('--part requires --stage and a simple name')
    if (args.destination is not None or args.file) and args.action != 'restore':
        parser.error('--destination and --file apply to restore')
    if args.action == 'plan':
        plan = [dict(stage=stage, files=len(items), bytes=sum(x['size'] for x in items))
                for stage in stages if (items := candidates(stage))]
        print(json.dumps(plan, indent=2))
    elif args.action == 'pack':
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
            list(pool.map(lambda stage: pack(stage, args.prune, args.part), stages))
    else:
        for stage in stages:
            if args.action == 'compact':
                compact(stage, args.part)
            elif args.action == 'restore':
                restore(stage, args.part, args.destination, set(args.file) if args.file else None)
            else:
                print(f'{stage}: {len(verify(archive_path(stage, args.part)))} verified')


if __name__ == '__main__':
    main()
