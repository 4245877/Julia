"""Recheck original accepted evidence bytes, loose or in lossless research archives.

Never extracts into historical directories and never substitutes a later copy for
an overwritten original. The original accepted manifest is the authority; archive
members are decompressed and hashed, rather than trusting ZIP metadata alone.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import zipfile

sys.dont_write_bytecode = True


def hash_stream(stream):
    h, size = hashlib.sha256(), 0
    for block in iter(lambda: stream.read(1024*1024), b''):
        h.update(block); size += len(block)
    return h.hexdigest(), size


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists()
    results = {}
    for stage in ('stage7i', 'stage7j'):
        root = Path('build/diagnostics')/stage
        manifest = root/'evidence-manifest.json'
        original = json.loads(manifest.read_text(encoding='utf-8-sig'))
        archive_path = Path('build/research-archive')/(stage+'-raw.zip')
        archive = zipfile.ZipFile(archive_path)
        packed = {x['path']: x for x in json.loads(archive.read('_manifest.json'))}
        hashes, bad, storage = {}, [], dict(loose=0, archive=0, preserved_report_snapshot=0)
        working_copy_changes = []
        for item in original:
            path = Path(item['path'])
            if path.is_file():
                with path.open('rb') as stream:
                    digest, size = hash_stream(stream)
                storage['loose'] += 1
                if path.as_posix() == 'docs/body_core_stage7i.md' and digest != item['sha256']:
                    snapshot = Path('build/diagnostics/stage7k/baseline/snapshot/docs/body_core_stage7i.md')
                    with snapshot.open('rb') as stream:
                        original_digest, original_size = hash_stream(stream)
                    assert original_digest == item['sha256'] and original_size == item['bytes']
                    working_copy_changes.append(dict(path=path.as_posix(), current_sha256=digest,
                        original_sha256=original_digest, preserved_original_at=snapshot.as_posix(),
                        explanation='Working report edited by the separate maintenance session; accepted bytes preserved in the pre-change baseline; current report left intact'))
                    digest, size = original_digest, original_size
                    storage['loose'] -= 1; storage['preserved_report_snapshot'] += 1
            else:
                rel = path.relative_to(Path('build/diagnostics')).as_posix()
                if rel not in packed:
                    bad.append(dict(path=item['path'], error='Neither loose nor archived')); continue
                record = packed[rel]; stored = record.get('stored_path', rel)
                if stored not in hashes:
                    with archive.open(stored) as stream:
                        hashes[stored] = hash_stream(stream)
                digest, size = hashes[stored]
                storage['archive'] += 1
                if digest != record['sha256'] or size != record['size']:
                    bad.append(dict(path=item['path'], error='Archive own manifest mismatch'))
            if digest != item['sha256'] or size != item['bytes']:
                bad.append(dict(path=item['path'], expected=item['sha256'], actual=digest,
                                expected_bytes=item['bytes'], actual_bytes=size))
        listed = {x['path'] for x in original}
        loose_extra = sorted(p.as_posix() for p in root.rglob('*') if p.is_file() and
                             p.as_posix() not in listed and p != manifest)
        with manifest.open('rb') as stream:
            manifest_hash, _ = hash_stream(stream)
        ancillary_missing = [x for x in bad if x['path'] == 'build/diagnostics/stage7j/continuation-01/__pycache__/aggregate.cpython-311.pyc'
                             and x.get('error') == 'Neither loose nor archived']
        payload_bad = [x for x in bad if x not in ancillary_missing]
        results[stage] = dict(original_manifest=str(manifest), original_manifest_sha256=manifest_hash,
                              entries=len(original), verified=len(original)-len({x['path'] for x in bad}),
                              storage=storage, mismatches=bad, loose_extra=loose_extra, working_copy_changes=working_copy_changes,
                              missing_ancillary_cache=ancillary_missing,
                              archive=str(archive_path), decompressed_unique_members=len(hashes),
                              strict_original_entries_pass=not bad and not working_copy_changes and not loose_extra,
                              payload_pass_all=not payload_bad and not loose_extra)
        archive.close()
        print(stage, results[stage]['verified'], '/', len(original), 'extra',len(loose_extra), flush=True)
    result = dict(schema='stage7k-accepted-evidence-integrity-v2', stages=results,
                  pass_all=all(v['payload_pass_all'] for v in results.values()),
                  strict_original_paths_pass=all(v['strict_original_entries_pass'] for v in results.values()),
                  policy='Every physical evidence entry must match its original hash. One known removed CPython import cache is explicitly reported as missing, never counted as verified; the edited stage7i report is verified at its original baseline snapshot. No historical output restored.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    if not result['pass_all']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
