"""Archive-aware exact CSV comparison for stage7k regressions (read-only).

Reference files may be loose under build/diagnostics or packed by tools/archive-diagnostics.py
into build/research-archive/<stage>-raw.zip (members named by their build/diagnostics-relative
path; `_manifest.json` gives size, SHA-256 and optional `stored_path` for deduplicated copies).
Archived members are streamed, their SHA-256 and size are verified against the manifest, and
nothing is extracted to disk. Comparison semantics equal tools/verify_stage7c.compare: identical
header, identical row count, no extra values, and every field not ending in `_us` equal as text.
"""
from pathlib import Path
import csv
import hashlib
import io
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DIAGNOSTICS = ROOT/'build/diagnostics'
ARCHIVES = ROOT/'build/research-archive'


class _HashingReader(io.RawIOBase):
    def __init__(self, stream):
        self.stream, self.sha, self.size = stream, hashlib.sha256(), 0
    def readable(self):
        return True
    def readinto(self, buffer):
        data = self.stream.read(len(buffer))
        self.sha.update(data); self.size += len(data)
        buffer[:len(data)] = data
        return len(data)


class References:
    """Resolve build/diagnostics-relative paths to loose files or verified archive members."""
    def __init__(self):
        self._open = {}

    def _archive(self, stage):
        if stage not in self._open:
            path = ARCHIVES/f'{stage}-raw.zip'
            if not path.is_file():
                self._open[stage] = None
            else:
                archive = zipfile.ZipFile(path)
                manifest = json.loads(archive.read('_manifest.json'))
                self._open[stage] = (archive, {item['path']: item for item in manifest}, path)
        return self._open[stage]

    def entry(self, rel):
        rel = Path(rel).as_posix()
        loose = DIAGNOSTICS/rel
        if loose.is_file():
            return dict(kind='loose', path=loose)
        found = self._archive(rel.split('/')[0])
        if found and rel in found[1]:
            archive, index, path = found
            return dict(kind='archive', archive=archive, item=index[rel], archive_path=path)
        raise FileNotFoundError(f'Reference neither loose nor archived: {rel}')

    def listing(self, prefix):
        """All reference CSV paths under a build/diagnostics-relative prefix (loose or archived)."""
        prefix = prefix.rstrip('/')+'/'
        names = {p.relative_to(DIAGNOSTICS).as_posix() for p in (DIAGNOSTICS/prefix).rglob('*.csv')} if (DIAGNOSTICS/prefix).is_dir() else set()
        found = self._archive(prefix.split('/')[0])
        if found:
            names |= {p for p in found[1] if p.startswith(prefix) and p.endswith('.csv')}
        return sorted(names)

    def open_text(self, rel):
        e = self.entry(rel)
        if e['kind'] == 'loose':
            return e['path'].open(newline='', encoding='utf-8'), None
        item = e['item']
        raw = _HashingReader(e['archive'].open(item.get('stored_path', item['path'])))
        text = io.TextIOWrapper(io.BufferedReader(raw, 1024*1024), newline='', encoding='utf-8')
        return text, (raw, item)


def compare(references, rel, actual):
    """Exact deterministic comparison of reference `rel` with the new file `actual`."""
    stream, check = references.open_text(rel)
    with stream as left, Path(actual).open(newline='', encoding='utf-8') as right:
        a, b = csv.DictReader(left), csv.DictReader(right)
        if a.fieldnames != b.fieldnames:
            raise RuntimeError(f'CSV schema mismatch: {actual}')
        fields = [f for f in a.fieldnames if not f.endswith('_us')]
        rows = 0
        sentinel = object()
        while True:
            l, r = next(a, sentinel), next(b, sentinel)
            if l is sentinel or r is sentinel:
                if l is not r:
                    raise RuntimeError(f'CSV row mismatch: {actual}: row {rows}')
                break
            if None in l or None in r:
                raise RuntimeError(f'Extra CSV values: {actual}')
            for field in fields:
                if l[field] != r[field]:
                    raise RuntimeError(f'{actual} row {rows} {field}: {l[field]} != {r[field]}')
            rows += 1
        if check:
            raw, item = check
            left.read()  # drain for a complete digest
            if raw.size != item['size'] or raw.sha.hexdigest() != item['sha256']:
                raise RuntimeError(f'Archived reference checksum mismatch: {rel}')
    return rows, len(fields)
