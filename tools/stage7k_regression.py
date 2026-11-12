"""Exact cross-version regressions of accepted physical contracts for stage7k.

Compares every deterministic CSV field (only *_us excluded) of new outputs produced by
the frozen stage7k binaries against accepted stage7i/stage7j evidence:
  --tests: accepted suite artifacts (stage7i optimized-suite, stage7j api/corpus suites)
  --load-parity: new CLI headless load-shift quiet/pulse/retarget vs stage7j accepted parity
  --com-run label=dir: new CLI headless optimized com-shift pulse/retarget/yaw vs accepted stage7j legacy parity
Old files define the contract; new extra files (coordinated-v1.csv header) are reported separately.
"""
from pathlib import Path
import argparse
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_stage7c import compare

J = Path('build/diagnostics/stage7j/final-validation-01')


def tree(old, new, prefix, records, skip_performance=True):
    assert old.is_dir() and new.is_dir(), (old, new)
    files = sorted(old.rglob('*.csv'))
    assert files, old
    for path in files:
        if skip_performance and 'performance' in path.name:
            continue
        rel = path.relative_to(old)
        rows, fields = compare(path, new/rel)
        records.append(dict(group=prefix, file=rel.as_posix(), rows=rows, fields=fields))
    extra = sorted(p.relative_to(new).as_posix() for p in new.rglob('*.csv') if not (old/p.relative_to(new)).exists())
    return extra


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tests', type=Path)
    parser.add_argument('--load-parity', type=Path)
    parser.add_argument('--com-run', action='append', default=[], help='label=new headless dir for shift-pulse/retarget/yaw')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    records, extras = [], {}
    if args.tests:
        for variant in ('cpu-release', 'vs-release'):
            for stage, suite in (('stage7i', 'optimized-suite'), ('stage7j', 'api-suite'), ('stage7j', 'corpus-suite')):
                old = J/'tests'/variant/'artifacts'/stage/suite
                new = args.tests/variant/'artifacts'/stage/suite
                extras[f'{variant}/{stage}/{suite}'] = tree(old, new, f'suite:{variant}:{stage}/{suite}', records)
    if args.load_parity:
        for label in ('quiet', 'pulse', 'retarget'):
            extras[f'load-{label}'] = tree(J/'parity'/f'{label}-headless', args.load_parity/f'{label}-headless', f'load-cli:{label}', records)
    for spec in args.com_run:
        label, folder = spec.split('=', 1)
        assert label in ('shift-pulse', 'retarget', 'yaw'), label
        extras[f'com-{label}'] = tree(J/'legacy-parity'/f'optimized-{label}-headless', Path(folder), f'com-cli:{label}', records)
    summary = dict(pass_all=True, comparisons=len(records), groups=sorted({r['group'] for r in records}),
                   rows=sum(r['rows'] for r in records), excluded='only *_us timing fields; performance workload CSVs excluded',
                   new_only_files=extras, records=records)
    (args.output/'comparison.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('PASS', len(records), 'exact accepted CSV histories;', {g: sum(r['group'] == g for r in records) for g in summary['groups']})


if __name__ == '__main__':
    main()
