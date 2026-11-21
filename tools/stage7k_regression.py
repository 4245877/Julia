"""Exact cross-version regressions of accepted physical contracts for stage7k.

Compares every deterministic CSV field (only *_us excluded) of new outputs produced by the
frozen stage7k binaries against accepted stage7h/stage7i/stage7j evidence. Accepted CSVs are
read loose or streamed from build/research-archive/<stage>-raw.zip with SHA-256 verification
(tools/stage7k_compare.py); nothing is restored or written outside --output.

  --stage7h-tests <tests>: same contract as tools/stage7i_regression.py — every stage7*
      CSV of stage7h/final-tests-03/<variant>/artifacts (performance workloads excluded) and
      the new reference-vs-optimized suite pairs (stage7h/cpu vs stage7i/optimized-suite)
  --tests <tests>: accepted stage7i optimized-suite and stage7j api/corpus suites
  --load-parity <dir>: new CLI headless load-shift quiet/pulse/retarget vs stage7j parity
  --com-run label=dir: new CLI headless optimized com-shift pulse/retarget/yaw vs stage7j legacy parity
New suites write under <tests>/<variant>/verification/artifacts (current CTest layout); the
pre-maintenance <tests>/<variant>/artifacts layout is accepted when present.
"""
from pathlib import Path
import argparse
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stage7k_compare import References, compare
from verify_stage7c import compare as compare_loose

J = 'stage7j/final-validation-01'


def artifacts(tests, variant):
    for candidate in (Path(tests)/variant/'verification/artifacts', Path(tests)/variant/'artifacts'):
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError(f'No suite artifacts for {variant} under {tests}')


def tree(references, old_prefix, new_root, group, records, skip_performance=True):
    files = references.listing(old_prefix)
    assert files, old_prefix
    for rel in files:
        name = rel.rsplit('/', 1)[-1]
        if skip_performance and 'performance' in name:
            continue
        sub = rel[len(old_prefix.rstrip('/'))+1:]
        rows, fields = compare(references, rel, Path(new_root)/sub)
        records.append(dict(group=group, file=sub, reference=rel, rows=rows, fields=fields))
    old = {rel[len(old_prefix.rstrip('/'))+1:] for rel in files}
    return sorted(p.relative_to(new_root).as_posix() for p in Path(new_root).rglob('*.csv') if p.relative_to(new_root).as_posix() not in old)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--stage7h-tests', type=Path)
    parser.add_argument('--tests', type=Path)
    parser.add_argument('--load-parity', type=Path)
    parser.add_argument('--com-run', action='append', default=[], help='label=new headless dir for shift-pulse/retarget/yaw')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    references = References()
    records, extras, excluded = [], {}, []
    if args.stage7h_tests:
        for variant in ('cpu-release', 'vs-release'):
            prefix = f'stage7h/final-tests-03/{variant}/artifacts'
            new = artifacts(args.stage7h_tests, variant)
            for rel in references.listing(prefix):
                sub = rel[len(prefix)+1:]
                if not sub.split('/')[0].startswith('stage7'):
                    continue
                if 'performance' in sub.rsplit('/', 1)[-1]:
                    excluded.append(rel); continue
                rows, fields = compare(references, rel, new/sub)
                records.append(dict(group=f'stage7h:{variant}', file=sub, reference=rel, rows=rows, fields=fields))
            ref, opt = new/'stage7h/cpu', new/'stage7i/optimized-suite'
            for path in sorted(ref.rglob('*.csv')):
                if 'performance' in path.name:
                    continue
                sub = path.relative_to(ref).as_posix()
                rows, fields = compare_loose(path, opt/sub)
                records.append(dict(group=f'reference-vs-optimized:{variant}', file=sub, reference=path.as_posix(), rows=rows, fields=fields))
    if args.tests:
        for variant in ('cpu-release', 'vs-release'):
            new = artifacts(args.tests, variant)
            for stage, suite in (('stage7i', 'optimized-suite'), ('stage7j', 'api-suite'), ('stage7j', 'corpus-suite')):
                extras[f'{variant}/{stage}/{suite}'] = tree(references, f'{J}/tests/{variant}/artifacts/{stage}/{suite}', new/stage/suite,
                                                            f'suite:{variant}:{stage}/{suite}', records)
    if args.load_parity:
        for label in ('quiet', 'pulse', 'retarget'):
            extras[f'load-{label}'] = tree(references, f'{J}/parity/{label}-headless', args.load_parity/f'{label}-headless', f'load-cli:{label}', records)
    for spec in args.com_run:
        label, folder = spec.split('=', 1)
        assert label in ('shift-pulse', 'retarget', 'yaw'), label
        extras[f'com-{label}'] = tree(references, f'{J}/legacy-parity/optimized-{label}-headless', Path(folder), f'com-cli:{label}', records)
    groups = sorted({r['group'] for r in records})
    summary = dict(pass_all=True, comparisons=len(records), groups=groups, rows=sum(r['rows'] for r in records),
                   by_group={g: sum(r['group'] == g for r in records) for g in groups},
                   excluded='only *_us timing fields; performance workload CSVs excluded', excluded_performance=excluded,
                   reference_storage='loose files or SHA-256-verified members of build/research-archive/<stage>-raw.zip',
                   new_only_files=extras, records=records)
    (args.output/'comparison.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('PASS', len(records), 'exact accepted CSV histories;', summary['by_group'])


if __name__ == '__main__':
    main()
