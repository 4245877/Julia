"""Compare preserved pre-change physical CSV with the new legacy-mode suites."""
import argparse
from pathlib import Path
from verify_stage7c import compare

p = argparse.ArgumentParser()
p.add_argument('--root', type=Path, default=Path('build/diagnostics/stage7d'))
a = p.parse_args()
baseline = a.root / 'baseline-desktop/artifacts'
current = a.root / 'final-cpu/artifacts'
with (a.root / 'legacy-baseline-comparison.txt').open('w', encoding='utf-8') as report:
    for stage in ('stage7a', 'stage7b', 'stage7c'):
        for reference in sorted((baseline / stage / 'cpu').rglob('*.csv')):
            if reference.name == 'performance.csv':
                continue
            actual = current / reference.relative_to(baseline)
            counts = compare(reference, actual)
            message = f'PASS {reference.relative_to(baseline)} rows/fields={counts}'
            print(message, flush=True)
            report.write(message+'\n')
