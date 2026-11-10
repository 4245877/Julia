"""Compare real headless and GLFW whole-body runs, including contact telemetry.

The runtime is C++; Python only orchestrates verification and reads CSV.
Run from C:/Julia: python tools/verify_stage7c.py
"""
import argparse
import csv
from pathlib import Path
import subprocess


def run(exe, arguments, directory):
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'process.log').open('w', encoding='utf-8') as log:
        result = subprocess.run([str(exe), *arguments], stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'{exe.name} failed ({result.returncode}): {directory}/process.log')


def compare(reference, actual, expected_rows=None):
    with reference.open(newline='') as left, actual.open(newline='') as right:
        a, b = csv.DictReader(left), csv.DictReader(right)
        if a.fieldnames != b.fieldnames:
            raise RuntimeError(f'CSV schema mismatch: {actual}')
        fields = [f for f in a.fieldnames if not f.endswith('_us')]
        lrows, rrows = list(a), list(b)
    if len(lrows) != len(rrows) or (expected_rows is not None and len(lrows) != expected_rows):
        raise RuntimeError(f'CSV row mismatch: {actual}: {len(lrows)} / {len(rrows)}')
    for row, (lrow, rrow) in enumerate(zip(lrows, rrows)):
        if None in lrow or None in rrow:
            raise RuntimeError(f'Extra CSV values: {actual}')
        for field in fields:
            if lrow[field] != rrow[field]:
                raise RuntimeError(f'{actual} row {row} {field}: {lrow[field]} != {rrow[field]}')
    return len(lrows), len(fields)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('build/diagnostics/stage7c/parity'))
    parser.add_argument('--compare-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cpu = Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe').resolve()
    desktop = Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe').resolve()
    with (args.output / 'parity.txt').open('w', encoding='utf-8') as report:
        for case in ('drop', 'tilt', 'pulse', 'internal', 'passive'):
            reference = args.output / f'{case}-headless'
            base = ['--case', case, '--ticks', '720']
            if not args.compare_only:
                run(cpu, base + ['--output', str(reference)], reference)
            for fps in (30, 60, 144):
                actual = args.output / f'{case}-{fps}'
                if not args.compare_only:
                    run(desktop, base + ['--fps', str(fps), '--verify', str(actual)], actual)
                counts = {}
                for filename, expected in [('state.csv', 721), ('joints.csv', 721*41), ('contacts.csv', None)]:
                    counts[filename] = compare(reference / filename, actual / filename, expected)
                gpu = (actual / 'gpu.txt').read_text()
                if ('GL_errors=0' not in gpu or 'callback_checks=PASS' not in gpu or f'renders={fps*6} ' not in gpu):
                    raise RuntimeError(f'GPU/callback acceptance failed: {actual}')
                message = f'PASS case={case} scheduler_hz={fps} rows/fields={counts}'
                print(message, flush=True)
                report.write(message + '\n')
                report.flush()


if __name__ == '__main__':
    main()
