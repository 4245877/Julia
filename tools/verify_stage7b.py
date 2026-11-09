"""Run actual CPU/GLFW benches and compare every deterministic CSV field.

From the repository root: python tools/verify_stage7b.py
Only the verification script uses Python; the bench runtime remains C++.
"""
import argparse
import csv
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('build/diagnostics/stage7b/parity'))
    parser.add_argument('--compare-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cpu = Path('build/cpu-release/apps/julia_arm_physics/julia_arm_physics.exe').resolve()
    desktop = Path('build/vs-release/apps/julia_arm_physics/julia_physics_desktop.exe').resolve()
    with (args.output / 'parity.txt').open('w', encoding='utf-8') as report:
        for side in ('left', 'right'):
            for case in ('impulse', 'contact', 'spatial'):
                reference = args.output / f'{side}-{case}-headless'
                base = ['--chain', 'full', '--side', side, '--case', case, '--ticks', '720']
                if not args.compare_only:
                    run(cpu, base + ['--output', str(reference)], reference)
                for fps in (30, 60, 144):
                    actual = args.output / f'{side}-{case}-{fps}'
                    if not args.compare_only:
                        run(desktop, base + ['--fps', str(fps), '--verify', str(actual)], actual)
                    rows, fields = compare(reference / 'state.csv', actual / 'state.csv')
                    gpu = (actual / 'gpu.txt').read_text()
                    if ('GL_errors=0' not in gpu or 'callback_checks=PASS' not in gpu
                            or f'renders={fps * 6} ' not in gpu):
                        raise RuntimeError(f'GPU/callback verification failed: {actual}')
                    message = f'PASS side={side} case={case} fps={fps} rows={rows} deterministic_fields={fields}'
                    print(message, flush=True)
                    report.write(message + '\n')
                    report.flush()


def run(exe, arguments, output):
    output.mkdir(parents=True, exist_ok=True)
    with (output / 'process.log').open('w', encoding='utf-8') as log:
        result = subprocess.run([str(exe), *arguments], stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'{exe.name} failed ({result.returncode}): {output}/process.log')


def compare(reference, actual):
    with reference.open(newline='') as a, actual.open(newline='') as b:
        left, right = csv.DictReader(a), csv.DictReader(b)
        if left.fieldnames != right.fieldnames:
            raise RuntimeError('CSV schema mismatch')
        fields = [f for f in left.fieldnames if not f.endswith('_us')]
        lrows, rrows = list(left), list(right)
        if len(lrows) != 720 or len(rrows) != 720:
            raise RuntimeError('Expected exactly 720 completed ticks')
        for tick, (lrow, rrow) in enumerate(zip(lrows, rrows), 1):
            for field in fields:
                if lrow[field] != rrow[field]:
                    raise RuntimeError(f'{actual}: tick {tick}, {field}: {lrow[field]} != {rrow[field]}')
        return len(lrows), len(fields)


if __name__ == '__main__':
    main()
