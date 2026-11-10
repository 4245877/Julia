"""Real C++ CLI / GLFW process parity. Preserve all runs, exclude timing only."""
import argparse
import csv
import subprocess
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
p.add_argument('--headless-build', type=Path, default=Path('build/cpu-release'))
p.add_argument('--desktop-build', type=Path, default=Path('build/vs-release'))
p.add_argument('--cases', nargs='+', default=['cycle', 'normal-disturbance', 'small-tangent', 'slip', 'lost', 'absent', 'foreign', 'weak', 'cancel', 'disable'])
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
summary = (a.output / 'comparison.txt').open('w', encoding='utf-8')
files = ['state.csv', 'links.csv', 'contacts.csv', 'drive-v1.csv', 'contact-v1.csv', 'contact-points-v1.csv']

def run(exe, args, output, desktop=False):
    output.mkdir(parents=True, exist_ok=True)
    command = [str(exe), *args, '--verify' if desktop else '--output', str(output)]
    (output / 'command.txt').write_text(subprocess.list2cmdline(command), encoding='utf-8')
    with (output / 'process.log').open('w', encoding='utf-8') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)

def compare(left, right):
    counts = []
    for name in files:
        with (left / name).open() as f, (right / name).open() as g:
            x, y = csv.DictReader(f), csv.DictReader(g)
            assert x.fieldnames == y.fieldnames, name
            columns = [c for c in x.fieldnames if not (c.endswith('_us') or c.endswith('_ms'))]
            count = 0
            for one, two in zip(x, y, strict=True):
                for col in columns:
                    assert one[col] == two[col], (name, count, col, one[col], two[col])
                count += 1
            counts.append(f'{name}:{count}x{len(columns)}')
    return ', '.join(counts)

headless = a.headless_build / 'apps/julia_arm_physics/julia_contact_physics.exe'
desktop = a.desktop_build / 'apps/julia_arm_physics/julia_contact_desktop.exe'
passed = 0
for side in ['left', 'right']:
    for case in a.cases:
        args = ['--side', side, '--case', case]
        reference = a.output / f'{side}-{case}-headless'
        run(headless, args, reference)
        for fps in [30, 60, 144]:
            target = a.output / f'{side}-{case}-{fps}'
            run(desktop, [*args, '--fps', str(fps)], target, True)
            result = compare(reference, target)
            gpu = (target / 'gpu.txt').read_text()
            assert 'GL_errors=0' in gpu and 'callback_checks=PASS' in gpu
            passed += 1
            summary.write(f'PASS {side} {case} {fps}: {result}\n')
            summary.flush()
            print(f'PASS {side} {case} {fps}', flush=True)
summary.write(f'PASS {passed}/{len(a.cases)*6}; exact text equality, timings excluded; OS input separate.\n')
