"""Fixed stage7k CLI/GLFW matrix; compare every deterministic CSV field exactly.

Native runs are final validation, not exploratory tuning. Invoke only after freezing
source and binaries. Timings are excluded only by the inherited *_us rule. Scheduler
rates 30/60/144 Hz are verification rates, not measured wall-clock FPS.
"""
from pathlib import Path
import argparse
import hashlib
import json

from verify_stage7h import run
from verify_stage7c import compare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--compare-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=args.compare_only)
    cpu = Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe')
    desktop = Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe')
    binaries = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (cpu, desktop)}
    records = []
    with (args.output/'parity.txt').open('w', encoding='utf-8') as log:
        for case, extra, label, ticks in [
            ('coord-shift', [], 'quiet', 8400),
            ('coord-shift', ['--coord-pulses', 'on'], 'pulse', 8400),
            ('coord-retarget', [], 'retarget', 7440),
        ]:
            command = ['--case', case, '--ticks', str(ticks), '--bounded-solver', 'optimized', *extra]
            headless = args.output/f'{label}-headless'
            if not args.compare_only:
                run(cpu, [*command, '--output'], headless)
            for fps in (30, 60, 144):
                actual = args.output/f'{label}-{fps}'
                if not args.compare_only:
                    run(desktop, [*command, '--fps', str(fps), '--verify'], actual)
                counts = {p.name: compare(p, actual/p.name) for p in headless.glob('*.csv')}
                assert 'coordinated-v1.csv' in counts and counts['coordinated-v1.csv'][0] == ticks+1
                gpu = (actual/'gpu.txt').read_text()
                assert 'GL_errors=0' in gpu and 'callback_checks=PASS' in gpu
                assert f'renders={(ticks*fps+119)//120} ' in gpu and f'steps={ticks*16} ' in gpu
                record = dict(scenario=label, fps=fps, counts=counts)
                records.append(record)
                message = f'PASS {label} {fps}Hz {counts}'
                print(message, flush=True)
                log.write(message+'\n')
                log.flush()
        log.write('PASS 9/9 real CLI/GLFW coordinated comparisons; timings separate; OS input not tested\n')
    assert binaries == {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (cpu, desktop)}
    (args.output/'binary-hashes.json').write_text(json.dumps(binaries, indent=2))
    (args.output/'comparisons.json').write_text(json.dumps(records, indent=2))


if __name__ == '__main__':
    main()
