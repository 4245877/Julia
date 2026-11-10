"""Real C++ headless/GLFW drive parity; timings excluded, historical artifacts untouched."""
import argparse
from pathlib import Path
from verify_stage7c import compare, run


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('build/diagnostics/stage7d/parity'))
    parser.add_argument('--compare-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    arm_cpu = Path('build/cpu-release/apps/julia_arm_physics/julia_arm_physics.exe').resolve()
    arm_gpu = Path('build/vs-release/apps/julia_arm_physics/julia_physics_desktop.exe').resolve()
    body_cpu = Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe').resolve()
    body_gpu = Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe').resolve()
    scenarios = []
    for side in ('left', 'right'):
        for case, drive in [('hold', 'servo-gravity'), ('retarget', 'servo-gravity'),
                            ('retarget', 'servo'), ('weak', 'servo-gravity'), ('impulse', 'servo-gravity')]:
            name = f'{side}-{case}-{drive}'
            flags = ['--chain', 'full', '--side', side, '--case', case, '--drive', drive]
            if case != 'weak':
                flags += ['--payload', '0.5']
            scenarios.append((name, arm_cpu, arm_gpu, flags, [('state.csv', 720), ('drive-v1.csv', 720*6), ('contacts.csv', None)]))
    for case in ('internal', 'drop', 'tilt'):
        scenarios.append((f'body-{case}-servo', body_cpu, body_gpu, ['--case', case, '--drive', 'servo'],
                          [('state.csv', 721), ('joints.csv', 721*41), ('drive-v1.csv', 721*41), ('contacts.csv', None)]))
    with (args.output / 'parity.txt').open('w', encoding='utf-8') as report:
        for name, cpu, gpu, flags, files in scenarios:
            base = flags + ['--ticks', '720']
            reference = args.output / (name + '-headless')
            if not args.compare_only:
                run(cpu, base + ['--output', str(reference)], reference)
            for fps in (30, 60, 144):
                actual = args.output / f'{name}-{fps}'
                if not args.compare_only:
                    run(gpu, base + ['--fps', str(fps), '--verify', str(actual)], actual)
                counts = {f: compare(reference / f, actual / f, n) for f, n in files}
                text = (actual / 'gpu.txt').read_text()
                if 'GL_errors=0' not in text or 'callback_checks=PASS' not in text or f'renders={fps*6} ' not in text:
                    raise RuntimeError(f'GPU/callback acceptance failed: {actual}')
                message = f'PASS {name} scheduler_hz={fps} rows/fields={counts}'
                print(message, flush=True)
                report.write(message + '\n')
                report.flush()


if __name__ == '__main__':
    main()
