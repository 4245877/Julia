"""Real C++ CLI/GLFW parity, including 1920 Hz quadrature samples. No simulation here."""
import argparse
from pathlib import Path
from verify_stage7c import compare
import subprocess


def run(exe, args, out):
    out.mkdir(parents=True, exist_ok=False)
    command = [str(exe.resolve()), *args, str(out)]
    (out / 'command.txt').write_text(subprocess.list2cmdline(command), encoding='utf8')
    with (out / 'process.log').open('w', encoding='utf8') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    cpu = Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe')
    gpu = Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe')
    with (a.output / 'parity.txt').open('w', encoding='utf8') as log:
        count = 0
        for case in ('drop', 'tilt', 'pulse', 'passive', 'internal'):
            ref = a.output / f'{case}-headless'
            base = ['--case', case, '--ticks', '720', '--observe', 'on']
            run(cpu, base + ['--output'], ref)
            for fps in (30, 60, 144):
                actual = a.output / f'{case}-{fps}'
                run(gpu, base + ['--fps', str(fps), '--verify'], actual)
                fields = {}
                for file in ('state.csv', 'joints.csv', 'contacts.csv', 'drive-v1.csv', 'support-v1.csv',
                             'dynamics-v1.csv', 'support-regions-v1.csv', 'support-points-v1.csv', 'dynamics-substeps-v1.csv'):
                    fields[file] = compare(ref / file, actual / file)
                text = (actual / 'gpu.txt').read_text()
                assert 'GL_errors=0' in text and 'callback_checks=PASS' in text
                assert f'renders={fps*6} ' in text and 'steps=11520 ' in text
                count += 1
                message = f'PASS {case} {fps}Hz rows/fields={fields}'
                print(message, flush=True)
                log.write(message + '\n'); log.flush()
        log.write(f'PASS {count}/15; timings excluded; actual CLI + GLFW processes; OS input not tested\n')


if __name__ == '__main__':
    main()
