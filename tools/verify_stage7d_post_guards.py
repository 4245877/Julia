"""Final repeat after numeric input guards: current CLI, current GLFW, prior matrix."""
import argparse
from pathlib import Path
from verify_stage7c import compare, run

p = argparse.ArgumentParser()
p.add_argument('--root', type=Path, default=Path('build/diagnostics/stage7d'))
a = p.parse_args()
out = a.root / 'post-guards-parity'
out.mkdir(parents=True, exist_ok=True)
with (out / 'parity.txt').open('w', encoding='utf-8') as report:
    for name, prior, body in [('left', 'left-retarget-servo-gravity', False),
                               ('right', 'right-retarget-servo-gravity', False),
                               ('body-internal', 'body-internal-servo', True),
                               ('body-drop', 'body-drop-servo', True)]:
        directory = out / (name+'-headless')
        if body:
            exe = Path('build/cpu-release/apps/julia_body_physics/julia_body_physics.exe').resolve()
            flags = ['--case', name.removeprefix('body-'), '--drive', 'servo']
            files = [('state.csv', 721), ('joints.csv', 721*41), ('drive-v1.csv', 721*41), ('contacts.csv', None)]
        else:
            exe = Path('build/cpu-release/apps/julia_arm_physics/julia_arm_physics.exe').resolve()
            flags = ['--chain', 'full', '--side', name, '--case', 'retarget', '--drive', 'servo-gravity', '--payload', '0.5']
            files = [('state.csv', 720), ('drive-v1.csv', 720*6), ('contacts.csv', None)]
        run(exe, flags+['--ticks', '720', '--output', str(directory)], directory)
        desktop = a.root / 'post-guards/artifacts/stage7d/gpu' / name
        old = a.root / 'parity-final' / (prior+'-headless')
        counts = {file: compare(directory/file, desktop/file, rows) for file, rows in files}
        for file, rows in files:
            compare(old/file, directory/file, rows)
        message = f'PASS {name}: current CLI/current GLFW and prior 30/60/144 matrix state unchanged; rows/fields={counts}'
        print(message, flush=True)
        report.write(message+'\n')
