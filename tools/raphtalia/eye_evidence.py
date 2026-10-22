"""Package actual julia_desktop eye framebuffer captures (Pillow, no model edits).

Run after --verify-eyes or CTest desktop_eyes. Raw PPM stays in build/.
"""
import argparse
import hashlib
import json
from pathlib import Path
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='build/vs-release/verification/desktop_eyes')
    parser.add_argument('--output', default='docs/raphtalia/eyes')
    args = parser.parse_args()
    source, output = ROOT / args.input, ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)

    def sheet(filename, samples, columns=3):
        tile, caption = 384, 28
        rows = (len(samples) + columns - 1) // columns
        canvas = Image.new('RGB', (tile * columns, (tile + caption) * rows), '#242932')
        draw = ImageDraw.Draw(canvas)
        for index, (name, label) in enumerate(samples):
            with Image.open(source / f'{name}.ppm') as frame:
                frame = frame.convert('RGB').resize((tile, tile), Image.Resampling.LANCZOS)
            x, y = index % columns * tile, index // columns * (tile + caption)
            canvas.paste(frame, (x, y + caption))
            draw.text((x + 8, y + 7), label, fill='white')
        canvas.save(output / filename)

    sheet('gaze.png', [(f'{name}_view_0', label) for name, label in [
        ('neutral', 'Forward'), ('gaze_left', 'Yaw +8 deg'), ('gaze_right', 'Yaw -8 deg'),
        ('gaze_up', 'Pitch +5 deg'), ('gaze_down', 'Pitch -5 deg'), ('gaze_diagonal', 'Yaw +8 / pitch +5'),
        ('blink_half', 'Blink L/R 0.5'), ('blink_full', 'Blink L/R 1.0'),
        ('gaze_diagonal_blink_half', 'Diagonal + half blink')]])
    sheet('blink_angles.png', [(f'{name}_view_{view}', f'{label}; view {view}')
        for name, label in [('blink_left', 'L=1 / R=0'), ('blink_right', 'L=0 / R=1'),
                            ('blink_asymmetric', 'L=0.25 / R=0.75')]
        for view in range(3)])
    sheet('body_target.png', [(f'{name}_view_{view}', f'{name}; view {view}')
        for name in ['TEST_head_torso_overlay', 'TEST_elbows_90_overlay', 'target_head_1']
        for view in [0, 3]], columns=2)
    for name in ['neutral', 'blink_full', 'gaze_diagonal_blink_half']:
        with Image.open(source / f'{name}_view_0.ppm') as frame:
            frame.save(output / f'{name}.png')
    motion_files = sorted((source / 'motion').glob('frame_*.ppm'), key=lambda p: int(p.stem.split('_')[-1]))
    if len(motion_files) != 360:
        raise ValueError(f'Expected 360 actual framebuffer motion frames, got {len(motion_files)}')
    frames = []
    for path in motion_files:
        with Image.open(path) as frame:
            frames.append(frame.convert('RGB').quantize(colors=128))
    # 30 fps encoded as 30/30/40 ms, preserving the 12-second simulated duration.
    frames[0].save(output / 'natural.gif', save_all=True, append_images=frames[1:],
                   duration=[30, 30, 40] * 120, loop=0, optimize=False)
    for name in ['desktop_eyes.txt', 'eye_samples.csv']:
        (output / name).write_bytes((source / name).read_bytes())
    files = [p for p in sorted(output.iterdir()) if p.is_file() and p.name != 'manifest.json']
    manifest = {'source': source.relative_to(ROOT).as_posix(),
                'capture': 'Actual julia_desktop GL_BACK RGB; simulated 60Hz time, 30fps GIF; imported diffuse materials',
                'files': [{'path': p.relative_to(ROOT).as_posix(), 'bytes': p.stat().st_size,
                           'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in files]}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(f'Packaged {len(files)} evidence files, 360 motion frames: {output}')


if __name__ == '__main__':
    main()
