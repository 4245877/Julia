"""Package 60 Hz gaze evidence captured by the real julia_desktop verifier.

Raw PPM and complete CSV stay in build/. Requires Pillow, matplotlib and an
installed ffmpeg only for offline evidence; adds no JULIA runtime dependencies.
"""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='build/vs-release/verification/desktop_eyes')
    parser.add_argument('--output', default='docs/raphtalia/eyes/gaze_motion')
    args = parser.parse_args()
    source, output = ROOT / args.input, ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)
    ffmpeg = shutil.which('ffmpeg')
    if not ffmpeg:
        raise RuntimeError('ffmpeg is needed to preserve the exact 60 FPS in the review videos')

    def eyes_crop(image):
        # Same face camera for all four sequences; retain both eyes and eyelids.
        width, height = image.size
        return image.crop((int(width * .08), int(height * .46),
                           int(width * .92), int(height * .77)))

    expected = {'short': 150, 'wide': 150, 'pursuit': 240, 'automatic_blink': 720}
    for name, count in expected.items():
        directory = source / 'gaze_motion' / name
        paths = sorted(directory.glob('frame_*.ppm'), key=lambda p: int(p.stem.split('_')[-1]))
        if len(paths) != count:
            raise ValueError(f'{name}: expected {count} actual framebuffer frames, got {len(paths)}')
        subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
                        '-framerate', '60', '-start_number', '0', '-i', str(directory / 'frame_%d.ppm'),
                        '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
                        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output / f'{name}.mp4')], check=True)
        if name in ('short', 'wide'):
            frames = []
            for path in paths:
                with Image.open(path) as image:
                    frames.append(eyes_crop(image.convert('RGB')).resize((648, 240), Image.Resampling.LANCZOS))
            # One shared palette avoids false texture/colour shimmer between frames.
            palette = frames[0].quantize(colors=96)
            quantized = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
            # GIF has 10 ms granularity: 20+10+20 ms exactly represents three
            # simulated 60 Hz frames. The MP4 above retains exact 60 FPS.
            quantized[0].save(output / f'{name}.gif', save_all=True, append_images=quantized[1:],
                              duration=([20, 10, 20] * ((count + 2) // 3))[:count], loop=0, optimize=True)

    # Show every physical intermediate pose of the first wide saccade at 60 Hz.
    indices = [35, 36, 37, 38, 39, 40, 41, 42, 43, 44]
    canvas = Image.new('RGB', (648 * 2, (240 + 28) * 5), '#242932')
    draw = ImageDraw.Draw(canvas)
    for panel, frame in enumerate(indices):
        with Image.open(source / 'gaze_motion/wide' / f'frame_{frame}.ppm') as image:
            tile = eyes_crop(image.convert('RGB')).resize((648, 240), Image.Resampling.LANCZOS)
        x, y = panel % 2 * 648, panel // 2 * 268
        canvas.paste(tile, (x, y + 28))
        draw.text((x + 10, y + 8), f'Frame {frame}; command + {(frame - 36 + 1) * 1000 / 60:.1f} ms', fill='white')
    canvas.save(output / 'wide_sequence.png')

    with (source / 'gaze_motion_samples.csv').open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    selected = [row for row in rows if row['sequence'] != 'automatic_10min']
    with (output / 'samples.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(selected)
    with (output / 'all_samples.csv.gz').open('wb') as stream:
        # A reproducible gzip header keeps evidence stable across repackaging.
        with gzip.GzipFile(filename='', mode='wb', fileobj=stream, mtime=0) as compressed:
            compressed.write((source / 'gaze_motion_samples.csv').read_bytes())

    # Compact comparison of tracking, autonomous translation and a blink that
    # overlaps the second wide saccade. All panels use the same face camera.
    automatic = [row for row in rows if row['sequence'] == 'visual_automatic_blink']
    first_saccade = next(index for index, row in enumerate(automatic) if row['state'] == 'Saccading')
    last_saccade = first_saccade
    while last_saccade + 1 < len(automatic) and automatic[last_saccade + 1]['state'] == 'Saccading':
        last_saccade += 1
    panels = [('pursuit', frame) for frame in [20, 80, 140]]
    panels += [('automatic_blink', frame) for frame in
               [max(0, first_saccade - 1), (first_saccade + last_saccade) // 2, last_saccade + 1]]
    panels += [('wide', frame) for frame in [90, 94, 100]]
    combined = Image.new('RGB', (324 * 3, 152 * 3), '#242932')
    draw = ImageDraw.Draw(combined)
    for panel, (name, frame) in enumerate(panels):
        row = next(row for row in selected if row['sequence'] == f'visual_{name}' and int(row['frame']) == frame)
        with Image.open(source / 'gaze_motion' / name / f'frame_{frame}.ppm') as image:
            tile = eyes_crop(image.convert('RGB')).resize((324, 120), Image.Resampling.LANCZOS)
        x, y = panel % 3 * 324, panel // 3 * 152
        combined.paste(tile, (x, y + 32))
        label = f"{name}; {float(row['time_seconds']):.3f}s; {row['state']}"
        detail = f"yaw {float(row['left_yaw']):+.2f} deg; blink {float(row['blink_left']):.2f}"
        draw.text((x + 6, y + 3), label, fill='white')
        draw.text((x + 6, y + 17), detail, fill='white')
    combined.save(output / 'tracking_automatic_blink.png')

    fig, axes = plt.subplots(2, 3, figsize=(12, 5.7), constrained_layout=True)
    for column, sequence in enumerate(['nearby_60', 'distant_60', 'tracking_stop']):
        values = [row for row in rows if row['sequence'] == sequence]
        time = [float(row['time_seconds']) for row in values]
        axes[0, column].plot(time, [float(row['left_yaw']) for row in values], label='actual L yaw')
        axes[0, column].plot(time, [float(row['right_yaw']) for row in values], '--', label='actual R yaw')
        axes[0, column].plot(time, [float(row['target_yaw']) for row in values], ':', label='target yaw')
        axes[1, column].plot(time, [float(row['actual_speed_degrees_per_second']) for row in values])
        axes[0, column].set_title(sequence.replace('_', ' '))
        axes[0, column].set_ylabel('yaw (degrees)')
        axes[1, column].set_ylabel('angular speed (degrees/s)')
        axes[1, column].set_xlabel('time (s)')
        for axis in axes[:, column]:
            axis.grid(alpha=.25)
        axes[0, column].legend(fontsize=7)
    fig.savefig(output / 'position_speed.png', dpi=150)
    plt.close(fig)

    report = dict(line.split('=', 1) for line in (source / 'desktop_eyes.txt').read_text().splitlines() if '=' in line)
    keys = [key for key in report if key.startswith(('temporal_', 'max_frame_', 'max_actual_', 'max_binocular_',
                                                   'actual_', 'pursuit_', 'frame_gap_', 'visual_'))]
    summary = {'source': source.relative_to(ROOT).as_posix(), 'process': report['process'],
               'renderer': report['GL_RENDERER'], 'window_visible': report['window_visible'],
               'checks': {key: report[key] for key in keys},
               'video': 'Actual julia_desktop framebuffer, same face camera; exact 60 FPS MP4; normal speed',
               'gif': 'Same 60 Hz poses encoded 20/10/20 ms (GIF timing granularity)',
               'limitations': 'Imported diffuse material only; original iris/pupil textures absent'}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    manifest = [{'file': path.name, 'bytes': path.stat().st_size,
                 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                for path in sorted(output.iterdir()) if path.is_file() and path.name != 'manifest.json']
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(f'Packaged 4 exact 60 FPS videos, 2 GIFs, actual bone traces and frame sequence: {output}')


if __name__ == '__main__':
    main()
