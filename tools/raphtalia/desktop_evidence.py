"""Package framebuffer evidence from julia_desktop and compare the old auditor.

Only converts existing PPM frames; never changes or renders the model itself.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def read_csv(path):
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('build/vs-release/verification/desktop_animation'))
    parser.add_argument('--oracle', type=Path, default=Path('build/raphtalia_tools/desktop_oracle'))
    parser.add_argument('--output', type=Path, default=Path('docs/raphtalia/desktop'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    samples = read_csv(args.source / 'samples.csv')
    oracle = read_csv(args.oracle / 'gpu_samples.csv')
    comparisons = []
    for row in samples:
        view = {'0': 'front', '1': 'back', '2': 'eyes_front'}[row['view']]
        name = row['clip_name'].replace('|', '_')
        other = next((s for s in oracle if s['clip_name'] == name and s['view'] == view
                      and abs(float(s['time_ticks']) / 30 - float(row['time_seconds'])) < 1e-8), None)
        if other is None:
            continue
        actual = np.asarray(Image.open(args.source / row['image'])).astype(np.int16)
        expected = np.asarray(Image.open(args.oracle / other['image'])).astype(np.int16)
        if actual.shape != expected.shape:
            raise ValueError('Matching resolution and MSAA are required; rerun current desktop verification')
        difference = np.abs(actual - expected)
        comparisons.append({'clip': row['clip_name'], 'seconds': float(row['time_seconds']),
            'view': view, 'changed_rgb_bytes': int(np.count_nonzero(difference)),
            'max_rgb_delta': int(difference.max()), 'mean_absolute_rgb_delta': float(difference.mean())})
    selected = [('TEST_blink_natural', [0, 3, 5, 7, 9]),
                ('TEST_blink_independent', [0, 2, 4, 10, 13]),
                ('TEST_blink_gaze', [0, 4, 8, 12, 20]),
                ('TEST_blink_head', [0, 4, 10, 16, 19])]
    sheet = Image.new('RGB', (1500, 4 * 245), '#edf0f2')
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 16)
    for y, (suffix, indices) in enumerate(selected):
        rows = [r for r in samples if r['clip_name'].endswith(suffix) and r['view'] == '2']
        for x, i in enumerate(indices):
            row = rows[i]
            with Image.open(args.source / row['image']) as image:
                # Same fixed crop for every pose; original PPM remains in build.
                tile = image.crop((0, 140, 768, 678)).resize((300, 210), Image.Resampling.LANCZOS)
            sheet.paste(tile, (x * 300, y * 245 + 35))
            draw.text((x * 300 + 6, y * 245 + 7),
                      suffix.removeprefix('TEST_blink_') + f"  {float(row['time_seconds']):.3f} s", fill='#17212b', font=font)
    sheet.save(args.output / 'blink_desktop_contact.png')
    for view, label in [(0, 'front'), (1, 'back'), (2, 'eyes')]:
        with Image.open(args.source / f'bind_{view}.ppm') as image:
            image.save(args.output / f'bind_{label}.png')
    body = Image.new('RGB', (5 * 240, 3 * 275), '#edf0f2')
    draw = ImageDraw.Draw(body)
    for i, clip in enumerate(dict.fromkeys(r['clip_name'] for r in samples)):
        rows = [r for r in samples if r['clip_name'] == clip and r['view'] == '0']
        row = rows[len(rows) // 2]
        with Image.open(args.source / row['image']) as image:
            body.paste(image.resize((240, 240), Image.Resampling.LANCZOS), ((i % 5) * 240, (i // 5) * 275 + 35))
        draw.text(((i % 5) * 240 + 5, (i // 5) * 275 + 5), clip.split('|')[-1].removeprefix('TEST_'), fill='#17212b', font=font)
    body.save(args.output / 'all_clips_desktop.png')
    shutil.copyfile(args.source / 'desktop_animation.txt', args.output / 'desktop_animation.txt')
    shutil.copyfile(args.source / 'samples.csv', args.output / 'samples.csv')
    shutil.copyfile(args.oracle / 'gpu_audit.txt', args.output / 'oracle_audit.txt')
    (args.output / 'comparison.json').write_text(json.dumps({
        'scope': 'Same 768x768 camera, MSAA4, model fit, assets/shaders; desktop runtime vs independent Assimp sampler.',
        'desktop_report_sha256': digest(args.source / 'desktop_animation.txt'),
        'oracle_report_sha256': digest(args.oracle / 'gpu_audit.txt'),
        'compared_images': len(comparisons),
        'maximum_mean_absolute_rgb_delta': max((r['mean_absolute_rgb_delta'] for r in comparisons), default=None),
        'comparisons': comparisons,
    }, indent=2), encoding='utf-8')
    print(f'Packaged desktop evidence; {len(comparisons)} matched images')


if __name__ == '__main__':
    main()
