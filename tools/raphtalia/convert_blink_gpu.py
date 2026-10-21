"""Convert selected current blink GPU frames and compact review sheets only."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def sheet(entries, output, columns, width=300, crop=(0, 320, 768, 620)):
    crop_width, crop_height = crop[2]-crop[0], crop[3]-crop[1]
    height = round(width*crop_height/crop_width)
    cell_height = height+46
    canvas = Image.new('RGB', (columns*width, ((len(entries)+columns-1)//columns)*cell_height), '#edf0f2')
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 14)
    except OSError:
        font = ImageFont.load_default()
    for index, (image, title) in enumerate(entries):
        x, y = (index % columns)*width, (index//columns)*cell_height
        with Image.open(image) as source:
            tile = source.convert('RGB').crop(crop).resize((width, height), Image.Resampling.LANCZOS)
        canvas.paste(tile, (x, y+46))
        draw.multiline_text((x+7, y+4), title, fill='#17212b', font=font, spacing=3)
    canvas.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='build/raphtalia_tools/blink/gpu')
    parser.add_argument('--before', default='build/raphtalia_tools/blink/gpu_before')
    parser.add_argument('--samples', default='build/raphtalia_tools/blink/gpu_samples.csv')
    parser.add_argument('--output', default='docs/raphtalia/blink/gpu')
    args = parser.parse_args()
    source, before, output = Path(args.source), Path(args.before), Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    all_samples = rows(source/'gpu_samples.csv')
    selected = [row.copy() for row in all_samples
                if 'TEST_blink_' in row['clip_name'] and row['view'] in ('eyes_front', 'face_front')]
    converted = []
    for row in selected:
        original = source/row['image']
        target = output/(original.stem+'.png')
        with Image.open(original) as image:
            image.save(target)
        row['image'] = target.name
        converted.append({'image': target.name, 'sha256': digest(target),
                          'source_ppm': str(original), 'source_sha256': digest(original)})
    with (output/'gpu_blink_samples.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected[0]))
        writer.writeheader(); writer.writerows(selected)
    for name in ('gpu_audit.txt', 'gpu_samples.csv', 'gpu_endpoints.csv', 'gpu_mix.csv'):
        shutil.copyfile(source/name, output/name)
    shutil.copyfile(before/'gpu_audit.txt', output/'before_gpu_audit.txt')
    shutil.copyfile(args.samples, output/'authored_samples.csv')
    comparisons = {}
    for view in ('eyes_front', 'face_front'):
        images = []
        for label, directory in (('before', before), ('after', source)):
            target = output/f'{label}_neutral_{view}.png'
            with Image.open(directory/f'gpu_neutral_{view}.ppm') as image:
                image.save(target)
                images.append(np.asarray(image.convert('RGB'), dtype=np.int16))
        delta = np.abs(images[0]-images[1])
        comparisons[view] = {'changed_rgb_bytes': int((delta != 0).sum()),
                             'max_rgb_delta': int(delta.max()),
                             'mean_absolute_rgb_delta': float(delta.mean())}
    sheet([(output/'before_neutral_eyes_front.png', 'Before blink stage: neutral'),
           (output/'after_neutral_eyes_front.png', 'After blink stage: neutral')],
          output/'neutral_before_after.png', 2, 480, (0, 0, 768, 768))
    eye_rows = [r for r in selected if r['view'] == 'eyes_front']
    natural = [r for r in eye_rows if r['clip_name'].endswith('TEST_blink_natural')]
    natural = sorted(natural, key=lambda r: float(r['authored_frame']))
    independent = [r for r in eye_rows if r['clip_name'].endswith('TEST_blink_independent')]
    left = [r for r in independent if 1 < float(r['authored_frame']) < 16]
    right = [r for r in independent if 30 < float(r['authored_frame']) < 40]
    entries = [(output/r['image'], f'Both: {r["sample_label"]}\nframe {float(r["authored_frame"]):.4g}') for r in natural[:5]]
    for title, group in (('Left', left), ('Right', right)):
        group.sort(key=lambda r: float(r['authored_frame']))
        entries.extend([(output/r['image'], f'{title}: {r["sample_label"]}\nframe {float(r["authored_frame"]):.4g}') for r in group])
        entries.append((output/'after_neutral_eyes_front.png', 'Neutral reference'))
    sheet(entries, output/'blink_levels_contact.png', 5)
    for suffix, name, columns in (('TEST_blink_gaze', 'blink_gaze_contact.png', 6),
                                  ('TEST_blink_head', 'blink_head_contact.png', 3)):
        group = [r for r in eye_rows if r['clip_name'].endswith(suffix)]
        group.sort(key=lambda r: ('100p00' not in r['sample_label'], float(r['authored_frame'])))
        entries = [(output/r['image'], f'{suffix[11:]} {r["sample_label"]}\nframe {float(r["authored_frame"]):.4g}') for r in group]
        sheet(entries, output/name, columns, 250 if columns == 6 else 400,
              (0, 280, 768, 650) if suffix.endswith('head') else (0, 320, 768, 620))
    report = {'scope': f'Only {len(selected)//2} representative blink states are converted, eyes_front and face_front ({len(selected)} PNGs). Full raw GPU samples and logs are retained separately. Compact sheets crop existing fixed-camera images for legibility.',
              'render_material_limit': 'Existing prepared FBX contains no Assimp texture slots; current unchanged runtime renders clay materials. This is deformation evidence, not a textured beauty render.',
              'source_audit': str(source/'gpu_audit.txt'),
              'source_audit_sha256': digest(source/'gpu_audit.txt'),
              'before_source_audit_sha256': digest(before/'gpu_audit.txt'),
              'selected_blink_states': len(selected)//2,
              'converted_png_count': len(converted),
              'neutral_rgb_comparison': comparisons, 'images': converted}
    (output/'manifest.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'images'}, indent=2))


if __name__ == '__main__':
    main()
