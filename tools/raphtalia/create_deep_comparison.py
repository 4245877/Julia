"""Build same-view body/clothing before/after evidence without changing assets.

The contact sheet has neutral labels. Numerical edge flags locate geometry
changes; neither their improvement nor a zero count establishes visual quality.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
ROWS = [
    ('torso_turn_30', 'waist', 'Поворот корпуса 30° — талия и пояс'),
    ('torso_bend_45', 'waist', 'Наклон корпуса 45° — талия и пояс'),
    ('overhead_turn', 'shoulder', 'Поднятые руки и поворот корпуса — плечи'),
    ('shoulder_forward_100', 'shoulder', 'Руки вперёд 100° — плечи и одежда'),
    ('knee_flex_120', 'knees', 'Сгибание коленей 120° — чулки и обувь'),
    ('ankle_-30', 'shoe', 'Сгибание стопы −30° — обувь'),
    ('hip_flex_90', 'hip', 'Сгибание бёдер 90° — таз и ноги'),
]
METRICS = ('max_ratio', 'p99', 'over_2', 'over_2_extension_gt_1mm', 'nonfinite')
RATIO_EPSILON = 1e-4


def record(path):
    return {'path': str(path), 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def font(size, bold=False, override=None):
    candidates = [override] if override else []
    windows = Path(os.environ.get('WINDIR', 'C:/Windows')) / 'Fonts'
    candidates += [windows / ('arialbd.ttf' if bold else 'arial.ttf'),
                   Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf' if bold else
                        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')]
    for path in candidates:
        if path and Path(path).is_file():
            return ImageFont.truetype(str(path), size)
    raise RuntimeError('A Cyrillic TrueType font is required; provide --font.')


def metric_delta(before, after):
    result = {}
    regressions = []
    for key in METRICS:
        if key not in before or key not in after:
            continue
        a, b = before[key], after[key]
        delta = b - a
        tolerance = RATIO_EPSILON if key in ('max_ratio', 'p99') else 0
        result[key] = {'before': a, 'after': b, 'delta': delta,
                       'increased': delta > tolerance,
                       'decreased': delta < -tolerance}
        if delta > tolerance:
            regressions.append(key)
    return {'metrics': result, 'increased_metric_flags': regressions}


def compare(before, after):
    result = {'poses': {}, 'regressions': [],
              'missing_poses': sorted(set(before['poses']) - set(after['poses'])),
              'extra_poses': sorted(set(after['poses']) - set(before['poses'])),
              'ratio_delta_flag_epsilon': RATIO_EPSILON,
              'limits': 'Positive deltas are diagnostic regressions, not an artistic verdict. Edge ratios do not measure volume, intersections, silhouette or cloth realism.'}
    for name in sorted(before['poses'].keys() & after['poses'].keys()):
        a, b = before['poses'][name], after['poses'][name]
        if a.get('rotations') != b.get('rotations'):
            raise ValueError(f'Pose rotations differ: {name}')
        entry = metric_delta(a, b)
        am = {m['mesh']: m for m in a['meshes']}
        bm = {m['mesh']: m for m in b['meshes']}
        entry['missing_meshes'] = sorted(set(am) - set(bm))
        entry['extra_meshes'] = sorted(set(bm) - set(am))
        entry['meshes'] = {}
        for mesh in sorted(am.keys() & bm.keys()):
            change = metric_delta(am[mesh], bm[mesh])
            entry['meshes'][mesh] = change
            if change['increased_metric_flags']:
                result['regressions'].append({'pose': name, 'mesh': mesh,
                                              'metrics': change['increased_metric_flags'],
                                              'deltas': {k: change['metrics'][k]['delta']
                                                         for k in change['increased_metric_flags']}})
        result['poses'][name] = entry
    result['summary'] = {
        'compared_poses': len(result['poses']),
        'pose_mesh_pairs_with_increased_flags': len(result['regressions']),
        'poses_with_increased_flags': len({r['pose'] for r in result['regressions']}),
        'pose_mesh_pairs_with_decreased_flags': sum(any(v['decreased'] for v in m['metrics'].values())
                                                   for p in result['poses'].values() for m in p['meshes'].values()),
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', default='docs/raphtalia/deep_deformation/before.json')
    parser.add_argument('--after', default='docs/raphtalia/deep_deformation/after.json')
    parser.add_argument('--output', default='docs/raphtalia/deep_deformation/comparison.png',
                        help='PNG path, or a directory in which comparison.png/json are written.')
    parser.add_argument('--tile', type=int, default=600)
    parser.add_argument('--font')
    args = parser.parse_args()
    paths = [(ROOT / p).resolve() for p in (args.before, args.after)]
    inputs = [record(p) for p in paths]
    before, after = [json.loads(p.read_text(encoding='utf-8')) for p in paths]
    result = compare(before, after)
    result['inputs'] = {'before': inputs[0], 'after': inputs[1],
                        'before_model_sha256': before.get('sha256'),
                        'after_model_sha256': after.get('sha256')}
    tile = args.tile
    if tile < 300:
        raise ValueError('--tile must be at least 300 pixels for readable labels.')
    title_height, row_title, caption, gap = 98, 48, 36, 12
    row_height = row_title + caption + tile + gap
    canvas = Image.new('RGB', (tile * 2, title_height + len(ROWS) * row_height), '#f4f4f2')
    draw = ImageDraw.Draw(canvas)
    heading = font(26, True, args.font)
    body = font(23, False, args.font)
    draw.text((20, 14), 'Raphtalia — деформации тела и одежды', font=heading, fill='#171717')
    draw.text((20, 54), 'Одинаковые позы, камеры и условия отображения', font=body, fill='#404040')
    result['image_pairs'] = []
    for row_index, (pose, view, label) in enumerate(ROWS):
        if pose not in before['poses'] or pose not in after['poses']:
            raise ValueError(f'Selected pose missing from a measurement file: {pose}')
        if before['views'].get(view) != after['views'].get(view):
            raise ValueError(f'View definitions differ: {view}. Regenerate both sides using the same VIEWS.')
        y = title_height + row_index * row_height
        draw.text((20, y + 8), label, font=body, fill='#171717')
        draw.text((20, y + row_title), 'До исправления', font=body, fill='#303030')
        draw.text((tile + 20, y + row_title), 'После исправления', font=body, fill='#303030')
        pair = {'pose': pose, 'view': view, 'view_definition_equal': True,
                'view_definition': before['views'][view], 'images': {}}
        for column, (tag, json_path) in enumerate(zip(('before', 'after'), paths)):
            path = json_path.with_name(json_path.stem + '_' + pose + '_' + view + '.png')
            if not path.is_file():
                raise FileNotFoundError(f'Missing comparison frame: {path}')
            with Image.open(path) as source:
                pair['images'][tag] = {**record(path), 'resolution': list(source.size)}
                im = source.convert('RGB')
                im.thumbnail((tile, tile), Image.Resampling.LANCZOS)
                x = column * tile + (tile - im.width) // 2
                top = y + row_title + caption + (tile - im.height) // 2
                canvas.paste(im, (x, top))
        if pair['images']['before']['resolution'] != pair['images']['after']['resolution']:
            raise ValueError(f'Input image resolutions differ: {pose}/{view}')
        result['image_pairs'].append(pair)
    output = (ROOT / args.output).resolve()
    if output.suffix.lower() != '.png':
        output = output / 'comparison.png'
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, optimize=True)
    result['contact_sheet'] = {**record(output), 'resolution': list(canvas.size), 'tile_pixels': tile}
    result['checks'] = {'same_pose_definitions': True, 'same_selected_views': True,
                         'same_image_resolutions': True, 'image_pairs_complete': len(result['image_pairs']) == len(ROWS),
                         'measurement_files_unchanged': inputs == [record(p) for p in paths],
                         'all_before_poses_present': not result['missing_poses'],
                         'mesh_sets_equal_in_common_poses': all(not p['missing_meshes'] and not p['extra_meshes']
                                                               for p in result['poses'].values())}
    output.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'output': str(output), 'checks': result['checks'], **result['summary']}, ensure_ascii=False, indent=2))
    if not all(result['checks'].values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
