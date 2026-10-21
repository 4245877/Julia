"""Read-only inversion of authored Blink curves for exact GPU closure samples.

Uses the saved candidate's controller curves to find 0/25/50/75/100% closing
states. Writes a CSV accepted by build_gpu_audit.ps1 -BlinkSamplesCsv. Does not
save the opened .blend and does not export or modify any geometry.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import bpy

ROOT = Path(__file__).resolve().parents[2]


def crossings(curve):
    keys = sorted(float(k.co.x) for k in curve.keyframe_points)
    frames = set(keys)
    for start, end in zip(keys, keys[1:]):
        low, high = float(curve.evaluate(start)), float(curve.evaluate(end))
        if high - low < 1e-6:
            continue
        for value in (.25, .5, .75):
            if not low < value < high:
                continue
            a, b = start, end
            for _ in range(50):
                mid = (a+b)/2
                if curve.evaluate(mid) < value:
                    a = mid
                else:
                    b = mid
            frames.add((a+b)/2)
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blend', default='build/raphtalia_tools/blink/candidate.blend')
    parser.add_argument('--output', default='build/raphtalia_tools/blink/gpu_samples.csv')
    args = parser.parse_args()
    source = (ROOT/args.blend).resolve()
    output = (ROOT/args.output).resolve()
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(source))
    rows = []
    for action in sorted(bpy.data.actions, key=lambda a: a.name):
        if not action.name.startswith('TEST_blink_'):
            continue
        curves = {side: next((c for c in action.fcurves
                             if c.data_path == f'["Blink_{side}"]'), None)
                  for side in ('L', 'R')}
        if any(c is None for c in curves.values()):
            raise ValueError(f'{action.name}: missing independent Blink curves')
        frames = sorted({round(f, 7) for c in curves.values() for f in crossings(c)})
        for frame in frames:
            left, right = (float(curves[s].evaluate(frame)) for s in ('L', 'R'))
            label = f'L{left*100:.2f}_R{right*100:.2f}'.replace('.', 'p')
            rows.append((action.name, f'{frame:.7f}', label))
    if not rows:
        raise ValueError('No TEST_blink_* actions found')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('clip_suffix', 'frame', 'label'))
        writer.writerows(rows)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
    print(f'Wrote {len(rows)} authored closure samples: {output}')
    print('Labels refer to the Blender control curves; FBX must use matching baked actions.')


if __name__ == '__main__':
    main()
