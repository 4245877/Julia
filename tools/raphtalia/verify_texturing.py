"""Read-only exact preservation and texture-binding checks for this task."""
import argparse
import hashlib
import json
from pathlib import Path

import bpy
import numpy as np

from blink_integrity import snapshot

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / 'build/raphtalia_tools/texturing'
DOC = ROOT / 'docs/raphtalia/texturing'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--base', default='assets/models/Raphtalia_prepared.blend')
    p.add_argument('--candidate', default='build/raphtalia_tools/texturing/Raphtalia_textured.blend')
    args = p.parse_args()
    base = snapshot(ROOT / args.base)
    candidate = snapshot(ROOT / args.candidate)
    checks = {key: base[key] == candidate[key] for key in (
        'bones', 'pose_constraints', 'actions', 'drivers', 'meshes', 'weights', 'object_constraints')}
    checks['evaluated_neutral_vertices_exact'] = all(np.array_equal(values, candidate['neutral_vertices'][name])
        for name, values in base['neutral_vertices'].items())
    checks['24_images_4k_packed'] = len(bpy.data.images) == 24 and all(
        list(i.size) == [4096, 4096] and i.packed_file and i.colorspace_settings.name == 'sRGB'
        for i in bpy.data.images)
    sources = json.loads((DOC / 'texture_sources.json').read_text())
    checks['texture_files_match_manifest'] = all(
        hashlib.sha256((ROOT / t['file']).read_bytes()).hexdigest() == t['sha256'] for t in sources['maps'])
    bindings = json.loads((DOC / 'material_bindings.json').read_text())
    checks['26_material_bindings'] = len(bindings) == 26 and sum(b['image'] is not None for b in bindings) == 25
    checks['28_meshes'] = len(candidate['meshes']) == 28
    checks['429_bones_15_actions'] = len(candidate['bones']) == 429 and len(candidate['actions']) == 15
    report = {'base': args.base, 'candidate': args.candidate,
              'base_sha256': base['sha256'], 'candidate_sha256': candidate['sha256'],
              'checks': checks, 'all_pass': all(checks.values()),
              'meshes': len(candidate['meshes']), 'vertices': sum(len(w) for w in candidate['weights'].values()),
              'faces': sum(len(m['polygons']) for m in candidate['meshes'].values()),
              'bones': len(candidate['bones']), 'actions': list(candidate['actions'])}
    (DOC / 'blend_preservation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    assert report['all_pass']


if __name__ == '__main__':
    main()
