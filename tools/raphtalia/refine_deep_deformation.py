"""Stage-three local body/clothing correction; writes explicit candidates only.

Start with the retained 241-bone/10-action prepared snapshot. The source FBX
and input BLEND are read-only. --baseline adds only the same diagnostic clip.
"""
import argparse
import hashlib
import json

import bpy

from reaudit import ROOT, reset
from refine_rig import export_fbx
from deep_pose_audit import add_clip


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='build/raphtalia_tools/deep/base/Raphtalia_prepared.blend')
    parser.add_argument('--output', default='build/raphtalia_tools/deep/candidate.blend')
    parser.add_argument('--baseline', action='store_true')
    parser.add_argument('--export', action='store_true')
    args = parser.parse_args()
    source = (ROOT / args.input).resolve()
    target = (ROOT / args.output).resolve()
    assert source != target, 'Never overwrite input snapshot'
    target.parent.mkdir(parents=True, exist_ok=True)
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(source))
    arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
    meshes = [o for o in bpy.data.objects if o.type == 'MESH']
    assert len(arm.data.bones) == 241 and len(bpy.data.actions) == 10, 'Expected the exact stage-two model'
    reset(arm)
    result = {'input': str(source.relative_to(ROOT)), 'sha256': source_hash, 'baseline': args.baseline, 'patches': []}
    if not args.baseline:
        from deep_body import apply as apply_body
        from deep_body import apply_limb_clothes
        from deep_cloth import apply as apply_cloth
        result['patches'].extend(apply_body(arm, meshes))
        result['patches'].extend(apply_limb_clothes(arm, meshes))
        result['patches'].extend(apply_cloth(arm, meshes))
    result['clip'] = add_clip(arm)
    reset(arm)
    arm.animation_data_create()
    arm.animation_data.action = bpy.data.actions['TEST_elbows_90']
    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = 361
    scene.frame_set(1)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(target), check_existing=False)
    target.with_suffix('.patch.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    if args.export:
        export_fbx(target.with_suffix('.fbx'), arm, meshes)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash
    print('CANDIDATE', target, len(arm.data.bones), len(bpy.data.actions), flush=True)


if __name__ == '__main__':
    main()
