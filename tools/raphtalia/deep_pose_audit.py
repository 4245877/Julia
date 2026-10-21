"""Read-only body/clothing range tests. Angles are deltas from the baked rest.

No facial pose is authored here. Identical named views work for before/after.
The measurements locate defects; they do not certify collision-free anatomy.
"""
import argparse
import hashlib
import json
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

from reaudit import ROOT, reset, find, turn, cache_meshes, measure, render


PEAKS = ['overhead_turn', 'squat', 'high_step', 'arm_combined', 'torso_combined', 'cloth_fan']


def definitions(arm):
    def pair(prefix, angle, axis=(1, 0, 0)):
        return [(find(arm, prefix, side), angle, axis) for side in ('l', 'r')]
    def spine(angle, axis):
        return [(name, angle / 2, axis) for name in ('spine_01.x_014', 'spine_02.x_015')]
    cases = {}
    for angle in (45, 90, 120, 135):
        cases[f'elbow_flex_{angle}'] = pair('forearm_stretch', -angle)
    for angle in (60, 90, 120, 135):
        cases[f'knee_flex_{angle}'] = pair('leg_stretch', angle)
    for angle in (60, 90, 110):
        cases[f'hip_flex_{angle}'] = pair('thigh_stretch', -angle)
    for angle in (45, 80):
        cases[f'hip_abduction_{angle}'] = [(find(arm, 'thigh_stretch', s), angle if s == 'l' else -angle, (0, 1, 0)) for s in ('l', 'r')]
    for angle in (60, 100, 140):
        cases[f'shoulder_forward_{angle}'] = pair('arm_stretch', -angle)
    for angle in (60, 100, 140):
        cases[f'shoulder_side_{angle}'] = [(find(arm, p, s), a if s == 'l' else -a, (0, 1, 0)) for s in ('l', 'r') for p, a in [('shoulder', 15), ('arm_stretch', angle - 15)]]
    for angle in (-60, 60):
        cases[f'forearm_roll_{angle}'] = [(find(arm, 'hand', s), angle, tuple(arm.data.bones[find(arm, 'forearm_stretch', s)].y_axis)) for s in ('l', 'r')]
    for angle in (-45, 45):
        cases[f'upper_arm_roll_{angle}'] = [(find(arm, 'arm_stretch', s), angle, tuple(arm.data.bones[find(arm, 'arm_stretch', s)].y_axis)) for s in ('l', 'r')]
    for angle in (-45, 45):
        cases[f'wrist_flex_{angle}'] = pair('hand', angle)
    for angle in (-25, 25):
        cases[f'wrist_side_{angle}'] = pair('hand', angle, (0, 0, 1))
    for angle in (-30, 30):
        cases[f'ankle_{angle}'] = pair('foot', angle)
        cases[f'toes_{angle}'] = pair('toes_01', angle)
    for angle in (-20, 20):
        cases[f'foot_roll_{angle}'] = [(find(arm, 'foot', s), angle, tuple(arm.data.bones[find(arm, 'foot', s)].y_axis)) for s in ('l', 'r')]
    for angle in (-30, 30, 45):
        cases[f'torso_bend_{angle}'] = spine(angle, (1, 0, 0))
    for axis, vector in [('side', (0, 1, 0)), ('turn', (0, 0, 1))]:
        for angle in (-30, 30, 45):
            cases[f'torso_{axis}_{angle}'] = spine(angle, vector)
    cases['overhead_turn'] = cases['shoulder_side_140'] + spine(30, (0, 0, 1))
    cases['squat'] = pair('thigh_stretch', -90) + pair('leg_stretch', 120) + pair('foot', -30) + spine(30, (1, 0, 0))
    cases['high_step'] = [(find(arm, 'thigh_stretch', 'l'), -100, (1, 0, 0)), (find(arm, 'leg_stretch', 'l'), 100, (1, 0, 0))] + spine(20, (1, 0, 0))
    cases['arm_combined'] = cases['shoulder_forward_60'] + cases['elbow_flex_120'] + cases['forearm_roll_60']
    # Compose rather than replace two rotations on the same bone (apply below).
    cases['torso_combined'] = spine(30, (1, 0, 0)) + spine(30, (0, 0, 1)) + cases['hip_flex_60'] + cases['knee_flex_90']
    cases['arms_open_turn'] = cases['shoulder_side_100'] + spine(-30, (0, 0, 1))
    cases['lean_crouch'] = spine(45, (1, 0, 0)) + cases['hip_flex_60'] + cases['knee_flex_90']
    cases['cloth_fan'] = spine(20, (0, 0, 1)) + [(b.name, 8 if b.parent and b.parent.name == 'root.x_01' else 5, (1, 0, 0)) for b in arm.data.bones if b.name.startswith('cloth.')]
    return cases


def apply_pose(arm, changes, strength=1):
    for name, angle, axis in changes:
        p = arm.pose.bones[name]
        q = p.bone.matrix_local.to_quaternion()
        delta = q.inverted() @ Quaternion(Vector(axis), np.radians(angle * strength)) @ q
        p.rotation_quaternion = p.rotation_quaternion @ delta
    bpy.context.view_layer.update()


def add_clip(arm):
    assert not bpy.data.actions.get('TEST_deep_body_cloth'), 'Refuse to apply twice'
    cases = definitions(arm)
    reset(arm)
    action = bpy.data.actions.new('TEST_deep_body_cloth')
    action.use_fake_user = True
    arm.animation_data_create()
    arm.animation_data.action = action
    frames = [(1, None)]
    for i, name in enumerate(PEAKS):
        frames += [(31 + i * 60, name), (61 + i * 60, None)]
    for frame, name in frames:
        for bone in arm.pose.bones:
            bone.matrix_basis = Matrix.Identity(4)
            bone.rotation_mode = 'QUATERNION'
        apply_pose(arm, cases.get(name, []))
        for bone in arm.pose.bones:
            for path in ('rotation_quaternion', 'location', 'scale'):
                bone.keyframe_insert(path, frame=frame, group=bone.name)
    for curve in action.fcurves:
        for key in curve.keyframe_points:
            key.interpolation = 'LINEAR'
    reset(arm)
    return {'action': action.name, 'frames': frames, 'fps': 30, 'duration_s': 12}


VIEWS = {
    'shoulder': ((0, 0, 3.24), 1.65, (1, -2, .35), ['body_tex_skin_0', 'shirt_tex_shirt_0', 'long_gloves_tex_glove_0', 'cuff_tex_cuff_0', 'cuff.001_tex_cuff_0', 'armor_tex_armor_0', 'shoulder_armor_tex_shoulderarmor_0']),
    'waist': ((0, -.03, 2.32), 1.4, (1, -2, .15), ['body_tex_skin_0', 'shirt_tex_shirt_0', 'belt_tex_belt_0', 'armor_tex_armor_0']),
    'knees': ((0, .18, 1.13), 1.65, (1.5, -2, .3), ['body_tex_skin_0', 'stocking_tex_sock_0', 'shoes_tex_shoes_0', 'kneepad_tex_kneepad_0']),
    'body': ((0, 0, 2.1), 4.8, (1, -2, .2), None),
    'side': ((0, -.2, 2.1), 4.8, (2, -.3, .15), None),
    'hip': ((0, .05, 2.12), 1.25, (1.5, 2, .15), ['body_tex_skin_0', 'stocking_tex_sock_0', 'underwear_tex_underwear_0']),
    'elbow': ((0, 0, 0), .95, (1.5, -2, .2), ['body_tex_skin_0', 'shirt_tex_shirt_0', 'long_gloves_tex_glove_0', 'cuff_tex_cuff_0', 'cuff.001_tex_cuff_0']),
    'wrist': ((0, 0, 0), .65, (1.5, -2, .2), ['body_tex_skin_0', 'long_gloves_tex_glove_0', 'cuff.001_tex_cuff_0']),
    'shoe': ((0, 0, 0), .85, (1.5, -2, .25), ['body_tex_skin_0', 'shoes_tex_shoes_0', 'stocking_tex_sock_0']),
}


def render_case(out, label):
    views = ['body']
    if label.startswith('shoulder') or label == 'overhead_turn': views += ['shoulder']
    if label.startswith('torso') or label == 'cloth_fan': views += ['waist']
    if label.startswith('knee'): views += ['knees']
    if label in ('squat', 'high_step'): views += ['side']
    if label.startswith('hip_flex'): views += ['hip']
    if label.startswith('elbow_flex') or label == 'arm_combined': views += ['elbow']
    if label.startswith('wrist_'): views += ['wrist']
    if label.startswith('ankle_') or label.startswith('foot_roll_'): views += ['shoe']
    for name in views:
        target, scale, view, objects = VIEWS[name]
        if name in ('elbow', 'wrist', 'shoe'):
            arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
            prefix = {'elbow': 'forearm_stretch', 'wrist': 'hand', 'shoe': 'foot'}[name]
            target = arm.matrix_world @ arm.pose.bones[find(arm, prefix, 'l')].head
        render(out.parent / (out.stem + '_' + label + '_' + name + '.png'), target, scale, view, objects)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--blend', default='assets/models/Raphtalia_prepared.blend')
    parser.add_argument('--output', default='docs/raphtalia/deep_deformation/before.json')
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--only', nargs='*')
    args = parser.parse_args()
    path = (ROOT / args.blend).resolve()
    out = (ROOT / args.output).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    before_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(path))
    arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
    meshes = [o for o in bpy.data.objects if o.type == 'MESH']
    reset(arm)
    cache = cache_meshes(meshes)
    cases = definitions(arm)
    result = {'sha256': before_hash, 'bones': len(arm.data.bones), 'poses': {}, 'views': VIEWS,
              'limits': 'Angles are additive rest deltas, not anatomical ROM. Edge ratios do not measure volume or collisions. No facial channels are posed.'}
    for label, changes in cases.items():
        if args.only and label not in args.only: continue
        reset(arm)
        apply_pose(arm, changes)
        result['poses'][label] = {'rotations': changes, **measure(meshes, cache, True)}
        if args.render and (args.only or label in PEAKS + ['knee_flex_120', 'shoulder_forward_100', 'torso_turn_30', 'torso_bend_45', 'hip_flex_90', 'elbow_flex_135', 'wrist_flex_45', 'ankle_-30']):
            render_case(out, label)
        row = result['poses'][label]
        print(label, round(row['max_ratio'], 3), row['over_2'], flush=True)
    result['file_unchanged'] = before_hash == hashlib.sha256(path.read_bytes()).hexdigest()
    result['summary'] = {'poses': len(result['poses']), 'nonfinite': sum(r['nonfinite'] for r in result['poses'].values())}
    out.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(result['summary'], flush=True)


if __name__ == '__main__':
    main()
