"""Read-only preservation audit for the body/clothing deformation stage (bpy 4.2).

Compares stored data, independently of evaluated scene pose. Weight edits and
additional actions/controls are allowed; existing authored data must be exact.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ORIGINAL_SHA256 = 'a602086ac9e580f2d265a489478922b75aae00410313b17a89bd846ad4f3b34c'
BODY_MESH = 'body_tex_skin_0'
# Explicit stage boundary: head meshes, hair and tail cannot acquire edits just
# because a clothing rivet happens to share a neck-bone influence.
EDITABLE_WEIGHT_MESHES = {
    BODY_MESH, 'armor_tex_armor_0', 'belt_tex_belt_0', 'bow_tex_bow_0',
    'cuff.001_tex_cuff_0', 'cuff_tex_cuff_0', 'long_gloves_tex_glove_0',
    'shirt.001_tex_shirt_0', 'shirt_tex_shirt_0', 'shoes_tex_shoes_0',
    'shoulder_armor_tex_shoulderarmor_0', 'stocking_tex_sock_0', 'underwear_tex_underwear_0',
}
HEAD_MESHES = {
    'body_tex_skin2_0', 'ear.001_tex_ear2_0', 'ear_tex_ear_0', 'eye_tex_eyes_0',
    'eyelashes_tex_eyelashes_0', 'hair.mesh.000_tex_hair1_0', 'hair_band_tex_band_0',
    'hightlight_hightlight_0', 'NurbsPath.001_tex_hair3_0', 'NurbsPath.002_tex_hair4_0',
    'NurbsPath.003_tex_hair5_0', 'NurbsPath.006_tex_hair6_0', 'NurbsPath.007_tex_hair7_0',
    'NurbsPath_tex_hair2_0',
}


def digest(value):
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                     separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def file_record(path):
    return {'path': str(path), 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def plain(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, bpy.types.ID):
        return {'id_type': value.bl_rna.identifier, 'name': value.name}
    if hasattr(value, 'to_dict'):
        return {k: plain(v) for k, v in value.to_dict().items()}
    try:
        return [plain(v) for v in value]
    except TypeError:
        raise TypeError(type(value).__name__)


def rna(value, skip=()):
    result = {}
    for prop in value.bl_rna.properties:
        key = prop.identifier
        if (key in {'rna_type', 'session_uid', 'is_valid', 'error_location', 'error_rotation',
                    'is_proxy_local', 'is_override_data'} or key in skip or
                prop.type == 'COLLECTION'):
            continue
        try:
            result[key] = plain(getattr(value, key))
        except (TypeError, AttributeError, ValueError):
            # Nested non-ID structures are handled explicitly by their owner.
            continue
    return result


def custom(value):
    return {k: plain(value[k]) for k in value.keys() if k != '_RNA_UI'}


def constraints(value):
    return [rna(c) for c in value.constraints]


def curve(fcurve):
    return {'rna': rna(fcurve, {'group'}),
            'group': fcurve.group.name if fcurve.group else None,
            'keys': [rna(k) for k in fcurve.keyframe_points],
            'sampled': [rna(k) for k in fcurve.sampled_points],
            'modifiers': [rna(m) for m in fcurve.modifiers]}


def drivers(owner):
    if not owner.animation_data:
        return {}
    return {f'{f.data_path}[{f.array_index}]':
            {'curve': curve(f), 'driver': rna(f.driver),
             'variables': [{'rna': rna(v), 'targets': [rna(t) for t in v.targets]}
                           for v in f.driver.variables]}
            for f in owner.animation_data.drivers}


def array_hash(collection, field, dimensions, dtype='<f4'):
    values = np.empty(len(collection) * dimensions, dtype=dtype)
    collection.foreach_get(field, values)
    return hashlib.sha256(values.tobytes()).hexdigest()


def material(material):
    result = {'rna': rna(material, {'preview', 'grease_pencil'}),
              'custom': custom(material)}
    if material.node_tree:
        result['nodes'] = {n.name: {'rna': rna(n, {'internal_links'}),
                                  'inputs': [{ 'name': s.name,
                                              'default': plain(s.default_value) if hasattr(s, 'default_value') else None}
                                             for s in n.inputs]}
                           for n in material.node_tree.nodes}
        result['links'] = sorted((l.from_node.name, l.from_socket.identifier,
                                  l.to_node.name, l.to_socket.identifier)
                                 for l in material.node_tree.links)
        result['drivers'] = drivers(material.node_tree)
    return result


def snapshot(path):
    bpy.ops.wm.open_mainfile(filepath=str(path))
    arms = [o for o in bpy.data.objects if o.type == 'ARMATURE']
    if len(arms) != 1:
        raise ValueError(f'Expected one armature, found {len(arms)}')
    arm = arms[0]
    bone_names = set(arm.data.bones.keys())
    protected_bones = {n for n in bone_names
                       if n.startswith(('head.', 'neck.', 'eye.', 'ear.', 'brow.'))}
    result = {'bones': {}, 'meshes': {}, 'weights': {}, 'face_mask': {},
              'actions': {}, 'controls': {}, 'pose_controls': {}, 'drivers': {}, 'constraints': {}}
    for bone in arm.data.bones:
        result['bones'][bone.name] = {
            'parent': bone.parent.name if bone.parent else None,
            'matrix_local': plain(bone.matrix_local), 'head': plain(bone.head_local),
            'tail': plain(bone.tail_local), 'deform': bone.use_deform,
            'inherit_scale': bone.inherit_scale,
            'use_inherit_rotation': bone.use_inherit_rotation,
            'use_local_location': bone.use_local_location,
            'custom': custom(bone)}
    for obj in bpy.data.objects:
        d = drivers(obj)
        if d:
            result['drivers']['object:' + obj.name] = d
        result['constraints']['object:' + obj.name] = constraints(obj)
        if obj.type == 'ARMATURE':
            for p in obj.pose.bones:
                result['constraints']['bone:' + p.name] = constraints(p)
                result['pose_controls'][p.name] = {
                    'custom': custom(p),
                    'custom_shape': p.custom_shape.name if p.custom_shape else None,
                    'custom_shape_transform': p.custom_shape_transform.name if p.custom_shape_transform else None,
                    'custom_shape_translation': plain(p.custom_shape_translation),
                    'custom_shape_rotation_euler': plain(p.custom_shape_rotation_euler),
                    'custom_shape_scale_xyz': plain(p.custom_shape_scale_xyz),
                    'use_custom_shape_bone_size': p.use_custom_shape_bone_size,
                    'lock_location': plain(p.lock_location), 'lock_rotation': plain(p.lock_rotation),
                    'lock_scale': plain(p.lock_scale), 'lock_rotation_w': p.lock_rotation_w,
                    'lock_rotations_4d': p.lock_rotations_4d,
                    'ik_stretch': p.ik_stretch, 'use_ik_rotation_control': p.use_ik_rotation_control,
                    'use_ik_linear_control': p.use_ik_linear_control,
                    'bone_collections': sorted(c.name for c in p.bone.collections)}
            result['controls'][obj.name] = {'custom': custom(obj),
                                           'transform': plain(obj.matrix_world)}
        elif obj.type == 'EMPTY' and (obj.name.startswith(('CTRL_', '_')) or
                                     obj.animation_data or obj.constraints):
            result['controls'][obj.name] = {
                'type': obj.type, 'parent': obj.parent.name if obj.parent else None,
                'matrix_parent_inverse': plain(obj.matrix_parent_inverse),
                'matrix_basis': plain(obj.matrix_basis), 'custom': custom(obj),
                'display_type': obj.empty_display_type, 'display_size': obj.empty_display_size,
                'hide_viewport': obj.hide_viewport, 'hide_render': obj.hide_render}
        if obj.type != 'MESH':
            continue
        mesh = obj.data
        co = np.array([v.co[:] for v in mesh.vertices], dtype=float)
        w = [{obj.vertex_groups[g.group].name: float(g.weight) for g in v.groups}
             for v in mesh.vertices]
        result['weights'][obj.name] = w
        result['face_mask'][obj.name] = [bool(obj.name in HEAD_MESHES or
                                             (obj.name == BODY_MESH and
                                              (p[2] >= 335 or any(n in protected_bones and weight > 0
                                                                 for n, weight in weights.items()))))
                                        for p, weights in zip(co, w)]
        shape_keys = None
        if mesh.shape_keys:
            shape_keys = {'rna': rna(mesh.shape_keys),
                          'blocks': {b.name: {'rna': rna(b),
                                              'co': array_hash(b.data, 'co', 3)}
                                     for b in mesh.shape_keys.key_blocks},
                          'drivers': drivers(mesh.shape_keys)}
        result['meshes'][obj.name] = {
            'vertices': len(mesh.vertices), 'positions_sha256': array_hash(mesh.vertices, 'co', 3),
            'edges_sha256': array_hash(mesh.edges, 'vertices', 2, '<i4'),
            'loops_sha256': array_hash(mesh.loops, 'vertex_index', 1, '<i4'),
            'polygon_starts_sha256': array_hash(mesh.polygons, 'loop_start', 1, '<i4'),
            'polygon_sizes_sha256': array_hash(mesh.polygons, 'loop_total', 1, '<i4'),
            'polygon_materials_sha256': array_hash(mesh.polygons, 'material_index', 1, '<i4'),
            'smooth_sha256': array_hash(mesh.polygons, 'use_smooth', 1, '?'),
            'normals_sha256': array_hash(mesh.corner_normals, 'vector', 3),
            'uv_layers': {layer.name: array_hash(layer.data, 'uv', 2) for layer in mesh.uv_layers},
            'materials': [m.name if m else None for m in mesh.materials],
            'matrix_world': plain(obj.matrix_world), 'shape_keys': shape_keys,
            'modifiers': [rna(m) for m in obj.modifiers]}
    result['materials'] = {m.name: material(m) for m in bpy.data.materials}
    result['images'] = {im.name: {'path': im.filepath, 'size': list(im.size),
                                'colorspace': im.colorspace_settings.name,
                                'packed_sha256': hashlib.sha256(im.packed_file.data).hexdigest()
                                                 if im.packed_file else None}
                        for im in bpy.data.images}
    result['actions'] = {a.name: {'frame_range': list(a.frame_range),
                                'curves': sorted([curve(c) for c in a.fcurves],
                                                 key=lambda c: (c['rna']['data_path'], c['rna']['array_index']))}
                         for a in bpy.data.actions}
    return result


def compare_preserved(base, candidate):
    missing = sorted(set(base) - set(candidate))
    changed = sorted(n for n in base.keys() & candidate.keys() if base[n] != candidate[n])
    return {'preserved': not missing and not changed, 'missing': missing,
            'changed': changed, 'added': sorted(set(candidate) - set(base)),
            'base_sha256': digest(base),
            'candidate_existing_sha256': digest({n: candidate.get(n) for n in base})}


def compare(base, candidate):
    result = {key: compare_preserved(base[key], candidate[key])
              for key in ('bones', 'meshes', 'materials', 'images', 'actions', 'controls', 'pose_controls',
                          'constraints', 'drivers')}
    edits = []
    face_changes = []
    invalid = []
    for name, before in base['weights'].items():
        after = candidate['weights'].get(name, [])
        if len(before) != len(after):
            invalid.append({'mesh': name, 'reason': 'vertex_count_changed'})
            continue
        changed = []
        for index, (a, b) in enumerate(zip(before, after)):
            if a != b:
                row = {'vertex': index, 'before': a, 'after': b,
                       'max_absolute_delta': max(abs(a.get(n, 0) - b.get(n, 0)) for n in a.keys() | b.keys())}
                changed.append(row)
                if base['face_mask'][name][index] or candidate['face_mask'][name][index]:
                    face_changes.append({'mesh': name, **row})
            positive = {n: w for n, w in b.items() if w > 1e-8}
            if (not positive or len(positive) > 4 or abs(sum(positive.values()) - 1) > 1e-5 or
                    any(not np.isfinite(w) or w < 0 or n not in candidate['bones'] for n, w in b.items())):
                invalid.append({'mesh': name, 'vertex': index, 'weights': b})
        if changed:
            edits.append({'mesh': name, 'changed_vertices': len(changed),
                          'max_absolute_delta': max(v['max_absolute_delta'] for v in changed),
                          'vertices': changed})
    result['weights'] = {'changed_vertices': sum(m['changed_vertices'] for m in edits),
                         'changed_meshes': len(edits), 'changes': edits,
                         'invalid_vertices': invalid}
    result['face'] = {'guard': 'All dedicated head/hair/face meshes; body_tex_skin_0 vertices with local Z >= 335 source units OR positive head/neck/eye/ear/brow weights, union of both scenes. Neck-linked clothing rivets remain clothing.',
                      'dedicated_meshes': sorted(HEAD_MESHES),
                      'protected_vertices': sum(sum(mask) for mask in base['face_mask'].values()),
                      'weights_exact': not face_changes, 'changed_vertices': face_changes}
    out_of_scope = [e['mesh'] for e in edits if e['mesh'] not in EDITABLE_WEIGHT_MESHES]
    result['scope'] = {'editable_weight_meshes': sorted(EDITABLE_WEIGHT_MESHES),
                       'changed_meshes_outside_scope': out_of_scope}
    # Entire skeleton is fixed at this stage; existing elements of controls and
    # actions must survive, and additional test actions are allowed.
    result['checks'] = {key + '_preserved': value['preserved']
                        for key, value in result.items() if 'preserved' in value}
    result['checks'].update({'bone_count_241': len(candidate['bones']) == len(base['bones']) == 241,
                             'no_extra_bones': not result['bones']['added'],
                             'no_extra_meshes': not result['meshes']['added'],
                             'face_weights_exact': not face_changes,
                             'weight_changes_within_body_clothing_scope': not out_of_scope,
                             'weights_valid_four_influences': not invalid})
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', default='build/raphtalia_tools/deep/base/Raphtalia_prepared.blend')
    p.add_argument('--candidate', default='assets/models/Raphtalia_prepared.blend')
    p.add_argument('--output', default='docs/raphtalia/deep_deformation/integrity.json')
    args = p.parse_args()
    base_path, candidate_path = ((ROOT / v).resolve() for v in (args.base, args.candidate))
    original = ROOT / 'assets/models/Raphtalia.fbx'
    backup = ROOT / 'assets/models/backups/Raphtalia.original.fbx'
    paths = {'base': base_path, 'candidate': candidate_path, 'original': original}
    if backup.exists():
        paths['original_backup'] = backup
    before = {n: file_record(path) for n, path in paths.items()}
    base = snapshot(base_path)
    candidate = snapshot(candidate_path)
    report = {'blender': bpy.app.version_string, 'files': before, **compare(base, candidate)}
    report['checks']['original_sha256_unchanged'] = before['original']['sha256'] == ORIGINAL_SHA256
    report['checks']['backup_sha256_unchanged'] = ('original_backup' not in before or
                                                  before['original_backup']['sha256'] == ORIGINAL_SHA256)
    report['checks']['files_stable_during_audit'] = before == {n: file_record(path) for n, path in paths.items()}
    report['result'] = 'PASS' if all(report['checks'].values()) else 'FAIL'
    output = (ROOT / args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({'result': report['result'], 'checks': report['checks'],
                      'changed_meshes': report['weights']['changed_meshes'],
                      'changed_vertices': report['weights']['changed_vertices']}, indent=2))
    if report['result'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
