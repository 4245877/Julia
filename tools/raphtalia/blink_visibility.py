"""Read-only orthographic skin-occlusion audit of the blink candidate.

Ray tests include evaluated iris, sclera and highlight sample points. Only the
evaluated body skin can occlude them: hair and lashes never hide a closure gap
from this test. Finite sampling cannot establish continuous surface coverage or
visibility between the selected cameras; visual inspection remains necessary.
"""
from __future__ import annotations

from pathlib import Path
import argparse
import hashlib
import json
import math
from itertools import product

import bpy
import numpy as np
from mathutils import Quaternion, Vector
from mathutils.bvhtree import BVHTree

from reaudit import reset

ROOT = Path(__file__).resolve().parents[2]
TARGETS = {
    'eye_tex_eyes_0': 'iris',
    'body_tex_skin2_0': 'sclera',
    'hightlight_hightlight_0': 'highlights',
}
GAZES = [
    ('straight', 0., 0.), ('up8', 0., 8.), ('down8', 0., -8.),
    ('left12', 12., 0.), ('right12', -12., 0.), ('diagonal10_6', 10., 6.),
]
STATES = [(f'both_{int(level*100):03}', level, level)
          for level in (0., .25, .5, .75, 1.)] + [
    ('left_closed', 1., 0.), ('right_closed', 0., 1.),
]


def record(path):
    return {'path': str(path), 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def evaluated_geometry(obj):
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = evaluated.to_mesh()
    try:
        coordinates = np.empty(len(mesh.vertices)*3, dtype=np.float64)
        mesh.vertices.foreach_get('co', coordinates)
        coordinates = coordinates.reshape(-1, 3)
        matrix = np.asarray(evaluated.matrix_world, dtype=np.float64)
        world = coordinates @ matrix[:3, :3].T + matrix[:3, 3]
        mesh.calc_loop_triangles()
        triangles = np.array([tuple(t.vertices) for t in mesh.loop_triangles], dtype=np.int32)
        return world, triangles
    finally:
        evaluated.to_mesh_clear()


def descriptors(obj):
    """Stable topology samples: vertices, every triangle edge midpoint, centers."""
    world, triangles = evaluated_geometry(obj)
    result = []
    edges = set()
    for triangle in triangles:
        for a, b in zip(triangle, np.roll(triangle, -1)):
            edges.add(tuple(sorted((int(a), int(b)))))
    entries = [(f'v{i}', (i,)) for i in range(len(world))]
    entries += [(f'e{a}_{b}', (a, b)) for a, b in sorted(edges)]
    entries += [(f't{i}', tuple(map(int, triangle))) for i, triangle in enumerate(triangles)]
    for sample_id, indices in entries:
        # Side is assigned once in the neutral source geometry, never by the
        # posed eye's position. Thus IDs remain comparable across gaze/blink.
        x = float(world[list(indices), 0].mean())
        if abs(x) < 1e-9:
            raise ValueError(f'{obj.name}/{sample_id}: target crosses eye center plane')
        result.append((sample_id, indices, 'L' if x > 0 else 'R'))
    return {'samples': result, 'vertices': len(world), 'triangles': triangles.tolist()}


def gaze(arm, yaw, pitch):
    for side in ('L', 'R'):
        bone = arm.pose.bones['eye.'+side]
        basis = bone.bone.matrix_local.to_quaternion()
        bone.rotation_mode = 'QUATERNION'
        bone.rotation_quaternion = (basis.inverted()
            @ Quaternion(Vector((0, 0, 1)), math.radians(yaw))
            @ Quaternion(Vector((1, 0, 0)), math.radians(pitch)) @ basis)


def state(arm, topology, left, right, epsilon, max_examples, view_yaw=0.):
    arm['Blink_L'] = float(left)
    arm['Blink_R'] = float(right)
    arm.update_tag()
    bpy.context.view_layer.update()
    skin_points, skin_triangles = evaluated_geometry(bpy.data.objects['body_tex_skin_0'])
    nonfinite_skin = int((~np.isfinite(skin_points).all(axis=1)).sum())
    if nonfinite_skin:
        return {'nonfinite_skin_vertices': nonfinite_skin, 'nonfinite_eye_samples': 0,
                'meshes': {}, 'visibility_sets': {}}, {}
    skin = BVHTree.FromPolygons([Vector(p) for p in skin_points],
                               skin_triangles.tolist(), all_triangles=True)
    result = {'nonfinite_skin_vertices': 0, 'nonfinite_eye_samples': 0, 'meshes': {}}
    visible_sets = {}
    theta = math.radians(view_yaw)
    ray_direction = Vector((math.sin(theta), math.cos(theta), 0.))
    for name, description in topology.items():
        points, triangles = evaluated_geometry(bpy.data.objects[name])
        if len(points) != description['vertices'] or triangles.tolist() != description['triangles']:
            raise ValueError(f'{name}: evaluated topology changed between states')
        by_side = {s: {'sample_count': 0, 'visible_count': 0, 'occluded_count': 0,
                      'coincident_depth_count': 0, 'nonfinite_count': 0,
                      'rays_without_skin_hit': 0, 'minimum_skin_depth_margin_m': None,
                      'visible_examples': []} for s in ('L', 'R')}
        visible_sets[name] = {s: set() for s in ('L', 'R')}
        for sample_id, indices, side in description['samples']:
            row = by_side[side]
            row['sample_count'] += 1
            point = points[list(indices)].mean(axis=0)
            if not np.isfinite(point).all():
                row['nonfinite_count'] += 1
                result['nonfinite_eye_samples'] += 1
                continue
            if point[1] <= -10:
                raise ValueError('Target lies in front of the fixed ray origin')
            # The central view keeps the specifically requested y=-10 origin.
            # Oblique orthographic rays start 10m before the sample along the
            # selected view direction; margin is measured along that ray.
            origin = (Vector((float(point[0]), -10., float(point[2])))
                      if abs(view_yaw) < 1e-12 else Vector(point)-ray_direction*10.)
            sample_distance = float((Vector(point)-origin).dot(ray_direction))
            hit, normal, triangle, distance = skin.ray_cast(origin, ray_direction, 25.)
            margin = float(sample_distance-distance) if hit is not None else None
            if margin is not None:
                previous = row['minimum_skin_depth_margin_m']
                row['minimum_skin_depth_margin_m'] = margin if previous is None else min(previous, margin)
                if abs(margin) <= epsilon:
                    row['coincident_depth_count'] += 1
            else:
                row['rays_without_skin_hit'] += 1
            # Meeting surfaces count as covered within an explicit world-space
            # tolerance. They are reported separately from strictly-ahead skin.
            occluded = margin is not None and margin >= -epsilon
            row['occluded_count' if occluded else 'visible_count'] += 1
            if not occluded:
                visible_sets[name][side].add(sample_id)
                if len(row['visible_examples']) < max_examples:
                    row['visible_examples'].append({'sample': sample_id,
                        'world_position_m': point.tolist(),
                        'first_skin_world_position_m': list(hit) if hit is not None else None,
                        'first_skin_y_m': float(hit.y) if hit is not None else None,
                        'skin_depth_margin_m': margin})
        result['meshes'][name] = {'role': TARGETS[name], 'sides': by_side}
    return result, visible_sets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blend', default='build/raphtalia_tools/blink/candidate.blend')
    parser.add_argument('--output', default='docs/raphtalia/blink/visibility.json')
    parser.add_argument('--depth-epsilon', type=float, default=1e-6)
    parser.add_argument('--max-examples', type=int, default=16)
    parser.add_argument('--view-yaws', type=float, nargs='+', default=[0., -25., 25.],
                        help='Orthographic camera directions in source world XY plane')
    parser.add_argument('--strict', action='store_true', help='Return nonzero if diagnostic checks fail')
    args = parser.parse_args()
    if args.depth_epsilon < 0 or not math.isfinite(args.depth_epsilon):
        parser.error('--depth-epsilon must be finite and nonnegative')
    if not all(math.isfinite(angle) and abs(angle) < 90 for angle in args.view_yaws):
        parser.error('--view-yaws must be finite and within (-90,90) degrees')
    source, output = (ROOT/args.blend).resolve(), (ROOT/args.output).resolve()
    if source == output:
        raise ValueError('Evidence output must differ from the model')
    before = record(source)
    bpy.ops.wm.open_mainfile(filepath=str(source))
    arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
    for prop in ('Blink_L', 'Blink_R'):
        if prop not in arm:
            raise ValueError(f'Missing {prop} controller')
    reset(arm)
    arm['Blink_L'] = arm['Blink_R'] = 0.
    arm.update_tag()
    bpy.context.view_layer.update()
    topology = {name: descriptors(bpy.data.objects[name]) for name in TARGETS}
    result = {'input': before, 'blender_version': bpy.app.version_string,
        'method': 'Evaluated body skin BVH; front orthographic rays from (sample.x,-10,sample.z) toward +Y; oblique rays rotated in source world XY. Only body_tex_skin_0 occludes. Samples: all vertices, triangle edge midpoints (split 2), and triangle barycenters.',
        'coordinate_space': 'World coordinates after evaluated matrix_world; depth values in model world metres. No implicit 0.01 rescale is applied.',
        'skin_matrix_world': [list(row) for row in bpy.data.objects['body_tex_skin_0'].matrix_world],
        'depth_epsilon_m': args.depth_epsilon,
        'view_yaws_degrees': args.view_yaws,
        'limitations': ['Finite samples cannot prove continuous coverage between samples.',
            'Only the listed fixed orthographic camera directions are tested; no unrestricted camera claim.',
            'Opaque skin occlusion only; texture alpha, lashes and hair are excluded.',
            'Eye sample sets may contain self-occluded/rear surfaces: all points are conservatively tested against skin, and neutral skin-visible subsets are separately tracked.',
            'Angles follow the existing Blender source-axis gaze convention.'],
        'sample_counts': {name: len(data['samples']) for name, data in topology.items()},
        'cases': [], 'checks': {}}
    closed_checks, opposite_checks, nonfinite = [], [], 0
    for view_yaw, (direction, yaw, pitch) in product(args.view_yaws, GAZES):
        reset(arm)
        gaze(arm, yaw, pitch)
        baseline = None
        for label, left, right in STATES:
            case, visible = state(arm, topology, left, right,
                                  args.depth_epsilon, args.max_examples, view_yaw)
            case.update({'gaze': direction, 'yaw_degrees': yaw, 'pitch_degrees': pitch,
                         'camera_yaw_degrees': view_yaw,
                         'state': label, 'Blink_L': left, 'Blink_R': right})
            nonfinite += case['nonfinite_skin_vertices'] + case['nonfinite_eye_samples']
            if baseline is None:
                baseline = visible
            for name, mesh in case['meshes'].items():
                for side, row in mesh['sides'].items():
                    original = baseline[name][side]
                    now = visible[name][side]
                    row['baseline_skin_visible_count'] = len(original)
                    row['baseline_visible_still_visible_count'] = len(original & now)
                    row['newly_skin_visible_count'] = len(now - original)
                    blink = left if side == 'L' else right
                    if blink == 1.:
                        passed = row['visible_count'] == 0
                        row['closed_skin_coverage_pass'] = passed
                        closed_checks.append(passed)
                    elif (left == 1. or right == 1.) and blink == 0.:
                        passed = now == original
                        row['wink_opposite_visible_set_preserved'] = passed
                        opposite_checks.append(passed)
            result['cases'].append(case)
            print(view_yaw, direction, label, {name: {s: mesh['sides'][s]['visible_count']
                  for s in ('L', 'R')} for name, mesh in case['meshes'].items()}, flush=True)
    after = record(source)
    result['input_stable_during_audit'] = before == after
    result['checks'] = {'all_values_finite': nonfinite == 0,
        'nonfinite_total': nonfinite, 'closed_mesh_side_checks': len(closed_checks),
        'closed_mesh_side_failures': sum(not value for value in closed_checks),
        'all_closed_eye_samples_covered_by_skin': bool(closed_checks) and all(closed_checks),
        'opposite_eye_checks': len(opposite_checks),
        'opposite_eye_failures': sum(not value for value in opposite_checks),
        'all_opposite_wink_visible_sets_preserved': bool(opposite_checks) and all(opposite_checks)}
    result['pass'] = (result['input_stable_during_audit']
        and result['checks']['all_values_finite']
        and result['checks']['all_closed_eye_samples_covered_by_skin']
        and result['checks']['all_opposite_wink_visible_sets_preserved'])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print('WROTE', output, 'PASS' if result['pass'] else 'FAIL', result['checks'], flush=True)
    if args.strict and not result['pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
