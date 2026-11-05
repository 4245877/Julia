"""Inspect the current mesh/UVs without modifying the asset."""
import hashlib
import json
from pathlib import Path

import bpy
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'docs/raphtalia/texturing'


def array_hash(collection, key, width, dtype=np.float32):
    values = np.empty(len(collection) * width, dtype=dtype)
    collection.foreach_get(key, values)
    return hashlib.sha256(values.tobytes()).hexdigest()


def mesh_signature(obj):
    mesh = obj.data
    return {
        'vertices': len(mesh.vertices), 'polygons': len(mesh.polygons),
        'loops': len(mesh.loops),
        'positions': array_hash(mesh.vertices, 'co', 3),
        'edges': array_hash(mesh.edges, 'vertices', 2, np.int32),
        'loop_vertices': array_hash(mesh.loops, 'vertex_index', 1, np.int32),
        'material_indices': array_hash(mesh.polygons, 'material_index', 1, np.int32),
        'uv': {uv.name: array_hash(uv.data, 'uv', 2) for uv in mesh.uv_layers},
        'weights': hashlib.sha256(json.dumps([
            [(g.group, g.weight) for g in v.groups] for v in mesh.vertices
        ], separators=(',', ':')).encode()).hexdigest(),
        'vertex_groups': [g.name for g in obj.vertex_groups],
        'matrix_world': [list(row) for row in obj.matrix_world],
        'materials': [m.name if m else None for m in mesh.materials],
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.open_mainfile(filepath=str(ROOT / 'assets/models/Raphtalia_prepared.blend'))
    report = {'source': 'assets/models/Raphtalia_prepared.blend',
              'blender': bpy.app.version_string,
              'images': [i.name for i in bpy.data.images], 'meshes': {}, 'materials': {}}
    for obj in bpy.data.objects:
        if obj.type != 'MESH':
            continue
        mesh = obj.data
        report['meshes'][obj.name] = mesh_signature(obj)
        uv = mesh.uv_layers.active
        co = np.array([v.co[:] for v in mesh.vertices])
        print(obj.name, len(mesh.vertices), len(mesh.polygons),
              'bbox', co.min(axis=0).round(3).tolist(), co.max(axis=0).round(3).tolist(),
              'UV', [u.name for u in mesh.uv_layers], flush=True)
        for slot, mat in enumerate(mesh.materials):
            if not mat:
                continue
            polys = [p for p in mesh.polygons if p.material_index == slot]
            if not polys:
                continue
            coords = np.array([uv.data[i].uv[:] for p in polys for i in p.loop_indices]) if uv else np.zeros((0, 2))
            r = {'mesh': obj.name, 'faces': len(polys), 'uv_min': coords.min(axis=0).tolist() if len(coords) else None,
                 'uv_max': coords.max(axis=0).tolist() if len(coords) else None,
                 'outside_0_1': int(np.any((coords < -1e-5) | (coords > 1.00001), axis=1).sum()),
                 'nodes': [{'type': n.type, 'name': n.name} for n in mat.node_tree.nodes] if mat.use_nodes else []}
            report['materials'].setdefault(mat.name, []).append(r)
            im = Image.new('RGB', (1024, 1024), '#181e29')
            d = ImageDraw.Draw(im)
            for poly in polys if uv else []:
                xy = [(float(uv.data[i].uv.x) * 1023, (1-float(uv.data[i].uv.y)) * 1023) for i in poly.loop_indices]
                d.line(xy + [xy[0]], fill='#9cbdce', width=1)
            im.save(OUT / (mat.name + '_uv.png'))
            print(' ', mat.name, len(polys), 'UV', r['uv_min'], r['uv_max'], flush=True)
    (OUT / 'before.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
