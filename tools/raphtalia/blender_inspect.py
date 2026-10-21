"""Inspect the unmodified FBX through Blender and render neutral evidence.

Run with Blender --background --python, or Python with the official bpy module.
"""
import bpy
import json
from pathlib import Path
from mathutils import Vector
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'docs/raphtalia'
OUT.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=str(ROOT / 'assets/models/Raphtalia.fbx'), use_anim=True,
                         automatic_bone_orientation=False, ignore_leaf_bones=True)
arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
meshes = [o for o in bpy.data.objects if o.type == 'MESH']
deps = bpy.context.evaluated_depsgraph_get()
report = {'blender': bpy.app.version_string, 'armature': arm.name,
          'armature_matrix': [list(r) for r in arm.matrix_world],
          'bones': [], 'meshes': [], 'materials': [], 'actions': [], 'images': []}
for b in arm.data.bones:
    report['bones'].append({'name': b.name, 'parent': b.parent.name if b.parent else None,
                           'head': list(b.head_local), 'tail': list(b.tail_local),
                           'matrix': [list(r) for r in b.matrix_local], 'deform': b.use_deform})
for o in meshes:
    ev = o.evaluated_get(deps)
    delta = [(a.co-b.co).length for a,b in zip(ev.data.vertices, o.data.vertices)]
    counts = [len([g for g in v.groups if g.weight > 1e-8]) for v in o.data.vertices]
    sums = [sum(g.weight for g in v.groups) for v in o.data.vertices]
    report['meshes'].append({'name': o.name, 'vertices': len(o.data.vertices),
      'polygons': len(o.data.polygons), 'triangles': sum(len(p.vertices)-2 for p in o.data.polygons),
      'bounds_world': [list(o.matrix_world @ Vector(v)) for v in o.bound_box],
      'groups': [{'name': g.name, 'weighted_vertices': sum(1 for v in o.data.vertices
                   if any(i.group == g.index and i.weight > 1e-8 for i in v.groups))} for g in o.vertex_groups],
      'max_influences': max(counts, default=0), 'unweighted': counts.count(0),
      'sum_range': [min(sums, default=0), max(sums, default=0)],
      'bind_eval_max_delta_local': max(delta, default=0),
      'modifiers': [{'name': m.name, 'type': m.type, 'object': m.object.name if m.type=='ARMATURE' and m.object else None} for m in o.modifiers],
      'shape_keys': [k.name for k in o.data.shape_keys.key_blocks] if o.data.shape_keys else [],
      'materials': [m.name if m else None for m in o.data.materials]})
for m in bpy.data.materials:
    report['materials'].append({'name': m.name, 'diffuse': list(m.diffuse_color),
        'nodes': [{'name': n.name, 'type': n.type} for n in m.node_tree.nodes] if m.use_nodes else []})
for a in bpy.data.actions:
    report['actions'].append({'name': a.name, 'range': list(a.frame_range), 'curves': len(a.fcurves)})
for i in bpy.data.images:
    report['images'].append({'name': i.name, 'size': list(i.size), 'path': i.filepath, 'packed': bool(i.packed_file)})
(OUT / 'blender_original.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
# Evidence uses imported material values; no materials are edited.
points = [o.matrix_world @ v.co for o in meshes for v in o.data.vertices]
lo = Vector(tuple(min(v[i] for v in points) for i in range(3)))
hi = Vector(tuple(max(v[i] for v in points) for i in range(3)))
center = (lo+hi)*.5
height = hi.z-lo.z
scene=bpy.context.scene
scene.render.engine='BLENDER_WORKBENCH'
scene.display.shading.light='STUDIO'
scene.display.shading.color_type='SINGLE'
scene.display.shading.single_color=(.65,.65,.65)
scene.display.shading.show_shadows=True
scene.display.shading.show_cavity=True
scene.display.shading.cavity_type='BOTH'
scene.render.resolution_x=1000; scene.render.resolution_y=1000; scene.render.resolution_percentage=100
scene.world=bpy.data.worlds.new('Audit world')
scene.world.color=(.2,.2,.2)
scene.view_settings.view_transform='Standard'
def camera(name, pos, target, scale):
    data=bpy.data.cameras.new(name); o=bpy.data.objects.new(name,data); scene.collection.objects.link(o)
    o.location=pos; o.rotation_euler=(Vector(target)-o.location).to_track_quat('-Z','Y').to_euler()
    data.type='ORTHO'; data.ortho_scale=scale; scene.camera=o
    return o
for pos,power,size in [((height*.6,-height, height*.9),3000,height),((-height*.6,-height*.4,height*.7),1800,height)]:
    data=bpy.data.lights.new('Audit softbox','AREA'); o=bpy.data.objects.new(data.name,data); scene.collection.objects.link(o)
    o.location=pos; data.energy=power * .03 * (height/4)**2; data.shape='DISK'; data.size=size
    o.rotation_euler=(center-o.location).to_track_quat('-Z','Y').to_euler()
camera('Audit camera', center+Vector((height*.12,-height*1.5,height*.06)),center,height*1.08)
scene.render.filepath=str(OUT/'original_front.png'); bpy.ops.render.render(write_still=True)
camera('Audit face', (center.x,-height,lo.z+height*.87),(center.x,center.y,lo.z+height*.87),height*.32)
scene.render.filepath=str(OUT/'original_face.png'); bpy.ops.render.render(write_still=True)
print(json.dumps({'bones':len(arm.data.bones),'vertices':sum(len(o.data.vertices) for o in meshes),
                  'meshes':len(meshes), 'actions':report['actions'], 'bounds':[list(lo),list(hi)]}))
