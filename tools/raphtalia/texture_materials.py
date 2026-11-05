"""Connect the reference maps without changing geometry, UVs, rig or clips."""
import argparse
import json
from pathlib import Path

import bpy
from mathutils import Vector

from texture_audit import mesh_signature
from reaudit import reset

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / 'build/raphtalia_tools/texturing'
DOC = ROOT / 'docs/raphtalia/texturing'


def connect():
    config = json.loads((DOC / 'reference_options.json').read_text())
    metadata = json.loads((DOC / 'reference_texture_manifest.json').read_text())
    textures = {t['uid']: ROOT / 'assets/textures/raphtalia' / t['name'] for t in metadata['results']}
    materials = {m['name']: m for m in config['materials'].values() if isinstance(m, dict)}
    bindings = []
    for mat in bpy.data.materials:
        source = materials[mat.name]
        channel = source['channels']['AlbedoPBR']
        mat.use_nodes = True
        mat.use_backface_culling = False
        mat.diffuse_color = (1, 1, 1, 1)
        nodes = mat.node_tree.nodes
        nodes.clear()
        output = nodes.new('ShaderNodeOutputMaterial')
        output.location = (620, 80)
        emission = nodes.new('ShaderNodeEmission')
        emission.label = 'Reference: unlit hand-painted colour'
        emission.location = (350, 180)
        principled = nodes.new('ShaderNodeBsdfPrincipled')
        principled.label = 'Optional lit material / FBX colour export'
        principled.location = (350, -80)
        principled.inputs['Roughness'].default_value = .8
        principled.inputs['Specular IOR Level'].default_value = .2
        mat['reference'] = 'https://skfb.ly/ouUGR'
        mat['reference_author'] = 'Newtaw'
        mat['display_style'] = 'Unlit; colour includes painted shadows and highlights'
        if channel.get('texture'):
            path = textures[channel['texture']['uid']]
            image = bpy.data.images.get(path.name) or bpy.data.images.load(str(path))
            image.colorspace_settings.name = 'sRGB'
            image.alpha_mode = 'NONE'
            image.pack()
            node = nodes.new('ShaderNodeTexImage')
            node.image = image
            node.name = 'Reference Base Color'
            node.location = (-180, 160)
            node.extension = 'REPEAT'
            node.interpolation = 'Linear'
            uv = nodes.new('ShaderNodeUVMap')
            uv.uv_map = 'UVMap'
            uv.location = (-400, 160)
            mat.node_tree.links.new(uv.outputs['UV'], node.inputs['Vector'])
            mat.node_tree.links.new(node.outputs['Color'], emission.inputs['Color'])
            mat.node_tree.links.new(node.outputs['Color'], principled.inputs['Base Color'])
            nodes.active = node
            binding = {'material': mat.name, 'image': path.name, 'uv_map': 'UVMap',
                       'uv_scale': [1, 1], 'uv_offset': [0, 0], 'alpha': 'OPAQUE'}
        else:
            color = (*channel.get('color', [1, 1, 1]), 1)
            emission.inputs['Color'].default_value = color
            principled.inputs['Base Color'].default_value = color
            mat.diffuse_color = color
            binding = {'material': mat.name, 'image': None, 'constant_linear_rgba': color}
        mat.node_tree.links.new(emission.outputs[0], output.inputs['Surface'])
        bindings.append(binding)
    return bindings


def camera(position, target, scale, width, height):
    scene = bpy.context.scene
    if not scene.camera:
        data = bpy.data.cameras.new('Texture Review Camera')
        obj = bpy.data.objects.new('Texture Review Camera', data)
        scene.collection.objects.link(obj)
        scene.camera = obj
    cam = scene.camera
    cam.data.type = 'ORTHO'
    cam.data.ortho_scale = scale
    cam.data.lens = 70
    cam.location = position
    cam.rotation_euler = (Vector(target) - cam.location).to_track_quat('-Z', 'Y').to_euler()
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100


def render(path, position, target, scale, width=1000, height=1200):
    scene = bpy.context.scene
    camera(position, target, scale, width, height)
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 16
    scene.cycles.use_denoising = False
    scene.render.film_transparent = True
    scene.world.color = (0, 0, 0)
    scene.view_settings.view_transform = 'Standard'
    scene.view_settings.look = 'None'
    scene.view_settings.exposure = 0
    scene.view_settings.gamma = 1
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--final', action='store_true')
    parser.add_argument('--input', default='assets/models/Raphtalia_prepared.blend')
    args = parser.parse_args()
    WORK.mkdir(parents=True, exist_ok=True)
    DOC.mkdir(parents=True, exist_ok=True)
    source = ROOT / args.input
    bpy.ops.wm.open_mainfile(filepath=str(source))
    before = {o.name: mesh_signature(o) for o in bpy.data.objects if o.type == 'MESH'}
    bindings = connect()
    after = {o.name: mesh_signature(o) for o in bpy.data.objects if o.type == 'MESH'}
    assert before == after, 'Character mesh/UV/weights must remain bit exact.'
    # Material-only FBX export supplies standard Blender material/texture records.
    for mat in bpy.data.materials:
        output = next(n for n in mat.node_tree.nodes if n.type == 'OUTPUT_MATERIAL')
        principled = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
        mat.node_tree.links.new(principled.outputs['BSDF'], output.inputs['Surface'])
    for obj in bpy.context.selected_objects:
        obj.select_set(False)
    for obj in bpy.data.objects:
        if obj.type == 'MESH':
            obj.select_set(True)
    bpy.ops.export_scene.fbx(filepath=str(WORK / 'material_template.fbx'), use_selection=True,
        object_types={'MESH'}, use_mesh_modifiers=False, bake_anim=False,
        path_mode='RELATIVE', embed_textures=False)
    for mat in bpy.data.materials:
        output = next(n for n in mat.node_tree.nodes if n.type == 'OUTPUT_MATERIAL')
        emission = next(n for n in mat.node_tree.nodes if n.type == 'EMISSION')
        mat.node_tree.links.new(emission.outputs[0], output.inputs['Surface'])
    out = ROOT / 'assets/models/Raphtalia_prepared.blend' if args.final else WORK / 'Raphtalia_textured.blend'
    for image in bpy.data.images:
        if image.source == 'FILE':
            image.filepath = '//../textures/raphtalia/' + image.name if args.final else str(ROOT / 'assets/textures/raphtalia' / image.name)
    bpy.context.preferences.filepaths.save_version = 0
    if args.final:
        camera((0, -10, 2.15), (0, 0, 2.15), 4.55, 1000, 1200)
        scene = bpy.context.scene
        scene.render.engine = 'CYCLES'
        scene.cycles.samples = 16
        scene.view_settings.view_transform = 'Standard'
        scene.view_settings.look = 'None'
        scene.render.film_transparent = True
        for screen in bpy.data.screens:
            for area in screen.areas:
                if area.type == 'VIEW_3D':
                    space = area.spaces.active
                    space.shading.type = 'MATERIAL'
                    space.region_3d.view_location = (0, 0, 2.15)
                    space.region_3d.view_distance = 5.5
                    space.region_3d.view_rotation = scene.camera.rotation_euler.to_quaternion()
    bpy.ops.wm.save_as_mainfile(filepath=str(out), check_existing=False)
    (DOC / 'material_bindings.json').write_text(json.dumps(bindings, indent=2), encoding='utf-8')
    print('Saved', out, '; preserved', len(before), 'meshes;', len(bindings), 'materials', flush=True)
    if args.render:
        arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
        reset(arm)
        for key in ('Blink_L', 'Blink_R'):
            arm[key] = 0.
        arm.update_tag()
        bpy.context.scene.frame_set(1)
        bpy.context.view_layer.update()
        render(DOC / 'front.png', (0, -10, 2.15), (0, 0, 2.15), 4.55)
        render(DOC / 'back.png', (0, 10, 2.15), (0, 0, 2.15), 4.55)
        render(DOC / 'three_quarter.png', (5, -10, 3.2), (0, 0, 2.15), 4.55)
        render(DOC / 'face.png', (0, -10, 3.83), (0, -.1, 3.83), .83, 1200, 1050)
        render(DOC / 'outfit.png', (0, -10, 2.65), (0, -.1, 2.65), 1.65, 1100, 1200)


if __name__ == '__main__':
    main()
