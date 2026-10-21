"""Consistent read-only close-ups for eyelid development and review."""
import argparse
from pathlib import Path
import bpy
from mathutils import Vector
from reaudit import reset

ROOT = Path(__file__).resolve().parents[2]

def setup(mode='texture', side=False):
    s=bpy.context.scene
    s.render.engine='BLENDER_WORKBENCH'
    s.display.shading.light='STUDIO'
    s.display.shading.color_type='TEXTURE' if mode=='texture' else 'SINGLE'
    s.display.shading.single_color=(.65,.65,.65)
    s.display.shading.show_cavity=False
    s.display.shading.show_shadows=False
    s.display.shading.show_specular_highlight=False
    s.display.shading.background_type='VIEWPORT'
    s.display.shading.background_color=(.14,.16,.19)
    s.view_settings.view_transform='Standard'
    s.render.resolution_x=1200;s.render.resolution_y=720
    s.render.resolution_percentage=100
    target=Vector((-.006,-.17,3.792))
    s.camera.location=target+Vector((.55 if side else 0,-8,.08))
    s.camera.rotation_euler=(target-s.camera.location).to_track_quat('-Z','Y').to_euler()
    s.camera.data.ortho_scale=.51
    s.render.image_settings.file_format='PNG'
    return s

def render(path,mode='texture',side=False):
    s=setup(mode,side);s.render.filepath=str(path)
    bpy.ops.render.render(write_still=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--blend',default='assets/models/Raphtalia_prepared.blend')
    p.add_argument('--out',default='docs/raphtalia/blink/original_open.png')
    p.add_argument('--level',type=float,default=0);p.add_argument('--mode',default='texture');p.add_argument('--side',action='store_true')
    args=p.parse_args();bpy.ops.wm.open_mainfile(filepath=str(ROOT/args.blend))
    arm=next(o for o in bpy.data.objects if o.type=='ARMATURE');reset(arm)
    for side in ('L','R'):
        if 'Blink_'+side in arm:arm['Blink_'+side]=args.level
    arm.update_tag();bpy.context.view_layer.update()
    render(ROOT/args.out,args.mode,args.side)
