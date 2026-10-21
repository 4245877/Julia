"""Exercise real Blender controls, with IK reach and length assertions."""
import argparse
import json
import bpy
import numpy as np
from mathutils import Vector
from reaudit import ROOT,reset,coordinates,render,find


def main():
    p=argparse.ArgumentParser();p.add_argument('--blend',default='assets/models/Raphtalia_prepared.blend')
    p.add_argument('--output',default='docs/raphtalia/reaudit/controls.json');args=p.parse_args()
    bpy.ops.wm.open_mainfile(filepath=str(ROOT/args.blend));arm=next(o for o in bpy.data.objects if o.type=='ARMATURE')
    reset(arm);meshes=[o for o in bpy.data.objects if o.type=='MESH'];base={o.name:coordinates(o) for o in meshes}
    report={'ik':[],'finger_curls':[],'brows':[],'twist':[]}
    def update():
        arm.update_tag(refresh={'OBJECT'});bpy.context.scene.frame_set(bpy.context.scene.frame_current)
        bpy.context.view_layer.update()
    def snapshot():return {b.name:b.matrix.copy() for b in arm.pose.bones}
    out=ROOT/args.output;out.parent.mkdir(parents=True,exist_ok=True)
    for side in ('l','r'):
        for limb,end,lower,upper in [('arm','hand','forearm_stretch','arm_stretch'),('leg','foot','leg_stretch','thigh_stretch')]:
            key='IK_'+limb+'.'+side.upper();target=bpy.data.objects['CTRL_'+end+'.'+side.upper()]
            rest=target.location.copy();reset(arm);before=snapshot();arm[key]=1;update()
            rest_jump=max((arm.pose.bones[n].matrix.translation-m.translation).length for n,m in before.items())*.01
            for offset in ((0,-8,5),(5,0,8)):
                target.location=rest+Vector(offset);update()
                effector=arm.matrix_world@arm.pose.bones[find(arm,end,side)].matrix.translation
                desired=target.matrix_world.translation;residual=(effector-desired).length
                length_errors=[]
                for prefix in (upper,lower):
                    b=arm.pose.bones[find(arm,prefix,side)]
                    length_errors.append(abs((b.tail-b.head).length-b.bone.length)*.01)
                report['ik'].append({'control':key,'offset_source_units':offset,'end_error_m':residual,
                                     'length_error_m':max(length_errors),'rest_switch_head_jump_m':rest_jump})
                assert residual<.0002,(key,residual)
                assert max(length_errors)<.0001,(key,length_errors)
            target.location=rest;arm[key]=0;update()
    reset(arm)
    for obj in bpy.data.objects:
        if obj.name.startswith('CTRL_curl_'):
            obj['curl_degrees']=20;obj.update_tag();update()
            movement=max(float(np.linalg.norm(coordinates(o)-base[o.name],axis=1).max()) for o in meshes)
            report['finger_curls'].append({'control':obj.name,'degrees':20,'max_movement_m':movement})
            assert movement>.001,obj.name
            obj['curl_degrees']=0;obj.update_tag();update()
    render(out.parent/'face_neutral.png',(0,-.12,3.78),.72)
    for s in ('L','R'):
        b=arm.pose.bones['brow.'+s];b.location=b.bone.matrix_local.to_quaternion().inverted()@Vector((0,0,.8))
        update();movement=float(np.linalg.norm(coordinates(bpy.data.objects['eyelashes_tex_eyelashes_0'])-base['eyelashes_tex_eyelashes_0'],axis=1).max())
        report['brows'].append({'bone':b.name,'raise_source_units':.8,'max_movement_m':movement})
        assert movement>.007
    render(out.parent/'face_brows_raised.png',(0,-.12,3.78),.72)
    reset(arm)
    for s in ('l','r'):
        hand=arm.pose.bones[find(arm,'hand',s)];hand.rotation_mode='XYZ';hand.rotation_euler.y=np.radians(60);update()
        helper=bpy.data.objects['_auto_twist.'+s.upper()];angle=float(helper.evaluated_get(bpy.context.evaluated_depsgraph_get()).rotation_euler.y)
        report['twist'].append({'side':s,'hand_local_y_degrees':60,'helper_degrees':float(np.degrees(angle))})
        assert abs(np.degrees(angle)-30)<.01,angle
    reset(arm);update()
    report['neutral_reset_max_m']=max(float(np.linalg.norm(coordinates(o)-base[o.name],axis=1).max()) for o in meshes)
    report['invalid_drivers']=[o.name+':'+f.data_path for o in bpy.data.objects if o.animation_data for f in o.animation_data.drivers if not f.driver.is_valid]
    assert report['neutral_reset_max_m']<1e-5
    assert not report['invalid_drivers']
    report['result']='PASS';out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
