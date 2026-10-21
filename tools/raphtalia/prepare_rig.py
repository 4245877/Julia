"""Reproducible conservative geometry-based refit; never writes the input FBX.

This is an approximate recovery of a mesh baked in a different pose from its
bind skeleton. The JSON evidence and report explicitly retain that limitation.
"""
import bpy
import json
import math
import csv
from pathlib import Path
from mathutils import Vector, Matrix, Quaternion
import numpy as np
from glove_skinning import transfer_glove_hand_weights,rigidify_wrist_cuffs

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'docs/raphtalia'
MODEL=ROOT/'assets/models/Raphtalia_prepared.fbx'
BLEND=ROOT/'assets/models/Raphtalia_prepared.blend'
OUT.mkdir(parents=True,exist_ok=True)

def dump(name,data):
    (OUT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

def import_source():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(ROOT/'assets/models/Raphtalia.fbx'),
       use_anim=False,ignore_leaf_bones=True,automatic_bone_orientation=False)
    arm=next(o for o in bpy.data.objects if o.type=='ARMATURE')
    meshes=[o for o in bpy.data.objects if o.type=='MESH']
    # Retain the original geometry and its relative placement. Only remove the
    # outer placement transform, using upright armature space, in metres.
    inv=arm.matrix_world.inverted()
    deps=bpy.context.evaluated_depsgraph_get()
    canonical={}
    for o in meshes:
        xform=inv@o.matrix_world
        ev=o.evaluated_get(deps)
        canonical[o.name]=[xform@v.co for v in ev.data.vertices]
    for o in [arm]+meshes:
        o.parent=None
        o.matrix_world=Matrix.Scale(.01,4)
    for o in meshes:
        for v,p in zip(o.data.vertices,canonical[o.name]): v.co=p
        for m in list(o.modifiers): o.modifiers.remove(m)
        mod=o.modifiers.new('Deform LBS / 4 weights','ARMATURE'); mod.object=arm
        mod.use_deform_preserve_volume=False
    for p in arm.pose.bones: p.matrix_basis=Matrix.Identity(4)
    # Keep source structural helpers in a hidden archive collection in .blend.
    archive=bpy.data.collections.new('Source transform helpers (archived)')
    bpy.context.scene.collection.children.link(archive)
    for o in list(bpy.data.objects):
        if o not in meshes and o!=arm:
            for c in list(o.users_collection): c.objects.unlink(o)
            archive.objects.link(o); o.hide_render=True; o.hide_viewport=True
    archive.hide_render=True; archive.hide_viewport=True
    return arm,meshes,canonical

def setup_render(meshes):
    scene=bpy.context.scene
    scene.render.engine='BLENDER_WORKBENCH'
    scene.display.shading.light='STUDIO'; scene.display.shading.color_type='SINGLE'
    scene.display.shading.single_color=(.65,.65,.65)
    scene.display.shading.show_cavity=True; scene.display.shading.cavity_type='BOTH'
    scene.display.shading.show_shadows=True
    scene.display.shading.background_type='WORLD'
    scene.world=bpy.data.worlds.new('Audit world'); scene.world.color=(.12,.12,.12)
    scene.render.resolution_x=1000; scene.render.resolution_y=1100
    scene.render.resolution_percentage=100
    scene.view_settings.view_transform='Standard'
    camdata=bpy.data.cameras.new('Review camera'); cam=bpy.data.objects.new('Review camera',camdata)
    scene.collection.objects.link(cam); scene.camera=cam; camdata.type='ORTHO'
    points=[o.matrix_world@v.co for o in meshes for v in o.data.vertices]
    lo=Vector(tuple(min(v[i] for v in points) for i in range(3)))
    hi=Vector(tuple(max(v[i] for v in points) for i in range(3)))
    return cam,lo,hi

def render(cam,lo,hi,name,detail=None):
    height=hi.z-lo.z; target=(lo+hi)*.5
    if detail=='face': target=Vector((target.x,-.1,lo.z+height*.88)); scale=height*.32
    elif detail=='hands': target=Vector((.43,-.19,2.04)); scale=1.0
    else: scale=height*1.15
    cam.location=target+Vector((height*.07,-height*1.7,height*.025))
    cam.rotation_euler=(target-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.ortho_scale=scale
    bpy.context.scene.render.filepath=str(OUT/(name+'.png'))
    bpy.ops.render.render(write_still=True)

def reset_pose(arm):
    arm.animation_data_clear()
    for p in arm.pose.bones: p.matrix_basis=Matrix.Identity(4)
    bpy.context.view_layer.update()

def turn(arm,name,degrees,axis=(1,0,0)):
    p=arm.pose.bones.get(name)
    if not p: return
    q=p.bone.matrix_local.to_quaternion()
    p.rotation_mode='QUATERNION'
    p.rotation_quaternion=q.inverted()@Quaternion(Vector(axis),math.radians(degrees))@q

def evaluate(meshes):
    bpy.context.view_layer.update(); deps=bpy.context.evaluated_depsgraph_get()
    return {o.name:np.array([v.co[:] for v in o.evaluated_get(deps).data.vertices],dtype=np.float64) for o in meshes}

def metrics(meshes,base,current):
    allmove=[]; ratios=[]; per=[]
    for o in meshes:
        p=base[o.name]; q=current[o.name]
        moved=np.linalg.norm(q-p,axis=1)*.01
        edges=np.array([e.vertices[:] for e in o.data.edges],dtype=np.int32)
        a=np.linalg.norm(p[edges[:,1]]-p[edges[:,0]],axis=1)
        b=np.linalg.norm(q[edges[:,1]]-q[edges[:,0]],axis=1)
        valid=a>1e-4; ratio=b[valid]/a[valid]
        allmove.extend(moved.tolist()); ratios.extend(ratio.tolist())
        per.append({'mesh':o.name,'max_movement_m':float(moved.max()),
                    'edge_stretch_p99':float(np.percentile(ratio,99)),
                    'edge_stretch_max':float(ratio.max()),
                    'edges_stretched_over_2x':int((ratio>2).sum())})
    r=np.array(ratios)
    return {'max_movement_m':max(allmove),'nonfinite_vertices':sum(int((~np.isfinite(v)).any(axis=1).sum()) for v in current.values()),
       'edge_ratio_p01':float(np.percentile(r,1)),'edge_ratio_p99':float(np.percentile(r,99)),
       'edge_ratio_max':float(r.max()),'edges_stretched_over_2x':int((r>2).sum()),'meshes':per}

def test_definitions(arm):
    return {
      'elbows_90':[('forearm_stretch.l_0159',90,(1,0,0)),('forearm_stretch.r_0138',90,(1,0,0))],
      'knees_90':[('leg_stretch.l_03',90,(1,0,0)),('leg_stretch.r_09',90,(1,0,0))],
      'head_torso':[('spine_02.x_015',15,(0,0,1)),('head.x_018',30,(0,0,1))],
      'shoulders_hips_ankles':[('arm_stretch.l_0158',25,(0,1,0)),('arm_stretch.r_0137',-25,(0,1,0)),
               ('thigh_stretch.l_02',25,(1,0,0)),('thigh_stretch.r_08',-20,(1,0,0)),
               ('foot.l_04',15,(1,0,0)),('foot.r_010',-15,(1,0,0)),
               ('arm_twist.l_0177',12,tuple(arm.data.bones['arm_twist.l_0177'].y_axis)),
               ('forearm_twist.r_0135',12,tuple(arm.data.bones['forearm_twist.r_0135'].y_axis))],
      'fingers_35':[(p.name,35,(1,0,0)) for p in arm.pose.bones if p.name.startswith(('c_index','c_middle','c_ring','c_pinky','c_thumb'))],
      'hair_tail_cloth':[('hair.d.000_093',10,(0,1,0)),('hair.h.000_0108',8,(0,1,0)),
                         ('c_tail_00.x_0178',18,(0,0,1)),('cloth.c.000_0182',10,(1,0,0))],
    }

def run_tests(arm,meshes,cam,lo,hi,prefix,definitions):
    reset_pose(arm); base=evaluate(meshes); results={}
    for name,changes in definitions.items():
        reset_pose(arm)
        for bone,angle,axis in changes: turn(arm,bone,angle,axis)
        results[name]={'rotations':changes,**metrics(meshes,base,evaluate(meshes))}
        render(cam,lo,hi,prefix+'_'+name)
        if name=='fingers_35': render(cam,lo,hi,prefix+'_fingers_detail','hands')
        if name=='eyes_ears': render(cam,lo,hi,prefix+'_eyes_ears_detail','face')
    reset_pose(arm)
    return results

def refit(arm,config,raw):
    bpy.context.view_layer.objects.active=arm; arm.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    edits=arm.data.edit_bones
    original_lengths={b.name:(b.tail-b.head).length for b in edits}
    changed=[]
    # Filled from measured transition regions; retain the uncertainty in JSON.
    suggestions=config['suggested_heads_armature_local']
    for name,data in suggestions.items():
        if name not in edits: continue
        # These source pivots are already near the anatomy. Broad surface
        # centroids (especially crotch/torso) are not reliable replacements.
        if name.startswith(('spine_','thigh_stretch.','shoulder.','arm_stretch.','head.','neck.')):
            continue
        if 'twist' in name: continue
        point=Vector(data['head'] if isinstance(data,dict) else data)
        b=edits[name]; previous=b.head.copy(); delta=point-previous
        if delta.length>.001:
            b.head=point; b.tail+=delta
            changed.append({'bone':name,'old_head':list(previous),'new_head':list(point),
                            'reason':data.get('method','geometry refit') if isinstance(data,dict) else 'geometry refit'})
    # Correct only corrupt zero bind pivots from intact FBX model transforms.
    for bone in raw['skeleton']:
        n=bone['name']
        if n.startswith('hair.') and n in edits and edits[n].head.length<.01:
            target=Vector(bone['armature_space_model_head'])
            old=edits[n].head.copy(); edits[n].head=target; edits[n].tail+=target-old
            changed.append({'bone':n,'old_head':list(old),'new_head':list(target),'reason':'zero bind pivot restored from source Model transform'})
    for side,upper,lower,hand,thigh,shin,foot in [
       ('l','arm_stretch.l_0158','forearm_stretch.l_0159','hand.l_0160','thigh_stretch.l_02','leg_stretch.l_03','foot.l_04'),
       ('r','arm_stretch.r_0137','forearm_stretch.r_0138','hand.r_0139','thigh_stretch.r_08','leg_stretch.r_09','foot.r_010')]:
        for prefix,start,end,fraction in [('arm_twist',upper,lower,0),('forearm_twist',lower,hand,.5),
                                          ('thigh_twist',thigh,shin,0),('leg_twist',shin,foot,.5)]:
            b=next(x for x in edits if x.name.startswith(prefix+'.'+side+'_'))
            previous=b.head.copy(); b.head=edits[start].head.lerp(edits[end].head,fraction)
            b.tail=edits[end].head
            changed.append({'bone':b.name,'old_head':list(previous),'new_head':list(b.head),
                            'reason':'align twist helper with refitted parent segment'})
    # Each cloth ribbon is an independent 3-bone chain; source connected some
    # ribbon roots to a neighbouring ribbon tip, creating a backwards jump.
    reparent=[]
    for b in edits:
        if b.name.startswith('cloth.') and b.name.split('.')[2].split('_')[0] in ('003','006'):
            old=b.parent.name if b.parent else None
            b.parent=edits['root.x_01']; b.use_connect=False
            reparent.append({'bone':b.name,'old_parent':old,'parent':'root.x_01'})
    tail_repairs=[]
    # Anatomical segment endpoints: real child joints, not old T-pose tails.
    for b in edits:
        children=[c for c in edits if c.parent==b and 'twist' not in c.name]
        if children:
            if b.name.startswith('hand.'):
                children=[c for c in children if c.name.startswith('c_middle1')]
            if len(children)==1 and (children[0].head-b.head).length>.05:
                b.tail=children[0].head
        elif b.name.startswith(('c_index','c_middle','c_ring','c_pinky','c_thumb','toes_')):
            cloud=config['weighted_groups'].get(b.name)
            if cloud:
                direction=Vector(cloud['center'])-b.head
                if direction.length>.05: b.tail=b.head+direction.normalized()*max(1.5,original_lengths[b.name])
        # Four source hair end markers are hundreds of units away from their
        # strand. They have no weights/children: continue the preceding segment
        # by one segment length rather than reproducing those corrupt markers.
        if b.name in ('hair.d.002_096','hair.e.002_099','hair.f.003_0103','hair.g.003_0107'):
            old=b.tail.copy(); segment=b.head-b.parent.head
            b.tail=b.head+segment
            tail_repairs.append({'bone':b.name,'old_tail':list(old),'new_tail':list(b.tail),
                                  'reason':'replace remote corrupt source end marker with continuation of preceding hair segment'})
        if b.name=='head.x_018':
            old=b.tail.copy(); b.tail=b.head+Vector((0,0,35))
            tail_repairs.append({'bone':b.name,'old_tail':list(old),'new_tail':list(b.tail),
                                  'reason':'head display/rest direction endpoint kept within scalp instead of beyond character bounds'})
        if (b.tail-b.head).length<.05: b.tail=b.head+Vector((0,0,1))
        # Project global forward as roll reference for predictable FK axes.
        b.align_roll(Vector((0,-1,0)))
    bpy.ops.object.mode_set(mode='OBJECT')
    reset_pose(arm)
    return changed,reparent,tail_repairs

def add_face(arm,meshes,face):
    bpy.context.view_layer.objects.active=arm
    bpy.ops.object.mode_set(mode='EDIT'); edits=arm.data.edit_bones
    for info in face['controls']:
        b=edits.new(info['bone']); b.head=info['head']; b.tail=Vector(info['head'])+Vector((0,4,0))
        b.parent=edits['head.x_018']; b.use_deform=True
    bpy.ops.object.mode_set(mode='OBJECT')
    assigned=[]
    inv=arm.matrix_world.inverted()
    for info in face['controls']:
        name=info['bone']; side=info['side']
        for o in meshes:
            if o.name not in info['meshes']: continue
            xform=inv@o.matrix_world
            group=o.vertex_groups.get(name) or o.vertex_groups.new(name=name)
            count=0
            for v in o.data.vertices:
                p=xform@v.co
                if p.x*side<=0: continue
                if info['kind']=='ear':
                    t=max(0,min(1,(p.z-info['base_z'])/info['fade_height']))
                    w=t*t*(3-2*t)
                else: w=1.0
                if w<=1e-7: continue
                old=[(g.group,g.weight) for g in v.groups]
                for idx,value in old: o.vertex_groups[idx].add([v.index],value*(1-w),'REPLACE')
                group.add([v.index],w,'REPLACE'); count+=1
            assigned.append({'bone':name,'mesh':o.name,'vertices':count})
    return assigned

def create_actions(arm,definitions):
    clips=[]
    for name,changes in definitions.items():
        reset_pose(arm); action=bpy.data.actions.new('TEST_'+name); arm.animation_data_create()
        arm.animation_data.action=action; action.use_fake_user=True
        for frame,strength in [(1,0),(16,.5),(31,1),(46,.5),(61,0)]:
            for p in arm.pose.bones:
                p.matrix_basis=Matrix.Identity(4); p.rotation_mode='QUATERNION'
            for bone,angle,axis in changes: turn(arm,bone,angle*strength,axis)
            for p in arm.pose.bones:
                p.keyframe_insert('rotation_quaternion',frame=frame,group=p.name)
                p.keyframe_insert('location',frame=frame,group=p.name)
                p.keyframe_insert('scale',frame=frame,group=p.name)
        for c in action.fcurves:
            for k in c.keyframe_points: k.interpolation='LINEAR'
        clips.append({'name':action.name,'frames':[1,61],'fps':30,'duration_s':2})
    reset_pose(arm)
    return clips

def main():
    arm,meshes,canonical=import_source()
    bpy.context.preferences.filepaths.save_version=0
    cam,lo,hi=setup_render(meshes)
    original_tests=run_tests(arm,meshes,cam,lo,hi,'before',test_definitions(arm))
    config=json.loads((OUT/'joint_geometry.json').read_text(encoding='utf-8'))
    raw=json.loads((OUT/'raw_original.json').read_text(encoding='utf-8'))
    changes,reparent,tail_repairs=refit(arm,config,raw)
    face=json.loads((OUT/'face_geometry.json').read_text(encoding='utf-8'))
    face_assignments=add_face(arm,meshes,face)
    glove_transfer=transfer_glove_hand_weights(meshes)
    dump('glove_transfer_applied.json',glove_transfer)
    cuff_cleanup=rigidify_wrist_cuffs(meshes)
    dump('cuff_transfer_applied.json',cuff_cleanup)
    arm.name='Raphtalia_Rig'; arm.data.name='Raphtalia_DeformSkeleton'
    arm.show_in_front=True
    arm['status']='Approximate geometry-based refit. Manual deformation and collision cleanup still required.'
    arm['source']='Raphtalia.fbx SHA256 A602086AC9E580F2D265A489478922B75AAE00410313B17A89BD846AD4F3B34C'
    categories={}
    for b in arm.data.bones:
        category='Face' if b.name.startswith(('eye.','ear.')) else 'Hair' if b.name.startswith('hair.') else 'Cloth' if b.name.startswith('cloth.') else 'Tail' if 'tail' in b.name else 'Fingers' if b.name.startswith(('c_index','c_middle','c_ring','c_pinky','c_thumb')) else 'Body'
        if category not in categories: categories[category]=arm.data.collections.new(category)
        categories[category].assign(b)
    definitions=test_definitions(arm)
    definitions['eyes_ears']=[('eye.L',8,(0,0,1)),('eye.R',8,(0,0,1)),('ear.L',12,(0,1,0)),('ear.R',-12,(0,1,0))]
    prepared_tests=run_tests(arm,meshes,cam,lo,hi,'prepared',definitions)
    render(cam,lo,hi,'prepared_neutral')
    render(cam,lo,hi,'prepared_face','face')
    bpy.context.view_layer.update()
    neutral=evaluate(meshes)
    neutral_delta=max(float(np.linalg.norm(neutral[o.name]-np.array([p[:] for p in canonical[o.name]]),axis=1).max())*.01 for o in meshes)
    clips=create_actions(arm,definitions)
    scene=bpy.context.scene; scene.render.fps=30; scene.frame_start=1; scene.frame_end=61; scene.frame_set(1)
    reset_pose(arm)
    # FBX exporter discovers animated armatures through AnimationData. Keep one
    # action assigned while all seven compatible actions are baked separately.
    arm.animation_data_create()
    arm.animation_data.action=bpy.data.actions[clips[0]['name']]
    scene.frame_set(1)
    for o in bpy.context.selected_objects: o.select_set(False)
    for o in meshes+[arm]: o.select_set(True)
    bpy.context.view_layer.objects.active=arm
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND),check_existing=False)
    bpy.ops.export_scene.fbx(filepath=str(MODEL),use_selection=True,object_types={'ARMATURE','MESH'},
        use_mesh_modifiers=False,add_leaf_bones=False,use_armature_deform_only=True,
        bake_anim=True,bake_anim_use_all_actions=True,bake_anim_use_nla_strips=False,
        bake_anim_use_all_bones=True,bake_anim_force_startend_keying=True,bake_anim_step=1,
        bake_anim_simplify_factor=0,path_mode='RELATIVE',embed_textures=False,
        axis_forward='-Z',axis_up='Y',apply_unit_scale=True,apply_scale_options='FBX_SCALE_ALL')
    hierarchy=[{'name':b.name,'parent':b.parent.name if b.parent else None,'head':list(b.head_local),'tail':list(b.tail_local)} for b in arm.data.bones]
    with (OUT/'skeleton_prepared.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f); w.writerow(['bone','parent','head_x','head_y','head_z','tail_x','tail_y','tail_z'])
        for b in hierarchy: w.writerow([b['name'],b['parent'],*b['head'],*b['tail']])
    dump('preparation.json',{'blender':bpy.app.version_string,'status':'approximate refit requiring manual cleanup',
      'bones':len(arm.data.bones),'mesh_count':len(meshes),'source_control_points':sum(len(o.data.vertices) for o in meshes),
      'materials':len(bpy.data.materials),'images':len(bpy.data.images),'changes':changes,'reparent':reparent,
      'tail_repairs':tail_repairs,
      'face_assignments':face_assignments,'glove_vertices_reweighted':glove_transfer['vertices_changed'],
      'cuff_cleanup':{k:v for k,v in cuff_cleanup.items() if k!='patch'},
      'neutral_max_delta_m':neutral_delta,'clips':clips,
      'before_tests':original_tests,'prepared_tests':prepared_tests,'skeleton':hierarchy,
      'coordinate_convention':'Upright original armature coordinates; 0.01 m/source unit. Export Y up -Z forward.'})
    print('PREPARED',len(arm.data.bones),'bones',MODEL)

if __name__=='__main__': main()
