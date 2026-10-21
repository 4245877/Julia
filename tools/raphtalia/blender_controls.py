"""Blender-only object controls, no extra control/deform bones in FBX.

The two added brow bones deform real disconnected eyebrow ink surfaces.
IK is opt-in. Finger curls are additive to existing FK. Export evaluates the
constraints; controls are not selected by the shared FBX export function.
"""
import math
import bpy
from mathutils import Matrix, Vector
from scipy.optimize import minimize_scalar
from reaudit import find, reset, definitions, turn


def prop(obj,name,value,minimum=0,maximum=1,description=''):
    obj[name]=value
    obj.id_properties_ui(name).update(min=minimum,max=maximum,description=description)


def variable(driver,name,obj,path):
    v=driver.variables.new();v.name=name;v.type='SINGLE_PROP'
    v.targets[0].id=obj;v.targets[0].data_path=path


def influence(constraint,arm,key):
    f=constraint.driver_add('influence');f.driver.expression='enabled'
    variable(f.driver,'enabled',arm,'["'+key+'"]')


def build(arm,meshes):
    reset(arm);collection=bpy.data.collections.new('Animation controls (Blender only)')
    bpy.context.scene.collection.children.link(collection)
    internal=bpy.data.collections.new('Constraint helpers (no export)');collection.children.link(internal)
    def empty(name,location=(0,0,0),kind='SPHERE',size=6,helper=False):
        obj=bpy.data.objects.new(name,None);(internal if helper else collection).objects.link(obj)
        obj.parent=arm;obj.location=location;obj.empty_display_type=kind;obj.empty_display_size=size
        obj.show_in_front=True;obj.hide_render=True
        if helper:obj.hide_set(True)
        return obj
    result={'ik':[],'finger_controls':[],'twist':[],'brows':[]}
    # Actual eyebrow components are separate islands above z=386; lashes are
    # below z=384. This selection is checked explicitly before adding bones.
    lashes=next(o for o in meshes if o.name=='eyelashes_tex_eyelashes_0')
    ids={s:[v.index for v in lashes.data.vertices if v.co.z>386 and v.co.x*sign>0] for s,sign in [('L',1),('R',-1)]}
    assert all(len(v)==17 for v in ids.values()),ids
    bpy.context.view_layer.objects.active=arm;arm.select_set(True);bpy.ops.object.mode_set(mode='EDIT')
    for side,indices in ids.items():
        center=sum((lashes.data.vertices[i].co for i in indices),Vector())/len(indices)
        b=arm.data.edit_bones.new('brow.'+side);b.head=center;b.tail=center+Vector((0,3,0))
        b.parent=arm.data.edit_bones['head.x_018'];b.use_deform=True
    bpy.ops.object.mode_set(mode='OBJECT')
    for side,indices in ids.items():
        g=lashes.vertex_groups.new(name='brow.'+side)
        for i in indices:
            for old in list(lashes.data.vertices[i].groups):lashes.vertex_groups[old.group].remove([i])
            g.add([i],1,'REPLACE')
        arm.data.collections['Face'].assign(arm.data.bones['brow.'+side])
        result['brows'].append({'bone':'brow.'+side,'mesh':lashes.name,'vertices':indices})
    # IK targets use armature coordinates, so the existing 0.01 object scale is
    # inherited exactly. No stretch and no scene-space scale hacks are used.
    for side in ('l','r'):
        for limb,upper,lower,end in [('arm','arm_stretch','forearm_stretch','hand'),('leg','thigh_stretch','leg_stretch','foot')]:
            key='IK_'+limb+'.'+side.upper();prop(arm,key,0,description='0: FK, 1: IK. Match targets before switching an animated FK pose.')
            up=arm.pose.bones[find(arm,upper,side)];lo=arm.pose.bones[find(arm,lower,side)];tip=arm.pose.bones[find(arm,end,side)]
            target=empty('CTRL_'+end+'.'+side.upper(),tip.bone.head_local,'CUBE',7)
            target.rotation_mode='QUATERNION';target.rotation_quaternion=tip.bone.matrix_local.to_quaternion()
            a=up.bone.head_local.copy();b=lo.bone.head_local.copy();c=tip.bone.head_local.copy();axis=(c-a).normalized()
            projection=a+axis*(b-a).dot(axis);bend=b-projection
            if bend.length<.01:bend=Vector((0,-1 if limb=='arm' else 1,0))
            pole=empty('CTRL_pole_'+limb+'.'+side.upper(),b+bend.normalized()*(c-a).length*.65,'SPHERE',5)
            reference=[up.matrix.copy(),lo.matrix.copy()]
            constraint=lo.constraints.new('IK');constraint.name=key+' (2 bones, no stretch)';constraint.target=target
            constraint.pole_target=pole;constraint.chain_count=2;constraint.use_stretch=False;constraint.iterations=100
            influence(constraint,arm,key);up.ik_stretch=0;lo.ik_stretch=0
            arm[key]=1;bpy.context.view_layer.update()
            # Calibrate pole angle against the actual asymmetrical rest pose.
            best=(float('inf'),0)
            for step in range(360):
                angle=math.radians(step-180);constraint.pole_angle=angle;bpy.context.view_layer.update()
                error=sum(sum((x-y).length_squared for x,y in zip(p.matrix.to_3x3(),m.to_3x3())) for p,m in zip((up,lo),reference))
                if error<best[0]:best=(error,angle)
            def objective(angle):
                constraint.pole_angle=angle;bpy.context.view_layer.update()
                return sum(sum((x-y).length_squared for x,y in zip(p.matrix.to_3x3(),m.to_3x3())) for p,m in zip((up,lo),reference))
            fit=minimize_scalar(objective,bounds=(best[1]-math.pi/180,best[1]+math.pi/180),method='bounded',options={'xatol':1e-8})
            best=(float(fit.fun),float(fit.x));constraint.pole_angle=best[1]
            orient=tip.constraints.new('COPY_ROTATION');orient.name=key+' target orientation';orient.target=target
            orient.owner_space='WORLD';orient.target_space='WORLD';influence(orient,arm,key)
            arm[key]=0;bpy.context.view_layer.update()
            result['ik'].append({'property':key,'target':target.name,'pole':pole.name,'end_bone':tip.name,
                                'solver_bone':lo.name,'pole_angle':best[1],'rest_orientation_fit_error':best[0]})
    # Additive phalanx curls use a transverse anatomical axis projected into
    # each bone's own coordinates, rather than one global rotation for all.
    for side in ('l','r'):
        across=(arm.data.bones[find(arm,'c_pinky1',side)].head_local-arm.data.bones[find(arm,'c_index1',side)].head_local).normalized()
        for digit in ('index','middle','ring','pinky','thumb'):
            first=arm.data.bones[find(arm,'c_'+digit+'1',side)]
            ctrl=empty('CTRL_curl_'+digit+'.'+side.upper(),first.head_local,'CIRCLE',3)
            prop(ctrl,'curl_degrees',0,-30,60,'Additive curl; the left rest hand is already bent. Start with 0..30 degrees.')
            for j in ('1','2','3'):
                bone=arm.pose.bones[find(arm,'c_'+digit+j,side)]
                axis=across-bone.bone.y_axis*across.dot(bone.bone.y_axis)
                axis=bone.bone.matrix_local.to_quaternion().inverted()@axis.normalized()
                if side=='r':axis=-axis
                helper=empty('_curl_'+bone.name,helper=True);helper.rotation_mode='QUATERNION'
                for index in range(4):
                    f=helper.driver_add('rotation_quaternion',index);variable(f.driver,'angle',ctrl,'["curl_degrees"]')
                    f.driver.expression='cos(angle*pi/360)' if index==0 else f'sin(angle*pi/360)*{axis[index-1]:.12g}'
                copy=bone.constraints.new('COPY_ROTATION');copy.name='Additive finger curl';copy.target=helper
                copy.owner_space='LOCAL';copy.target_space='LOCAL';copy.mix_mode='AFTER'
            result['finger_controls'].append(ctrl.name)
    # Half wrist twist added to the matching forearm helper using swing/twist
    # decomposition. Existing FK rotations remain editable and are not replaced.
    prop(arm,'auto_forearm_twist',1,description='Half of wrist local Y twist; additive to manual forearm helper FK.')
    for side in ('l','r'):
        bone=arm.pose.bones[find(arm,'forearm_twist',side)];hand=find(arm,'hand',side)
        helper=empty('_auto_twist.'+side.upper(),helper=True)
        f=helper.driver_add('rotation_euler',1);f.driver.expression='.5*twist*enabled'
        v=f.driver.variables.new();v.name='twist';v.type='TRANSFORMS';v.targets[0].id=arm
        v.targets[0].bone_target=hand;v.targets[0].transform_type='ROT_Y';v.targets[0].transform_space='LOCAL_SPACE'
        v.targets[0].rotation_mode='SWING_TWIST_Y';variable(f.driver,'enabled',arm,'["auto_forearm_twist"]')
        copy=bone.constraints.new('COPY_ROTATION');copy.name='Half wrist longitudinal twist';copy.target=helper
        copy.owner_space='LOCAL';copy.target_space='LOCAL';copy.mix_mode='AFTER';copy.use_x=False;copy.use_z=False
        result['twist'].append({'bone':bone.name,'source':hand,'fraction':.5})
    # Pose collections expose existing deform bones for body, gaze and ears.
    # No look-at constraint that silently changes the neutral iris direction.
    arm['controls_help']='IK_* sliders in Object custom properties. CTRL_hand/foot + poles. CTRL_curl_* custom property. Face collection: eye/ear/brow FK. Body root: root.x_01.'
    reset(arm)
    return result


def add_tests(arm):
    # One compact wider-range regression clip; retained seven original Actions
    # stay comparable. Every checkpoint has explicit zero scale/location reset.
    cases=definitions(arm)
    action=bpy.data.actions.new('TEST_practical_ranges');action.use_fake_user=True
    reset(arm);arm.animation_data_create();arm.animation_data.action=action
    sequence=[(1,None),(31,'elbow_-120'),(61,None),(91,'knee_120'),(121,None),
              (151,'finger_curl_30'),(181,None),(211,'combined'),(241,None)]
    for frame,name in sequence:
        for p in arm.pose.bones:p.matrix_basis=Matrix.Identity(4);p.rotation_mode='QUATERNION'
        for bone,angle,axis in cases.get(name,[]):turn(arm,bone,angle,axis)
        for p in arm.pose.bones:
            for path in ('rotation_quaternion','location','scale'):p.keyframe_insert(path,frame=frame,group=p.name)
    for c in action.fcurves:
        for k in c.keyframe_points:k.interpolation='LINEAR'
    reset(arm);action=bpy.data.actions.new('TEST_brows_wrist');action.use_fake_user=True
    arm.animation_data_create();arm.animation_data.action=action
    for frame,strength in [(1,0),(16,.5),(31,1),(46,.5),(61,0)]:
        for p in arm.pose.bones:p.matrix_basis=Matrix.Identity(4);p.rotation_mode='QUATERNION'
        for side in ('l','r'):
            hand=find(arm,'hand',side);axis=arm.data.bones[hand].y_axis
            turn(arm,hand,60*strength,axis)
            # Brow bones have local Z along global Z (their rest Y is forward).
            p=arm.pose.bones['brow.'+side.upper()]
            p.location=p.bone.matrix_local.to_quaternion().inverted()@Vector((0,0,.8*strength))
        for p in arm.pose.bones:
            for path in ('rotation_quaternion','location','scale'):p.keyframe_insert(path,frame=frame,group=p.name)
    for c in action.fcurves:
        for k in c.keyframe_points:k.interpolation='LINEAR'
    reset(arm)
    action=bpy.data.actions.new('TEST_toes_30');action.use_fake_user=True
    arm.animation_data_create();arm.animation_data.action=action
    for frame,angle in [(1,0),(16,-30),(31,0),(46,30),(61,0)]:
        for p in arm.pose.bones:p.matrix_basis=Matrix.Identity(4);p.rotation_mode='QUATERNION'
        for side in ('l','r'):turn(arm,find(arm,'toes_01',side),angle,(1,0,0))
        for p in arm.pose.bones:
            for path in ('rotation_quaternion','location','scale'):p.keyframe_insert(path,frame=frame,group=p.name)
    for c in action.fcurves:
        for k in c.keyframe_points:k.interpolation='LINEAR'
    reset(arm)
