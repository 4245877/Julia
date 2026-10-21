"""Author an explicit eyelid-rib candidate from the retained 241/11 snapshot.

The source and original assets are never written. Positive, local weights
close the upper/lower lids, preserving the surrounding face and existing rig.
No shape keys or changes to the C++ runtime are needed for baked node TRS.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import bpy
import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from reaudit import reset, turn
from refine_rig import export_fbx
from blink_lash_mapping import classify_lashes
from blink_local_surface import refine

ROOT=Path(__file__).resolve().parents[2]

def data():
    audit=json.loads((ROOT/'docs/raphtalia/blink/topology.json').read_text(encoding='utf-8'))
    result={}
    for side,d in audit['eyes'].items():
        rows={v['unique_id']:v for v in d['loop']}
        up=np.array([rows[i]['co'] for i in d['upper_unique_ids']])
        low=np.array([rows[i]['co'] for i in d['lower_unique_ids']])
        t=np.linspace(0,1,9);dense=np.linspace(0,1,33)
        seam=low+.22*(up-low)
        seam[:,1]=np.minimum(low[:,1],up[:,1])-.16
        # A deep U inherited from the open lower rim turns the corner fan inside
        # out. Use a shallow relaxed seam, tangent to each outer canthus spoke.
        seam[:,2]=(1-t)*up[0,2]+t*up[-1,2]-.9*np.sin(math.pi*t)**2
        seam[[0,-1]]=up[[0,-1]]
        outer=d['geodesic_shells']['3']
        corner_left=np.array(min(outer,key=lambda v:v['co'][0])['co'])
        corner_right=np.array(max(outer,key=lambda v:v['co'][0])['co'])
        left_slope=(up[0,2]-corner_left[2])/(up[0,0]-corner_left[0])
        right_slope=(up[-1,2]-corner_right[2])/(up[-1,0]-corner_right[0])
        seam[1,2]=up[0,2]+left_slope*(seam[1,0]-up[0,0])
        seam[-2,2]=up[-1,2]+right_slope*(seam[-2,0]-up[-1,0])
        target=CubicSpline(t,seam,axis=0,bc_type='natural')(dense)
        target[:,0]=PchipInterpolator(t,seam[:,0])(dense)
        target[:,1]-=.8*np.sin(math.pi*dense)
        target[:5,2]=up[0,2]+left_slope*(target[:5,0]-up[0,0])
        target[-5:,2]=up[-1,2]+right_slope*(target[-5:,0]-up[-1,0])
        upper=np.array([np.interp(dense,t,up[:,j]) for j in range(3)]).T
        lower=np.array([np.interp(dense,t,low[:,j]) for j in range(3)]).T
        result[side]=(upper,lower,target)
    return audit,result

def interpolate_weights(xs,x):
    i=int(np.clip(np.searchsorted(xs,x)-1,0,len(xs)-2))
    t=float(np.clip((x-xs[i])/(xs[i+1]-xs[i]),0,1))
    return [(i,1-t),(i+1,t)]

def build(arm):
    audit,eyes=data();head=arm.data.bones['head.x_018']
    annuli={}
    for side,d in audit['eyes'].items():
        center=np.mean([v['co'] for v in d['loop']],axis=0);annuli[side]=[]
        for vertices in [d['loop']]+[d['geodesic_shells'][str(i)] for i in (1,2,3)]:
            vertices=sorted(vertices,key=lambda v:math.atan2(v['co'][2]-center[2],v['co'][0]-center[0]))
            left=int(np.argmin([v['co'][0] for v in vertices]));vertices=vertices[left:]+vertices[:left]
            assert len(vertices)==16 and int(np.argmax([v['co'][0] for v in vertices]))==8
            annuli[side].append(vertices)
    skin=bpy.data.objects['body_tex_skin_0'];lashes=bpy.data.objects['eyelashes_tex_eyelashes_0']
    original_skin_points=np.array([v.co[:] for v in skin.data.vertices])
    _,source_weld=np.unique(np.round(original_skin_points,4),axis=0,return_inverse=True)
    source_normals=np.zeros((int(source_weld.max())+1,3))
    for f in skin.data.polygons:
        a,b,c=f.vertices;n=np.cross(original_skin_points[b]-original_skin_points[a],original_skin_points[c]-original_skin_points[a])
        for i in (a,b,c):source_normals[source_weld[i]]+=n
    source_normals/=np.maximum(np.linalg.norm(source_normals,axis=1)[:,None],1e-20)
    source_normals=source_normals[source_weld]
    lash_mapping=classify_lashes(lashes.data,{s:v[0][::4] for s,v in eyes.items()},{s:v[1][::4] for s,v in eyes.items()})
    surfaces={obj.name:refine(obj,passes=2) for obj in (skin,lashes)}
    for o in bpy.context.selected_objects:o.select_set(False)
    arm.select_set(True);bpy.context.view_layer.objects.active=arm
    bpy.ops.object.mode_set(mode='EDIT')
    names={};deltas={};surface_bones={};surface_deltas={}
    for side,(up,low,target_curve) in eyes.items():
        for region,curve in [('upper',up),('lower',low)]:
            k=side+'.'+region;names[k]=[];deltas[k]=[]
            for i,p in enumerate(curve):
                # Corner ribs have zero motion and use the existing head bone.
                name=f'lid.{region}.{side}.{i:02d}' if i not in (0,len(curve)-1) else 'head.x_018'
                if i not in (0,len(curve)-1):
                    b=arm.data.edit_bones.new(name);b.head=Vector(p);b.tail=b.head+Vector((0,1,0))
                    b.parent=arm.data.edit_bones[head.name];b.use_deform=True
                names[k].append(name)
                # Corresponding contour knots must share x as well as y/z.
                # Interpolating only z at the two different sets of x knots
                # leaves triangular cracks between their polygon chords.
                target=target_curve[i].copy()
                overlap=math.sin(math.pi*(i-4)/(len(curve)-9)) if 4<i<len(curve)-5 else 0.
                target[2]+=(-.005 if region=='upper' else .005)*overlap
                delta=target-p
                if i in (0,len(curve)-1):delta[:]=0
                deltas[k].append(delta)
        seam=target_curve[::4]
        closed_loop=np.concatenate([seam,seam[-2:0:-1]])
        outer=np.array([v['co'] for v in annuli[side][3]])
        for row,radius in ((1,.30),(2,.65)):
            for i,v in enumerate(annuli[side][row]):
                name=f'lid.surface.{side}.{row}.{i:02d}';p=np.array(v['co'])
                b=arm.data.edit_bones.new(name);b.head=Vector(p);b.tail=b.head+Vector((0,1,0));b.parent=arm.data.edit_bones[head.name]
                target=(1-radius)*closed_loop[i]+radius*outer[i]
                surface_bones[(side,row,i)]=name;surface_deltas[name]=(side,target-p)
    bpy.ops.object.mode_set(mode='OBJECT')
    col=arm.data.collections.get('Eyelids') or arm.data.collections.new('Eyelids')
    for side in eyes:
        key='Blink_'+side;arm[key]=0.
        arm.id_properties_ui(key).update(min=0.,max=1.,soft_min=0.,soft_max=1.,description=f'{side} eyelids: 0 open, 1 closed; fitted shared seam')
        for region in ('upper','lower'):
            k=side+'.'+region
            for i,(name,delta) in enumerate(zip(names[k],deltas[k])):
                if name=='head.x_018':continue
                col.assign(arm.data.bones[name]);p=arm.pose.bones[name]
                p.lock_rotation=(True,True,True);p.lock_scale=(True,True,True)
                for axis in (0,1,2):
                    curve=p.driver_add('location',axis);d=curve.driver;d.type='SCRIPTED'
                    v=d.variables.new();v.name='blink';v.type='SINGLE_PROP'
                    v.targets[0].id=arm;v.targets[0].data_path=f'["{key}"]'
                    arc=-.28*math.sin(math.pi*i/(len(names[k])-1)) if axis==1 else 0.
                    d.expression=f'({delta[axis]:.9g})*min(max(blink,0),1)+({arc:.9g})*sin(pi*min(max(blink,0),1))'
    for name,(side,delta) in surface_deltas.items():
        col.assign(arm.data.bones[name]);p=arm.pose.bones[name]
        p.lock_rotation=(True,True,True);p.lock_scale=(True,True,True)
        for axis in (0,1,2):
            curve=p.driver_add('location',axis);d=curve.driver;d.type='SCRIPTED'
            v=d.variables.new();v.name='blink';v.type='SINGLE_PROP';v.targets[0].id=arm;v.targets[0].data_path=f'["Blink_{side}"]'
            arc=-.12 if axis==1 else 0.
            d.expression=f'({delta[axis]:.9g})*min(max(blink,0),1)+({arc:.9g})*sin(pi*min(max(blink,0),1))'
    patch=[]
    def assign(obj,index,side,region,x,strength):
        old={obj.vertex_groups[g.group].name:float(g.weight) for g in obj.data.vertices[index].groups if g.weight>1e-8}
        assert old=={'head.x_018':1.},(obj.name,index,old)
        new={'head.x_018':1-strength} if strength<1-1e-7 else {}
        for i,w in interpolate_weights(eyes[side][0 if region=='upper' else 1][:,0],x):
            if strength*w>1e-7:
                n=names[side+'.'+region][i];new[n]=new.get(n,0)+strength*w
        total=sum(new.values());new={n:w/total for n,w in new.items()}
        for g in list(obj.data.vertices[index].groups):obj.vertex_groups[g.group].remove([index])
        for n,w in new.items():(obj.vertex_groups.get(n) or obj.vertex_groups.new(name=n)).add([index],w,'REPLACE')
        patch.append({'mesh':obj.name,'vertex':index,'before':old,'after':new})
    # Follow the actual annular edge loops. A rectangular x/z mask cuts through
    # the inner canthus fans and can invert their tiny triangles during closure.
    source_bindings={};boundary_fields={}
    for side,rings in annuli.items():
        for row,vertices in enumerate(rings[:3]):
            for i,v in enumerate(vertices):
                if row:
                    binding={surface_bones[(side,row,i)]:1.}
                else:
                    region='lower' if i<=8 else 'upper';j=i if i<=8 else 16-i
                    binding={names[side+'.'+region][j*4]:1.}
                    for source in v['source_ids']:boundary_fields[source]=(side,region,j/8)
                for source in v['source_ids']:source_bindings[source]=binding
    def set_weights(obj,index,new):
        old={obj.vertex_groups[g.group].name:float(g.weight) for g in obj.data.vertices[index].groups if g.weight>1e-8}
        new={n:w for n,w in new.items() if w>1e-7};total=sum(new.values());new={n:w/total for n,w in new.items()}
        assert len(new)<=4,(obj.name,index,new)
        if old==new:return
        for g in list(obj.data.vertices[index].groups):obj.vertex_groups[g.group].remove([index])
        for n,w in new.items():(obj.vertex_groups.get(n) or obj.vertex_groups.new(name=n)).add([index],w,'REPLACE')
        patch.append({'mesh':obj.name,'vertex':index,'before':old,'after':new})
    for v,provenance in zip(skin.data.vertices,surfaces[skin.name]['vertex_provenance']):
        if not any(source in source_bindings for source in provenance):continue
        new={}
        for source,weight in provenance.items():
            for name,value in source_bindings.get(source,{'head.x_018':1.}).items():new[name]=new.get(name,0)+weight*value
        # Keep the two very narrow outer-canthus fans affine. Curving only their
        # subdivided boundary folds the tiny interior triangles at full contact.
        affine_corner=any(set(provenance).issubset(support) for support in ({28,29,30},{253,259,260}))
        if not affine_corner and all(source in boundary_fields for source in provenance):
            charts=[boundary_fields[source] for source in provenance]
            active=[m for m in charts if m[2] not in (0,1)]
            if active:
                side,region,_=active[0];t=sum(weight*boundary_fields[source][2] for source,weight in provenance.items())
                curve=eyes[side][0 if region=='upper' else 1];x=float(np.interp(t,np.linspace(0,1,len(curve)),curve[:,0]))
                new={}
                for i,w in interpolate_weights(curve[:,0],x):
                    n=names[side+'.'+region][i];new[n]=new.get(n,0)+w
        set_weights(skin,v.index,new)
    skin_bvh=BVHTree.FromPolygons([v.co for v in skin.data.vertices],[list(f.vertices) for f in skin.data.polygons],all_triangles=True)
    def nearest_skin_field(co):
        point,_,face,_=skin_bvh.find_nearest(co)
        ids=list(skin.data.polygons[face].vertices);a,b,c=[skin.data.vertices[i].co for i in ids]
        v0=b-a;v1=c-a;v2=point-a;d00=v0.dot(v0);d01=v0.dot(v1);d11=v1.dot(v1);d20=v2.dot(v0);d21=v2.dot(v1)
        denom=d00*d11-d01*d01
        if abs(denom)<1e-15:return None
        v=(d11*d20-d01*d21)/denom;w=(d00*d21-d01*d20)/denom
        coeff=np.maximum([1-v-w,v,w],0);coeff/=sum(coeff)
        aggregate={}
        for i,weight in zip(ids,coeff):
            for g in skin.data.vertices[i].groups:
                name=skin.vertex_groups[g.group].name;aggregate[name]=aggregate.get(name,0)+weight*g.weight
        if len(aggregate)>4:
            aggregate=dict(sorted(aggregate.items(),key=lambda kv:kv[1],reverse=True)[:4])
        return aggregate
    for i,provenance in enumerate(surfaces[lashes.name]['vertex_provenance']):
        regions={}
        for source,weight in provenance.items():
            region=lash_mapping[source]['region'];regions[region]=regions.get(region,0)+weight
        if 'skin_detail' in regions:
            mapped=nearest_skin_field(lashes.data.vertices[i].co)
            if mapped:set_weights(lashes,i,mapped)
        elif 'upper' in regions or 'lower' in regions:
            region=max(('upper','lower'),key=lambda r:regions.get(r,0))
            co=lashes.data.vertices[i].co;side='L' if co.x>0 else 'R'
            assign(lashes,i,side,region,co.x,regions[region])
    # Source FBX marks nearly every skin edge sharp. A stretched sharp-normal
    # fan looks folded even when its surface is monotone. Smooth only moving
    # eyelid skin, fading to the retained normals at the fixed surrounding rows.
    points=np.array([v.co[:] for v in skin.data.vertices])
    # Interpolate smooth ORIGINAL normals. Recomputing on planar subdivision
    # creates flat face-interior normals and scalloped shading at new points.
    accumulated=np.array([sum((source_normals[i]*w for i,w in provenance.items()),start=np.zeros(3))
                          for provenance in surfaces[skin.name]['vertex_provenance']])
    accumulated/=np.maximum(np.linalg.norm(accumulated,axis=1)[:,None],1e-20)
    old=np.array([n.vector[:] for n in skin.data.corner_normals]);new=old.copy();normal_ids=[];smooth_vertices=set()
    for loop in skin.data.loops:
        x,y,z=points[loop.vertex_index]
        radius=math.sqrt(((abs(x)-9.1)/8.3)**2+((z-378.1)/8.8)**2)
        if radius>=1 or y>=-10:continue
        w=float(np.clip((1-radius)/.3,0,1));alpha=w*w*(3-2*w)
        n=(1-alpha)*old[loop.index]+alpha*accumulated[loop.vertex_index]
        new[loop.index]=n/max(np.linalg.norm(n),1e-20)
        if alpha>.999:
            new[loop.index]=0.;smooth_vertices.add(loop.vertex_index)
        normal_ids.append(loop.index)
    smooth_edges=[]
    for edge in skin.data.edges:
        if all(i in smooth_vertices for i in edge.vertices):edge.use_edge_sharp=False;smooth_edges.append(edge.index)
    skin.data.normals_split_custom_set(new.tolist())
    return {'method':'Upper/lower eyelid ribs and annular corrective cage; fitted shared seam',
            'bone_names':names,'closed_deltas':{k:[v.tolist() for v in ds] for k,ds in deltas.items()},'weight_patch':patch,
            'surface_refinement':surfaces,'skin_chart':'Annular surface cage; closed rows at 30/65 percent from common seam to fixed outer ring',
            'normal_smoothing':{'mesh':skin.name,'loop_ids':normal_ids,'smooth_edge_ids':smooth_edges,
              'method':'Automatic smooth normals in moving eyelids, interpolated original normals in continuous orbital fade'},
            'corner_affine_supports':[[28,29,30],[253,259,260]],
            'surface_closed_deltas':{name:delta.tolist() for name,(_,delta) in surface_deltas.items()},
            'driver_contract':[{'data_path':c.data_path,'axis':c.array_index,'expression':c.driver.expression,
                                'property':c.driver.variables[0].targets[0].data_path}
                               for c in arm.animation_data.drivers if c.data_path.startswith('pose.bones["lid.')],
            'motion_units':'Armature-local source units; mesh/armature world scale 0.01. Bone rotations/scales remain at rest.'}

def clips(arm):
    cases={
      'TEST_blink_natural':[(1,0,0),(9,0,0),(12,1,1),(14,1,1),(20,0,0),(31,0,0)],
      'TEST_blink_independent':[(1,0,0),(8,0,0),(12,1,0),(15,1,0),(22,0,0),(30,0,0),(34,0,1),(37,0,1),(44,0,0),(61,0,0)],
      'TEST_blink_gaze':[(1,0,0),(9,0,0),(12,1,1),(18,0,0),(27,0,0),(30,1,1),(36,0,0),(45,0,0),(48,1,1),(54,0,0),(63,0,0),(66,1,1),(72,0,0),(81,0,0),(84,1,1),(90,0,0),(99,0,0),(102,1,1),(108,0,0),(121,0,0)],
      'TEST_blink_head':[(1,0,0),(17,0,0),(20,1,1),(22,1,1),(28,0,0),(47,0,0),(50,1,1),(52,1,1),(58,0,0),(77,0,0),(80,1,1),(82,1,1),(88,0,0),(121,0,0)]}
    result=[]
    for name,keys in cases.items():
        reset(arm);arm['Blink_L']=0.;arm['Blink_R']=0.
        a=bpy.data.actions.new(name);a.use_fake_user=True;arm.animation_data.action=a
        for f,l,r in keys:
            for s,value in [('L',l),('R',r)]:
                arm['Blink_'+s]=float(value);arm.keyframe_insert(data_path=f'["Blink_{s}"]',frame=f,group='Blink controls')
        if name=='TEST_blink_gaze':
            # Six practically useful gaze directions; blinks occur while gaze holds.
            poses=[(1,0,0),(18,0,0),(24,0,8),(36,0,8),(42,0,-8),(54,0,-8),(60,12,0),(72,12,0),(78,-12,0),(90,-12,0),(96,10,6),(108,10,6),(115,0,0),(121,0,0)]
            for f,yaw,pitch in poses:
                for side in ('L','R'):
                    p=arm.pose.bones['eye.'+side];q=p.bone.matrix_local.to_quaternion()
                    from mathutils import Quaternion
                    p.rotation_mode='QUATERNION';p.rotation_quaternion=q.inverted()@Quaternion(Vector((0,0,1)),math.radians(yaw))@Quaternion(Vector((1,0,0)),math.radians(pitch))@q
                    p.keyframe_insert(data_path='rotation_quaternion',frame=f,group='Gaze')
        if name=='TEST_blink_head':
            for f,yaw,pitch,roll,gaze,brow in [(1,0,0,0,0,0),(18,18,-5,4,-8,.35),(40,-18,6,-6,8,-.2),(72,12,5,8,-6,.4),(100,0,0,0,0,0),(121,0,0,0,0,0)]:
                for bone,angle,axis in [('head.x_018',yaw,(0,0,1)),('neck.x_017',pitch,(1,0,0))]:
                    turn(arm,bone,angle,axis);p=arm.pose.bones[bone]
                    if bone=='head.x_018':
                        from mathutils import Quaternion
                        q=p.bone.matrix_local.to_quaternion();p.rotation_quaternion@=q.inverted()@Quaternion(Vector((0,1,0)),math.radians(roll))@q
                    p.keyframe_insert(data_path='rotation_quaternion',frame=f,group='Head')
                for side in ('L','R'):
                    turn(arm,'eye.'+side,gaze,(0,0,1));p=arm.pose.bones['eye.'+side];p.keyframe_insert(data_path='rotation_quaternion',frame=f,group='Gaze')
                    p=arm.pose.bones['brow.'+side];p.location=p.bone.matrix_local.to_quaternion().inverted()@Vector((0,0,brow));p.keyframe_insert(data_path='location',frame=f,group='Existing brows')
        for fc in a.fcurves:
            for k in fc.keyframe_points:
                k.interpolation='BEZIER';k.handle_left_type='AUTO_CLAMPED';k.handle_right_type='AUTO_CLAMPED'
        result.append({'name':name,'frames':list(a.frame_range),'blink_keys':keys})
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',default='build/raphtalia_tools/blink/base/Raphtalia_prepared.blend')
    p.add_argument('--output',default='build/raphtalia_tools/blink/candidate.blend');p.add_argument('--export',action='store_true')
    args=p.parse_args();src=(ROOT/args.input).resolve();out=(ROOT/args.output).resolve();assert src!=out
    sha=hashlib.sha256(src.read_bytes()).hexdigest();bpy.ops.wm.open_mainfile(filepath=str(src))
    arm=next(o for o in bpy.data.objects if o.type=='ARMATURE');assert len(arm.data.bones)==241 and len(bpy.data.actions)==11
    reset(arm);result=build(arm);result['source_sha256']=sha;result['clips']=clips(arm)
    reset(arm);arm['Blink_L']=0.;arm['Blink_R']=0.;arm.update_tag();bpy.context.scene.frame_set(1)
    arm.animation_data.action=bpy.data.actions['TEST_blink_natural'];bpy.context.scene.frame_end=121
    bpy.context.preferences.filepaths.save_version=0
    out.parent.mkdir(parents=True,exist_ok=True);bpy.ops.wm.save_as_mainfile(filepath=str(out),check_existing=False)
    out.with_suffix('.patch.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if args.export:export_fbx(out.with_suffix('.fbx'),arm,[o for o in bpy.data.objects if o.type=='MESH'],bake_step=.25)
    assert hashlib.sha256(src.read_bytes()).hexdigest()==sha
    print('BUILT',out,'bones',len(arm.data.bones),'actions',len(bpy.data.actions),'patched',len(result['weight_patch']))

if __name__=='__main__':main()
