"""Incremental local repair and Blender controls; never edits source Raphtalia.fbx.

Input must be the output of prepare_rig.py, or the retained pre-audit snapshot.
Writes only explicitly named candidate outputs. No global automatic weighting.
"""
import argparse
import json
import math
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector
from scipy.spatial import cKDTree

from reaudit import ROOT, reset, find


def get_weights(obj):
    names=[g.name for g in obj.vertex_groups]
    w=np.zeros((len(obj.data.vertices),len(names)))
    for v in obj.data.vertices:
        for g in v.groups:w[v.index,g.group]=g.weight
    return names,w


def set_weights(obj,names,w,selected):
    changes=[]
    for i in np.flatnonzero(selected):
        old={obj.vertex_groups[g.group].name:float(g.weight) for g in obj.data.vertices[i].groups if g.weight>1e-8}
        row=w[i];top=np.argsort(row)[-4:];total=row[top].sum()
        new={names[j]:float(row[j]/total) for j in top if row[j]/total>1e-8}
        if max(abs(old.get(n,0)-new.get(n,0)) for n in old.keys()|new.keys())<1e-6:continue
        for g in list(obj.data.vertices[i].groups):obj.vertex_groups[g.group].remove([int(i)])
        for n,value in new.items():obj.vertex_groups[n].add([int(i)],value,'REPLACE')
        changes.append({'vertex':int(i),'before':old,'after':new})
    return changes


def smooth_local(obj,arm,iterations=6):
    names,w=get_weights(obj);original=w.copy();co=np.array([v.co[:] for v in obj.data.vertices])
    mask=np.zeros(len(co));regions=[]
    # Soft compact support leaves vertices beyond each local repair untouched.
    for prefix,radius in [('forearm_stretch',23),('leg_stretch',25),('foot',25),('toes_01',25),('hand',13),('arm_stretch',26),('head',16)]:
        if prefix=='foot' and obj.name not in ('shoes_tex_shoes_0','stocking_tex_sock_0'):continue
        if prefix=='toes_01' and obj.name not in ('body_tex_skin_0','shoes_tex_shoes_0'):continue
        if prefix=='hand' and obj.name not in ('body_tex_skin_0','long_gloves_tex_glove_0'):continue
        if prefix=='forearm_stretch' and obj.name not in ('body_tex_skin_0','shirt_tex_shirt_0','long_gloves_tex_glove_0'):continue
        if prefix=='leg_stretch' and obj.name not in ('body_tex_skin_0','stocking_tex_sock_0'):continue
        if prefix in ('arm_stretch','head') and obj.name not in ('body_tex_skin_0','shirt_tex_shirt_0'):continue
        for side in ('l','r'):
            if prefix=='head' and side=='r':continue
            b=arm.data.bones['head.x_018' if prefix=='head' else find(arm,prefix,side)];distance=np.linalg.norm(co-np.array(b.head_local),axis=1)
            t=np.clip((radius-distance)/(.65*radius),0,1);mask=np.maximum(mask,t*t*(3-2*t))
            regions.append({'joint':b.name,'radius_source_units':radius})
    if obj.name in ('body_tex_skin_0','long_gloves_tex_glove_0'):
        # Compact knuckle transitions; finger tips retain original weights.
        for side in ('l','r'):
            for digit in ('index','middle','ring','pinky','thumb'):
                for joint in ('2','3'):
                    b=arm.data.bones[find(arm,'c_'+digit+joint,side)]
                    d=np.linalg.norm(co-np.array(b.head_local),axis=1);t=np.clip((3.2-d)/2.2,0,1)
                    mask=np.maximum(mask,.85*t*t*(3-2*t))
    if obj.name=='body_tex_skin_0':
        # Keep the already graded proximal thumb fan. Wrist smoothing there
        # increased two 60-degree thumb stress flags in the rejected candidate.
        for side in ('l','r'):
            first=find(arm,'c_thumb1',side);second=find(arm,'c_thumb2',side)
            close=np.linalg.norm(co-np.asarray(arm.data.bones[first].head_local),axis=1)<8
            keep=close&(original[:,names.index(first)]>.2)&(original[:,names.index(second)]<.5)
            mask[keep]=0
    if obj.name=='shirt_tex_shirt_0':
        # Waistband seam: abrupt spine-only versus pelvis/cloth assignments
        # stretched a 2.13 mm edge to 18.16 mm under just 5 degrees of spine yaw.
        # Diffuse only near the existing independent ribbon roots.
        for b in arm.data.bones:
            if b.name.startswith('cloth.') and b.parent.name=='root.x_01':
                d=np.linalg.norm(co-np.asarray(b.head_local),axis=1);t=np.clip((14-d)/9,0,1)
                mask=np.maximum(mask,t*t*(3-2*t))
    # Mesh edges plus coincident UV seam vertices. Never nearest-surface jumps.
    edges=[tuple(e.vertices) for e in obj.data.edges]
    edges+=list(cKDTree(co).query_pairs(1e-4))
    e=np.asarray(edges,int);length=np.linalg.norm(co[e[:,1]]-co[e[:,0]],axis=1)
    strength=1/np.maximum(length,.1)**2
    ids=np.r_[e[:,0],e[:,1]];other=np.r_[e[:,1],e[:,0]];strength=np.r_[strength,strength]
    denom=np.bincount(ids,weights=strength,minlength=len(co));valid=denom>0
    for _ in range(iterations):
        total=np.zeros_like(w);np.add.at(total,ids,w[other]*strength[:,None])
        mean=total/np.maximum(denom[:,None],1e-30)
        # Screened diffusion: fixed original anchor prevents runaway blur.
        w[valid]=(1-mask[valid,None])*original[valid]+mask[valid,None]*(.6*w[valid]+.4*mean[valid])
    # Narrow folded ankle seams have virtually adjacent endpoints assigned to
    # opposite rigid segments. Keep each connected short seam coherent, rather
    # than making the entire boot rigid (which would break toes and shaft).
    if obj.name in ('shoes_tex_shoes_0','stocking_tex_sock_0','body_tex_skin_0','shirt_tex_shirt_0'):
        ankle=np.minimum.reduce([np.linalg.norm(co-np.asarray(arm.data.bones[find(arm,'foot',s)].head_local),axis=1) for s in ('l','r')])
        toe=np.minimum.reduce([np.linalg.norm(co-np.asarray(arm.data.bones[find(arm,'toes_01',s)].head_local),axis=1) for s in ('l','r')])
        local=(ankle<12) if obj.name!='body_tex_skin_0' else (toe<14)
        if obj.name=='shoes_tex_shoes_0':local|=toe<14
        if obj.name=='shirt_tex_shirt_0':
            local=np.minimum.reduce([np.linalg.norm(co-np.asarray(b.head_local),axis=1)
                                     for b in arm.data.bones if b.name.startswith('cloth.') and b.parent.name=='root.x_01'])<14
        parent=list(range(len(co)))
        def root(i):
            while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
            return i
        for (a,b),length in zip(e,np.linalg.norm(co[e[:,1]]-co[e[:,0]],axis=1)):
            if length<.5 and local[a] and local[b]:parent[root(int(b))]=root(int(a))
        groups={}
        for i in range(len(co)):groups.setdefault(root(i),[]).append(i)
        for group in groups.values():
            if len(group)>1:
                w[group]=w[group].mean(axis=0);mask[group]=1
    changes=set_weights(obj,names,w,mask>0)
    return {'mesh':obj.name,'regions':regions,'iterations':iterations,'changes':changes}


def export_fbx(path,arm,meshes,bake_step=1):
    for o in bpy.context.selected_objects:o.select_set(False)
    for o in meshes+[arm]:o.select_set(True)
    bpy.context.view_layer.objects.active=arm
    bpy.ops.export_scene.fbx(filepath=str(path),use_selection=True,object_types={'ARMATURE','MESH'},
        use_mesh_modifiers=False,add_leaf_bones=False,use_armature_deform_only=True,
        bake_anim=True,bake_anim_use_all_actions=True,bake_anim_use_nla_strips=False,
        bake_anim_use_all_bones=True,bake_anim_force_startend_keying=True,bake_anim_step=bake_step,
        bake_anim_simplify_factor=0,path_mode='RELATIVE',embed_textures=False,
        axis_forward='-Z',axis_up='Y',apply_unit_scale=True,apply_scale_options='FBX_SCALE_ALL')


def main():
    p=argparse.ArgumentParser();p.add_argument('--input',default='build/raphtalia_tools/reaudit/before.blend')
    p.add_argument('--output',default='build/raphtalia_tools/reaudit/candidate.blend')
    p.add_argument('--iterations',type=int,default=6);p.add_argument('--export',action='store_true')
    p.add_argument('--controls',action='store_true')
    args=p.parse_args();bpy.ops.wm.open_mainfile(filepath=str((ROOT/args.input).resolve()))
    arm=next(o for o in bpy.data.objects if o.type=='ARMATURE');meshes=[o for o in bpy.data.objects if o.type=='MESH']
    if len(arm.data.bones)!=239 or 'brow.L' in arm.data.bones:
        raise ValueError('Expected the 239-bone first-stage prepared rig; do not apply the refinement twice.')
    reset(arm);patches=[]
    for obj in meshes:
        if obj.name in ('body_tex_skin_0','shirt_tex_shirt_0','stocking_tex_sock_0','shoes_tex_shoes_0','long_gloves_tex_glove_0'):
            patches.append(smooth_local(obj,arm,args.iterations))
    out=(ROOT/args.output).resolve();out.parent.mkdir(parents=True,exist_ok=True)
    if args.controls:
        from blender_controls import build,add_tests
        controls=build(arm,meshes);add_tests(arm)
        out.with_suffix('.controls.json').write_text(json.dumps(controls,indent=2),encoding='utf-8')
    bpy.context.preferences.filepaths.save_version=0
    bpy.context.scene.frame_end=241
    arm.animation_data_create();arm.animation_data.action=bpy.data.actions['TEST_elbows_90'];bpy.context.scene.frame_set(1)
    bpy.ops.wm.save_as_mainfile(filepath=str(out),check_existing=False)
    (out.with_suffix('.weights.json')).write_text(json.dumps(patches,indent=2),encoding='utf-8')
    if args.export:export_fbx(out.with_suffix('.fbx'),arm,meshes)
    print('LOCAL PATCH',[(x['mesh'],len(x['changes'])) for x in patches])


if __name__=='__main__':main()
