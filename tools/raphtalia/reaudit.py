"""Read-only, reproducible wide-range audit of a prepared .blend (bpy 4.2).

python tools/raphtalia/reaudit.py --blend ... --output ... [--render]
Angles are pose deltas from the asymmetrical baked rest, not clinical ROM.
Strain flags are diagnostics, never a certificate of anatomical quality.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

ROOT = Path(__file__).resolve().parents[2]


def reset(arm):
    if arm.animation_data:
        arm.animation_data.action=None
        for track in arm.animation_data.nla_tracks:track.mute=True
    for p in arm.pose.bones:
        p.matrix_basis = Matrix.Identity(4)
        p.rotation_mode = 'QUATERNION'
    for o in bpy.data.objects:
        if o.type == 'MESH' and o.data.shape_keys:
            for k in o.data.shape_keys.key_blocks:
                k.value = 0
    bpy.context.view_layer.update()


def find(arm, prefix, side):
    return next(b.name for b in arm.data.bones if b.name.startswith(prefix+'.'+side+'_'))


def turn(arm, name, angle, axis):
    p = arm.pose.bones[name]
    q = p.bone.matrix_local.to_quaternion()
    p.rotation_quaternion = q.inverted() @ Quaternion(Vector(axis), math.radians(angle)) @ q


def definitions(arm):
    cases = {}
    def pair(prefix, angle, axis):
        return [(find(arm, prefix, s), angle, axis) for s in ('l','r')]
    for angle in (-120,-90,-45,45,90,120):
        cases[f'elbow_{angle}'] = pair('forearm_stretch', angle, (1,0,0))
    for angle in (45,90,120):
        cases[f'knee_{angle}'] = pair('leg_stretch', angle, (1,0,0))
    for angle in (-35,35):
        cases[f'ankle_{angle}'] = pair('foot', angle, (1,0,0))
    for angle in (-30,30):
        cases[f'toes_{angle}'] = pair('toes_01',angle,(1,0,0))
    for angle in (-60,60):
        cases[f'wrist_roll_{angle}'] = [(find(arm,'hand',s),angle,tuple(arm.data.bones[find(arm,'forearm_stretch',s)].y_axis)) for s in ('l','r')]
        cases[f'wrist_flex_{angle}'] = pair('hand',angle,(1,0,0))
    for angle in (60,100):
        cases[f'shoulder_raise_{angle}'] = [(find(arm,'arm_stretch',s),angle if s=='l' else -angle,(0,1,0)) for s in ('l','r')]
    for angle in (-45,45):
        cases[f'hip_{angle}'] = pair('thigh_stretch',angle,(1,0,0))
    # Across-palm direction defines a hinge transverse to each phalanx.
    # Signed consistently with the already curled left hand; right is mirrored.
    for angle in (-30,30,60):
        edits=[]
        for side in ('l','r'):
            index = arm.data.bones[find(arm,'c_index1',side)]
            pinky = arm.data.bones[find(arm,'c_pinky1',side)]
            across = (pinky.head_local-index.head_local).normalized()
            for b in arm.data.bones:
                if b.name.startswith(('c_index','c_middle','c_ring','c_pinky','c_thumb')) and f'.{side}_' in b.name:
                    axis=across-b.y_axis*across.dot(b.y_axis)
                    edits.append((b.name,angle if side=='l' else -angle,tuple(axis.normalized())))
        cases[f'finger_curl_{angle}']=edits
    for side in ('l','r'):
        for digit in ('index','middle','ring','pinky','thumb'):
            cases[f'{digit}_{side}_60']=[r for r in cases['finger_curl_60'] if r[0].startswith('c_'+digit) and f'.{side}_' in r[0]]
    cases['shoulder_coordinated']=[(find(arm,prefix,s),angle if s=='l' else -angle,(0,1,0))
        for s in ('l','r') for prefix,angle in [('shoulder',15),('arm_stretch',80)]]
    cases['torso_head']=[('spine_02.x_015',30,(0,0,1)),('neck.x_017',15,(1,0,0)),('head.x_018',60,(0,0,1))]
    cases['secondary_all']=[(b.name, 8 if b.name.startswith('hair.') else 12,
                             (1,0,0) if b.name.startswith('cloth.') else (0,0,1))
                            for b in arm.data.bones if b.name.startswith(('hair.','cloth.','c_tail_'))]
    for axis,vec,angle in [('horizontal',(0,0,1),12),('vertical',(1,0,0),8)]:
        for sign in (-1,1):
            cases[f'gaze_{axis}_{sign}']=[('eye.L',angle*sign,vec),('eye.R',angle*sign,vec)]
    cases['ears_20']=[('ear.L',20,(0,1,0)),('ear.R',-20,(0,1,0))]
    cases['combined']=cases['elbow_-90']+cases['knee_90']+cases['finger_curl_30']+cases['torso_head']+cases['gaze_horizontal_1']+cases['ears_20']+cases['secondary_all']
    return cases


def coordinates(obj):
    ev=obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    co=np.empty(len(ev.data.vertices)*3);ev.data.vertices.foreach_get('co',co)
    mat=np.asarray(ev.matrix_world)
    return (np.c_[co.reshape(-1,3),np.ones(len(ev.data.vertices))]@mat.T)[:,:3]


def cache_meshes(meshes):
    result={}
    for o in meshes:
        p=coordinates(o);e=np.array([x.vertices[:] for x in o.data.edges],dtype=int)
        lengths=np.linalg.norm(p[e[:,1]]-p[e[:,0]],axis=1)
        result[o.name]=(p,e,lengths)
    return result


def measure(meshes,cache,details=False):
    rows=[]
    for o in meshes:
        p,e,a=cache[o.name];q=coordinates(o)
        lengths=np.linalg.norm(q[e[:,1]]-q[e[:,0]],axis=1)
        ratio=lengths/np.maximum(a,1e-30);valid=a>1e-6
        row={'mesh':o.name,'max_ratio':float(ratio[valid].max()),'over_2':int(((ratio>2)&valid).sum()),
             'over_2_extension_gt_1mm':int(((ratio>2)&valid&(lengths-a>.001)).sum()),
             'p99':float(np.percentile(ratio[valid],99)),
             'nonfinite':int((~np.isfinite(q).all(1)).sum())}
        if details:
            ids=np.argsort(np.where(valid,ratio,0))[-5:][::-1]
            row['worst']=[{'vertices':e[i].tolist(),'ratio':float(ratio[i]),'rest_mm':float(a[i]*1000),
                           'posed_mm':float(lengths[i]*1000),
                           'weights':[{o.vertex_groups[g.group].name:float(g.weight) for g in o.data.vertices[v].groups if g.weight>1e-8} for v in e[i]]} for i in ids]
        rows.append(row)
    return {'max_ratio':max(r['max_ratio'] for r in rows),'over_2':sum(r['over_2'] for r in rows),
            'over_2_extension_gt_1mm':sum(r['over_2_extension_gt_1mm'] for r in rows),
            'nonfinite':sum(r['nonfinite'] for r in rows),'meshes':rows}


def render(path,target,scale,view=(0,-1,0.08),objects=None):
    scene=bpy.context.scene;cam=scene.camera
    target=Vector(target);cam.location=target+Vector(view)*8
    cam.rotation_euler=(target-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.ortho_scale=scale
    scene.render.resolution_x=800;scene.render.resolution_y=800
    scene.render.resolution_percentage=100
    scene.render.filepath=str(path)
    hidden=[]
    if objects:
        for o in bpy.data.objects:
            if o.type=='MESH' and o.name not in objects:
                hidden.append((o,o.hide_render));o.hide_render=True
    bpy.ops.render.render(write_still=True)
    for o,value in hidden:o.hide_render=value


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--blend',default='assets/models/Raphtalia_prepared.blend')
    parser.add_argument('--output',default='docs/raphtalia/reaudit/after.json');parser.add_argument('--render',action='store_true')
    args=parser.parse_args();path=(ROOT/args.blend).resolve();out=ROOT/args.output;out.parent.mkdir(parents=True,exist_ok=True)
    bpy.ops.wm.open_mainfile(filepath=str(path));arm=next(o for o in bpy.data.objects if o.type=='ARMATURE')
    meshes=[o for o in bpy.data.objects if o.type=='MESH'];reset(arm);cache=cache_meshes(meshes)
    cases=definitions(arm);report={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
      'blender':bpy.app.version_string,'bones':len(arm.data.bones),'roots':[b.name for b in arm.data.bones if not b.parent],
      'meshes':len(meshes),'vertices':sum(len(o.data.vertices) for o in meshes),'weights':{},'poses':{},
      'limits':'Pose delta stress tests; ratios do not assess volume, collisions or artistic naturalness.'}
    for o in meshes:
        weights=[[g.weight for g in v.groups if g.weight>1e-8] for v in o.data.vertices]
        report['weights'][o.name]={'max_influences':max(map(len,weights)), 'unweighted':sum(not w for w in weights),
            'max_sum_error':max(abs(sum(w)-1) for w in weights),
            'unknown_groups':sorted({o.vertex_groups[g.group].name for v in o.data.vertices for g in v.groups if g.weight>1e-8 and o.vertex_groups[g.group].name not in arm.data.bones})}
    # Legacy actions reproduced from actual saved data, not from old JSON.
    for action in list(bpy.data.actions):
        reset(arm);arm.animation_data_create();arm.animation_data.action=action
        bpy.context.scene.frame_set(31);bpy.context.view_layer.update()
        report['poses'][action.name]=measure(meshes,cache,True)
    for name,changes in cases.items():
        reset(arm)
        for bone,angle,axis in changes:turn(arm,bone,angle,axis)
        bpy.context.view_layer.update()
        report['poses'][name]={'rotations':changes,**measure(meshes,cache,True)}
        if args.render and name in ('elbow_-120','knee_120','ankle_35','finger_curl_30','combined','secondary_all','shoulder_raise_100'):
            render(out.parent/(out.stem+'_'+name+'.png'),(0,0,2.1),4.8)
            if name.startswith('elbow'):render(out.parent/(out.stem+'_elbow_detail.png'),(.4,-.1,2.65),1.2,(1,-1,.2))
            if name.startswith('knee'):render(out.parent/(out.stem+'_knee_detail.png'),(0,.1,1.15),1.5,(1,-1,.2))
            if name.startswith('ankle'):render(out.parent/(out.stem+'_shoe_detail.png'),(-.28,-.3,.25),.8,(1,-1,.2))
            if name.startswith('finger'):
                for side,x,y,z in [('l',.55,-.38,1.96),('r',-.8,.16,1.98)]:
                    render(out.parent/(out.stem+'_hand_'+side+'.png'),(x,y,z),.6,(1,-1,.1),['body_tex_skin_0','long_gloves_tex_glove_0','cuff.001_tex_cuff_0'])
        if args.render and name.startswith(('gaze_','ears_')):
            render(out.parent/(out.stem+'_'+name+'.png'),(0,-.12,3.78),.72)
    reset(arm)
    report['summary']={'poses':len(report['poses']),'nonfinite':sum(p['nonfinite'] for p in report['poses'].values()),
                       'worst_ratio':max(p['max_ratio'] for p in report['poses'].values())}
    out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report['summary']))
    for n,p in report['poses'].items():print(n,round(p['max_ratio'],3),p['over_2'],p['over_2_extension_gt_1mm'])


if __name__=='__main__':main()
