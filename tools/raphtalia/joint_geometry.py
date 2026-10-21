"""Read-only armature/skin geometry alignment audit of original Raphtalia FBX."""
from pathlib import Path
import json
import numpy as np
import bpy

ROOT = Path(__file__).resolve().parents[2]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=str(ROOT/'assets/models/Raphtalia.fbx'), use_anim=False,
                         automatic_bone_orientation=False, ignore_leaf_bones=True)
arm=next(o for o in bpy.data.objects if o.type=='ARMATURE')
meshes=[o for o in bpy.data.objects if o.type=='MESH']
world_to_arm=arm.matrix_world.inverted()
cloud={b.name:[] for b in arm.data.bones}
weights={b.name:[] for b in arm.data.bones}
records=[]
allrecords=[]
for o in meshes:
    transform=world_to_arm @ o.matrix_world
    groupnames={g.index:g.name for g in o.vertex_groups}
    for v in o.data.vertices:
        co=np.array(transform @ v.co,dtype=float)
        influences={groupnames[g.group]:g.weight for g in v.groups if g.weight>1e-8}
        for n,w in influences.items():
            cloud[n].append(co); weights[n].append(w)
        allrecords.append((co,influences))
        if o.name=='body_tex_skin_0': records.append((co,influences))

result={'coordinate_space':'armature local; object transforms included', 'weighted_groups':{},'shared_body_regions':[], 'mesh_matrices':{o.name:[list(r) for r in o.matrix_world] for o in meshes}}
for n,points in cloud.items():
    if not points: continue
    p=np.array(points);w=np.array(weights[n]);center=np.average(p,axis=0,weights=w)
    centered=p-center
    cov=(centered*w[:,None]).T@centered / w.sum()
    eigval,eigvec=np.linalg.eigh(cov)
    b=arm.data.bones[n]
    result['weighted_groups'][n]={'n':len(p),'weight_sum':float(w.sum()),'center':center.tolist(),
         'min':p.min(axis=0).tolist(),'max':p.max(axis=0).tolist(),
         'pca_values':eigval.tolist(),'pca_directions':eigvec.T.tolist(),
         'head':list(b.head_local),'tail':list(b.tail_local),
         'head_to_weighted_centroid':float(np.linalg.norm(np.array(b.head_local)-center))}

# Blend-weight transition clouds give independent evidence of geometric joint placement.
for b in arm.data.bones:
    if not b.parent: continue
    if not any(s in b.name for s in ['stretch','hand.','c_index','c_middle','c_ring','c_pinky','c_thumb','foot.','neck.','head.','spine_']): continue
    pairs=[(co, inf[b.name]*inf[b.parent.name]) for co,inf in records if b.name in inf and b.parent.name in inf]
    if not pairs: continue
    p=np.array([r[0] for r in pairs]);w=np.array([r[1] for r in pairs]);center=np.average(p,axis=0,weights=w)
    result['shared_body_regions'].append({'child':b.name,'parent':b.parent.name,'n':len(p),'center':center.tolist(),
       'min':p.min(axis=0).tolist(),'max':p.max(axis=0).tolist(),'head_to_center':float(np.linalg.norm(np.array(b.head_local)-center))})

result['shared_secondary_regions']=[]
for b in arm.data.bones:
    if not b.parent or not b.name.startswith(('hair.','cloth.','c_tail_')): continue
    pairs=[(co,inf[b.name]*inf[b.parent.name]) for co,inf in allrecords if b.name in inf and b.parent.name in inf]
    if not pairs: continue
    p=np.array([r[0] for r in pairs]);w=np.array([r[1] for r in pairs]);center=np.average(p,axis=0,weights=w)
    result['shared_secondary_regions'].append({'child':b.name,'parent':b.parent.name,'n':len(p),'center':center.tolist(),
       'min':p.min(axis=0).tolist(),'max':p.max(axis=0).tolist(),'head_to_center':float(np.linalg.norm(np.array(b.head_local)-center))})

suggestions={}
for r in result['shared_body_regions']:
    name=r['child']
    spread=np.array(r['max'])-np.array(r['min'])
    # These are skin transition centroids, not recovered anatomical joint centers.
    confidence='medium' if max(spread)<40 and r['n']>=20 else 'low'
    if name.startswith('thigh_'): confidence='low'
    suggestions[name]={'head':r['center'],'method':'parent-child weight-product surface centroid on body mesh',
      'confidence':confidence,'sample_count':r['n'],'sample_aabb_extent':spread.tolist(),
      'original_head_displacement':r['head_to_center'],
      'caveat':'Surface centroid estimates a pivot; density and nonuniform skin weights can bias its center.'}

for b in arm.data.bones:
    if not b.name.startswith('toes_01.'): continue
    pairs=[(co,inf[b.name]*inf[b.parent.name]) for co,inf in allrecords if b.name in inf and b.parent.name in inf]
    if not pairs: continue
    p=np.array([r[0] for r in pairs]);w=np.array([r[1] for r in pairs]);center=np.average(p,axis=0,weights=w)
    suggestions[b.name]={'head':center.tolist(),'method':'foot-to-toe transition on all weighted meshes',
      'confidence':'medium','sample_count':len(p),'sample_aabb_extent':(p.max(axis=0)-p.min(axis=0)).tolist(),
      'original_head_displacement':float(np.linalg.norm(np.array(b.head_local)-center)),
      'caveat':'Includes shoe layers; use visible ball-of-foot cross-section as manual reference.'}

# Keep root and clavicle attachment points; unlike the limbs they have no trustworthy narrow transition region.
for b in arm.data.bones:
    if b.name.startswith(('Skeleton_4_','root.x','shoulder.')):
        suggestions[b.name]={'head':list(b.head_local),'method':'retain original attachment pivot',
          'confidence':'low','sample_count':0,'caveat':'No narrow parent-child body transition; do not infer this point from broad torso clouds.'}

# Auxiliary deform bones are parallel children rather than extra anatomical joints.
for side in ('l','r'):
    def find(prefix): return next(b for b in arm.data.bones if b.name.startswith(prefix+'.'+side+'_'))
    for twist,prox,dist,fraction in [('arm_twist','arm_stretch','forearm_stretch',0.0),
          ('forearm_twist','forearm_stretch','hand',0.5),
          ('thigh_twist','thigh_stretch','leg_stretch',0.0),
          ('leg_twist','leg_stretch','foot',0.5)]:
        tb,pb,db=find(twist),find(prox),find(dist)
        p=np.array(suggestions[pb.name]['head']);d=np.array(suggestions[db.name]['head'])
        suggestions[tb.name]={'head':((1-fraction)*p+fraction*d).tolist(),
           'tail':d.tolist(),'method':'parallel twist helper placed along refit parent segment',
           'confidence':'low','sample_count':0,
           'caveat':'Preserve original weights. Twist rotations must be driven consistently with main parent; do not animate helper as a second elbow/knee.'}

result['suggested_heads_armature_local']=suggestions
result['secondary_refit_recommendation']={
    'rule':'Do not automatically replace hair/cloth/tail pivots with all centroid samples. Narrow parent-child blend regions may guide manual refit; wide separated clouds indicate multiple garment regions and are unsuitable as a joint center.',
    'narrow_candidates':[r['child'] for r in result['shared_secondary_regions'] if r['n']>=8 and max(np.array(r['max'])-np.array(r['min']))<30],
    'warning':'Original geometry is already asymmetrically posed. Changing armature rest matrices while retaining geometry and weights rebinds the visible pose, but cannot recover original unposed anatomy exactly.'}
result['orientation_recommendation']={
    'anatomical_chain_tails':'Use next joint head as main bone tail: humerus->elbow, forearm->wrist, thigh->knee, shin->ankle, each finger segment->next knuckle.',
    'roll':'For humerus and forearm, align a local transverse axis with the plane through shoulder/elbow/wrist. For legs, use hip/knee/ankle plane; keep consistent sign per side. Blender local Y must point toward child. Roll is not recoverable exactly from skin clouds.',
    'terminal_fingers':'Use direction from last knuckle toward distal weighted cloud centroid, with original distal phalanx length or visible fingertip bounds. Do not use original world X directions.',
    'evaluation':'Candidate refit needs explicit moderate and extreme bending tests, including neighboring twist groups, and face/garment intersection review. No automatic anatomical-quality guarantee.'}

out=ROOT/'docs/raphtalia/joint_geometry.json'
out.write_text(json.dumps(result,indent=2),encoding='utf-8')
for n,d in result['weighted_groups'].items():
    if any(s in n for s in ['arm_stretch','forearm_stretch','hand.','thigh_stretch','leg_stretch','foot.','head.','neck.']):
        print(n,'bone',[round(a,2) for a in d['head']],'weighted center',[round(a,2) for a in d['center']], 'range', [round(a,2) for a in d['min']], [round(a,2) for a in d['max']])
print('PAIR REGIONS')
for r in result['shared_body_regions']:
    print(r['child'],r['parent'],r['n'],[round(a,2) for a in r['center']])
