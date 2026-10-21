"""Read-only geometry evidence for canthus artefacts in a blink candidate."""
import argparse
import hashlib
import json
from pathlib import Path
from collections import Counter
import bpy
import numpy as np
from collections import defaultdict
from mathutils import Matrix
from mathutils.bvhtree import BVHTree

ROOT=Path(__file__).resolve().parents[2]


def segment_triangle(a,b,triangle,eps=1e-8):
    e1=triangle[1]-triangle[0];e2=triangle[2]-triangle[0];d=b-a
    h=np.cross(d,e2);det=float(e1@h)
    if abs(det)<eps:return None
    inv=1/det;s=a-triangle[0];u=float(s@h)*inv
    if u<=eps or u>=1-eps:return None
    q=np.cross(s,e1);v=float(d@q)*inv
    if v<=eps or u+v>=1-eps:return None
    t=float(e2@q)*inv
    if t<=eps or t>=1-eps:return None
    return (a+t*d).tolist()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--blend',default='build/raphtalia_tools/blink/candidate.blend')
    parser.add_argument('--output',default='build/raphtalia_tools/blink/corner_audit.json')
    parser.add_argument('--dense',action='store_true',help='Sample 0,10,...,100 percent')
    parser.add_argument('--winks',action='store_true',help='Also sample independent left/right closure')
    args=parser.parse_args()
    source=(ROOT/args.blend).resolve();source_sha256=hashlib.sha256(source.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(source))
    arm=next(o for o in bpy.data.objects if o.type=='ARMATURE')
    arm.animation_data.action=None
    for track in arm.animation_data.nla_tracks:track.mute=True
    for bone in arm.pose.bones:bone.matrix_basis=Matrix.Identity(4)
    obj=bpy.data.objects['body_tex_skin_0'];mesh=obj.data
    p=np.array([v.co[:] for v in mesh.vertices]);faces=np.array([f.vertices[:] for f in mesh.polygons],dtype=int)
    influences=np.array([sum(g.weight for g in v.groups if obj.vertex_groups[g.group].name.startswith('lid.')) for v in mesh.vertices])
    def face_region(face_index):
        weights=Counter()
        for vertex in faces[face_index]:
            for g in mesh.vertices[vertex].groups:
                name=obj.vertex_groups[g.group].name
                if name.startswith('lid.'):weights[name.split('.')[1]]+=g.weight
        return weights.most_common(1)[0][0] if weights else 'fixed'
    normals=np.cross(p[faces[:,1]]-p[faces[:,0]],p[faces[:,2]]-p[faces[:,0]])
    area=np.linalg.norm(normals,axis=1)*.5
    touched=(influences[faces]>1e-7).any(1)
    topology=json.loads((ROOT/'docs/raphtalia/blink/topology.json').read_text())
    corners=np.array([e['loop'][0]['co'] for e in topology['eyes'].values()])
    corner_coords=[]
    for side,eye in topology['eyes'].items():
        rows={v['unique_id']:v['co'] for v in eye['loop']}
        corner_coords.extend([(side+'.'+str(k),np.array(rows[eye['upper_unique_ids'][k]])) for k in (0,-1)])
    distance=np.min(np.stack([np.linalg.norm(p[:,[0,2]]-co[[0,2]],axis=1) for _,co in corner_coords]),axis=0)
    near=(distance[faces].min(1)<2.5)&touched
    _,weld=np.unique(np.round(p,4),axis=0,return_inverse=True)
    edge_faces=defaultdict(list)
    for index,face in enumerate(faces):
        ids=weld[face]
        for a,b in zip(ids,np.roll(ids,-1)):
            if a!=b:edge_faces[tuple(sorted((int(a),int(b))))].append(index)
    pairs=np.array([v for v in edge_faces.values() if len(v)==2 and near[v].any()],dtype=int)
    rest_unit=normals/np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-30)
    rest_adjacent_cos=np.sum(rest_unit[pairs[:,0]]*rest_unit[pairs[:,1]],axis=1)
    evaluated=lambda:np.array([v.co[:] for v in obj.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.vertices])
    result={'source':str(source),'source_sha256':source_sha256,
            'scope':'Numerical geometry evidence, not an artistic-quality certificate.',
            'corners':{name:co.tolist() for name,co in corner_coords},'poses':{}}
    levels=np.linspace(0,1,11).tolist() if args.dense else (0,.25,.5,.75,1)
    cases=[(str(blink),blink,blink) for blink in levels]
    if args.winks:
        cases.extend((f'left_{blink:.1f}',blink,0) for blink in levels)
        cases.extend((f'right_{blink:.1f}',0,blink) for blink in levels)
    for case,left,right in cases:
        arm['Blink_L']=left;arm['Blink_R']=right;arm.update_tag();bpy.context.scene.frame_set(1);bpy.context.view_layer.update()
        q=evaluated()
        n=np.cross(q[faces[:,1]]-q[faces[:,0]],q[faces[:,2]]-q[faces[:,0]])
        areas=np.linalg.norm(n,axis=1)*.5
        projected_valid=(np.abs(normals[:,1])>1e-5)&touched
        flips=np.flatnonzero(projected_valid&(normals[:,1]*n[:,1]<0))
        # Projection sign is unstable when a surface turns edge-on. Retain the
        # raw count, but distinguish it from a substantial projected reversal.
        projected_magnitude=np.abs(n[:,1])/np.maximum(np.abs(normals[:,1]),1e-30)
        substantial_flips=flips[(np.abs(n[flips,1])>1e-4)&(projected_magnitude[flips]>.01)]
        degenerate=np.flatnonzero(touched&(area>1e-6)&(areas<1e-7))
        ratios=areas/np.maximum(area,1e-30)
        cosine=np.sum(n*normals,axis=1)/np.maximum(np.linalg.norm(n,axis=1)*np.linalg.norm(normals,axis=1),1e-30)
        unit=n/np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-30)
        adjacent_cos=np.sum(unit[pairs[:,0]]*unit[pairs[:,1]],axis=1)
        new_hinges=np.flatnonzero((rest_adjacent_cos>.5)&(adjacent_cos<-.5))
        def face_row(i):
            return {'face':int(i),'vertices':faces[i].tolist(),'rest':p[faces[i]].tolist(),
                    'posed':q[faces[i]].tolist(),'rest_normal':normals[i].tolist(),'posed_normal':n[i].tolist(),
                    'area_ratio':float(ratios[i]),'normal_cosine':float(cosine[i]),
                    'region':face_region(i),
                    'lid_influences':influences[faces[i]].tolist(),
                    'groups':[{obj.vertex_groups[g.group].name:float(g.weight) for g in mesh.vertices[v].groups} for v in faces[i]]}
        ids=np.flatnonzero(near)
        worst=ids[np.argsort(ratios[ids])[:8]]
        # BVH candidates + strict edge/interior crossing. Welded contacts and
        # coplanar/edge contacts are excluded, because closure joins two rims.
        local_faces=np.flatnonzero(near)
        local_triangles=faces[local_faces]
        tree=BVHTree.FromPolygons(q.tolist(),local_triangles.tolist(),all_triangles=True,epsilon=0.)
        candidates=sorted({tuple(sorted(pair)) for pair in tree.overlap(tree) if pair[0]!=pair[1]})
        intersections=[]
        for a,b in candidates:
            ia=local_triangles[a];ib=local_triangles[b]
            if set(map(tuple,np.round(p[ia],4)))&set(map(tuple,np.round(p[ib],4))):continue
            ta=q[ia];tb=q[ib];point=None
            for triangle,other in ((ta,tb),(tb,ta)):
                for start,end in zip(triangle,np.roll(triangle,-1,axis=0)):
                    point=segment_triangle(start,end,other)
                    if point is not None:break
                if point is not None:break
            if point is not None:intersections.append({'faces':[int(local_faces[a]),int(local_faces[b])],'point':point,
                'regions':[face_region(int(local_faces[a])),face_region(int(local_faces[b]))]})
        result['poses'][case]={'Blink_L':float(left),'Blink_R':float(right),
            'touched_faces':int(touched.sum()),'near_corner_faces':int(near.sum()),
            'front_projection_flips':len(flips),'near_corner_projection_flips':int(near[flips].sum()),
            'substantial_front_projection_reversals':len(substantial_flips),
            'projection_reversal_definition':'Raw signs retained; substantial additionally requires abs posed cross-y>1e-4 and >1% of rest abs cross-y. A 2D sign reversal alone does not certify a 3D fold.',
            'degenerate_triangles':degenerate.tolist(),'min_touched_area_ratio':float(ratios[touched].min()),
            'near_corner_min_normal_cosine':float(cosine[near].min()),
            'flip_faces':[face_row(i) for i in flips[:5]],'near_corner_smallest_faces':[face_row(i) for i in worst[:5]],
            'strict_nonadjacent_intersections':intersections,'bvh_candidate_pairs':len(candidates)}
        result['poses'][case]['new_adjacent_folds_over_120_degrees']=[
            {'faces':pairs[i].tolist(),'rest_angle_degrees':float(np.degrees(np.arccos(np.clip(rest_adjacent_cos[i],-1,1)))),
             'posed_angle_degrees':float(np.degrees(np.arccos(np.clip(adjacent_cos[i],-1,1))))}
            for i in new_hinges]
    out=(ROOT/args.output).resolve();out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    summary={'source_sha256':source_sha256,'samples':len(cases),
             'max_new_adjacent_folds_over_120_degrees':max(len(p['new_adjacent_folds_over_120_degrees']) for p in result['poses'].values()),
             'max_degenerate_triangles':max(len(p['degenerate_triangles']) for p in result['poses'].values()),
             'max_raw_projection_sign_changes':max(p['front_projection_flips'] for p in result['poses'].values()),
             'max_substantial_projection_reversals':max(p['substantial_front_projection_reversals'] for p in result['poses'].values()),
             'min_touched_area_ratio':min(p['min_touched_area_ratio'] for p in result['poses'].values()),
             'states_with_strict_intersections':{k:len(p['strict_nonadjacent_intersections']) for k,p in result['poses'].items() if p['strict_nonadjacent_intersections']}}
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
