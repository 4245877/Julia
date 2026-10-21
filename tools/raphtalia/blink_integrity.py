"""Read-only preservation audit for the blink candidate versus the 241/11 base.

This checks preservation and finite/normalized skinning data, not artistic quality.
The two .blend files are loaded sequentially and are never saved.
"""
import argparse
import hashlib
import json
from pathlib import Path
import bpy
import numpy as np
from mathutils import Matrix

ROOT = Path(__file__).resolve().parents[2]


def scalar(value):
    if value is None or isinstance(value,(str,int,float,bool)):
        return value
    if isinstance(value,bpy.types.ID):
        return {'id':value.name,'type':value.bl_rna.identifier}
    try:
        return [scalar(v) for v in value]
    except TypeError:
        return str(value)


def rna_properties(item):
    result={}
    for prop in item.bl_rna.properties:
        key=prop.identifier
        if key=='rna_type' or prop.type=='COLLECTION':continue
        try:result[key]=scalar(getattr(item,key))
        except (AttributeError,TypeError,ValueError):continue
    return result


def curve_data(curve):
    return {'data_path':curve.data_path,'array_index':curve.array_index,
            'extrapolation':curve.extrapolation,
            'keys':[{'co':list(k.co),'left':list(k.handle_left),'right':list(k.handle_right),
                     'left_type':k.handle_left_type,'right_type':k.handle_right_type,
                     'interpolation':k.interpolation,'easing':k.easing,
                     'amplitude':k.amplitude,'back':k.back,'period':k.period,
                     'type':k.type} for k in curve.keyframe_points],
            'samples':[list(s.co) for s in curve.sampled_points],
            'modifiers':[rna_properties(m) for m in curve.modifiers]}


def driver_data(curve):
    d=curve.driver
    return {'curve':curve_data(curve),'type':d.type,'expression':d.expression,
            'use_self':d.use_self,
            'variables':[{'name':v.name,'type':v.type,'targets':[rna_properties(t) for t in v.targets]}
                         for v in d.variables]}


def digest_array(values,dtype):
    a=np.asarray(values,dtype=dtype)
    return {'shape':list(a.shape),'sha256':hashlib.sha256(a.tobytes()).hexdigest()}


def neutral(arm):
    if arm.animation_data:
        arm.animation_data.action=None
        for t in arm.animation_data.nla_tracks:t.mute=True
    for p in arm.pose.bones:
        p.matrix_basis=Matrix.Identity(4)
        p.rotation_mode='QUATERNION'
    for key in ('Blink_L','Blink_R'):
        if key in arm:arm[key]=0.
    for o in bpy.data.objects:
        if o.type=='MESH' and o.data.shape_keys:
            for k in o.data.shape_keys.key_blocks:k.value=0.
    arm.update_tag()
    bpy.context.scene.frame_set(1)
    bpy.context.view_layer.update()


def mesh_geometry(obj):
    mesh=obj.data
    return {'vertices':digest_array([v.co[:] for v in mesh.vertices],'<f8'),
            'vertex_normals':digest_array([v.normal[:] for v in mesh.vertices],'<f8'),
            'corner_normals':digest_array([v.vector[:] for v in mesh.corner_normals],'<f8'),
            'edges':digest_array([e.vertices[:] for e in mesh.edges],'<i8'),
            'loop_vertex_indices':digest_array([l.vertex_index for l in mesh.loops],'<i8'),
            'polygons':[{'vertices':list(p.vertices),'material_index':p.material_index,
                         'smooth':p.use_smooth} for p in mesh.polygons],
            'uv':{layer.name:digest_array([v.uv[:] for v in layer.data],'<f8') for layer in mesh.uv_layers},
            'material_slots':[slot.material.name if slot.material else None for slot in obj.material_slots],
            'matrix_world':[list(row) for row in obj.matrix_world],
            'modifiers':[rna_properties(m) for m in obj.modifiers],
            'shape_keys':{k.name:digest_array([v.co[:] for v in k.data],'<f8') for k in mesh.shape_keys.key_blocks}
                         if mesh.shape_keys else {}}


def geometry_details(obj):
    mesh=obj.data
    return {'points':np.array([v.co[:] for v in mesh.vertices]),
            'faces':[tuple(map(int,p.vertices)) for p in mesh.polygons],
            'corner_normals':np.array([n.vector[:] for n in mesh.corner_normals]),
            'uv':{layer.name:np.array([v.uv[:] for v in layer.data]) for layer in mesh.uv_layers},
            'materials':[p.material_index for p in mesh.polygons],
            'smooth':[p.use_smooth for p in mesh.polygons]}


def snapshot(path):
    bpy.ops.wm.open_mainfile(filepath=str(path))
    arm=next(o for o in bpy.data.objects if o.type=='ARMATURE')
    result={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'bones':{b.name:{'parent':b.parent.name if b.parent else None,'deform':b.use_deform,
                      'matrix_local':[list(row) for row in b.matrix_local],
                      'head':list(b.head_local),'tail':list(b.tail_local),
                      'inherit_scale':b.inherit_scale,'use_inherit_rotation':b.use_inherit_rotation,
                      'use_local_location':b.use_local_location}
                     for b in arm.data.bones},
            'pose_constraints':{p.name:[rna_properties(c) for c in p.constraints] for p in arm.pose.bones},
            'actions':{a.name:{'frame_range':list(a.frame_range),'curves':[curve_data(f) for f in a.fcurves]}
                       for a in bpy.data.actions},'drivers':{},'meshes':{},'weights':{},'materials':{},
            'images':{},'object_constraints':{},'mesh_details':{}}
    owners=[*bpy.data.objects,*bpy.data.materials,*bpy.data.node_groups]
    for owner in owners:
        if owner.animation_data and owner.animation_data.drivers:
            result['drivers'][owner.bl_rna.identifier+':'+owner.name]={
                f.data_path+':'+str(f.array_index):driver_data(f) for f in owner.animation_data.drivers}
    for obj in bpy.data.objects:
        result['object_constraints'][obj.name]=[rna_properties(c) for c in obj.constraints]
        if obj.type!='MESH':continue
        result['meshes'][obj.name]=mesh_geometry(obj)
        result['mesh_details'][obj.name]=geometry_details(obj)
        result['weights'][obj.name]=[{obj.vertex_groups[g.group].name:float(g.weight) for g in v.groups
                                     if g.weight>1e-8} for v in obj.data.vertices]
    for mat in bpy.data.materials:
        result['materials'][mat.name]={'diffuse':list(mat.diffuse_color),
            'use_nodes':mat.use_nodes,'blend_method':mat.blend_method,
            'nodes':{n.name:{'properties':rna_properties(n),
                           'inputs':{s.name:scalar(s.default_value) for s in n.inputs if hasattr(s,'default_value')}}
                     for n in mat.node_tree.nodes} if mat.node_tree else {},
            'links':[(l.from_node.name,l.from_socket.name,l.to_node.name,l.to_socket.name)
                     for l in mat.node_tree.links] if mat.node_tree else []}
    for img in bpy.data.images:
        packed=bytes(img.packed_file.data) if img.packed_file else None
        result['images'][img.name]={'filepath':img.filepath,'source':img.source,'size':list(img.size),
             'colorspace':img.colorspace_settings.name,
             'packed_sha256':hashlib.sha256(packed).hexdigest() if packed is not None else None}
    neutral(arm)
    depsgraph=bpy.context.evaluated_depsgraph_get()
    result['neutral_vertices']={}
    for obj in bpy.data.objects:
        if obj.type!='MESH':continue
        evaluated=obj.evaluated_get(depsgraph)
        result['neutral_vertices'][obj.name]=np.array([v.co[:] for v in evaluated.data.vertices],dtype=float)
    return result


def compare(base,candidate,patch=None):
    failures=[]
    checks={}
    def check(name,value,detail=None):
        checks[name]={'pass':bool(value)}
        if detail is not None:checks[name]['details']=detail
        if not value:failures.append(name)
    surfaces=(patch or {}).get('surface_refinement',{})
    def preserved_map(name,skip=()):
        missing=sorted(set(base[name])-set(candidate[name]))
        changed=sorted(k for k in base[name] if k not in skip and k in candidate[name] and base[name][k]!=candidate[name][k])
        check('preserve_'+name,not missing and not changed,{'missing':missing,'changed':changed})
    for category in ('bones','pose_constraints','actions','materials','images','object_constraints'):
        preserved_map(category)
    preserved_map('meshes',skip=surfaces)
    surface_validation={};surface_errors=[]
    for name,refinement in surfaces.items():
        if name not in ('body_tex_skin_0','eyelashes_tex_eyelashes_0'):
            surface_errors.append({'mesh':name,'reason':'refinement outside authorized eye meshes'});continue
        old=base['mesh_details'][name];new=candidate['mesh_details'][name]
        vp=[{int(k):float(v) for k,v in row.items()} for row in refinement['vertex_provenance']]
        fp=refinement['face_provenance']
        if len(vp)!=len(new['points']) or len(fp)!=len(new['faces']):
            surface_errors.append({'mesh':name,'reason':'provenance count mismatch'});continue
        expected=np.array([sum(old['points'][i]*w for i,w in weights.items()) for weights in vp])
        row={'original_coordinates_bit_exact':bool(np.array_equal(new['points'][:len(old['points'])],old['points'])),
             'provenance_max_distance':float(np.linalg.norm(new['points']-expected,axis=1).max(initial=0)),
             'max_provenance_sum_error':max(abs(sum(w.values())-1) for w in vp),
             'old_vertex_indices_identity':all(vp[i]=={i:1.} for i in range(len(old['points']))),
             'invalid_faces':[],'uv_max_error':{},'material_indices_exact':True,'smooth_flags_exact':True}
        source_areas=np.array([np.linalg.norm(np.cross(old['points'][b]-old['points'][a],old['points'][c]-old['points'][a]))*.5 for a,b,c in old['faces']])
        areas=np.zeros(len(old['faces']));source_normals=old['corner_normals'];expected_normals=[]
        expected_uv={uv:[] for uv in old['uv']}
        for face_index,(face,source) in enumerate(zip(new['faces'],fp)):
            old_face=old['faces'][source];loop_map={v:source*3+i for i,v in enumerate(old_face)}
            a,b,c=new['points'][list(face)]
            areas[source]+=np.linalg.norm(np.cross(b-a,c-a))*.5
            row['material_indices_exact'] &= new['materials'][face_index]==old['materials'][source]
            row['smooth_flags_exact'] &= new['smooth'][face_index]==old['smooth'][source]
            for vertex in face:
                weights=vp[vertex]
                if not set(weights).issubset(old_face):
                    row['invalid_faces'].append(face_index);continue
                normal=sum(source_normals[loop_map[i]]*w for i,w in weights.items());normal/=max(np.linalg.norm(normal),1e-20)
                expected_normals.append(normal)
                for uv in old['uv']:
                    expected_uv[uv].append(sum(old['uv'][uv][loop_map[i]]*w for i,w in weights.items()))
        row['source_face_area_max_error']=float(np.max(np.abs(areas-source_areas),initial=0))
        for uv,expected_uv_values in expected_uv.items():
            if uv not in new['uv'] or len(expected_uv_values)!=len(new['uv'][uv]):row['uv_max_error'][uv]=float('inf')
            else:row['uv_max_error'][uv]=float(np.max(np.abs(np.array(expected_uv_values)-new['uv'][uv]),initial=0))
        smoothing=(patch or {}).get('normal_smoothing',{})
        allowed_normals=set(smoothing.get('loop_ids',[])) if smoothing.get('mesh')==name else set()
        normal_diff=np.linalg.norm(new['corner_normals']-np.asarray(expected_normals),axis=1)
        outside=np.array([i for i in range(len(normal_diff)) if i not in allowed_normals],dtype=int)
        row['documented_smoothed_loops']=len(allowed_normals)
        row['normal_max_error_outside_documented_smoothing']=float(normal_diff[outside].max(initial=0))
        row['normal_max_error_all']=float(normal_diff.max(initial=0))
        object_fields=('matrix_world','material_slots','modifiers','shape_keys')
        row['object_fields_exact']=all(base['meshes'][name][k]==candidate['meshes'][name][k] for k in object_fields)
        row['pass']=(row['original_coordinates_bit_exact'] and row['old_vertex_indices_identity'] and
                     row['provenance_max_distance']<1e-4 and row['max_provenance_sum_error']<1e-8 and
                     not row['invalid_faces'] and row['source_face_area_max_error']<1e-3 and
                     all(error<1e-6 for error in row['uv_max_error'].values()) and
                     row['normal_max_error_outside_documented_smoothing']<1e-3 and
                     row['material_indices_exact'] and row['smooth_flags_exact'] and row['object_fields_exact'])
        surface_validation[name]=row
        if not row['pass']:surface_errors.append({'mesh':name,'reason':'refinement transfer validation failed'})
    check('documented_surface_refinement',not surface_errors,{'meshes':surface_validation,'errors':surface_errors})
    missing_drivers=[];changed_drivers=[]
    for owner,drivers in base['drivers'].items():
        for key,value in drivers.items():
            new=candidate['drivers'].get(owner,{}).get(key)
            if new is None:missing_drivers.append(owner+'/'+key)
            elif new!=value:changed_drivers.append(owner+'/'+key)
    check('preserve_existing_drivers',not missing_drivers and not changed_drivers,
          {'missing':missing_drivers,'changed':changed_drivers})
    topology_path=ROOT/'docs/raphtalia/blink/topology.json'
    topology=json.loads(topology_path.read_text(encoding='utf-8'))
    orbital_skin_ids=set()
    for eye in topology['eyes'].values():
        for row in eye['loop']:
            orbital_skin_ids.update(row['source_ids'])
        for level,rows in eye['geodesic_shells'].items():
            if int(level)>5:continue
            for row in rows:orbital_skin_ids.update(row['source_ids'])
    weight_changes={};invalid=[];outside=[];max_influences=0;max_sum_error=0.
    for name,old in base['weights'].items():
        new=candidate['weights'].get(name,[])
        expected_weights=old
        if name in surfaces:
            expected_weights=[]
            for provenance in surfaces[name]['vertex_provenance']:
                weights={}
                for source,coefficient in provenance.items():
                    for group,value in old[int(source)].items():weights[group]=weights.get(group,0)+value*coefficient
                expected_weights.append(weights)
        if len(new)!=len(expected_weights):invalid.append({'mesh':name,'reason':'vertex count changed without matching provenance'});continue
        changed=[i for i,(a,b) in enumerate(zip(expected_weights,new)) if a!=b]
        if changed:weight_changes[name]=changed
        if changed and name not in ('body_tex_skin_0','eyelashes_tex_eyelashes_0'):
            outside.append({'mesh':name,'vertices':changed})
        if name=='body_tex_skin_0':
            outside_orbit=[]
            for vertex in changed:
                sources=(set(map(int,surfaces[name]['vertex_provenance'][vertex])) if name in surfaces else {vertex})
                if not sources.issubset(orbital_skin_ids):outside_orbit.append(vertex)
            if outside_orbit:
                outside.append({'mesh':name,'vertices':outside_orbit,
                                'reason':'outside original socket and five geodesic skin rows'})
        for i,weights in enumerate(new):
            values=list(weights.values());max_influences=max(max_influences,len(values))
            error=abs(sum(values)-1);max_sum_error=max(max_sum_error,error)
            if len(values)>4 or error>1e-5 or not values or any(not np.isfinite(v) or v<0 for v in values):
                invalid.append({'mesh':name,'vertex':i,'weights':weights})
            if any(b not in candidate['bones'] for b in weights):
                invalid.append({'mesh':name,'vertex':i,'reason':'unknown bone'})
            if i in changed:
                if any(not (b=='head.x_018' or b.startswith('lid.')) for b in weights):
                    outside.append({'mesh':name,'vertex':i,'reason':'changed unrelated influence'})
                if expected_weights[i]!={'head.x_018':1.0}:
                    outside.append({'mesh':name,'vertex':i,'reason':'changed non-head source weight'})
    check('weights_normalized_finite_max4',not invalid,{'max_influences':max_influences,
          'max_sum_error':max_sum_error,'invalid':invalid})
    check('weights_only_head_skin_and_lashes',not outside,{'violations':outside,
          'changed_vertex_counts':{name:len(ids) for name,ids in weight_changes.items()}})
    neutral=[]
    for name,old in base['neutral_vertices'].items():
        new=candidate['neutral_vertices'].get(name)
        expected=old
        if name in surfaces:
            expected=np.array([sum(old[int(i)]*w for i,w in weights.items()) for weights in surfaces[name]['vertex_provenance']])
        if new is None or new.shape!=expected.shape:neutral.append({'mesh':name,'exact':False,'shape_changed':True});continue
        delta=np.linalg.norm(new-expected,axis=1)
        neutral.append({'mesh':name,'exact':bool(np.array_equal(expected,new)),
                        'max_local_distance':float(delta.max()),'different_vertices':int((delta>0).sum())})
    check('neutral_evaluation_bit_exact',all(row['exact'] for row in neutral),neutral)
    check('neutral_evaluation_within_1e_4_source_unit',all(row.get('max_local_distance',float('inf'))<=1e-4 for row in neutral))
    return {'pass':not failures,'checks':checks,'failed_checks':failures,
            'preservation_with_float_tolerance_pass':not [f for f in failures if f!='neutral_evaluation_bit_exact'],
            'base_bones':len(base['bones']),'candidate_bones':len(candidate['bones']),
            'added_bones':sorted(set(candidate['bones'])-set(base['bones'])),
            'base_actions':sorted(base['actions']),'added_actions':sorted(set(candidate['actions'])-set(base['actions'])),
            'weight_changes':weight_changes,
            'limitations':'Read-only integrity and numerical skinning checks. These do not certify naturalness, intersections, eyelash appearance, export or runtime playback.'}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--base',default='build/raphtalia_tools/blink/base/Raphtalia_prepared.blend')
    parser.add_argument('--candidate',default='build/raphtalia_tools/blink/candidate.blend')
    parser.add_argument('--output',default='docs/raphtalia/blink/integrity.json')
    parser.add_argument('--patch',help='Intentional surface provenance; defaults to candidate.with_suffix(.patch.json)')
    args=parser.parse_args()
    base_path=(ROOT/args.base).resolve();candidate_path=(ROOT/args.candidate).resolve()
    patch_path=(ROOT/args.patch).resolve() if args.patch else candidate_path.with_suffix('.patch.json')
    patch=json.loads(patch_path.read_text(encoding='utf-8')) if patch_path.is_file() else {}
    base=snapshot(base_path);candidate=snapshot(candidate_path);report=compare(base,candidate,patch)
    report.update({'base':str(base_path),'candidate':str(candidate_path),
                   'base_sha256':base['sha256'],'candidate_sha256':candidate['sha256'],
                   'blender':bpy.app.version_string})
    out=(ROOT/args.output).resolve();out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    summary={k:report[k] for k in ('pass','preservation_with_float_tolerance_pass','failed_checks',
             'base_bones','candidate_bones','base_sha256','candidate_sha256')}
    summary['added_bones_count']=len(report['added_bones']);summary['added_actions']=report['added_actions']
    print(json.dumps(summary,ensure_ascii=False))
    for name,check in report['checks'].items():print(name,check['pass'])


if __name__=='__main__':main()
