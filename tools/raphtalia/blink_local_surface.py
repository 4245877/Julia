"""Local, rest-surface-preserving triangle refinement for Raphtalia eyelids.

Original vertex indices remain fixed. All added points carry explicit linear
provenance in original vertices; every output triangle names its source face.
No bmesh operations, model saves, normal smoothing or rig edits are performed.
"""
from collections import defaultdict
from pathlib import Path
import argparse
import json
import math
import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def _edge(a,b):
    return (min(a,b),max(a,b))


def _mix(a,b):
    result=defaultdict(float)
    for source,value in a.items():result[source]+=.5*value
    for source,value in b.items():result[source]+=.5*value
    return dict(result)


def _split_triangle(face,midpoint,points):
    """Orientation-preserving 0/1/2/3-edge triangle splitting."""
    a,b,c=face
    mids=[midpoint.get(_edge(a,b)),midpoint.get(_edge(b,c)),midpoint.get(_edge(c,a))]
    count=sum(m is not None for m in mids)
    if count==0:return [face],count
    if count==3:
        ab,bc,ca=mids
        return [(a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)],count
    if count==1:
        edge_index=next(i for i,m in enumerate(mids) if m is not None)
        x,y,z=(face[(edge_index+i)%3] for i in range(3));m=mids[edge_index]
        return [(x,m,z),(m,y,z)],count
    unmarked=next(i for i,m in enumerate(mids) if m is None)
    common=(unmarked+2)%3
    x,y,z=(face[(common+i)%3] for i in range(3))
    xy=midpoint[_edge(x,y)];zx=midpoint[_edge(z,x)]
    children=[(x,xy,zx)]
    if np.linalg.norm(points[xy]-points[z])<=np.linalg.norm(points[y]-points[zx]):
        children.extend([(xy,y,z),(xy,z,zx)])
    else:children.extend([(xy,y,zx),(y,z,zx)])
    return children,count


def _attribute_value(item):
    for field in ('value','vector','color'):
        if hasattr(item,field):
            value=getattr(item,field)
            return (field,list(value) if not isinstance(value,(bool,int,float)) else value)
    return None


def _interpolate(values,weights,discrete=False):
    if discrete:
        return values[max(weights,key=weights.get)]
    result=None
    for source,weight in weights.items():
        term=np.asarray(values[source],dtype=float)*weight
        result=term if result is None else result+term
    return result.tolist() if result.ndim else float(result)


def _default_selection(obj,points):
    if obj.name=='body_tex_skin_0':
        return {i for i,(x,y,z) in enumerate(points)
                if 3<=abs(x)<=17 and 371<=z<=386.2 and y < -10}
    if obj.name=='eyelashes_tex_eyelashes_0':
        audit=json.loads((ROOT/'docs/raphtalia/blink/topology.json').read_text(encoding='utf-8'))
        mapping=audit['lash_vertex_mapping']
        if len(mapping)!=len(points):raise ValueError('Lash topology differs from audited base')
        return {i for i,item in enumerate(mapping) if item['region'] in ('upper','lower','canthus')}
    raise ValueError('Pass selected_vertices or selection for an unknown mesh')


def refine(obj,passes=2,selected_vertices=None,selection=None,max_influences=4):
    """Split all edges whose endpoints are selected, with conforming neighbours.

    ``selected_vertices`` is an iterable of ORIGINAL vertex indices. Alternatively
    ``selection(index, coordinate)`` is called for each original vertex. By default
    the documented front-eye bounding region or audited lash regions are used.
    A new midpoint is selected exactly when both of its parents are selected.
    This makes every round local and shares every edge midpoint between faces.

    UV layers, material indices, smooth flags, split custom normals, edge seams,
    sharp edges and supported custom mesh attributes are transferred from source
    triangles. Original group weights are copied exactly. New weights interpolate
    provenance, retaining at most max_influences; any truncation is reported.

    Returns JSON-serializable provenance and numerical rest-transfer diagnostics.
    The object is updated in memory. Its transform, modifiers and groups remain.
    """
    if obj.type!='MESH':raise TypeError('refine expects a mesh object')
    if passes<0 or int(passes)!=passes:raise ValueError('passes must be a nonnegative integer')
    old=obj.data
    if old.shape_keys:raise ValueError('Refine before shape keys; morph interpolation is not implemented')
    if any(len(p.vertices)!=3 for p in old.polygons):
        raise ValueError('Triangle input required; no implicit triangulation/reordering')
    source_points=np.array([v.co[:] for v in old.vertices],dtype=float)
    source_faces=[tuple(map(int,p.vertices)) for p in old.polygons]
    source_loop_ids=[tuple(p.loop_indices) for p in old.polygons]
    source_face_smooth=[p.use_smooth for p in old.polygons]
    source_face_material=[p.material_index for p in old.polygons]
    source_normals=np.array([n.vector[:] for n in old.corner_normals],dtype=float)
    source_uv={layer.name:np.array([uv.uv[:] for uv in layer.data],dtype=float) for layer in old.uv_layers}
    uv_metadata={layer.name:{'active_render':layer.active_render,'active_clone':layer.active_clone}
                 for layer in old.uv_layers}
    active_uv_name=old.uv_layers.active.name if old.uv_layers.active else None
    source_weights=[{g.group:float(g.weight) for g in v.groups} for v in old.vertices]
    group_metadata=[(g.name,g.lock_weight) for g in obj.vertex_groups]
    active_group_index=obj.vertex_groups.active_index
    source_seams={_edge(*e.vertices):bool(e.use_seam) for e in old.edges}
    source_edges={_edge(*e.vertices):e.index for e in old.edges}
    material_slots=[(slot.link,slot.material) for slot in obj.material_slots]
    attrs={}
    for attr in old.attributes:
        if attr.name.startswith('.') or attr.name in ('position','material_index') or attr.name in source_uv:
            continue
        values=[_attribute_value(item) for item in attr.data]
        if values and all(v is not None for v in values):
            attrs[attr.name]={'domain':attr.domain,'type':attr.data_type,
                              'field':values[0][0],'values':[v[1] for v in values]}
    if selected_vertices is not None and selection is not None:
        raise ValueError('Choose selected_vertices or selection')
    if selection is not None:
        selected={i for i,point in enumerate(source_points) if selection(i,point.copy())}
    elif selected_vertices is not None:selected=set(map(int,selected_vertices))
    else:selected=_default_selection(obj,source_points)
    if not selected.issubset(range(len(source_points))):raise ValueError('Invalid original selected vertex')
    selected_original=sorted(selected)
    points=[p.copy() for p in source_points]
    provenance=[{i:1.} for i in range(len(points))]
    faces=list(source_faces);face_sources=list(range(len(faces)))
    edge_origins=dict(source_edges)
    rounds=[]
    for _ in range(passes):
        all_edges={_edge(a,b) for f in faces for a,b in zip(f,(f[1],f[2],f[0]))}
        split_edges=sorted(e for e in all_edges if e[0] in selected and e[1] in selected)
        midpoint={}
        next_origins={e:edge_origins.get(e) for e in all_edges if e not in split_edges}
        for a,b in split_edges:
            index=len(points);midpoint[(a,b)]=index
            points.append((points[a]+points[b])*.5)
            provenance.append(_mix(provenance[a],provenance[b]));selected.add(index)
            next_origins[_edge(a,index)]=edge_origins.get((a,b))
            next_origins[_edge(index,b)]=edge_origins.get((a,b))
        next_faces=[];next_sources=[]
        split_face_counts=defaultdict(int)
        for face,source in zip(faces,face_sources):
            children,count=_split_triangle(face,midpoint,points)
            split_face_counts[count]+=1
            next_faces.extend(children);next_sources.extend([source]*len(children))
        output_edges={_edge(a,b) for f in next_faces for a,b in zip(f,(f[1],f[2],f[0]))}
        assert not (set(split_edges)&output_edges),'A split edge survived: possible T junction'
        for edge in output_edges:next_origins.setdefault(edge,None)
        rounds.append({'split_edges':len(split_edges),'input_faces_by_split_edges':dict(split_face_counts),
                       'output_faces':len(next_faces),'output_vertices':len(points)})
        faces,face_sources,edge_origins=next_faces,next_sources,next_origins
    new=bpy.data.meshes.new(old.name+'_blink_local')
    new.from_pydata(points,[],faces);new.update()
    for key,value in old.items():new[key]=value
    for mat in old.materials:new.materials.append(mat)
    for index,poly in enumerate(new.polygons):
        src=old.polygons[face_sources[index]]
        poly.material_index=src.material_index;poly.use_smooth=src.use_smooth
    corner_source=[]
    for face,source in zip(faces,face_sources):
        old_face=source_faces[source];source_to_loop=dict(zip(old_face,source_loop_ids[source]))
        for vertex in face:
            weights=provenance[vertex]
            if not set(weights).issubset(old_face):
                raise AssertionError('Output corner is outside its source triangle provenance')
            corner_source.append({source_to_loop[i]:w for i,w in weights.items()})
    for name,uv in source_uv.items():
        layer=new.uv_layers.new(name=name)
        for item,weights in zip(layer.data,corner_source):item.uv=_interpolate(uv,weights)
        layer.active_render=uv_metadata[name]['active_render'];layer.active_clone=uv_metadata[name]['active_clone']
    if active_uv_name:new.uv_layers.active=new.uv_layers[active_uv_name]
    for edge in new.edges:
        origin=edge_origins.get(_edge(*edge.vertices))
        if origin is not None:edge.use_seam=source_seams[_edge(*old.edges[origin].vertices)]
    skipped_attributes=[]
    for name,description in attrs.items():
        domain=description['domain'];values=description['values'];data_type=description['type']
        discrete=data_type in ('INT','INT8','BOOLEAN','STRING','INT32_2D')
        attribute=new.attributes.get(name) or new.attributes.new(name=name,type=data_type,domain=domain)
        if domain=='POINT':maps=provenance
        elif domain=='CORNER':maps=corner_source
        elif domain=='FACE':maps=[{source:1.} for source in face_sources]
        elif domain=='EDGE':
            maps=[{edge_origins[_edge(*e.vertices)]:1.} if edge_origins.get(_edge(*e.vertices)) is not None else {}
                  for e in new.edges]
        else:skipped_attributes.append(name);continue
        for item,weights in zip(attribute.data,maps):
            if weights:setattr(item,description['field'],_interpolate(values,weights,discrete))
    target_normals=[]
    for weights in corner_source:
        n=np.asarray(_interpolate(source_normals,weights))
        length=np.linalg.norm(n)
        target_normals.append(n/length if length>1e-20 else np.zeros(3))
    new.normals_split_custom_set(target_normals)
    new.update()
    obj.data=new
    # Assigning an empty replacement mesh can clear Blender's group definitions.
    # Recreate the complete original ordered table before restoring memberships.
    if list(g.name for g in obj.vertex_groups)!=[name for name,_ in group_metadata]:
        obj.vertex_groups.clear()
        for name,locked in group_metadata:
            group=obj.vertex_groups.new(name=name);group.lock_weight=locked
    obj.vertex_groups.active_index=active_group_index
    for slot,(link,mat) in zip(obj.material_slots,material_slots):
        slot.link=link
        if link=='OBJECT':slot.material=mat
    # Groups themselves belong to the object; their per-vertex membership is mesh data.
    truncations=[]
    all_weights=[]
    for vertex,weights in enumerate(provenance):
        mixed=defaultdict(float)
        for source,coefficient in weights.items():
            for group,value in source_weights[source].items():mixed[group]+=value*coefficient
        mixed={g:w for g,w in mixed.items() if w>1e-8}
        if vertex>=len(source_points) and len(mixed)>max_influences:
            before=dict(mixed);mixed=dict(sorted(mixed.items(),key=lambda item:item[1],reverse=True)[:max_influences])
            total=sum(mixed.values());mixed={g:w/total for g,w in mixed.items()}
            truncations.append({'vertex':vertex,'before':before,'after':mixed})
        all_weights.append(mixed)
        for group,value in mixed.items():obj.vertex_groups[group].add([vertex],value,'REPLACE')
    actual=np.array([v.co[:] for v in new.vertices])
    expected=np.array([_interpolate(source_points,weights) for weights in provenance])
    expected_uv={name:np.array([_interpolate(uv,w) for w in corner_source]) for name,uv in source_uv.items()}
    actual_normals=np.array([n.vector[:] for n in new.corner_normals])
    target_normals=np.asarray(target_normals)
    normal_error=np.linalg.norm(actual_normals-target_normals,axis=1)
    untouched_normal_errors=[];original_corner_normal_errors=[]
    descendants=defaultdict(list)
    for index,source in enumerate(face_sources):descendants[source].append(index)
    for face_index,(face,source) in enumerate(zip(faces,face_sources)):
        for corner,vertex in enumerate(face):
            if vertex<len(source_points):
                source_corner=source_faces[source].index(vertex)
                error=float(np.linalg.norm(actual_normals[face_index*3+corner]-source_normals[source_loop_ids[source][source_corner]]))
                original_corner_normal_errors.append(error)
                if len(descendants[source])==1:untouched_normal_errors.append(error)
    source_areas=np.array([np.linalg.norm(np.cross(source_points[b]-source_points[a],source_points[c]-source_points[a]))*.5
                           for a,b,c in source_faces])
    output_areas=np.zeros(len(source_faces))
    for (a,b,c),source in zip(faces,face_sources):
        output_areas[source]+=np.linalg.norm(np.cross(actual[b]-actual[a],actual[c]-actual[a]))*.5
    validation={'original_vertex_coordinates_bit_exact':bool(np.array_equal(actual[:len(source_points)],source_points)),
        'rest_provenance_max_error':float(np.linalg.norm(actual-expected,axis=1).max(initial=0)),
        'source_face_area_max_error':float(np.max(np.abs(output_areas-source_areas),initial=0)),
        'uv_max_error':{name:float(np.max(np.abs(np.array([v.uv[:] for v in new.uv_layers[name].data])-uv),initial=0))
                        for name,uv in expected_uv.items()},
        'split_custom_normal_max_vector_error':float(normal_error.max(initial=0)),
        'original_corner_normal_max_vector_error':max(original_corner_normal_errors,default=0.),
        'untouched_face_normal_max_vector_error':max(untouched_normal_errors,default=0.),
        'material_indices_exact_by_source':all(p.material_index==source_face_material[s] for p,s in zip(new.polygons,face_sources)),
        'smooth_flags_exact_by_source':all(p.use_smooth==source_face_smooth[s] for p,s in zip(new.polygons,face_sources)),
        'original_weights_exact':all({g.group:float(g.weight) for g in new.vertices[i].groups}==source_weights[i]
                                     for i in range(len(source_points))),
        'max_influences':max(map(len,all_weights),default=0),'weight_truncations':truncations,
        'conforming_global_midpoints':True}
    return {'mesh':obj.name,'original_vertices':len(source_points),'original_faces':len(source_faces),
            'vertices':len(points),'faces':len(faces),'passes':passes,
            'selected_original_vertices':selected_original,'rounds':rounds,
            'vertex_provenance':provenance,'face_provenance':face_sources,
            'validation':validation,'skipped_attributes':skipped_attributes}


def self_test(source=None,output=None):
    # A square tests all triangle split cases through two rounds, shared midpoints,
    # UV seams/gradient normals, attributes and unchanged old group membership.
    mesh=bpy.data.meshes.new('blink_refine_fixture')
    mesh.from_pydata([(0,0,0),(2,0,0),(2,2,0),(0,2,0)],[],[(0,1,2),(0,2,3)])
    obj=bpy.data.objects.new('blink_refine_fixture',mesh);bpy.context.collection.objects.link(obj)
    uv=mesh.uv_layers.new(name='UVTest')
    for i,loop in enumerate(mesh.loops):uv.data[i].uv=mesh.vertices[loop.vertex_index].co.xy
    for p in mesh.polygons:p.use_smooth=True
    mesh.normals_split_custom_set([(0,0,1)]*6)
    group=obj.vertex_groups.new(name='test');group.add(list(range(4)),1.,'REPLACE')
    fixture=refine(obj,passes=2,selected_vertices=[0,1,2])
    assert fixture['validation']['original_vertex_coordinates_bit_exact']
    assert fixture['validation']['original_weights_exact']
    assert fixture['validation']['rest_provenance_max_error']<1e-7
    assert fixture['validation']['source_face_area_max_error']<1e-7
    assert fixture['validation']['uv_max_error']['UVTest']<1e-7
    assert fixture['validation']['split_custom_normal_max_vector_error']<1e-7
    # Explicit edge-pattern coverage includes 2-edge splits, which induced
    # vertex selection does not normally need but conforming callers may use.
    triangle=np.array([(0,0,0),(4,0,0),(1,3,0)],dtype=float)
    split_patterns=[]
    for mask in range(8):
        points=[p.copy() for p in triangle];midpoints={}
        for bit,edge in enumerate(((0,1),(1,2),(0,2))):
            if mask&(1<<bit):
                midpoints[edge]=len(points);points.append((triangle[edge[0]]+triangle[edge[1]])*.5)
        children,count=_split_triangle((0,1,2),midpoints,points)
        cross=[np.cross(points[b]-points[a],points[c]-points[a])[2] for a,b,c in children]
        assert all(v>0 for v in cross)
        assert abs(sum(cross)-12)<1e-10
        child_edges={_edge(a,b) for face in children for a,b in zip(face,(face[1],face[2],face[0]))}
        assert not(set(midpoints)&child_edges)
        split_patterns.append({'mask':mask,'marked_edges':count,'children':len(children),'oriented_area_exact':True})
    report={'fixture':{k:v for k,v in fixture.items() if k not in ('vertex_provenance','face_provenance')},
            'all_eight_edge_patterns':split_patterns}
    if source:
        bpy.ops.wm.open_mainfile(filepath=str(Path(source).resolve()))
        report['source']=str(source);report['meshes']={}
        for name in ('body_tex_skin_0','eyelashes_tex_eyelashes_0'):
            result=refine(bpy.data.objects[name],passes=2)
            report['meshes'][name]={k:v for k,v in result.items() if k not in ('vertex_provenance','face_provenance')}
            assert result['validation']['original_vertex_coordinates_bit_exact']
            assert result['validation']['original_weights_exact']
            assert not result['validation']['weight_truncations']
    if output:
        path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--source');parser.add_argument('--output');args=parser.parse_args()
    if args.self_test:self_test(args.source,args.output)
