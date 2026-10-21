"""Read-only evidence for the blink design. Never saves/modifies a model."""
from pathlib import Path
from collections import Counter, defaultdict, deque
import json
import bpy
import numpy as np
from scipy.spatial import cKDTree
from blink_lash_mapping import classify_lashes

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'build/raphtalia_tools/blink/base/Raphtalia_prepared.blend'
OUT = ROOT / 'docs/raphtalia/blink/topology.json'
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')

def topology(obj):
    points = np.array([v.co[:] for v in obj.data.vertices])
    unique, inverse = np.unique(np.round(points, 4), axis=0, return_inverse=True)
    edges = Counter(); adj = defaultdict(set)
    for f in obj.data.polygons:
        ids = inverse[list(f.vertices)]
        for a,b in zip(ids, np.roll(ids,-1)):
            if a != b:
                edges[tuple(sorted((int(a),int(b))))] += 1
                adj[int(a)].add(int(b)); adj[int(b)].add(int(a))
    boundary = defaultdict(set)
    for (a,b),n in edges.items():
        if n == 1: boundary[a].add(b); boundary[b].add(a)
    loops=[]; seen=set()
    for start in boundary:
        if start in seen: continue
        cur=start; prev=None; path=[]
        while cur not in path:
            path.append(cur); seen.add(cur)
            todo=boundary[cur]-({prev} if prev is not None else set())
            if not todo:break
            prev,cur=cur,next(iter(todo))
        loops.append(path)
    return points,unique,inverse,adj,loops

def components(obj):
    adj=defaultdict(set)
    for e in obj.data.edges:
        a,b=e.vertices; adj[a].add(b);adj[b].add(a)
    seen=set(); result=[]
    for start in range(len(obj.data.vertices)):
        if start in seen:continue
        pending=[start];ids=[]
        while pending:
            cur=pending.pop()
            if cur in seen:continue
            seen.add(cur);ids.append(cur);pending.extend(adj[cur]-seen)
        result.append(ids)
    return sorted(result,key=len,reverse=True)

body=bpy.data.objects['body_tex_skin_0'];p,u,inv,adj,loops=topology(body)
result={'source':str(SOURCE.relative_to(ROOT)),'blender':bpy.app.version_string,
        'units':'Mesh local coordinates; original source units, approximately centimetres. World scale 0.01.',
        'eyes':{},'face_meshes':{},'lash_components':[],'rejected_method_analysis':[]}
for side,sign in [('L',1),('R',-1)]:
    loop=next(v for v in loops if u[v].mean(0)[0]*sign>0)
    left=int(np.argmin(u[loop,0]));loop=loop[left:]+loop[:left];right=int(np.argmax(u[loop,0]))
    paths=[loop[:right+1],loop[right:]+loop[:1]]
    paths=[sorted(x,key=lambda i:u[i,0]) for x in paths]
    paths.sort(key=lambda x:u[x,2].mean());lower,upper=paths
    dist={i:0 for i in loop};q=deque(loop)
    while q:
        i=q.popleft()
        if dist[i]>=5:continue
        for j in adj[i]:
            if j not in dist:dist[j]=dist[i]+1;q.append(j)
    def row(i):
        ids=np.flatnonzero(inv==i)
        return {'unique_id':int(i),'source_ids':ids.tolist(),'co':u[i].tolist(),
                'weights':[{body.vertex_groups[g.group].name:float(g.weight) for g in body.data.vertices[v].groups} for v in ids],
                'neighbours':sorted(adj[i])}
    result['eyes'][side]={'loop':[row(i) for i in loop],
        'upper_unique_ids':upper,'lower_unique_ids':lower,
        'geodesic_shells':{str(level):[row(i) for i,d in dist.items() if d==level] for level in range(1,6)}}

for name in ['body_tex_skin_0','body_tex_skin2_0','eye_tex_eyes_0','hightlight_hightlight_0','eyelashes_tex_eyelashes_0']:
    obj=bpy.data.objects[name]
    result['face_meshes'][name]={'vertices':len(obj.data.vertices),'faces':len(obj.data.polygons),
        'shape_keys':[k.name for k in obj.data.shape_keys.key_blocks] if obj.data.shape_keys else [],
        'modifiers':[{'type':m.type,'name':m.name} for m in obj.modifiers],
        'weight_groups':{vg.name:sum(any(g.group==vg.index and g.weight>1e-8 for g in v.groups) for v in obj.data.vertices) for vg in obj.vertex_groups if any(any(g.group==vg.index and g.weight>1e-8 for g in v.groups) for v in obj.data.vertices)}}

lashes=bpy.data.objects['eyelashes_tex_eyelashes_0'];lp=np.array([v.co[:] for v in lashes.data.vertices])
for comp in components(lashes):
    co=lp[comp];side='L' if co[:,0].mean()>0 else 'R'
    upper=result['eyes'][side]['upper_unique_ids'];lower=result['eyes'][side]['lower_unique_ids']
    du,iu=cKDTree(u[upper]).query(co);dl,il=cKDTree(u[lower]).query(co)
    result['lash_components'].append({'side':side,'count':len(comp),'indices':comp,
       'bounds':[co.min(0).tolist(),co.max(0).tolist()],'mean':co.mean(0).tolist(),
       'unique_positions':len(np.unique(np.round(co,4),axis=0)),
       'upper_nearer_vertices':int((du<dl).sum()),'lower_nearer_vertices':int((dl<du).sum()),
       'vertices':[{'id':int(v),'co':lp[v].tolist(),'upper_distance':float(a),'lower_distance':float(b)} for v,a,b in zip(comp,du,dl)]})

result['lash_vertex_mapping'] = classify_lashes(lashes.data,
    {s:u[result['eyes'][s]['upper_unique_ids']] for s in ('L','R')},
    {s:u[result['eyes'][s]['lower_unique_ids']] for s in ('L','R')})

result['controls']={'bones':[{'name':b.name,'parent':b.parent.name if b.parent else None,'deform':b.use_deform,
                            'head':list(b.head_local),'tail':list(b.tail_local)} for b in arm.data.bones if any(t in b.name.lower() for t in ['eye','brow','head','neck'])],
                   'drivers':[{'data_path':d.data_path,'array_index':d.array_index,'expression':d.driver.expression} for d in arm.animation_data.drivers] if arm.animation_data else []}
result['rejected_method_analysis']=[
 'Socket boundary has only 16 vertices. No separate anatomical upper/lower lid shell exists in base model.',
 'Rejected probe collapsed both upper/lower boundary paths toward 20% of opening height and solved 3D Laplacian over a hard distance<8 region.',
 'Laplacian has no surface, curvature, area or collision constraints. It pulls existing cheek/upper-lid triangle fans toward a line, compressing/inverting them and making a flat skin sheet rather than a curved lid.',
 'Y target was min(lower_y,upper_y)-sin(pi*t); this geometric projection is unrelated to actual cornea/sclera surface, so can fold the skin or intersect it at intermediate closure.',
 'All lash vertices below z384 used 3 nearest socket vertices, without separating upper rim, lower accents, duplicate seams or preserving local lash ribbon cross sections.',
 'Three-nearest interpolation can mix upper and lower boundary displacement; lower 8-vertex accents drift independently because their basis is not an explicit lower-lid path.',
 'The same interpolation moves eyelash ribbon width and tips inconsistently, shearing the original silhouette.',
 'Source has textured iris patches and sclera shell, not a complete spherical eyeball. Lid trajectories require design against the actual visible surfaces and gaze envelope.'
]
OUT.parent.mkdir(parents=True,exist_ok=True)
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('WROTE',OUT)
for side,data in result['eyes'].items():
    print(side,'UPPER',[(i,u[i].tolist()) for i in data['upper_unique_ids']])
    print(side,'LOWER',[(i,u[i].tolist()) for i in data['lower_unique_ids']])
    print('SHELLS',{k:len(v) for k,v in data['geodesic_shells'].items()})
for c in result['lash_components']:
    print('LASH',c['side'],c['count'],'MEAN',c['mean'],'NEAREST',c['upper_nearer_vertices'],c['lower_nearer_vertices'])
