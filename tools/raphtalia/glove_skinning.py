"""Bounded glove-hand weight transfer; geometry and source weights are untouched.

Call only on the prepared meshes after their coordinates share armature space.
The function changes selected glove vertex weights in memory and returns a patch.
"""
import numpy as np
from mathutils.bvhtree import BVHTree

def rigidify_wrist_cuffs(meshes):
    """Remove tiny skin-weight residue from the two rigid wrist bracelets."""
    cuff=next(o for o in meshes if o.name=='cuff.001_tex_cuff_0')
    patch=[]
    for v in cuff.data.vertices:
        name='forearm_twist.l_0176' if v.co.x>0 else 'forearm_twist.r_0135'
        old={cuff.vertex_groups[g.group].name:float(g.weight) for g in v.groups if g.weight>1e-8}
        if len(old)==1 and old.get(name,0)>1-1e-7: continue
        for g in list(v.groups): cuff.vertex_groups[g.group].remove([v.index])
        group=cuff.vertex_groups.get(name) or cuff.vertex_groups.new(name=name)
        group.add([v.index],1.0,'REPLACE')
        patch.append({'vertex':v.index,'old_weights':old,'weights':{name:1.0}})
    return {'mesh':cuff.name,'vertices_changed':len(patch),
      'method':'Rigid wrist bracelets: each side follows the forearm twist helper that already carried >99% of total source influence; remove incidental hand/forearm residue.',
      'patch':patch}

def transfer_glove_hand_weights(meshes,max_distance_cm=1.5,smooth_iterations=16):
    body=next(o for o in meshes if o.name=='body_tex_skin_0')
    glove=next(o for o in meshes if o.name=='long_gloves_tex_glove_0')
    body.data.calc_loop_triangles()
    tris=[tuple(t.vertices) for t in body.data.loop_triangles]
    bvh=BVHTree.FromPolygons([v.co for v in body.data.vertices],tris,all_triangles=True)
    weights=[{body.vertex_groups[g.group].name:float(g.weight) for g in v.groups if g.weight>1e-8} for v in body.data.vertices]
    patch=[];skipped=[]
    for v in glove.data.vertices:
        old={glove.vertex_groups[g.group].name:float(g.weight) for g in v.groups if g.weight>1e-8}
        hand_weight=sum(w for n,w in old.items() if n.startswith(('hand.','c_index','c_middle','c_ring','c_pinky','c_thumb')))
        if hand_weight<.5: continue
        nearest,normal,index,distance=bvh.find_nearest(v.co)
        if index is None or distance>max_distance_cm:
            skipped.append({'vertex':v.index,'distance_cm':distance});continue
        ids=tris[index];p=[np.array(body.data.vertices[i].co,dtype=float) for i in ids]
        x=np.array(nearest,dtype=float)
        solution=np.linalg.lstsq(np.stack((p[1]-p[0],p[2]-p[0]),axis=1),x-p[0],rcond=None)[0]
        bary=np.maximum([1-solution.sum(),solution[0],solution[1]],0);bary/=bary.sum()
        w={}
        for i,f in zip(ids,bary):
            for n,a in weights[i].items(): w[n]=w.get(n,0)+float(f*a)
        w=dict(sorted(w.items(),key=lambda kv:kv[1],reverse=True)[:4]);total=sum(w.values())
        if total<=1e-8: continue
        w={n:a/total for n,a in w.items() if a/total>1e-8}
        for g in list(v.groups): glove.vertex_groups[g.group].remove([v.index])
        for n,a in w.items():
            group=glove.vertex_groups.get(n) or glove.vertex_groups.new(name=n)
            group.add([v.index],a,'REPLACE')
        patch.append({'vertex':v.index,'distance_cm':float(distance),'old_weights':old,'weights':w})
    # The glove has very short connected webbing edges whose endpoints were
    # assigned to separate rigid fingers. Body transfer alone still retains a
    # steep nearest-surface gradient. Diffuse only along actual glove edges,
    # prioritizing those short edges; do not merge nearby disconnected fingers.
    selected={x['vertex'] for x in patch}
    names=sorted({n for x in patch for n in x['weights']})
    ni={n:i for i,n in enumerate(names)}
    values=np.zeros((len(glove.data.vertices),len(names)))
    for x in patch:
        for n,w in x['weights'].items(): values[x['vertex'],ni[n]]=w
    edges=np.array([tuple(e.vertices) for e in glove.data.edges if all(int(i) in selected for i in e.vertices)],dtype=int)
    if len(edges):
        coords=np.array([v.co[:] for v in glove.data.vertices])
        length=np.linalg.norm(coords[edges[:,1]]-coords[edges[:,0]],axis=1)
        strength=np.exp(-(length/1.0)**2)/np.maximum(length,.1)**2
        ids=np.concatenate((edges[:,0],edges[:,1]));others=np.concatenate((edges[:,1],edges[:,0]));edgew=np.tile(strength,2)
        denom=np.zeros(len(glove.data.vertices));np.add.at(denom,ids,edgew)
        valid=denom>1e-12
        for iteration in range(smooth_iterations):
            total=np.zeros_like(values);np.add.at(total,ids,values[others]*edgew[:,None])
            values[valid]=.5*values[valid]+.5*total[valid]/denom[valid,None]
        for x in patch:
            i=x['vertex'];row=values[i];top=np.argsort(row)[-4:];total=float(row[top].sum())
            w={names[j]:float(row[j]/total) for j in top if row[j]/total>1e-8}
            for g in list(glove.data.vertices[i].groups): glove.vertex_groups[g.group].remove([i])
            for n,a in w.items():
                group=glove.vertex_groups.get(n) or glove.vertex_groups.new(name=n)
                group.add([i],a,'REPLACE')
            x['weights']=w
    return {'mesh':glove.name,'body':body.name,'max_distance_cm':max_distance_cm,'smooth_iterations':smooth_iterations,
       'method':'nearest body triangle barycentric skin-weight interpolation; selected only vertices with >=50% existing hand/finger influence; inverse-length-weighted smoothing along actual glove edges; maximum four normalized influences',
       'vertices_changed':len(patch),'vertices_skipped':skipped,'patch':patch}
