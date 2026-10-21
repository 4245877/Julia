"""Local, exportable clothing repairs for the 241-bone prepared rig.

Only existing vertex weights are changed. Call apply(arm, meshes) on a copy of
the current model; this module never opens, saves or exports a model.
"""
import numpy as np


def components(obj):
    parent=list(range(len(obj.data.vertices)))
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    for edge in obj.data.edges:
        a,b=map(int,edge.vertices);parent[root(b)]=root(a)
    result={}
    for i in range(len(parent)):result.setdefault(root(i),[]).append(i)
    return list(result.values())


def apply_shoulders(arm, meshes):
    """Filter two physical sleeve blends without adding bone influences."""
    from deep_body import _graph, _transition, _smoothstep
    from reaudit import find
    from refine_rig import get_weights, set_weights
    obj=next(o for o in meshes if o.name=='shirt_tex_shirt_0')
    names,w=get_weights(obj);co=np.array([v.co[:] for v in obj.data.vertices])
    graph=_graph(obj,co);selected=np.zeros(len(co),dtype=bool);regions=[]
    for side in ('l','r'):
        # First smooth shoulder-girdle-to-chest support, then the sleeve's
        # arm-to-girdle transition. Keep each existing primary/twist split.
        for prefixes,pivot,radius,band in [
            (('shoulder','arm_stretch','arm_twist'),'shoulder',38.,18.),
            (('arm_stretch','arm_twist'),'arm_stretch',32.,15.),
        ]:
            columns=[names.index(find(arm,p,side)) for p in prefixes]
            head=np.asarray(arm.data.bones[find(arm,pivot,side)].head_local)
            distance=np.linalg.norm(co-head,axis=1)
            mask=_smoothstep((radius-distance)/band)
            # _transition leaves absent and rigid families unchanged. Tiny
            # existing influences may grow: retaining the old .1 gate locked
            # near-rigid vertices next to soft sleeve vertices into a crease.
            selected|=_transition(w,columns,mask,graph)
            regions.append({'joint':find(arm,pivot,side),'radius_source_units':radius,
                            'method':'welded scalar sleeve transition; original bone support and twist split'})
    changes=set_weights(obj,names,w,selected)
    return [{'mesh':obj.name,'method':'local sleeve and shoulder-girdle transitions','regions':regions,'changes':changes}]


def apply_bow(arm, meshes):
    """Keep the neck bow's band, knot, loops and tails with the upper chest.

    The accessory has separate front/back/edge islands. Its old bilateral
    clavicle gradients pulled the knot apart and shortened its tails during
    a unilateral shoulder raise. A coherent existing chest attachment keeps
    its authored shape, including coincident seam copies, without a new rig.
    """
    from refine_rig import get_weights, set_weights
    obj=next(o for o in meshes if o.name=='bow_tex_bow_0')
    names,w=get_weights(obj)
    chest='spine_03.x_016'
    assert chest in arm.data.bones and chest in names
    w[:]=0.;w[:,names.index(chest)]=1.
    changes=set_weights(obj,names,w,np.ones(len(w),dtype=bool))
    return [{'mesh':obj.name,'method':'coherent neck bow and collar attachment to existing upper chest',
             'bone':chest,'changes':changes}]


def apply(arm, meshes, include_armor=True, include_sleeves=True):
    """Repair pelvis garment attachment; return an exact reversible patch log."""
    patches=[]
    for obj in meshes:
        cloth=obj.name in ('shirt_tex_shirt_0','shirt.001_tex_shirt_0','belt_tex_belt_0')
        armor=include_armor and obj.name in ('armor_tex_armor_0','shoulder_armor_tex_shoulderarmor_0')
        if not (cloth or armor):
            continue
        if cloth and 'root.x_01' not in obj.vertex_groups:obj.vertex_groups.new(name='root.x_01')
        names=[g.name for g in obj.vertex_groups]
        w=np.zeros((len(obj.data.vertices),len(names)))
        for v in obj.data.vertices:
            for g in v.groups:w[v.index,g.group]=g.weight
        old=w.copy();co=np.array([v.co[:] for v in obj.data.vertices])
        if armor:
            if obj.name=='shoulder_armor_tex_shoulderarmor_0':
                # One plate and its rivets must remain coherent. One rivet
                # was incorrectly attached 100% to the neck; the other parts
                # were split across shoulder/arm and small head influences.
                w[:]=0.;w[:,names.index('arm_twist.r_0156')]=1.
            else:
                # Breastplate shell, thickness and trim are rigid together.
                # Leave the two tall clavicle straps as authored.
                selected=[]
                for ids in components(obj):
                    if co[ids,2].max()<324.:selected+=ids
                w[selected]=0.;w[selected,names.index('spine_02.x_015')]=1.
        else:
            root=names.index('root.x_01');spine=names.index('spine_01.x_014')
            # The skirt chains start at the pelvis. Its waistband and rings
            # sit below the lumbar pivot; rotating them with the lumbar spine
            # pulled the fasteners away. Fade back to the unchanged torso.
            t=np.clip((co[:,2]-242.)/34.,0.,1.);t=t*t*(3.-2.*t)
            transfer=w[:,spine]*(1.-t);w[:,spine]-=transfer;w[:,root]+=transfer
        if obj.name=='belt_tex_belt_0':
            # The closed belt is one pelvis attachment, not nine independent
            # skirt ribbons. This retains its neutral form and buckle.
            w[:]=0.;w[:,root]=1.
        if obj.name=='shirt_tex_shirt_0':
            # Two split rings, their backing and rims are separate topology
            # islands. Keep each entire metal fitting with the same belt.
            for ids in components(obj):
                p=co[ids]
                if len(ids)<=128 and p[:,2].min()>220. and p[:,2].max()<242.:
                    w[ids]=0.;w[ids,root]=1.
        changes=[]
        for i in np.flatnonzero(np.max(np.abs(w-old),axis=1)>1e-7):
            # JULIA accepts four influences; transfer can introduce a fifth.
            row=w[i];top=np.argsort(row)[-4:];total=row[top].sum()
            after={names[j]:float(row[j]/total)for j in top if row[j]/total>1e-8}
            before={names[j]:float(v)for j,v in enumerate(old[i])if v>1e-8}
            for g in list(obj.data.vertices[int(i)].groups):obj.vertex_groups[g.group].remove([int(i)])
            for name,value in after.items():obj.vertex_groups[name].add([int(i)],value,'REPLACE')
            changes.append({'vertex':int(i),'before':before,'after':after})
        method='coherent rigid armor shell; existing clavicle straps retained' if armor else 'pelvis garment attachment; smooth lumbar transition; coherent belt and metal rings'
        patches.append({'mesh':obj.name,'method':method,'changes':changes})
    if include_sleeves:patches.extend(apply_shoulders(arm,meshes))
    patches.extend(apply_bow(arm,meshes))
    return patches
