"""Compact LBS weight corrections for the prepared body; no model is saved.

The scalar transition filters preserve each vertex's existing bone support and
its split within the proximal/distal families. They never introduce fifth
influences or alter the manual twist relationship. Coincident seam vertices
are filtered on the same welded graph to avoid UV boundary locking.
"""
import numpy as np
from scipy.spatial import cKDTree

from reaudit import find, reset
from refine_rig import get_weights, set_weights


def _smoothstep(value):
    t = np.clip(value, 0, 1)
    return t * t * (3 - 2 * t)


def _graph(obj, co):
    parent = np.arange(len(co))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for a, b in cKDTree(co).query_pairs(1e-4):
        parent[root(b)] = root(a)
    labels = np.array([root(i) for i in range(len(co))])
    _, inverse = np.unique(labels, return_inverse=True)
    count = np.bincount(inverse)
    center = np.stack([np.bincount(inverse, weights=co[:,j])/count for j in range(3)], axis=1)
    edges = inverse[np.array([e.vertices[:] for e in obj.data.edges])]
    edges = np.unique(np.sort(edges[edges[:,0] != edges[:,1]], axis=1), axis=0)
    strength = 1 / np.maximum(np.linalg.norm(center[edges[:,0]]-center[edges[:,1]], axis=1), .5)
    ids = np.r_[edges[:,0], edges[:,1]]
    other = np.r_[edges[:,1], edges[:,0]]
    strength = np.r_[strength, strength]
    denom = np.bincount(ids, weights=strength, minlength=len(count))
    return inverse, count, ids, other, strength, denom


def _transition(w, selected_columns, mask, graph, iterations=18):
    """Filter one physical blend, retaining the original per-family split."""
    inverse, count, ids, other, strength, denom = graph
    original = w[:, selected_columns].sum(1)
    welded = np.bincount(inverse, weights=original) / count
    compact = np.bincount(inverse, weights=mask) / count
    value = welded.copy()
    for _ in range(iterations):
        mean = np.bincount(ids, weights=value[other] * strength, minlength=len(count)) / np.maximum(denom, 1e-30)
        value = (1-compact) * welded + compact * (.25*welded + .75*mean)
    desired = value[inverse]
    # Compact support: absent families remain absent, exactly rigid skin is
    # unchanged, and the original four-bone support is preserved.
    valid = (original > 1e-6) & (original < 1-1e-6) & (mask > 0)
    distal = selected_columns
    proximal = [i for i in range(w.shape[1]) if i not in distal]
    w[np.ix_(valid, distal)] *= (desired[valid]/original[valid])[:,None]
    w[np.ix_(valid, proximal)] *= ((1-desired[valid])/(1-original[valid]))[:,None]
    return valid


def apply(arm, meshes):
    body = next(o for o in meshes if o.name == 'body_tex_skin_0')
    names, w = get_weights(body)
    co = np.array([v.co[:] for v in body.data.vertices])
    graph = _graph(body, co)
    selected = np.zeros(len(co), dtype=bool)
    regions = []
    # The previous top-four pruning alternated a 3.5% residual between two
    # non-adjacent spine segments on adjacent upper chest vertices. Fold only
    # that low residual into their common dominant chest segment.
    centers = [np.array((s*13, -30, 311.5)) for s in (-1, 1)]
    distance = np.minimum.reduce([np.linalg.norm(co-center, axis=1) for center in centers])
    mask = _smoothstep((12-distance)/5)
    dominant = names.index('spine_02.x_015')
    for source in ('spine_01.x_014', 'spine_03.x_016'):
        column = names.index(source)
        amount = w[:, column] * mask * (w[:,column] < .10) * (w[:,dominant] > .5)
        w[:, column] -= amount
        w[:, dominant] += amount
        selected |= amount > 1e-8
    regions.append({'region':'upper_chest_support', 'method':'minor spine residual to common chest segment', 'radius_source_units':12})
    for side in ('l', 'r'):
        upper = [names.index(find(arm, p, side)) for p in ('arm_stretch', 'arm_twist')]
        head = np.asarray(arm.data.bones[find(arm,'arm_stretch',side)].head_local)
        radius = np.linalg.norm(co-head, axis=1)
        total = w[:,upper].sum(1)
        mask = _smoothstep((32-radius)/15) * _smoothstep(total/.1) * _smoothstep((1-total)/.1)
        selected |= _transition(w, upper, mask, graph)
        regions.append({'region':'axilla.'+side, 'method':'welded scalar arm-to-torso transition', 'radius_source_units':32})
        thigh = [names.index(find(arm,p,side)) for p in ('thigh_stretch','thigh_twist')]
        head = np.asarray(arm.data.bones[find(arm,'thigh_stretch',side)].head_local)
        radius = np.linalg.norm(co-head,axis=1)
        total = w[:,thigh].sum(1)
        mask = _smoothstep((33-radius)/16) * _smoothstep(total/.08) * _smoothstep((1-total)/.08)
        selected |= _transition(w, thigh, mask, graph)
        regions.append({'region':'hip.'+side, 'method':'welded scalar thigh-to-pelvis transition', 'radius_source_units':33})
    # Face and neck excluded independently of any region formula.
    selected &= co[:,2] < 335
    changes = set_weights(body, names, w, selected)
    return [{'mesh':body.name,'regions':regions,'changes':changes}]


def apply_limb_clothes(arm, meshes):
    """Motion-constrained stocking fold and local boot-ankle transitions.

    SciPy runs only while authoring the static weights. The resulting model
    still uses exactly the existing four-influence linear skinning in JULIA.
    """
    patches = []
    stocking = next((o for o in meshes if o.name == 'stocking_tex_sock_0'), None)
    if stocking:
        changes, fit = _fit_stocking(arm, stocking)
        patches.append({'mesh':stocking.name,'regions':[{'region':'knee_folds','method':'bounded motion-constrained scalar blend fit','radius_source_units':30}], 'fit':fit,'changes':changes})
    obj = next((o for o in meshes if o.name == 'shoes_tex_shoes_0'), None)
    if obj is not None:
        names, w = get_weights(obj)
        co = np.array([v.co[:] for v in obj.data.vertices])
        distance = np.minimum.reduce([np.linalg.norm(co-np.asarray(arm.data.bones[find(arm,'foot',s)].head_local),axis=1) for s in ('l','r')])
        local = distance<17
        parent = np.arange(len(co))
        def root(i):
            while parent[i]!=i:
                parent[i]=parent[parent[i]]
                i=parent[i]
            return i
        edges = [tuple(e.vertices) for e in obj.data.edges]
        edges += list(cKDTree(co).query_pairs(1e-4))
        # Two folded collar seams have almost adjacent endpoints assigned to
        # opposite segments. Make each short connected fold coherent; do not
        # diffuse the boot shaft, sole, or articulated toe.
        for a,b in edges:
            if local[a] and local[b] and np.linalg.norm(co[a]-co[b])<1.2:
                parent[root(b)]=root(a)
        groups = {}
        for i in np.flatnonzero(local):
            groups.setdefault(root(i),[]).append(i)
        selected = np.zeros(len(co),dtype=bool)
        components = []
        for group in groups.values():
            if len(group)<2:
                continue
            mean = w[group].mean(0)
            if (mean>1e-8).sum()>4:
                raise ValueError('Unexpected fifth boot-fold influence; refuse arbitrary pruning')
            w[group]=mean
            selected[group]=True
            components.append(len(group))
        changes = set_weights(obj,names,w,selected)
        patches.append({'mesh':obj.name,'regions':[{'region':'ankle_folded_seams','radius_source_units':17,'connected_edge_limit_mm':12,'method':'coherent weights within isolated short connected folds','component_sizes':components}],'changes':changes})
    reset(arm)
    return patches


def _fit_stocking(arm, obj):
    """Retain correct folds while reducing specific, reproduced overstrain.

    Only the scalar shin-versus-thigh blend is fitted. Existing support and
    primary/helper proportions are fixed. Constraints include ordinary knee
    flexion, deep flexion, transverse motion, and longitudinal helper twist;
    interpolation and unseen poses are verified separately by the audit.
    """
    from scipy.optimize import least_squares
    from scipy.sparse import coo_matrix
    from deep_pose_audit import apply_pose

    names, w = get_weights(obj)
    co = np.array([v.co[:] for v in obj.data.vertices])
    homogeneous = np.c_[co, np.ones(len(co))]
    inverse, count, *_ = _graph(obj, co)
    mask = np.zeros(len(co))
    distal = []
    for side in ('l', 'r'):
        distal += [names.index(find(arm,p,side)) for p in ('leg_stretch','leg_twist')]
        head = np.asarray(arm.data.bones[find(arm,'leg_stretch',side)].head_local)
        mask = np.maximum(mask, _smoothstep((30-np.linalg.norm(co-head,axis=1))/12))
    total = w[:,distal].sum(1)
    mask *= (total>1e-5) & (total<1-1e-5)
    original = np.bincount(inverse, weights=total) / count
    compact = np.bincount(inverse, weights=mask) / count
    unknown = np.flatnonzero(compact>1e-4)
    mapping = np.full(len(count), -1)
    mapping[unknown] = np.arange(len(unknown))
    vertex_variable = mapping[inverse]
    edges = np.array([e.vertices[:] for e in obj.data.edges])
    rest = np.linalg.norm(co[edges[:,0]]-co[edges[:,1]], axis=1)
    valid = (rest>.01) & ((vertex_variable[edges[:,0]]>=0) | (vertex_variable[edges[:,1]]>=0))
    edges, rest = edges[valid], rest[valid]
    cases = {}
    for angle in (60,90,120,135):
        cases['knee_'+str(angle)] = [(find(arm,'leg_stretch',s),angle,(1,0,0)) for s in ('l','r')]
    for angle in (-30,30):
        for axisname, axis in [('y',(0,1,0)), ('z',(0,0,1))]:
            cases['shin_'+axisname+'_'+str(angle)] = [(find(arm,'leg_stretch',s),angle,axis) for s in ('l','r')]
    for angle in (-45,45):
        cases['twist_'+str(angle)] = [(find(arm,'leg_twist',s),angle,tuple(arm.data.bones[find(arm,'leg_twist',s)].y_axis)) for s in ('l','r')]
    data = []
    for label, changes in cases.items():
        reset(arm)
        apply_pose(arm, changes)
        qdistal, qproximal = np.zeros_like(co), np.zeros_like(co)
        for j, name in enumerate(names):
            if not np.any(w[:,j]):
                continue
            bone = arm.pose.bones[name]
            matrix = np.asarray(bone.matrix @ bone.bone.matrix_local.inverted())
            q = (homogeneous @ matrix.T)[:,:3] * w[:,j,None]
            if j in distal:
                qdistal += q
            else:
                qproximal += q
        qdistal /= np.maximum(total[:,None], 1e-30)
        qproximal /= np.maximum(1-total[:,None], 1e-30)
        delta = qdistal-qproximal
        q = qproximal+total[:,None]*delta
        ratio = np.linalg.norm(q[edges[:,0]]-q[edges[:,1]],axis=1)/rest
        cap = np.where(ratio>1.7, 1.7, np.maximum(ratio+.02,1.05))
        floor = np.minimum(ratio-.02,.5)
        if label.startswith('twist'):
            cap = np.maximum(ratio+.02,1.05)
        data.append((qproximal,delta,cap,floor))

    def evaluate(values, jacobian=False):
        scalar = original.copy()
        scalar[unknown] = values
        blend = scalar[inverse]
        residuals, rows, cols, entries = [], [], [], []
        offset = 0
        for qproximal, delta, cap, floor in data:
            q = qproximal+blend[:,None]*delta
            vector = q[edges[:,0]]-q[edges[:,1]]
            length = np.linalg.norm(vector,axis=1)
            ratio = length/rest
            residuals += [np.maximum(ratio-cap,0)*5, np.minimum(ratio-floor,0)*5]
            if jacobian:
                unit = vector/np.maximum(length[:,None],1e-30)/rest[:,None]
                for endpoint, sign in [(0,1),(1,-1)]:
                    variable = vertex_variable[edges[:,endpoint]]
                    gradient = (unit*delta[edges[:,endpoint]]).sum(1)*sign*5
                    for shift, active in [(0,ratio>cap),(len(edges),ratio<floor)]:
                        use = (variable>=0)&active
                        rows += (offset+shift+np.flatnonzero(use)).tolist()
                        cols += variable[use].tolist()
                        entries += gradient[use].tolist()
            offset += 2*len(edges)
        residuals.append((values-original[unknown])*.3)
        if jacobian:
            rows += (offset+np.arange(len(values))).tolist()
            cols += np.arange(len(values)).tolist()
            entries += [.3]*len(values)
            return coo_matrix((entries,(rows,cols)),shape=(offset+len(values),len(values))).tocsr()
        return np.concatenate(residuals)

    lower = np.maximum(original[unknown]-.25*compact[unknown],1e-8)
    upper = np.minimum(original[unknown]+.25*compact[unknown],1-1e-8)
    start = np.maximum(lower+1e-10,np.minimum(upper-1e-10,original[unknown]))
    fit = least_squares(evaluate,start,jac=lambda values:evaluate(values,True),bounds=(lower,upper),max_nfev=100,ftol=1e-8,xtol=1e-8)
    if not np.isfinite(fit.x).all():
        raise ValueError('Nonfinite stocking fit; refuse to apply')
    scalar = original.copy()
    scalar[unknown] = fit.x
    desired = scalar[inverse]
    selected = (mask>0)&(total>1e-6)&(total<1-1e-6)
    proximal = [j for j in range(w.shape[1]) if j not in distal]
    w[np.ix_(selected,distal)] *= (desired[selected]/total[selected])[:,None]
    w[np.ix_(selected,proximal)] *= ((1-desired[selected])/(1-total[selected]))[:,None]
    changes = set_weights(obj,names,w,selected)
    return changes, {'poses':list(cases), 'variables':len(unknown), 'edges':len(edges), 'evaluations':fit.nfev, 'objective':float(fit.cost), 'status':int(fit.status), 'message':fit.message, 'method':'bounded sparse least squares; fixed 100-evaluation budget; offline static weights only'}
