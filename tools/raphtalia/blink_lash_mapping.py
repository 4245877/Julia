"""Topology-aware lash classification for the unchanged Raphtalia source mesh.

No bpy import or side effects. Coordinates must share the skin's local space.
The map deliberately keeps the lower ribbon/accents and existing brow separate.
"""
from collections import defaultdict
import numpy as np
from scipy.spatial import cKDTree


def mesh_components(mesh):
    adjacency = defaultdict(set)
    for edge in mesh.edges:
        a, b = map(int, edge.vertices)
        adjacency[a].add(b)
        adjacency[b].add(a)
    remaining = set(range(len(mesh.vertices)))
    result = []
    while remaining:
        pending = [next(iter(remaining))]
        component = []
        while pending:
            vertex = pending.pop()
            if vertex not in remaining:
                continue
            remaining.remove(vertex)
            component.append(vertex)
            pending.extend(adjacency[vertex] & remaining)
        result.append(component)
    return result


def classify_lashes(mesh, upper_by_side, lower_by_side):
    """Return source-vertex maps with region and original horizontal rib parameter.

    Upper/lower arrays contain the 9 original socket coordinates sorted by x.
    The two common endpoints are fixed canthi. Each moving upper lash vertex
    receives only the two rib translations bracketing its ORIGINAL x. This keeps
    ribbon thickness and flare offsets instead of wrapping tips onto the skin.

    The 11-vertex crease is labelled skin_detail: transfer weights from the skin
    surface there rather than attaching it directly to the moving socket edge.
    """
    points = np.array([v.co[:] for v in mesh.vertices])
    output = [None] * len(points)
    for component in mesh_components(mesh):
        points_component = points[component]
        side = 'L' if points_component[:, 0].mean() > 0 else 'R'
        upper = np.asarray(upper_by_side[side])
        lower = np.asarray(lower_by_side[side])
        upper = upper[np.argsort(upper[:, 0])]
        lower = lower[np.argsort(lower[:, 0])]
        du = cKDTree(upper).query(points_component)[0]
        dl = cKDTree(lower).query(points_component)[0]
        for vertex, distance_upper, distance_lower in zip(component, du, dl):
            if len(component) == 17:
                region = 'brow'
            elif len(component) == 11:
                region = 'skin_detail'
            elif len(component) == 8:
                region = 'lower'
            elif len(component) in (55, 32):
                region = ('canthus' if abs(distance_upper-distance_lower)<1e-4 else
                          'upper' if distance_upper<distance_lower else 'lower')
            else:
                raise ValueError(f'Unexpected lash component ({len(component)} vertices)')
            x = float(points[vertex, 0])
            segment = int(np.clip(np.searchsorted(upper[:,0], x)-1, 0, len(upper)-2))
            fraction = float(np.clip((x-upper[segment,0]) /
                                     (upper[segment+1,0]-upper[segment,0]),0,1))
            output[vertex] = {'side': side, 'region': region, 'component_size': len(component),
                              'rib_indices': [segment,segment+1],
                              'rib_weights': [1-fraction,fraction], 'source_x':x}
    # Ensure split normal/UV seams across disconnected ribbons share identical maps.
    canonical = {}
    for vertex, point in enumerate(points):
        key = tuple(np.round(point,4))
        if key in canonical:
            other = canonical[key]
            if output[vertex]['region'] != output[other]['region']:
                raise ValueError(f'Conflicting duplicate lash classification {vertex}/{other}')
            output[vertex] = dict(output[other])
        else:
            canonical[key] = vertex
    return output
