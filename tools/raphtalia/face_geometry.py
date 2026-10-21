"""Read-only connected-component and pivot feasibility audit for Raphtalia's face.

Run with the official bpy Python module or Blender --background --python.
All coordinates include object transforms and are in armature local space.
"""
from pathlib import Path
import collections
import json

import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=str(ROOT / "assets/models/Raphtalia.fbx"),
                         use_anim=False, automatic_bone_orientation=False,
                         ignore_leaf_bones=True)
arm = next(o for o in bpy.data.objects if o.type == "ARMATURE")
to_arm = arm.matrix_world.inverted()
targets = {o.name: o for o in bpy.data.objects if o.type == "MESH" and
           (o.name.startswith(("eye_", "eyelashes_", "hightlight_", "ear_", "ear."))
            or o.name in ("body_tex_skin_0", "body_tex_skin2_0"))}


def components(mesh, points, weld=False):
    parents = list(range(len(points)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    def union(i, j):
        parents[root(j)] = root(i)
    for e in mesh.edges:
        union(int(e.vertices[0]), int(e.vertices[1]))
    if weld:
        shared = {}
        for i, p in enumerate(points):
            key = tuple(np.round(p, 5))
            if key in shared:
                union(i, shared[key])
            shared[key] = i
    out = collections.defaultdict(list)
    for i in range(len(points)):
        out[root(i)].append(i)
    return sorted(out.values(), key=len, reverse=True)


def describe(points, indices, include_indices=True):
    p = points[indices]
    result = {"vertices": len(indices), "center": p.mean(axis=0).tolist(),
              "min": p.min(axis=0).tolist(), "max": p.max(axis=0).tolist()}
    if include_indices:
        result["indices"] = [int(i) for i in indices]
    return result


def welded_boundaries(mesh, points):
    unique, inverse = np.unique(np.round(points, 5), axis=0, return_inverse=True)
    use = collections.Counter()
    for poly in mesh.polygons:
        indices = inverse[list(poly.vertices)]
        for a, b in zip(indices, np.roll(indices, -1)):
            if a != b:
                use[tuple(sorted((int(a), int(b))))] += 1
    adjacency = collections.defaultdict(set)
    for (a, b), count in use.items():
        if count == 1:
            adjacency[a].add(b); adjacency[b].add(a)
    groups = []
    visited = set()
    for i in adjacency:
        if i in visited:
            continue
        todo = [i]; ids = []
        while todo:
            j = todo.pop()
            if j in visited:
                continue
            visited.add(j); ids.append(j)
            todo.extend(adjacency[j] - visited)
        groups.append(describe(unique, ids, include_indices=False))
    return {"unique_vertices": len(unique), "boundary_edges": sum(v == 1 for v in use.values()),
            "nonmanifold_edges_over_two_faces": sum(v > 2 for v in use.values()),
            "boundary_components": sorted(groups, key=lambda d: d["vertices"], reverse=True)}


def sphere_fit(points):
    # Algebraic fit is only a geometric diagnostic, not a validated eye pivot.
    A = np.c_[2 * points, np.ones(len(points))]
    sol, *_ = np.linalg.lstsq(A, np.sum(points ** 2, axis=1), rcond=None)
    center = sol[:3]
    radius_sq = sol[3] + center @ center
    dist = np.linalg.norm(points - center, axis=1)
    return {"center": center.tolist(), "radius": float(np.sqrt(max(radius_sq, 0))),
            "radial_rmse": float(np.sqrt(np.mean((dist - dist.mean()) ** 2))),
            "condition_number": float(np.linalg.cond(A)),
            "warning": "Least squares fit to visible patch only; pivot needs pose/render validation."}


result = {"coordinate_space": "armature local with mesh object transforms included",
          "source": "assets/models/Raphtalia.fbx", "meshes": {},
          "recommendations": {}}
all_points = {}
for name, obj in targets.items():
    mat = to_arm @ obj.matrix_world
    points = np.asarray([mat @ v.co for v in obj.data.vertices], dtype=float)
    all_points[name] = points
    groups = components(obj.data, points)
    welded = components(obj.data, points, weld=True)
    side_indices = {"left_positive_x": np.flatnonzero(points[:, 0] > 0),
                    "right_negative_x": np.flatnonzero(points[:, 0] < 0),
                    "center_plane": np.flatnonzero(np.abs(points[:, 0]) <= 1e-6)}
    edge_use = collections.Counter()
    for p in obj.data.polygons:
        vv = list(p.vertices)
        for i, v in enumerate(vv):
            edge_use[tuple(sorted((int(v), int(vv[(i + 1) % len(vv)]))))] += 1
    report = {"vertices": len(points), "polygons": len(obj.data.polygons),
              "connected_components": [describe(points, c, include_indices=name != "body_tex_skin_0") for c in groups],
              "welded_component_sizes": [len(c) for c in welded],
              "boundary_edges": sum(v == 1 for v in edge_use.values()),
              "nonmanifold_edges_over_two_faces": sum(v > 2 for v in edge_use.values()),
              "side_regions": {k: describe(points, ids, include_indices=name != "body_tex_skin_0") for k, ids in side_indices.items() if len(ids)},
              "welded_boundaries": welded_boundaries(obj.data, points)}
    if name.startswith("eye_") or name == "body_tex_skin2_0":
        report["sphere_fits_per_side"] = {k: sphere_fit(points[ids]) for k, ids in side_indices.items() if k != "center_plane" and len(ids)}
    result["meshes"][name] = report

# Ear base is estimated from the lowest part of each disconnected ear, with
# outer and inner geometry pooled so both surfaces can use the same controller.
ear_points = np.vstack([p for n, p in all_points.items() if n.startswith(("ear_", "ear."))])
ear_pivots = {}
for side, sign in [("left_positive_x", 1), ("right_negative_x", -1)]:
    p = ear_points[ear_points[:, 0] * sign > 0]
    cutoff = np.quantile(p[:, 2], .15)
    base = p[p[:, 2] <= cutoff]
    ear_pivots[side] = {"base_center_lowest_15_percent": base.mean(axis=0).tolist(),
                        "base_z_cutoff": float(cutoff), "bounds_min": p.min(axis=0).tolist(),
                        "bounds_max": p.max(axis=0).tolist(),
                        "tip_center_highest_10_percent": p[p[:, 2] >= np.quantile(p[:, 2], .9)].mean(axis=0).tolist()}
result["recommendations"] = {
    "eye_controls": {"geometry": "Visible eye patches, not full eyeball spheres. Test conservative gaze angles to avoid sliding/clipping.",
                     "weighting": "Rigid per-side eye and corresponding highlight vertices; do not attach eyelashes to gaze bones.",
                     "pivot": "Use fitted sphere centers only after practical pose/render check."},
    "ear_controls": {"pivots": ear_pivots,
                     "weighting": "Use identical per-side height ramp on outer and inner ear surfaces; blend root to head, tip to ear bone.",
                     "collision": "Geometry does not include collision response; inspect hair intersection at each desired pose."},
    "blink": "No eyelid morphs or eyelid bones. Eyelashes are separate geometry; moving them alone cannot close the skin/eye opening. A proper blink requires sculpted eyelid shapes or manual facial skin weights followed by visual validation.",
    "jaw": "No jaw/tongue/lip controls, no morph targets. A mouth-region component with internal depth exists in the body mesh; a proper speaking mouth still requires manual lip/cheek/inner-mouth weights and corrective morph authoring. Complete speech, teeth or tongue articulation have not been established.",
    "limits": "This file records geometry feasibility only; it does not validate deformation or save changes to the model."
}
result["controls"] = []
eye_fits = result["meshes"]["eye_tex_eyes_0"]["sphere_fits_per_side"]
for label, sign, suffix in [("left_positive_x", 1, "L"), ("right_negative_x", -1, "R")]:
    result["controls"].append({
        "bone": "eye." + suffix, "head": eye_fits[label]["center"],
        "side": sign, "kind": "eye",
        "meshes": ["eye_tex_eyes_0", "hightlight_hightlight_0"],
        "suggested_test_degrees": 8,
        "pivot_source": "least-squares sphere fit to corresponding iris patch",
    })
    result["controls"].append({
        "bone": "ear." + suffix,
        "head": ear_pivots[label]["base_center_lowest_15_percent"],
        "side": sign, "kind": "ear",
        "base_z": ear_pivots[label]["bounds_min"][2], "fade_height": 10,
        "meshes": ["ear_tex_ear_0", "ear.001_tex_ear2_0"],
        "suggested_test_degrees": 12,
        "pivot_source": "mean of pooled outer and inner ear's lowest 15 percent vertices",
    })
body_components = result["meshes"]["body_tex_skin_0"]["connected_components"]
result["facial_topology_notes"] = {
    "body_welded_boundaries": result["meshes"]["body_tex_skin_0"]["welded_boundaries"],
    "mouth_region_components": [c for c in body_components if c["min"][2] > 350 and c["max"][2] < 367],
    "mouth_region_interpretation": "227-vertex component spans z356.959..365.932 and depth y-21.769..-0.964; evidence for an existing inner-mouth surface, not a tested speech rig.",
    "eye_socket_interpretation": "After position welding, the body has exactly two 16-edge boundary loops at eye sockets; sclera boundary loops match them.",
}
out = ROOT / "docs/raphtalia/face_geometry.json"
out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
for name, data in result["meshes"].items():
    print(name, "components", [c["vertices"] for c in data["connected_components"]],
          "welded", data["welded_component_sizes"], "bounds", {k: {x: v[x] for x in ('vertices','center','min','max')} for k,v in data["side_regions"].items()})
print("EAR PIVOTS", ear_pivots)
print("EYE FIT", result["meshes"]["eye_tex_eyes_0"]["sphere_fits_per_side"])
