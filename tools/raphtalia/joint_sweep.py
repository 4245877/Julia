"""Independent small-angle numerical joint sweep of the prepared Blender rig.

Every bone receives +5 degrees independently about its LOCAL X, Y and Z axes.
This catches numerical/exploding deformation and remote secondary pivots; it
does not establish anatomical quality, collision-free motion or full ranges.
The model is opened read-only and never saved.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import time

import bpy
from mathutils import Matrix, Quaternion
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / "assets/models/Raphtalia_prepared.blend"
OUT = ROOT / "docs/raphtalia/joint_sweep.json"
MIN_EDGE_M = 1e-6  # 1e-4 source units * 0.01 m/source unit.
PRACTICAL_EDGE_M = 1e-4  # Additional filter to reveal 0.1 mm and longer edges.


def file_hash():
    return hashlib.sha256(PATH.read_bytes()).hexdigest()


def positions(obj, deps):
    evaluated = obj.evaluated_get(deps)
    mesh = evaluated.data
    raw = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
    mesh.vertices.foreach_get("co", raw)
    co = raw.reshape(-1, 3)
    mat = np.asarray(evaluated.matrix_world, dtype=float)
    return (mat @ np.c_[co, np.ones(len(co))].T).T[:, :3]


def main():
    global PATH,OUT
    parser=argparse.ArgumentParser();parser.add_argument('--blend',default=str(PATH));parser.add_argument('--output',default=str(OUT))
    args=parser.parse_args();PATH=(ROOT/args.blend).resolve();OUT=(ROOT/args.output).resolve()
    started = time.monotonic()
    before_hash = file_hash()
    bpy.ops.wm.open_mainfile(filepath=str(PATH))
    arm = next(o for o in bpy.data.objects if o.type == "ARMATURE")
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    if arm.animation_data:
        arm.animation_data.action = None
        for track in arm.animation_data.nla_tracks:
            track.mute = True
    for bone in arm.pose.bones:
        bone.matrix_basis = Matrix.Identity(4)
        bone.rotation_mode = "QUATERNION"
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    cached = {}
    for obj in meshes:
        base = positions(obj, deps)
        edges_flat = np.empty(len(obj.data.edges) * 2, dtype=np.int32)
        obj.data.edges.foreach_get("vertices", edges_flat)
        edges = edges_flat.reshape(-1, 2)
        lengths = np.linalg.norm(base[edges[:, 0]] - base[edges[:, 1]], axis=1)
        names = {group.index: group.name for group in obj.vertex_groups}
        weights = [{names[g.group]: float(g.weight) for g in vertex.groups if g.weight > 1e-8} for vertex in obj.data.vertices]
        cached[obj.name] = {"base": base, "edges": edges, "lengths": lengths,
                            "requested_mask": lengths > MIN_EDGE_M,
                            "practical_mask": lengths > PRACTICAL_EDGE_M,
                            "weights": weights}
    bones = list(arm.pose.bones)
    rows = []
    for index, bone in enumerate(bones):
        head = np.asarray(arm.matrix_world @ bone.bone.head_local)
        direct_points = []; direct_weights = []
        for obj in meshes:
            cache = cached[obj.name]
            idx = [i for i, w in enumerate(cache["weights"]) if bone.name in w]
            if idx:
                direct_points.extend(cache["base"][idx])
                direct_weights.extend(cache["weights"][i][bone.name] for i in idx)
        if direct_points:
            center = np.average(direct_points, axis=0, weights=direct_weights)
            direct_distance = float(np.linalg.norm(center - head))
        else:
            center = None; direct_distance = None
        for axis_name, axis in (("X", (1, 0, 0)), ("Y", (0, 1, 0)), ("Z", (0, 0, 1))):
            bone.rotation_quaternion = Quaternion(axis, math.radians(5))
            bpy.context.view_layer.update()
            row = {"bone": bone.name, "axis": axis_name, "angle_degrees": 5,
                   "head_world_m": head.tolist(), "direct_weighted_vertices": len(direct_points),
                   "direct_weighted_centroid_m": center.tolist() if center is not None else None,
                   "head_to_direct_weighted_centroid_m": direct_distance,
                   "nonfinite_vertices": 0, "max_movement_m": 0.0,
                   "edges_stretched_over_2_requested": 0,
                   "edges_stretched_over_2_0_1mm": 0,
                   "max_edge_ratio_requested": 0.0,
                   "max_edge_ratio_0_1mm": 0.0, "meshes_with_stretch_over_2": []}
            worst = None
            for obj in meshes:
                cache = cached[obj.name]; deformed = positions(obj, deps)
                row["nonfinite_vertices"] += int((~np.isfinite(deformed).all(axis=1)).sum())
                movement = np.linalg.norm(deformed - cache["base"], axis=1)
                row["max_movement_m"] = max(row["max_movement_m"], float(movement.max()))
                edges = cache["edges"]; lengths = np.linalg.norm(deformed[edges[:, 0]] - deformed[edges[:, 1]], axis=1)
                ratios = np.divide(lengths, cache["lengths"], out=np.zeros_like(lengths), where=cache["lengths"] > 0)
                requested = cache["requested_mask"]; practical = cache["practical_mask"]
                bad_requested = int(((ratios > 2) & requested).sum())
                bad_practical = int(((ratios > 2) & practical).sum())
                row["edges_stretched_over_2_requested"] += bad_requested
                row["edges_stretched_over_2_0_1mm"] += bad_practical
                reqmax = float(ratios[requested].max()) if requested.any() else 0
                pracmax = float(ratios[practical].max()) if practical.any() else 0
                row["max_edge_ratio_requested"] = max(row["max_edge_ratio_requested"], reqmax)
                row["max_edge_ratio_0_1mm"] = max(row["max_edge_ratio_0_1mm"], pracmax)
                if bad_requested:
                    row["meshes_with_stretch_over_2"].append({"mesh": obj.name, "edges_requested": bad_requested, "edges_0_1mm": bad_practical})
                if reqmax and (worst is None or reqmax > worst["ratio"]):
                    ei = int(np.argmax(np.where(requested, ratios, -1)))
                    a, b = map(int, edges[ei])
                    worst = {"mesh": obj.name, "vertices": [a, b], "ratio": reqmax,
                             "rest_length_m": float(cache["lengths"][ei]), "deformed_length_m": float(lengths[ei]),
                             "rest_positions_m": cache["base"][[a, b]].tolist(),
                             "endpoint_weights": [cache["weights"][a], cache["weights"][b]]}
            row["worst_edge"] = worst
            row["secondary_chain"] = bone.name.startswith(("hair.", "cloth.")) or "tail" in bone.name
            row["twist_bone"] = "_twist" in bone.name
            rows.append(row)
            bone.matrix_basis = Matrix.Identity(4)
        if index % 40 == 0:
            print("SWEEP", index + 1, "/", len(bones), flush=True)
    after_hash = file_hash()
    bad = [r for r in rows if r["edges_stretched_over_2_requested"]]
    secondary = [r for r in bad if r["secondary_chain"]]
    result = {
        "blender": bpy.app.version_string, "file": str(PATH.relative_to(ROOT)),
        "sha256": before_hash, "file_unchanged": before_hash == after_hash,
        "duration_s": time.monotonic() - started,
        "method": f"{len(bones)} individual bones receive +5 degree local X, Y and Z rotations, one axis at a time; neutral between tests.",
        "edge_threshold_m": MIN_EDGE_M, "edge_threshold_source_units": 1e-4,
        "additional_edge_threshold_m": PRACTICAL_EDGE_M,
        "summary": {"bones": len(bones), "poses": len(rows),
                    "nonfinite_vertex_total": sum(r["nonfinite_vertices"] for r in rows),
                    "poses_with_edges_stretched_over_2": len(bad),
                    "secondary_poses_with_edges_stretched_over_2": len(secondary),
                    "poses_with_edges_over_0_1mm_stretched_over_2": sum(r["edges_stretched_over_2_0_1mm"] > 0 for r in rows),
                    "max_movement_m": max(r["max_movement_m"] for r in rows),
                    "max_edge_ratio_requested": max(r["max_edge_ratio_requested"] for r in rows)},
        "flagged_secondary_poses": [{k: r[k] for k in ("bone", "axis", "head_to_direct_weighted_centroid_m", "max_movement_m", "edges_stretched_over_2_requested", "edges_stretched_over_2_0_1mm", "max_edge_ratio_requested")} for r in secondary],
        "poses": rows,
        "limits": ["A small-angle numerical sweep is not full-range anatomical validation.",
                   "Ratio flags can expose thin/tiny triangles and mixed weights, not only wrong pivots; inspect worst_edge records and renders.",
                   "No mesh intersection, self-collision, volume preservation or animation blending test is performed."],
    }
    result["axis_group_summaries"] = {}
    for name, selected in {
        "local_X_Z": [r for r in rows if r["axis"] in ("X", "Z")],
        "local_Y": [r for r in rows if r["axis"] == "Y"],
        "twist_bones_local_Y_longitudinal": [r for r in rows if r["axis"] == "Y" and r["twist_bone"]],
        "twist_bones_X_Z_swing_stress": [r for r in rows if r["axis"] in ("X", "Z") and r["twist_bone"]],
    }.items():
        flagged = [r for r in selected if r["edges_stretched_over_2_requested"]]
        result["axis_group_summaries"][name] = {
            "poses": len(selected), "flagged_poses": len(flagged),
            "nonfinite_vertices": sum(r["nonfinite_vertices"] for r in selected),
            "max_edge_ratio_requested": max((r["max_edge_ratio_requested"] for r in selected), default=0),
            "max_edge_ratio_0_1mm": max((r["max_edge_ratio_0_1mm"] for r in selected), default=0),
            "flagged": [{k: r[k] for k in ("bone", "axis", "max_edge_ratio_requested", "edges_stretched_over_2_requested", "edges_stretched_over_2_0_1mm")} for r in flagged],
        }
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result["summary"], indent=2), flush=True)
    print("AXIS GROUPS", result["axis_group_summaries"], flush=True)
    print("SECONDARY FLAGGED", result["flagged_secondary_poses"], flush=True)


if __name__ == "__main__":
    main()
