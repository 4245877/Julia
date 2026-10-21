"""Read-only Blender FBX roundtrip verification of prepared Raphtalia assets.

Run: PYTHONPATH=build/raphtalia_tools/bpy python tools/raphtalia/roundtrip_check.py
Does not save any opened scene. The source is compared in upright armature space,
using the preparation convention of 0.01 metres per source coordinate unit.
"""
from __future__ import annotations

from pathlib import Path
import argparse
import hashlib
import json

import bpy
from mathutils import Matrix
import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs/raphtalia"
SOURCE = ROOT / "assets/models/Raphtalia.fbx"
PREPARED = ROOT / "assets/models/Raphtalia_prepared.fbx"
BLEND = ROOT / "assets/models/Raphtalia_prepared.blend"
EXPECTED_SOURCE_HASH = "a602086ac9e580f2d265a489478922b75aae00410313b17a89bd846ad4f3b34c"
POSITION_TOLERANCE_M = 1e-5
UV_TOLERANCE = 1e-6


def file_record(path):
    data = path.read_bytes()
    return {"path": str(path.relative_to(ROOT)), "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest()}


def import_fbx(path, ignore_leaf_bones=False):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(path), use_anim=True,
                             automatic_bone_orientation=False,
                             ignore_leaf_bones=ignore_leaf_bones)


def armature():
    return next(o for o in bpy.data.objects if o.type == "ARMATURE")


def action_label(action):
    # Blender's FBX importer prefixes takes with the armature name.
    name = action.name
    if "TEST_" in name:
        return name[name.index("TEST_"):]
    return name


def actions():
    return {action_label(a): a for a in bpy.data.actions}


def sample_frames(action, label):
    if label.startswith('TEST_blink_'):
        blink_curves=[c for c in action.fcurves if c.data_path in ('["Blink_L"]','["Blink_R"]')]
        keyed=sorted({float(k.co.x) for c in blink_curves for k in c.keyframe_points})
        frames=set(keyed)|{(a+b)/2 for a,b in zip(keyed,keyed[1:])}
        for curve in blink_curves:
            for a,b in zip(keyed,keyed[1:]):
                if curve.evaluate(a)>.001 or curve.evaluate(b)<.999:continue
                for target in (.25,.5,.75):
                    lo,hi=a,b
                    for _ in range(35):
                        mid=(lo+hi)*.5
                        if curve.evaluate(mid)<target:lo=mid
                        else:hi=mid
                    frames.add(round((lo+hi)*.5,6))
        return sorted(frames)
    if label.startswith('TEST_deep_'):
        # Inspect every authored pose and the transitions between them. Do not
        # assume a particular duration or silently miss multi-peak clips.
        keyed = sorted({float(k.co.x) for curve in action.fcurves
                        if curve.data_path.startswith('pose.bones[')
                        for k in curve.keyframe_points} | set(map(float, action.frame_range)))
        return sorted(set(keyed) | {(a + b) / 2 for a, b in zip(keyed, keyed[1:])})
    return ((1,16,31,46,61,76,91,106,121,136,151,166,181,196,211,226,241)
            if label == 'TEST_practical_ranges' else (1,16,31,46,61))


def set_action(label=None, frame=1):
    arm = armature()
    arm.animation_data_create()
    arm.animation_data.action = actions().get(label)
    for key in ('Blink_L','Blink_R'):
        if key in arm:arm[key]=0.
    for bone in arm.pose.bones:
        bone.matrix_basis = Matrix.Identity(4)
    whole = int(np.floor(frame))
    bpy.context.scene.frame_set(whole, subframe=float(frame - whole))
    bpy.context.view_layer.update()


def mesh_snapshot(source=False, include_surface=True):
    deps = bpy.context.evaluated_depsgraph_get()
    inverse_arm = armature().matrix_world.inverted()
    result = {}
    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        evaluated = obj.evaluated_get(deps)
        mesh = evaluated.to_mesh(preserve_all_data_layers=True, depsgraph=deps)
        transform = Matrix.Scale(.01, 4) @ inverse_arm @ evaluated.matrix_world if source else evaluated.matrix_world
        matrix = np.asarray(transform, dtype=float)
        co = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
        mesh.vertices.foreach_get("co", co)
        co = co.reshape(-1, 3)
        positions = (matrix @ np.c_[co, np.ones(len(co))].T).T[:, :3]
        record = {"positions": positions}
        if include_surface:
            faces = [tuple(int(v) for v in p.vertices) for p in mesh.polygons]
            flat_faces = np.asarray([v for p in faces for v in (len(p), *p)], dtype="<i4")
            uv_layers = {}
            for layer in mesh.uv_layers:
                values = np.empty(len(layer.data) * 2, dtype=np.float64)
                layer.data.foreach_get("uv", values)
                uv_layers[layer.name] = values.reshape(-1, 2)
            normals = np.asarray([n.vector[:] for n in mesh.corner_normals], dtype=float)
            normal_matrix = np.linalg.inv(matrix[:3, :3]).T
            normals = (normal_matrix @ normals.T).T
            normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-30)
            record.update({"vertices": len(mesh.vertices), "polygons": len(mesh.polygons),
                           "triangles": sum(len(p) - 2 for p in faces), "corners": len(mesh.loops),
                           "topology_sha256": hashlib.sha256(flat_faces.tobytes()).hexdigest(),
                           "uv_layers": uv_layers, "normals": normals,
                           "has_custom_normals": bool(mesh.has_custom_normals),
                           "material_slots": [m.name if m else None for m in obj.data.materials],
                           "polygon_material_indices": np.asarray([p.material_index for p in mesh.polygons]),
                           "smooth_faces": np.asarray([p.use_smooth for p in mesh.polygons])})
        result[obj.name] = record
        evaluated.to_mesh_clear()
    return result


def material_snapshot():
    result = {}
    for material in bpy.data.materials:
        shader = next((n for n in material.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None) if material.use_nodes else None
        result[material.name] = {"diffuse_color": list(material.diffuse_color),
                                 "roughness": material.roughness, "metallic": material.metallic,
                                 "principled": {key: list(shader.inputs[key].default_value) if key == "Base Color" else float(shader.inputs[key].default_value)
                                                for key in ("Base Color", "Metallic", "Roughness", "Alpha", "Specular IOR Level") if key in shader.inputs} if shader else None}
    return result


def skeleton_snapshot():
    arm = armature()
    return {bone.name: {"parent": bone.parent.name if bone.parent else None,
                        "head_world": list(arm.matrix_world @ bone.head_local),
                        "tail_world": list(arm.matrix_world @ bone.tail_local),
                        "deform": bool(bone.use_deform)} for bone in arm.data.bones}


def animations_snapshot():
    result = {}
    for label, action in actions().items():
        face_channels = {}
        for bone in ("eye.L", "eye.R", "ear.L", "ear.R"):
            prefix = 'pose.bones["' + bone + '"]'
            curves = [c for c in action.fcurves if c.data_path.startswith(prefix)]
            face_channels[bone] = {"curves": len(curves),
                                    "keyframes": sum(len(c.keyframe_points) for c in curves),
                                    "rotation_curves": sum("rotation" in c.data_path for c in curves)}
        result[label] = {"name": action.name, "frame_range": list(action.frame_range),
                         "fcurves": len(action.fcurves), "face_channels": face_channels}
    return result


def weights_snapshot():
    return {o.name:[{o.vertex_groups[g.group].name:float(g.weight) for g in v.groups if g.weight>1e-8} for v in o.data.vertices]
            for o in bpy.data.objects if o.type=='MESH'}


def compare_weights(a,b):
    errors=[];mismatches=[]
    for name in sorted(set(a)&set(b)):
        if len(a[name])!=len(b[name]):mismatches.append(name);continue
        errors.extend(max((abs(x.get(n,0)-y.get(n,0)) for n in x.keys()|y.keys()),default=0) for x,y in zip(a[name],b[name]))
    return {'vertex_count_mismatches':mismatches,'missing_meshes':sorted(set(a)-set(b)),
            'extra_meshes':sorted(set(b)-set(a)),'max_weight_delta':max(errors,default=0),
            'vertices_over_1e_6':sum(x>1e-6 for x in errors)}


def compare_geometry(reference, candidate, surfaces=True):
    result = {"missing_meshes": sorted(set(reference) - set(candidate)),
              "extra_meshes": sorted(set(candidate) - set(reference)), "meshes": {}}
    max_position = 0.0
    max_uv = 0.0
    max_normal_angle = 0.0
    surface_ok = True
    for name in sorted(set(reference) & set(candidate)):
        ref = reference[name]; test = candidate[name]
        a = ref["positions"]; b = test["positions"]
        a_to_b = cKDTree(b).query(a)[0]; b_to_a = cKDTree(a).query(b)[0]
        max_nearest = float(max(a_to_b.max(), b_to_a.max()))
        max_position = max(max_position, max_nearest)
        entry = {"reference_vertices": len(a), "candidate_vertices": len(b),
                 "symmetric_nearest_max_m": max_nearest,
                 "symmetric_nearest_rms_m": float(np.sqrt(np.mean(np.r_[a_to_b, b_to_a] ** 2)))}
        if len(a) == len(b):
            entry["corresponding_vertices_max_m"] = float(np.linalg.norm(a - b, axis=1).max())
        if surfaces:
            entry.update({"polygons_equal": ref["polygons"] == test["polygons"],
                          "triangles_equal": ref["triangles"] == test["triangles"],
                          "corners_equal": ref["corners"] == test["corners"],
                          "topology_equal": ref["topology_sha256"] == test["topology_sha256"],
                          "material_slots_equal": ref["material_slots"] == test["material_slots"],
                          "polygon_material_indices_equal": np.array_equal(ref["polygon_material_indices"], test["polygon_material_indices"]),
                          "smooth_faces_equal": np.array_equal(ref["smooth_faces"], test["smooth_faces"]),
                          "reference_has_custom_normals": ref["has_custom_normals"],
                          "candidate_has_custom_normals": test["has_custom_normals"],
                          "uv_layer_names_equal": list(ref["uv_layers"]) == list(test["uv_layers"]),
                          "uv_layers": {}})
            for uv in sorted(set(ref["uv_layers"]) & set(test["uv_layers"])):
                av = ref["uv_layers"][uv]; bv = test["uv_layers"][uv]
                delta = float(np.max(np.abs(av - bv))) if av.shape == bv.shape else None
                entry["uv_layers"][uv] = {"reference_corners": len(av), "candidate_corners": len(bv), "max_absolute_delta": delta}
                if delta is None:
                    surface_ok = False
                else:
                    max_uv = max(max_uv, delta)
            if entry["topology_equal"] and ref["normals"].shape == test["normals"].shape:
                cos = np.einsum("ij,ij->i", ref["normals"], test["normals"])
                angles = np.degrees(np.arccos(np.clip(cos, -1, 1)))
                entry["normal_angle_degrees"] = {"max": float(angles.max()), "rms": float(np.sqrt(np.mean(angles ** 2)))}
                max_normal_angle = max(max_normal_angle, float(angles.max()))
            else:
                entry["normal_angle_degrees"] = None
            surface_ok &= all(entry[k] for k in ("polygons_equal", "triangles_equal", "corners_equal", "topology_equal", "material_slots_equal", "polygon_material_indices_equal", "smooth_faces_equal", "uv_layer_names_equal"))
        result["meshes"][name] = entry
    result["max_symmetric_nearest_m"] = max_position
    result["positions_within_10_micrometres"] = max_position <= POSITION_TOLERANCE_M
    result["max_uv_absolute_delta"] = max_uv if surfaces else None
    result["max_normal_angle_degrees"] = max_normal_angle if surfaces else None
    result["surface_structure_preserved"] = bool(surface_ok and max_uv <= UV_TOLERANCE) if surfaces else None
    return result


def compare_materials(reference, candidate):
    diffs = {}
    for name in sorted(set(reference) & set(candidate)):
        a = reference[name]; b = candidate[name]
        vals = {key: float(np.max(np.abs(np.asarray(a[key]) - np.asarray(b[key])))) for key in ("diffuse_color", "roughness", "metallic")}
        vals["principled"] = {k: float(np.max(np.abs(np.asarray(a["principled"][k]) - np.asarray(b["principled"][k])))) for k in set(a["principled"] or {}) & set(b["principled"] or {})}
        diffs[name] = vals
    return {"missing": sorted(set(reference) - set(candidate)), "extra": sorted(set(candidate) - set(reference)), "deltas": diffs}


def main():
    global PREPARED,BLEND
    parser=argparse.ArgumentParser();parser.add_argument('--blend',default=str(BLEND));parser.add_argument('--fbx',default=str(PREPARED))
    parser.add_argument('--output',default=str(OUT/'roundtrip.json'));args=parser.parse_args()
    BLEND=(ROOT/args.blend).resolve();PREPARED=(ROOT/args.fbx).resolve()
    files_before = {key: file_record(path) for key, path in (("source", SOURCE), ("prepared_blend", BLEND), ("prepared_fbx", PREPARED))}
    # Match the preparation import exactly: source-generated *_end helpers are
    # intentionally ignored, while the final export has no helper leaves.
    import_fbx(SOURCE, ignore_leaf_bones=True)
    # Preserve the importer's original neutral pose, including its tiny bind
    # compensation transforms. The preparation bakes this evaluated surface.
    bpy.context.view_layer.update()
    source_meshes = mesh_snapshot(source=True)
    source_materials = material_snapshot()
    bpy.ops.wm.open_mainfile(filepath=str(BLEND))
    blend_animations = animations_snapshot()
    labels = sorted(blend_animations)
    set_action(labels[0], 1)
    blend_meshes = mesh_snapshot()
    blend_materials = material_snapshot()
    blend_skeleton = skeleton_snapshot()
    blend_weights=weights_snapshot()
    blend_samples = {}
    for label in labels:
        frames=sample_frames(actions()[label], label)
        for frame in frames:
            set_action(label, frame)
            blend_samples[(label, frame)] = mesh_snapshot(include_surface=False)
    import_fbx(PREPARED)
    fbx_animations = animations_snapshot()
    set_action(labels[0], 1)
    fbx_meshes = mesh_snapshot()
    fbx_materials = material_snapshot()
    fbx_skeleton = skeleton_snapshot()
    fbx_weights=weights_snapshot()
    sample_results = []
    for (label, frame), reference in blend_samples.items():
        if label not in fbx_animations:
            sample_results.append({"action": label, "frame": frame, "missing": True})
            continue
        set_action(label, frame)
        check = compare_geometry(reference, mesh_snapshot(include_surface=False), surfaces=False)
        sample_results.append({"action": label, "frame": frame, "max_symmetric_nearest_m": check["max_symmetric_nearest_m"],
                               "positions_within_10_micrometres": check["positions_within_10_micrometres"],
                               "mesh_maxima_m": {name: data["symmetric_nearest_max_m"] for name, data in check["meshes"].items()}})
    common_bones = set(blend_skeleton) & set(fbx_skeleton)
    head_deltas = {name: float(np.linalg.norm(np.asarray(blend_skeleton[name]["head_world"]) - np.asarray(fbx_skeleton[name]["head_world"]))) for name in common_bones}
    parent_mismatches = [name for name in sorted(common_bones) if blend_skeleton[name]["parent"] != fbx_skeleton[name]["parent"]]
    result = {
        "blender": bpy.app.version_string,
        "files": files_before,
        "source_hash_unchanged": files_before["source"]["sha256"] == EXPECTED_SOURCE_HASH,
        "source_to_prepared_neutral": compare_geometry(source_meshes, blend_meshes),
        "blend_to_fbx_neutral": compare_geometry(blend_meshes, fbx_meshes),
        "materials_source_to_blend": compare_materials(source_materials, blend_materials),
        "materials_blend_to_fbx": compare_materials(blend_materials, fbx_materials),
        "skeleton": {"blend_bones": len(blend_skeleton), "fbx_bones": len(fbx_skeleton),
                     "missing": sorted(set(blend_skeleton) - set(fbx_skeleton)), "extra": sorted(set(fbx_skeleton) - set(blend_skeleton)),
                     "parent_mismatches": parent_mismatches,
                     "max_head_position_delta_m": max(head_deltas.values(), default=0),
                     "head_position_deltas_m": head_deltas,
                     "note": "Leaf tail lengths are importer conventions; head transforms and sampled animation are tested."},
        "weights_blend_to_fbx":compare_weights(blend_weights,fbx_weights),
        "animations_blend": blend_animations, "animations_fbx": fbx_animations,
        "missing_actions": sorted(set(blend_animations) - set(fbx_animations)),
        "extra_actions": sorted(set(fbx_animations) - set(blend_animations)),
        "animation_samples": sample_results,
        "limits": ["Compares actual sampled Blender deformation; does not validate game blending, transitions, IK, collisions or artistic motion quality.",
                   "Symmetric nearest-vertex distances are not surface distances after local subdivision. Use the blink integrity provenance checks for unchanged rest surfaces and UV transfer.",
                   "Source's outer placement transform is removed using the documented preparation coordinate convention.",
                   "Normal comparisons are corner-wise and meaningful when topology order is equal."],
    }
    files_after = {key: file_record(path) for key, path in (("source", SOURCE), ("prepared_blend", BLEND), ("prepared_fbx", PREPARED))}
    result["files_stable_during_check"] = files_before == files_after
    result["summary"] = {
        "source_neutral_max_delta_m": result["source_to_prepared_neutral"]["max_symmetric_nearest_m"],
        "roundtrip_neutral_max_delta_m": result["blend_to_fbx_neutral"]["max_symmetric_nearest_m"],
        "sampled_animation_max_delta_m": max((r.get("max_symmetric_nearest_m", 0) for r in sample_results), default=0),
        "sample_count": len(sample_results),
        "bones": [len(blend_skeleton), len(fbx_skeleton)],
        "clips": [len(blend_animations), len(fbx_animations)],
        "source_hash_unchanged": result["source_hash_unchanged"],
        "files_stable_during_check": result["files_stable_during_check"],
    }
    (ROOT / args.output).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result["summary"], indent=2))
    print("SURFACE SOURCE", {k: v for k, v in result["source_to_prepared_neutral"].items() if k != "meshes"})
    print("SURFACE FBX", {k: v for k, v in result["blend_to_fbx_neutral"].items() if k != "meshes"})


if __name__ == "__main__":
    main()
