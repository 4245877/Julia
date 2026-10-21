"""Read-only binary FBX structural and skin audit (no Autodesk dependency).

Usage: python tools/raphtalia/fbx_audit.py input.fbx --output report.json
The report uses source FBX control points, before importer triangulation/splitting.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import struct
import zlib

import numpy as np
from scipy.spatial.transform import Rotation


class Node:
    def __init__(self, name, props, children):
        self.name, self.props, self.children = name, props, children

    def child(self, name):
        return next((c for c in self.children if c.name == name), None)

    def value(self, name, default=None):
        n = self.child(name)
        return n.props[0] if n and n.props else default


class BinaryFBX:
    def __init__(self, path):
        self.path = Path(path)
        self.data = self.path.read_bytes()
        if not self.data.startswith(b"Kaydara FBX Binary  \x00\x1a\x00"):
            raise ValueError("Expected binary FBX")
        self.version = struct.unpack_from("<I", self.data, 23)[0]
        self.wide = self.version >= 7500
        self.header_size = 25 if self.wide else 13
        self.offset = 27
        self.nodes = []
        while self.offset + self.header_size <= len(self.data):
            node = self.read_node()
            if node is None:
                break
            self.nodes.append(node)

    def read_property(self):
        t = chr(self.data[self.offset]); self.offset += 1
        scalars = {"Y": ("h", 2), "C": ("?", 1), "I": ("i", 4), "F": ("f", 4), "D": ("d", 8), "L": ("q", 8)}
        if t in scalars:
            fmt, size = scalars[t]
            result = struct.unpack_from("<" + fmt, self.data, self.offset)[0]
            self.offset += size
            return result
        if t in "fdlibc":
            length, encoding, stored = struct.unpack_from("<III", self.data, self.offset)
            self.offset += 12
            raw = self.data[self.offset:self.offset + stored]; self.offset += stored
            if encoding == 1:
                raw = zlib.decompress(raw)
            elif encoding != 0:
                raise ValueError(f"Unsupported array encoding {encoding}")
            dt = {"f": "<f4", "d": "<f8", "l": "<i8", "i": "<i4", "b": "?", "c": "i1"}[t]
            result = np.frombuffer(raw, dtype=dt)
            if result.size != length:
                raise ValueError("FBX array length mismatch")
            return result
        if t in "SR":
            length = struct.unpack_from("<I", self.data, self.offset)[0]; self.offset += 4
            raw = self.data[self.offset:self.offset + length]; self.offset += length
            return raw.decode("utf-8", errors="replace") if t == "S" else raw
        raise ValueError(f"Unknown FBX property {t!r}")

    def read_node(self):
        fmt = "<QQQB" if self.wide else "<IIIB"
        end, count, _length, name_len = struct.unpack_from(fmt, self.data, self.offset)
        self.offset += self.header_size
        if end == 0:
            return None
        name = self.data[self.offset:self.offset + name_len].decode("utf-8"); self.offset += name_len
        props = [self.read_property() for _ in range(count)]
        children = []
        while self.offset < end:
            child = self.read_node()
            if child is None:
                break
            children.append(child)
        if self.offset != end:
            raise ValueError(f"Invalid node end for {name}: {self.offset} != {end}")
        return Node(name, props, children)

    def root(self, name):
        return next((n for n in self.nodes if n.name == name), None)


def clean_name(value):
    return str(value).split("\x00", 1)[0]


def properties(node):
    block = node.child("Properties70")
    if block is None:
        return {}
    return {str(p.props[0]): p.props[4:] for p in block.children if p.name == "P"}


def simple_property_map(node):
    result = {}
    for key, vals in properties(node).items():
        result[key] = vals[0] if len(vals) == 1 else vals
    return result


def vector(v, default):
    return np.asarray(v if v is not None else default, dtype=float)


def trans(v):
    m = np.eye(4); m[:3, 3] = v
    return m


def rotation(v, order="XYZ"):
    # FBX Euler order XYZ means Rx is applied first to column vectors (Rz Ry Rx).
    m = np.eye(4); m[:3, :3] = Rotation.from_euler(order.lower(), v, degrees=True).as_matrix()
    return m


def local_matrix(node):
    """FBX transform stack, applicable to the audited file's default XYZ order.

    Non-default inheritance is reported separately; evaluated armature validation
    should also use Blender/Assimp for unsupported inherit-scale semantics.
    """
    p = properties(node)
    get = lambda k, d: vector(p.get(k), d)
    order_idx = int(p.get("RotationOrder", [0])[0])
    order = ["XYZ", "XZY", "YZX", "YXZ", "ZXY", "ZYX", "XYZ"][order_idx]
    T = trans(get("Lcl Translation", [0, 0, 0]))
    Roff = trans(get("RotationOffset", [0, 0, 0])); Rp = trans(get("RotationPivot", [0, 0, 0]))
    Pre = rotation(get("PreRotation", [0, 0, 0])); R = rotation(get("Lcl Rotation", [0, 0, 0]), order)
    Post = rotation(get("PostRotation", [0, 0, 0]))
    Soff = trans(get("ScalingOffset", [0, 0, 0])); Sp = trans(get("ScalingPivot", [0, 0, 0]))
    S = np.diag([*get("Lcl Scaling", [1, 1, 1]), 1.0])
    return T @ Roff @ Rp @ Pre @ R @ np.linalg.inv(Post) @ np.linalg.inv(Rp) @ Soff @ Sp @ S @ np.linalg.inv(Sp)


def summarize_numbers(values):
    a = np.asarray(values)
    return {"min": float(a.min()), "max": float(a.max()), "mean": float(a.mean())} if a.size else None


def audit(path):
    fbx = BinaryFBX(path)
    objects = fbx.root("Objects")
    by_id = {int(n.props[0]): n for n in objects.children}
    children = collections.defaultdict(list); parents = collections.defaultdict(list)
    connections = []
    for n in fbx.root("Connections").children:
        if n.name != "C":
            continue
        typ, src, dst, *more = n.props
        src, dst = int(src), int(dst)
        connections.append((typ, src, dst, more))
        if typ == "OO":
            children[dst].append(src); parents[src].append(dst)
    models = {i: n for i, n in by_id.items() if n.name == "Model"}
    bones = {i: n for i, n in models.items() if n.props[2] == "LimbNode"}
    meshes = {i: n for i, n in by_id.items() if n.name == "Geometry" and n.props[2] == "Mesh"}
    clusters = {i: n for i, n in by_id.items() if n.name == "Deformer" and n.props[2] == "Cluster"}
    skins = {i: n for i, n in by_id.items() if n.name == "Deformer" and n.props[2] == "Skin"}
    globals_cache = {}
    def world(i, visiting=None):
        if i in globals_cache:
            return globals_cache[i]
        if i not in models:
            return np.eye(4)
        visiting = (visiting or set()).copy()
        if i in visiting:
            raise ValueError("Model hierarchy cycle")
        visiting.add(i)
        parent = next((p for p in parents[i] if p in models), None)
        globals_cache[i] = world(parent, visiting) @ local_matrix(models[i])
        return globals_cache[i]
    def identity(i):
        n = by_id.get(i)
        return {"id": i, "name": clean_name(n.props[1]), "kind": n.name, "subtype": str(n.props[2]) if len(n.props) > 2 else ""} if n else {"id": i}
    bone_root = next((i for i in bones if not any(p in bones for p in parents[i])), None)
    armature_id = next((p for p in parents[bone_root] if p in models), None) if bone_root else None
    armature_inverse = np.linalg.inv(world(armature_id))
    bind_world = {}
    for n in by_id.values():
        if n.name == "Pose":
            for p in n.children:
                if p.name == "PoseNode":
                    bind_world[int(p.value("Node"))] = np.asarray(p.value("Matrix")).reshape(4, 4).T
    matrix_library = []
    matrix_lookup = {}
    def matrix_ref(a):
        m = np.asarray(a).reshape(4, 4).T
        signature = tuple(m.ravel())
        if signature not in matrix_lookup:
            matrix_lookup[signature] = len(matrix_library)
            matrix_library.append({"matrix_row_major": m.tolist(), "determinant": float(np.linalg.det(m)), "finite": bool(np.isfinite(m).all())})
        return matrix_lookup[signature]
    skeleton = []
    for i, n in bones.items():
        bone_children = [c for c in children[i] if c in bones]
        p = simple_property_map(n)
        skeleton.append({**identity(i), "parent": next((identity(p) for p in parents[i] if p in models), None), "children": [identity(c) for c in bone_children], "local_translation": p.get("Lcl Translation", [0, 0, 0]), "local_rotation": p.get("Lcl Rotation", [0, 0, 0]), "local_scale": p.get("Lcl Scaling", [1, 1, 1]), "pre_rotation": p.get("PreRotation", [0, 0, 0]), "post_rotation": p.get("PostRotation", [0, 0, 0]), "rotation_order": p.get("RotationOrder", 0), "inherit_type": p.get("InheritType", 0), "world_head_source_units": world(i)[:3, 3].tolist(), "armature_space_model_head": (armature_inverse @ world(i))[:3, 3].tolist(), "armature_space_bind_head": (armature_inverse @ bind_world[i])[:3, 3].tolist() if i in bind_world else None, "bind_model_head_distance": float(np.linalg.norm(world(i)[:3, 3] - bind_world[i][:3, 3])) if i in bind_world else None, "children_distances_source_units": {clean_name(models[c].props[1]): float(np.linalg.norm(world(c)[:3, 3] - world(i)[:3, 3])) for c in bone_children}})
    cluster_details = []
    empty_cluster_bone_counts = collections.Counter()
    for i, n in clusters.items():
        ids = n.value("Indexes", np.array([], dtype=int)); weights = n.value("Weights", np.array([], dtype=float))
        link_bones = [c for c in children[i] if c in models]
        if len(ids) == 0:
            empty_cluster_bone_counts.update(link_bones)
            continue
        matrix_info = {}
        for key in ("Transform", "TransformLink", "TransformAssociateModel"):
            a = n.value(key)
            if a is not None:
                matrix_info[key + "_matrix_ref"] = matrix_ref(a)
        cluster_details.append({**identity(i), "bones": [identity(c) for c in link_bones], "skins": [p for p in parents[i] if p in skins], "indices": int(len(ids)), "unique_indices": int(len(np.unique(ids))), "weight_stats": summarize_numbers(weights), "negative_weights": int((weights < 0).sum()), "nonfinite_weights": int((~np.isfinite(weights)).sum()), **matrix_info})
    mesh_reports = []
    all_positions = []
    for i, n in meshes.items():
        vertices = n.value("Vertices").reshape(-1, 3)
        pvi = n.value("PolygonVertexIndex", np.array([], dtype=int))
        ends = np.flatnonzero(pvi < 0)
        sizes = np.diff(np.r_[-1, ends])
        skin_ids = [c for c in children[i] if c in skins]
        cluster_ids = [c for s in skin_ids for c in children[s] if c in clusters]
        sums = np.zeros(len(vertices)); counts = np.zeros(len(vertices), dtype=int)
        maxweights = np.zeros(len(vertices)); invalid_indices = 0
        weights_by_point = [[] for _ in vertices]
        bone_weight_records = []
        for cid in cluster_ids:
            c = clusters[cid]; idx = c.value("Indexes", np.array([], dtype=int)); w = c.value("Weights", np.array([], dtype=float))
            valid = (idx >= 0) & (idx < len(vertices)); invalid_indices += int((~valid).sum())
            idx, w = idx[valid], w[valid]
            np.add.at(sums, idx, w); np.add.at(counts, idx, w > 1e-8); np.maximum.at(maxweights, idx, w)
            for vi, wi in zip(idx, w):
                if wi > 1e-8:
                    weights_by_point[int(vi)].append(float(wi))
            bone = next((c for c in children[cid] if c in models), None)
            active = idx[w > 1e-8]
            if len(active):
                bone_weight_records.append({"bone": identity(bone) if bone else None, "cluster": cid, "weighted_control_points": len(active), "total_weight": float(w.sum()), "weighted_centroid_source_units": ((vertices[idx] * w[:, None]).sum(axis=0) / w.sum()).tolist() if w.sum() else None, "weighted_bounds_min": vertices[active].min(axis=0).tolist() if len(active) else None, "weighted_bounds_max": vertices[active].max(axis=0).tolist() if len(active) else None})
        lost4 = np.asarray([sum(sorted(w, reverse=True)[4:]) for w in weights_by_point])
        mesh_models = [p for p in parents[i] if p in models]
        model = mesh_models[0] if mesh_models else None
        wm = world(model)
        wp = (wm @ np.c_[vertices, np.ones(len(vertices))].T).T[:, :3]
        all_positions.append(wp)
        mesh_reports.append({**identity(i), "models": [identity(m) for m in mesh_models], "control_points": len(vertices), "polygon_corners": len(pvi), "polygons": len(ends), "polygon_size_histogram": {str(int(k)): int(v) for k, v in zip(*np.unique(sizes, return_counts=True))}, "triangles_if_fan_triangulated": int(np.maximum(sizes - 2, 0).sum()), "local_bounds_min": vertices.min(axis=0).tolist(), "local_bounds_max": vertices.max(axis=0).tolist(), "world_bounds_min": wp.min(axis=0).tolist(), "world_bounds_max": wp.max(axis=0).tolist(), "finite_vertices": bool(np.isfinite(vertices).all()), "skin_ids": skin_ids, "cluster_ids": cluster_ids, "weight_sums": summarize_numbers(sums), "unweighted_control_points": int((sums < 1e-8).sum()), "non_normalized_control_points_1e_5": int((np.abs(sums - 1) > 1e-5).sum()), "influence_histogram": {str(int(k)): int(v) for k, v in zip(*np.unique(counts, return_counts=True))}, "max_influences": int(counts.max()), "control_points_over_4_influences": int((counts > 4).sum()), "weight_lost_truncating_to_4": summarize_numbers(lost4), "control_points_losing_over_1_percent_to_4": int((lost4 > .01).sum()), "invalid_cluster_indices": invalid_indices, "bone_weights": bone_weight_records, "layers": [{"name": c.name, "properties": [str(v) for v in c.props]} for c in n.children if c.name.startswith("LayerElement")], "material_ids": [c for m in mesh_models for c in children[m] if c in by_id and by_id[c].name == "Material"]})
    pose_reports = []
    bind_nodes = {}
    for i, n in by_id.items():
        if n.name == "Pose":
            pnodes = []
            for p in n.children:
                if p.name == "PoseNode":
                    obj = int(p.value("Node")); mat = np.asarray(p.value("Matrix")).reshape(4, 4).T
                    key = (obj, tuple(mat.ravel()))
                    if key not in bind_nodes:
                        bind_nodes[key] = {"node": identity(obj), "matrix_ref": matrix_ref(p.value("Matrix")), "world_difference_max": float(np.max(np.abs(mat - world(obj)))) if obj in models else None}
                    pnodes.append(obj)
            pose_reports.append({**identity(i), "node_ids": pnodes})
    type_counts = collections.Counter((n.name, str(n.props[2]) if len(n.props) > 2 else "") for n in by_id.values())
    materials = [{**identity(i), "properties": simple_property_map(n)} for i, n in by_id.items() if n.name == "Material"]
    textures = []
    for i, n in by_id.items():
        if n.name in ("Texture", "Video"):
            filenames = {c.name: clean_name(c.props[0]) for c in n.children if c.name in ("FileName", "Filename", "RelativeFilename") and c.props}
            textures.append({**identity(i), **filenames, "embedded_content_bytes": len(n.value("Content", b"")), "properties": simple_property_map(n), "connections": [{"type": t, "source": identity(s), "target": identity(d), "property": more} for t, s, d, more in connections if s == i or d == i]})
    anims = [{**identity(i), "properties": simple_property_map(n), "key_count": int(len(n.value("KeyTime", []))), "children": [identity(c) for c in children[i]]} for i, n in by_id.items() if n.name.startswith("Animation")]
    morphs = [{**identity(i), "vertex_offsets": int(len(n.value("Indexes", []))), "properties": simple_property_map(n)} for i, n in by_id.items() if n.name == "Geometry" and n.props[2] == "Shape" or n.name == "Deformer" and n.props[2] in ("BlendShape", "BlendShapeChannel")]
    apos = np.vstack(all_positions)
    return {
        "source": str(Path(path)),
        "sha256": hashlib.sha256(fbx.data).hexdigest(),
        "file_bytes": len(fbx.data),
        "fbx_version": fbx.version,
        "global_settings": simple_property_map(fbx.root("GlobalSettings")),
        "object_counts": {k + (":" + v if v else ""): count for (k, v), count in type_counts.items()},
        "model_nodes": [{**identity(i), "parents": [identity(p) for p in parents[i] if p in models], "properties": simple_property_map(n)} for i, n in models.items() if i not in bones],
        "bone_count": len(bones),
        "bone_roots": [identity(i) for i in bones if not any(p in bones for p in parents[i])],
        "armature_node": identity(armature_id),
        "skeleton": skeleton,
        "meshes": mesh_reports,
        "nonempty_clusters": cluster_details,
        "empty_cluster_count": sum(empty_cluster_bone_counts.values()),
        "empty_cluster_counts_by_bone": [{"bone": identity(i), "count": count} for i, count in empty_cluster_bone_counts.items()],
        "matrix_library": matrix_library,
        "poses": pose_reports,
        "unique_bind_nodes": list(bind_nodes.values()),
        "materials": materials,
        "textures": textures,
        "animations": anims,
        "morphs": morphs,
        "combined_world_bounds_min": apos.min(axis=0).tolist(),
        "combined_world_bounds_max": apos.max(axis=0).tolist(),
        "limitations": [
            "Raw audit does not prove deformation quality; practical pose tests are required.",
            "World transforms use standard FBX pivot/pre/post rotation stack with parent matrix multiplication; non-default inheritance requires importer cross-check.",
            "Counts are original FBX control points, not importer split vertices.",
            "Only nonempty clusters are detailed; empty clusters are counted by bone. Exact repeated matrices are deduplicated by matrix_ref.",
            "Bind pose entries shared by multiple Pose objects are deduplicated in unique_bind_nodes.",
            "model_nodes contains non-bone nodes; bone model properties are in skeleton.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    print(json.dumps({k: result[k] for k in ("source", "sha256", "fbx_version", "object_counts", "bone_count", "bone_roots", "combined_world_bounds_min", "combined_world_bounds_max")}, ensure_ascii=False, indent=2))
    for m in result["meshes"]:
        print(json.dumps({k: m[k] for k in ("name", "control_points", "polygons", "max_influences", "unweighted_control_points", "non_normalized_control_points_1e_5", "control_points_over_4_influences", "control_points_losing_over_1_percent_to_4")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
