"""Read-only material provenance audit; never guesses maps from material names.

Run with the project's bpy PYTHONPATH. Does not save BLEND or export FBX.
"""
import argparse
import hashlib
import json
from pathlib import Path

import bpy
from PIL import Image
from fbx_audit import BinaryFBX, clean_name


def inspect_fbx(path):
    objects = BinaryFBX(path).root('Objects').children
    return {
        'path': path.as_posix(),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'materials': [clean_name(n.props[1]) for n in objects if n.name == 'Material'],
        'textures': [clean_name(n.props[1]) for n in objects if n.name == 'Texture'],
        'videos': [clean_name(n.props[1]) for n in objects if n.name == 'Video'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='docs/raphtalia/desktop/materials.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    bpy.ops.wm.open_mainfile(filepath=str(root / 'assets/models/Raphtalia_prepared.blend'))
    report = {
        'blender_version': bpy.app.version_string,
        'blend_sha256': hashlib.sha256((root / 'assets/models/Raphtalia_prepared.blend').read_bytes()).hexdigest(),
        'blend_images': [{'name': i.name, 'path': i.filepath, 'packed': bool(i.packed_file)} for i in bpy.data.images],
        'blend_materials': [],
        'fbx': [inspect_fbx(root / 'assets/models' / name) for name in ('Raphtalia.fbx', 'Raphtalia_prepared.fbx', 'Raphtalia1.fbx')],
        'available_images': [],
    }
    for material in bpy.data.materials:
        tree = material.node_tree if material.use_nodes else None
        report['blend_materials'].append({
            'name': material.name,
            'diffuse': list(material.diffuse_color),
            'image_nodes': [{'node': n.name, 'image': n.image.name if n.image else None}
                            for n in tree.nodes if n.type == 'TEX_IMAGE'] if tree else [],
            'links': [(l.from_node.name, l.from_socket.name, l.to_node.name, l.to_socket.name)
                      for l in tree.links] if tree else [],
        })
    for path in sorted((root / 'assets/textures').glob('*.png')):
        with Image.open(path) as image:
            report['available_images'].append({'path': path.relative_to(root).as_posix(),
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'size': image.size,
                'mode': image.mode, 'alpha_range': image.getchannel('A').getextrema() if 'A' in image.getbands() else None})
    out = root / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Material evidence: {out}; BLEND images={len(report["blend_images"])}')


if __name__ == '__main__':
    main()
