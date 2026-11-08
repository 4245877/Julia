"""Package current authoring assets without historical reports or Blender."""
import argparse
import hashlib
import json
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
README = """# Raphtalia - textured model

Open models/Raphtalia_prepared.blend for the editable rig and packed textures.
Extract the whole archive when importing models/Raphtalia_prepared.fbx:
its texture references require the sibling textures/raphtalia directory.
The prepared asset contains 429 bones, 15 baked clips and 24 sRGB colour maps.
Blender controls/drivers are authoring data; FBX contains their baked result.
manifest.json records the SHA256 of every packaged asset.

Texture artwork: Newtaw, Raphtalia - The Rising of the Shield Hero.
Reference: https://skfb.ly/ouUGR
See texture_sources.json for attribution and source information.
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'build/packages/Raphtalia_textured_4K.zip')
    args = parser.parse_args()
    files = {
        'models/Raphtalia_prepared.blend': ROOT / 'assets/models/Raphtalia_prepared.blend',
        'models/Raphtalia_prepared.fbx': ROOT / 'assets/models/Raphtalia_prepared.fbx',
        'texture_sources.json': ROOT / 'assets/metadata/raphtalia/texturing/texture_sources.json',
    }
    for path in sorted((ROOT / 'assets/textures/raphtalia').glob('*.png')):
        files['textures/raphtalia/' + path.name] = path
    manifest = {
        name: {'bytes': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        for name, path in files.items()
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, path in files.items():
            archive.write(path, name)
        archive.writestr('README.md', README)
        archive.writestr('manifest.json', json.dumps(manifest, indent=2))
    with zipfile.ZipFile(args.output) as archive:
        if archive.testzip() is not None:
            raise RuntimeError('Archive CRC verification failed')
        for name, record in manifest.items():
            if hashlib.sha256(archive.read(name)).hexdigest() != record['sha256']:
                raise RuntimeError('Archive hash mismatch: ' + name)
    print(f'Verified {len(files)} files: {args.output} ({args.output.stat().st_size} bytes)')


if __name__ == '__main__':
    main()
