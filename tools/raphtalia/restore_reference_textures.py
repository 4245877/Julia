"""Restore publicly served reference colour maps; never downloads mesh data."""
import hashlib
import io
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import requests
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
MODEL = '4b283bf024c3405ea6b8a9c68d0e1c5b'
OUT = ROOT / 'assets/textures/raphtalia'
METADATA = ROOT / 'assets/metadata/raphtalia/texturing'
DOC = ROOT / 'build/diagnostics/texturing'
RAW = DOC / 'raw'


def viewer_pixels(image, key):
    """Convert the public viewer's pv=1 pixel layout to top-left PNG layout.

    The format orders 8x8 blocks along alternating diagonals, rotates blocks,
    and offsets this sequence by the public per-image block index.
    Format reference: https://github.com/Corpocore/sfrip/blob/main/lib/descramble.js
    Independent NumPy block implementation; no external code is executed.
    """
    src = np.asarray(image.convert('RGBA'))
    height, width, channels = src.shape
    assert height % 8 == 0 and width % 8 == 0
    nx, ny = width // 8, height // 8
    order = []
    for diagonal in range(nx + ny - 1):
        xs = list(range(max(0, diagonal-ny+1), min(nx-1, diagonal)+1))
        if diagonal % 2:
            xs.reverse()
        order.extend((diagonal-x)*nx+x for x in xs)
    order = np.array(order, dtype=np.int32)
    blocks = src.reshape(ny, 8, nx, 8, channels).transpose(0, 2, 1, 3, 4).reshape(-1, 8, 8, channels)
    sequence = blocks[order].copy()
    sequence[1::4] = sequence[1::4, :, ::-1].copy()
    sequence[2::4] = sequence[2::4].transpose(0, 2, 1, 3).copy()
    sequence[3::4] = np.rot90(sequence[3::4], -1, axes=(1, 2)).copy()
    sequence = np.roll(sequence, int(key), axis=0)
    sequence[1::4] = sequence[1::4, :, ::-1].copy()
    sequence[2::4] = sequence[2::4].transpose(0, 2, 1, 3).copy()
    sequence[3::4] = np.rot90(sequence[3::4], 1, axes=(1, 2)).copy()
    output = np.empty_like(sequence)
    output[order] = sequence
    output = output.reshape(ny, nx, 8, 8, channels).transpose(0, 2, 1, 3, 4).reshape(height, width, channels)
    # The local FBX/BLEND UV V convention is the inverse of the web viewer's
    # top-left display convention. Keep the unflipped raster for these UVs.
    return Image.fromarray(output)


def restore(item):
    candidates = [im for im in item['images'] if im.get('url')]
    best = max(candidates, key=lambda im: (
        im['width'] * im['height'], im['url'].split('?')[0].endswith('.png')))
    raw_path = RAW / item['name']
    if raw_path.exists():
        content = raw_path.read_bytes()
    else:
        session = requests.Session()
        session.trust_env = False
        response = session.get(best['url'], timeout=60)
        response.raise_for_status()
        content = response.content
        raw_path.write_bytes(content)
    with Image.open(io.BytesIO(content)) as image:
        image.load()
        assert image.size == (best['width'], best['height'])
        assert image.format == 'PNG', 'Keep the lossless source, not a JPEG derivative.'
        dimensions, mode = image.size, image.mode
        decoded = viewer_pixels(image, best['pk']) if best.get('pv') == 1 else image.convert('RGBA')
        alpha = decoded.getchannel('A').getextrema()
    path = OUT / item['name']
    if alpha == (255, 255):
        decoded = decoded.convert('RGB')
    decoded.save(path)
    print(item['name'], dimensions, mode, 'alpha', alpha, flush=True)
    return {'material': path.stem, 'file': path.relative_to(ROOT).as_posix(),
            'size': dimensions, 'mode': decoded.mode, 'source_mode': mode, 'alpha_range': alpha,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'source_bytes_sha256': hashlib.sha256(content).hexdigest(),
            'public_pixel_layout': {'pv': best.get('pv'), 'pk': best.get('pk')},
            'uv_orientation': 'Raster adapted to the unchanged local FBX UV V direction',
            'source_texture_uid': item['uid'], 'source_url': best['url']}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    DOC.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    metadata = json.loads((METADATA / 'reference_texture_manifest.json').read_text(encoding='utf-8-sig'))
    with ThreadPoolExecutor(max_workers=6) as pool:
        maps = list(pool.map(restore, metadata['results']))
    report = {'reference': 'https://skfb.ly/ouUGR',
              'canonical_reference': 'https://sketchfab.com/3d-models/raphtalia-the-rising-of-the-shield-hero-' + MODEL,
              'author': 'Newtaw', 'method': 'Lossless colour maps restored from the public reference viewer; no generated artwork or geometry downloaded.',
              'maps': maps}
    (DOC / 'texture_sources.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
