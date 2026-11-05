"""Package the verified current asset and validate all FBX texture references."""
import hashlib
import json
import posixpath
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / 'docs/raphtalia/texturing'
OUT = ROOT / 'deliverables/raphtalia'

README = '''# Рафталия — текстурированная модель 4K

Подготовлено 5 ноября 2026.

Откройте models/Raphtalia_prepared.blend: все 24 изображения упакованы,
материалы настроены на unlit-стиль референса. Сохранены 28 мешей,
42 041 вершина, 68 944 полигона, исходные UV и веса, 429 костей и 15 Actions.

Для импорта FBX распакуйте архив целиком. Файл
models/Raphtalia_prepared.fbx использует относительные ссылки на
textures/raphtalia/*.png. Сохраните расположение этих папок.
В FBX остаются все 15 запечённых анимаций. Blender-контролы и drivers
находятся в BLEND; FBX хранит их ранее подготовленный результат.

24 цветовые карты имеют размер 4096 × 4096 и цветовое пространство sRGB.
Для точного вида референса используйте unlit/base-color shader:
тени, линии и блики уже нарисованы в изображениях. Normal, bump и
roughness-текстуры не нужны для исходного shadeless-оформления.
В освещаемом движке внешний вид зависит от света и шейдера.

25 материалов используют карты, tex_skin и tex_skin2 разделяют карту кожи.
Материал hightlight — постоянный светлый цвет на геометрии бликов без UV.

Источник оформления: Newtaw, Raphtalia — The Rising of the Shield Hero.
https://sketchfab.com/3d-models/raphtalia-the-rising-of-the-shield-hero-4b283bf024c3405ea6b8a9c68d0e1c5b
Короткая ссылка пользователя: https://skfb.ly/ouUGR
Карты восстановлены из публичного просмотрщика и согласованы с локальными UV.
Исходное авторство изображения и дизайна не изменяется.

preview.png показывает проверенные ракурсы. manifest.json содержит SHA256.
Полный технический отчёт находится в проекте:
docs/raphtalia/texturing/REPORT_RU.md
'''


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    files = {
        'models/Raphtalia_prepared.blend': ROOT/'assets/models/Raphtalia_prepared.blend',
        'models/Raphtalia_prepared.fbx': ROOT/'assets/models/Raphtalia_prepared.fbx',
        'preview.png': DOC/'preview.png',
        'texture_sources.json': DOC/'texture_sources.json',
    }
    for path in sorted((ROOT/'assets/textures/raphtalia').glob('*.png')):
        files['textures/raphtalia/'+path.name] = path
    manifest = {'date': '2026-11-05', 'reference': 'https://skfb.ly/ouUGR',
                'files': {name: {'bytes': path.stat().st_size,
                    'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                    for name, path in files.items()}}
    archive = OUT/'Raphtalia_textured_4K.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for name, path in files.items():
            z.write(path, name)
        z.writestr('README_RU.md', README)
        z.writestr('manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2))
    (OUT/'README_RU.md').write_text(README, encoding='utf-8')
    assimp = json.loads((DOC/'assimp.json').read_text(encoding='utf-8-sig'))['project_import']
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for mat in assimp['materials']:
            for texture in mat['textures']:
                target = posixpath.normpath('models/'+texture['path'].replace('\\', '/'))
                assert target in z.namelist(), (mat['name'], target)
        for name, record in manifest['files'].items():
            assert hashlib.sha256(z.read(name)).hexdigest() == record['sha256']
    report = {'archive': archive.relative_to(ROOT).as_posix(), 'bytes': archive.stat().st_size,
              'sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
              'payload_files': len(files), 'crc_check': 'PASS',
              'payload_hashes': 'PASS', 'all_fbx_texture_paths_resolve_inside_archive': True}
    (DOC/'package_verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
