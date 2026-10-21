"""Generate the current Russian report and asset manifest from verified evidence.

Refuses stale model hashes or mismatched before/after pictures. No model, rig,
runtime source, historical evidence, or secondary text report is modified.
"""
from pathlib import Path
import csv
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'docs/raphtalia'
STAGE = OUT / 'deep_deformation'
ORIGINAL = 'a602086ac9e580f2d265a489478922b75aae00410313b17a89bd846ad4f3b34c'


def read(path):
    return json.loads((OUT / path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def csv_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def main():
    blend = ROOT / 'assets/models/Raphtalia_prepared.blend'
    fbx = ROOT / 'assets/models/Raphtalia_prepared.fbx'
    blend_hash, fbx_hash = sha(blend), sha(fbx)
    before, after = (read('deep_deformation/' + n + '.json') for n in ('before', 'after'))
    old, legacy = (read('deep_deformation/legacy_' + n + '.json') for n in ('before', 'after'))
    integrity = read('deep_deformation/integrity.json')
    comparison = read('deep_deformation/comparison.json')
    patch = read('deep_deformation/weight_patch.json')
    ai = read('deep_deformation/assimp_after.json')
    controls = read('deep_deformation/controls.json')
    structure = read('deep_deformation/structure.json')
    limb = read('deep_deformation/limb_validation.json')
    sleeve = read('deep_deformation/sleeve_validation.json')
    rt, sweep = read('roundtrip.json'), read('joint_sweep.json')
    sweep_before = read('deep_deformation/joint_sweep_before.json')
    archive = ROOT / 'assets/models/backups/Raphtalia_prepared.pre_deep_deformation_2026-10-21.zip'
    with zipfile.ZipFile(archive) as retained:
        base_hash = hashlib.sha256(retained.read('Raphtalia_prepared.blend')).hexdigest()
    assert before['sha256'] == old['sha256'] == patch['sha256'] == base_hash
    assert sweep_before['sha256'] == base_hash and sweep_before['file_unchanged']
    assert after['sha256'] == legacy['sha256'] == sweep['sha256'] == blend_hash, 'Stale Blender pose evidence'
    assert before['file_unchanged'] and after['file_unchanged']
    assert limb['source_sha256'] == sleeve['input']['sha256'] == base_hash
    assert limb['source_unchanged']
    assert integrity['files']['base']['sha256'] == base_hash
    assert integrity['files']['candidate']['sha256'] == blend_hash, 'Stale integrity audit'
    assert integrity['result'] == 'PASS' and all(integrity['checks'].values())
    assert rt['files']['prepared_blend']['sha256'] == blend_hash
    assert rt['files']['prepared_fbx']['sha256'] == fbx_hash, 'Stale FBX roundtrip'
    assert rt['source_hash_unchanged'] and rt['files_stable_during_check']
    assert not rt['missing_actions'] and not rt['extra_actions']
    assert not rt['skeleton']['parent_mismatches']
    assert rt['skeleton']['blend_bones'] == rt['skeleton']['fbx_bones'] == 241
    assert rt['source_to_prepared_neutral']['surface_structure_preserved']
    assert rt['blend_to_fbx_neutral']['surface_structure_preserved']
    assert rt['weights_blend_to_fbx']['max_weight_delta'] < 1e-6
    assert rt['summary']['sampled_animation_max_delta_m'] < 2e-5
    clip_count = len(rt['animations_blend'])
    assert clip_count == len(rt['animations_fbx']) == 11
    assert controls['result'] == structure['result'] == 'PASS'
    assert after['summary']['nonfinite'] == legacy['summary']['nonfinite'] == 0
    assert sweep['summary']['nonfinite_vertex_total'] == 0 and sweep['file_unchanged']
    assert comparison['inputs']['before_model_sha256'] == base_hash
    assert comparison['inputs']['after_model_sha256'] == blend_hash
    assert all(comparison['checks'].values()), 'Before/after views are not comparable'
    for n in ('before', 'after'):
        assert comparison['inputs'][n]['sha256'] == sha(STAGE / (n + '.json'))
    for pair in comparison['image_pairs']:
        for image in pair['images'].values():
            assert sha(Path(image['path'])) == image['sha256'], 'Stale comparison picture'
    assert sha(Path(comparison['contact_sheet']['path'])) == comparison['contact_sheet']['sha256']
    assert ai.get('input_sha256') == fbx_hash and ai.get('input_stable_during_audit'), 'Stale Assimp audit; rerun build_assimp_audit.ps1'
    for mode in ('source_validation', 'project_import'):
        data = ai[mode]
        assert data['loaded'] and data['animation_count'] == clip_count
        assert data['unique_weighted_bone_names'] == 241 and data['max_positive_influences'] <= 4
        for key in ('unweighted_vertices', 'invalid_weight_entries', 'nonunit_weight_vertices_epsilon_1e_5',
                    'unresolved_bone_entries', 'nonfinite_global_bind_matrices', 'nonfinite_bind_deformed_vertices'):
            assert data[key] == 0, (mode, key)
        for action in data['animations']:
            assert action['nonfinite_key_entries'] == action['sampled_nonfinite_total'] == 0
            assert action['unresolved_or_ambiguous_channels'] == 0
        assert not any(n['name'].startswith(('CTRL_', '_auto_twist', '_curl_')) for n in data['nodes'])
    gpu = dict(line.split('=', 1) for line in (STAGE / 'gpu_after/gpu_audit.txt').read_text(encoding='utf-8').splitlines() if '=' in line)
    assert gpu.get('input_sha256') == fbx_hash and gpu.get('input_stable_during_audit') == 'YES', 'Stale GPU audit; rerun build_gpu_audit.ps1 and converter'
    assert gpu['animation_clips'] == str(clip_count) and gpu['clips_with_pixel_changes'] == str(clip_count)
    assert gpu['result'] == 'PASS' and gpu['gl_errors'] == '0' and gpu['bind_reset'] == 'EXACT'
    samples = csv_rows(STAGE / 'gpu_after/gpu_samples.csv')
    peaks = {float(r['fraction']) for r in samples if r['clip_name'].endswith('TEST_deep_body_cloth')}
    assert len(peaks) == 6
    endpoints = csv_rows(STAGE / 'gpu_after/gpu_endpoints.csv')
    assert not any(int(e['end_vs_start_changed_rgb_bytes']) for e in endpoints)
    files = [ROOT / n for n in (
        'assets/models/Raphtalia.fbx', 'assets/models/backups/Raphtalia.original.fbx',
        'assets/models/Raphtalia_prepared.blend', 'assets/models/Raphtalia_prepared.fbx',
        'assets/models/backups/Raphtalia_prepared.pre_deep_deformation_2026-10-21.zip',
        'assets/models/backups/Raphtalia_prepared.pre_reaudit_2026-10-21.zip')]
    assert sha(files[0]) == sha(files[1]) == ORIGINAL
    manifest = [{'path': p.relative_to(ROOT).as_posix(), 'bytes': p.stat().st_size, 'sha256': sha(p).upper()}
                for p in files]
    weights = integrity['weights']
    weight_rows = '\n'.join(f"| `{m['mesh']}` | {m['changed_vertices']} |" for m in weights['changes'])
    bow_changed = any(m['mesh'] == 'bow_tex_bow_0' for m in weights['changes'])
    bow_note = ''
    bow_visual = ''
    bow_limit = ''
    if bow_changed:
        bow = read('deep_deformation/bow_validation.json')
        bow_vertices = next(m['changed_vertices'] for m in weights['changes'] if m['mesh'] == 'bow_tex_bow_0')
        assert bow['changed_vertices'] == bow_vertices
        assert bow['sha256'] == bow['small_angle_reproduction_source']['sha256'] == base_hash
        assert bow['file_unchanged'] and bow['small_angle_reproduction_source']['file_unchanged']
        assert bow['render_source']['file_unchanged_during_trial']
        assert bow['replay_matches_historical_render_trial']['passed']
        assert bow['callable_check']['same_trial_weights']
        assert bow['callable_check']['changed_vertices_from_immutable_base'] == bow_vertices
        assert bow['callable_check']['candidate_idempotent_changed_vertices'] == 0
        assert bow['view']['same_pose_camera_settings']
        assert all(p['after']['over_2'] == p['after']['nonfinite'] == 0 for p in bow['poses'].values())
        for pair in bow['image_pairs']:
            for im in pair['images'].values():
                assert sha(ROOT / im['path']) == im['sha256'], 'Stale bow comparison picture'
        bow_note = ('\n- **Бант у воротника:** лента, узел, петли, короткие концы и отдельные островки толщины '
                    'согласованы с существующей `spine_03.x_016`. Разные clavicle-веса раньше растягивали '
                    'малые рёбра узла при одностороннем движении плеча. Геометрия аксессуара и шея не менялись. '
                    f"[Изолированная проверка](deep_deformation/bow_validation.json): {len(bow['poses'])} пар поз; "
                    'окончательная модель дополнительно проверена общим integrity/roundtrip/GPU.')
        bow_visual = (' Дополнительно сохранены пары банта при [движении ключицы до](deep_deformation/before_bow_clavicle.png) '
                      '/ [после](deep_deformation/after_bow_clavicle.png), '
                      '[движении шеи до](deep_deformation/before_bow_neck.png) / [после](deep_deformation/after_bow_neck.png) '
                      'и [верхней груди до](deep_deformation/before_bow_upper_chest.png) / [после](deep_deformation/after_bow_upper_chest.png). '
                      'Последняя пара также показывает сохранённое пересечение концов банта с бронёй.')
        bow_limit = (' При отдельном локальном наклоне верхней груди +20° концы банта пересекают нагрудник '
                     'так же, как до исправления крепления; эта коллизия не устранена.')
    selected = [('torso_turn_30', 'Поворот корпуса 30°'), ('torso_bend_45', 'Наклон корпуса 45°'),
                ('overhead_turn', 'Руки вверх + поворот'), ('shoulder_forward_100', 'Руки вперёд 100°'),
                ('knee_flex_120', 'Колени 120°'), ('ankle_-30', 'Стопы −30°'),
                ('hip_flex_90', 'Бёдра 90°'), ('squat', 'Глубокое приседание'), ('cloth_fan', 'Движение цепочек ткани')]
    metric_rows = '\n'.join(f"| {label} | {before['poses'][key]['max_ratio']:.3f} / {before['poses'][key]['over_2']} | {after['poses'][key]['max_ratio']:.3f} / {after['poses'][key]['over_2']} |"
                            for key, label in selected)
    regression_count = len(comparison['regressions'])
    max_p99 = max((r['deltas'].get('p99', 0) for r in comparison['regressions']), default=0)
    regression_metrics = sorted({m for r in comparison['regressions'] for m in r['metrics']})
    source, imported = ai['source_validation'], ai['project_import']
    import_rows = '\n'.join(f"| {label} | {source[k]} | {imported[k]} |" for k, label in (
        ('node_count', 'Узлы'), ('mesh_count', 'Meshes'), ('vertices', 'Вершины'), ('triangles', 'Треугольники'),
        ('material_count', 'Материалы'), ('animation_count', 'Клипы'), ('max_positive_influences', 'Максимум влияний')))
    neutral_rgb = max(int(e['start_vs_bind_changed_rgb_bytes']) for e in endpoints)
    neutral_palette = max(float(e['start_vs_bind_max_palette_delta']) for e in endpoints)
    strict_misses = sum(not s['positions_within_10_micrometres'] for s in rt['animation_samples'])
    positive_bones = {b['name'] for m in source['meshes'] for b in m['bones'] if b['weight_entries'] > 0}
    text = f'''# Raphtalia — деформации тела и одежды

Дата: **21 октября 2026**. Текущий результат: **241 деформирующая кость, {clip_count} TEST-клипов**, прежние Blender-контроллеры и локально улучшенные веса тела и костюма. Исходная точка этого этапа — фактическая подготовленная модель с 241 костью и 10 Actions из сохранённого архива. Состояние проверено заново в Blender, через экспорт FBX, Assimp и настоящий OpenGL-путь JULIA.

Работа ограничена телом и одеждой. Лицо, его кости, веса, Actions, ограничения и drivers сохранены; моргание, речь и эмоции не разрабатывались. Новых костей, Shape Keys, симуляции ткани или изменений C++-рендерера нет. Глубокие складки и пересечения юбки полностью устранёнными не объявляются.

## Файлы и возможность отката

| Файл | Состояние |
|---|---|
| [Raphtalia_prepared.blend](../../assets/models/Raphtalia_prepared.blend) | Текущая редактируемая модель и существующие средства управления |
| [Raphtalia_prepared.fbx](../../assets/models/Raphtalia_prepared.fbx) | Те же исправленные статические веса и {clip_count} запечённых клипов |
| [Архив перед этим этапом](../../assets/models/backups/Raphtalia_prepared.pre_deep_deformation_2026-10-21.zip) | Точные BLEND/FBX состояния 241/10 и manifest |
| [Оригинал](../../assets/models/Raphtalia.fbx) / [его backup](../../assets/models/backups/Raphtalia.original.fbx) | Оба SHA256 `{ORIGINAL.upper()}`; не изменены |
| [Манифест](deliverable_manifest.json) | Размеры и SHA256 текущих файлов и обоих prepared-архивов |
| [Команды и управление](../../tools/raphtalia/README.md), [runtime contract](runtime_audit.md) | Воспроизведение, экспорт и ограничения JULIA |

Нейтральные позиции вершин, топология, UV, материалы, corner normals, все 241 rest matrices/родители и прежние 10 Actions совпадают с началом этапа точно. Сохранены свойства и формы контроллеров, IK/FK, ограничения и drivers. [integrity.json](deep_deformation/integrity.json) содержит хэши, все проверки и каждую изменённую вершину. Защита лица охватывает все head/hair meshes и верхнюю часть body; neck-вес заклёпки брони не превращает эту деталь одежды в лицевую геометрию. Любые изменения весов за пределами явного списка тела/костюма запрещены проверкой.

## Воспроизведённые проблемы и приоритеты

| Приоритет | Реальный дефект и условия | Причина |
|---|---|---|
| 1 | Пояс, кольца и верх юбочных панелей ломаются при yaw корпуса 30°, наклоне 45° и комбинированной позе | Низ одежды привязан к lumbar spine, тогда как цепочки ткани начинаются от таза; отдельные части креплений имеют разные веса |
| 1 | Короткие складки чулка у колена вытягиваются при 90–135°, локальные швы обуви растягиваются при сгибании стопы | Неравномерный переход thigh/shin и противоположные назначения соседних вершин сложенного шва |
| 2 | Нагрудник и его отделка деформируются как мягкая ткань, плечевые заклёпки расходятся при поднятии руки | Части одной жёсткой детали имеют разные влияния; одна заклёпка полностью следует шее |
| 2 | Узел и ленты банта растягиваются при одностороннем движении ключицы | Отдельные островки одного аксессуара имеют разные clavicle-веса |
| 2 | Переходы подмышки, рукава и таза дают локальные заломы при подъёме рук, twist и глубоком сгибании бедра | Неравномерный переход влияний по UV-разрывам; малый spine residual чередуется между соседними сегментами |
| 3 | Сильное уплощение глубокого сгиба и проникновение коленей через перед юбки | Ограничения исходной низкополигональной формы и LBS; свободная ткань не имеет автоматических коллизий |

Дефекты воспроизведены по реальным вершинам и позам, а не перенесены из прежнего отчёта: [before.json](deep_deformation/before.json), [сравнение](deep_deformation/comparison.png). Углы — добавки к асимметричной запечённой rest pose, а не медицинский диапазон движения. Исправные суставы и иерархия не перестраивались.

## Выполненные локальные исправления

Изменены **{weights['changed_vertices']} уникальных вершин в {weights['changed_meshes']} meshes**:

| Mesh | Изменено вершин |
|---|---:|
{weight_rows}

- **Тело:** переходы рука/торс в подмышках, бедро/таз и малые остаточные spine influences верхней груди. Фильтруется скалярная доля двух физических семейств по графу рёбер и совпадающим seam-вершинам. Существующие кости влияния и отношения основной/twist внутри семейства сохраняются. Лицо и шея исключены отдельно; локти, кисти и стопы тела не подвергались общему сглаживанию.
- **Рукава:** отдельно согласованы shoulder-girdle/chest и arm/girdle переходы; жёсткие и отсутствующие семейства влияний не создаются заново. Манжеты и перчатки сохранены. Дополнительная [проверка рукавов](deep_deformation/sleeve_validation.json): {sleeve['summary']['samples']} позовых samples, включая прежние клипы и знаковые стресс-повороты; роста max ratio/over_2 в этой выборке нет.
- **Чулки:** ограниченная offline-подгонка только доли shin/thigh по нескольким движениям, с сохранением списка влияний и primary/helper split. Бюджет оптимизатора — 100 вычислений; остановка по бюджету не трактуется как доказательство сходимости. Полученные статические веса проверены на дополнительных движениях, которые не входили в подбор.
- **Обувь:** согласованы лишь короткие связные складки у голеностопа. Подошва, носок и голенище не превращены в один жёсткий блок. [limb_validation.json](deep_deformation/limb_validation.json) содержит {limb['samples_per_stage']} samples на вариант, независимые углы, поперечные/продольные повороты и прежние Actions.
- **Пояс и свободные панели:** attachment нижней части согласован с `root.x_01`, выше сохранён плавный переход к lumbar spine. Замкнутый пояс, его пряжка и целые островки металлических колец следуют единому attachment. Существующие 54 кости ткани и их родители не менялись.
- **Броня:** shell/толщина/отделка нагрудника согласованы с chest; высокие clavicle straps оставлены. Плечевая пластина и заклёпки следуют одной существующей arm-twist кости. Это локальное исправление жёсткой детали, а не глобальная rigid-привязка одежды.
{bow_note}

Все итоговые веса нормализованы, не более четырёх влияний на вершину. Полный журнал before/after: [weight_patch.json](deep_deformation/weight_patch.json). Исходные twist drivers и управление остаются рабочими. Blender-зависимых corrective drivers/morphs в результате нет: исправления работают в том же LBS, который использует JULIA.

**Отклонённые варианты:** общий knee smoothing ухудшал отдельные сгибы; широкая диффузия обуви давала паразитную мягкость и ухудшала промежуточные движения. Автоматический подъём юбочных цепочек от бедра менял силуэт и растягивал швы панелей: [три отклонённых варианта](deep_deformation/rejected_lift.json). Эти изменения не включены. Численное улучшение не использовалось для оправдания заметного визуального ухудшения.

## Визуальные сравнения и численные результаты

[Семь пар «до → после»](deep_deformation/comparison.png) используют одинаковые позы, камеры, видимые объекты, масштаб и освещение: талия при повороте/наклоне, плечи в двух движениях, колени, голеностоп и таз. [comparison.json](deep_deformation/comparison.json) проверяет определения поз/камер, хэши изображений и содержит per-pose/per-mesh дельты.

Видимое улучшение наиболее выражено у пояса, круглых креплений, верхних панелей и края нагрудника. Рукава и переходы ног улучшены локально; внешняя разница отдельных кадров невелика. В OpenGL также проверены все шесть пиков нового клипа с близкими видами плеча, талии, колена и обуви. Отсутствие новых разрывов не означает отсутствия всех пересечений.{bow_visual}

Таблица: максимальное отношение длины ребра к rest / число рёбер >2×. Исключены почти нулевые rest-рёбра. Эти показатели локализуют растяжение, но не измеряют естественность, потерю объёма и коллизии.

| Поза | До | После |
|---|---:|---:|
{metric_rows}

В {regression_count} парах pose/mesh присутствуют увеличившиеся диагностические показатели ({', '.join(regression_metrics) or 'нет'}); наибольшая прибавка p99 — {max_p99:.5f}. Все дельты сохранены, а не скрыты усреднением. Оценка принятых компромиссов опирается также на локальные кадры и независимые проверки промежуточных движений. Модель не заявляется идеальной в каждом крайнем положении.

## Практические испытания и экспорт

| Проверка | Фактический результат |
|---|---|
| Новый body/clothing audit | {len(after['poses'])} отдельных и комбинированных поз; NaN/Inf = 0 |
| Прежний широкий аудит | {old['summary']['poses']} поз до / {legacy['summary']['poses']} после; дополнительная запись — новый TEST Action; NaN/Inf = 0 |
| Каждая локальная ось всех костей, +5° | {sweep['summary']['poses']} вращения; NaN/Inf = 0. Поз с рёбрами >2×: {sweep_before['summary']['poses_with_edges_stretched_over_2']} → {sweep['summary']['poses_with_edges_stretched_over_2']}; max ratio {sweep_before['summary']['max_edge_ratio_requested']:.3f} → {sweep['summary']['max_edge_ratio_requested']:.3f}. Это малые углы; метрики не измеряют объём и не заменяют глубокие позы: [до](deep_deformation/joint_sweep_before.json) / [после](joint_sweep.json) |
| Контроллеры и структура | [controls.json](deep_deformation/controls.json) / [structure.json](deep_deformation/structure.json): PASS; проверка сохранности — PASS |
| Blender → FBX → Blender | {rt['summary']['sample_count']} sampled poses, {clip_count}/{clip_count} Actions, 241/241 костей, родители и веса сохранены |
| Assimp {ai['assimp_version']} | Source validation и точные project postprocess flags; по девять CPU samples каждого клипа в каждом режиме, нечисловых значений нет |
| Реальный OpenGL | {gpu['rendered_images']} сохранённых изображений, все {clip_count} клипов меняют rendering, GL errors = {gpu['gl_errors']}, bind reset = EXACT |

Roundtrip: максимальная ошибка нейтрали {rt['summary']['roundtrip_neutral_max_delta_m'] * 1000:.5f} мм, анимационной поверхности {rt['summary']['sampled_animation_max_delta_m'] * 1000:.5f} мм. Порог 0.01 мм превышен в {strict_misses} samples; все ниже 0.02 мм. UV/топология/материалы сохранены; небольшая разница corner normals после FBX измерена и не выдаётся за побайтовое совпадение shading. Полные данные: [roundtrip.json](roundtrip.json).

Текущий FBX через Assimp:

| Показатель | Source validation | Project postprocess |
|---|---:|---:|
{import_rows}

В обоих режимах invalid/unweighted/nonunit vertices, unresolved bones/channels и нечисловые ключи/матрицы/поверхности — ноль. 241 palette entry включает пустые clusters; положительные vertex assignments имеют {len(positive_bones)} имён костей. Два вырожденных треугольника удаляет прежний project pipeline. Текстуры/UV/материалы не удалялись и не переназначались. [assimp_after.json](deep_deformation/assimp_after.json).

GPU использует существующие `GLModelLoader` и `OpenGLRenderer`, скрытый GL 3.3 context и независимый TRS/quaternion sampler. Проверены настоящие экспортированные движения, не только успешная загрузка. Начало и конец каждого клипа совпадают побайтово на всех десяти ракурсах. Animation-neutral отличается от imported bind на малую величину: max palette delta {neutral_palette:.8g}, максимум {neutral_rgb} RGB bytes; поэтому равенство animation-neutral и bind не заявляется. Отдельная повторная отрисовка bind точна. Сохраняются прежние masked-composition и пять TRS crossfade-проверок.

GPU evidence: [протокол](deep_deformation/gpu_after/gpu_audit.txt), [samples](deep_deformation/gpu_after/gpu_samples.csv), [endpoints](deep_deformation/gpu_after/gpu_endpoints.csv), [body/clothing closeups](deep_deformation/gpu_after/gpu_body_cloth_contact.png), [baseline](deep_deformation/gpu_before/gpu_body_cloth_contact.png). SHA256 в Blender/Assimp/GPU evidence и манифесте привязаны к текущим файлам; генератор отчёта отклоняет устаревшие результаты.

## Оставшиеся ограничения и следующий этап

1. Глубокие сгибы коленей/локтей сохраняют характерное для LBS уплощение и исходную гранёность. Общий DQS/Preserve Volume только в Blender нарушил бы соответствие JULIA, поэтому не включён.
2. При глубоком приседании и высоком шаге колени/бёдра могут проходить через юбку. Существующие цепочки позволяют ручное позирование панелей; автоматических коллизий и физики ткани нет. Умеренные движения устойчивее, но универсальная collision-free сертификация не заявляется.
3. При крайнем подъёме вперёд плечевая пластина может располагаться почти ребром и тесно прилегать к рукаву; под грудью в сильном наклоне остаётся жёсткое сжатие.{bow_limit} Эти места требуют авторинга конкретных движений и дальнейшей локальной проверки.
4. Исходная асимметричная rest pose, масштаб и ориентации локальных осей не изменены. Внешние движения требуют согласования retargeting, контактов и root motion. Автоматического FK/IK matching и полного foot-roll нет; прежнее управление сохранено.
5. Основное приложение по-прежнему статическое: standalone sampler не добавляет animation player, state machine или morph playback. Новые веса уже совместимы с runtime skinning; воспроизведение клипов — отдельная интеграционная задача.
6. Крайние сгибания запястья +45/+60° и пальцев сохраняют прежние ограничения сжатия/пересечения. Эти зоны проверены, но их веса на этом этапе не менялись; успешный finite/GPU-тест не означает естественной формы в крайних позах.

Для следующего этапа: авторить движения с ручными ключами существующих юбочных цепочек и проверить контакты; затем отдельно рассматривать совместимые pose-correctives, только с реальной поддержкой в JULIA и новым roundtrip/GPU сравнением. Лицевая анимация остаётся отдельной будущей задачей.

Исходный `Raphtalia.fbx`, текстуры, другие модели и ранние backups сохранены. Промежуточные пробы находятся в build и не заменяют final evidence; очистка затрагивает только явно созданные на этом этапе временные файлы: [журнал](deep_deformation/cleanup.json). `reaudit/` и `history/2026-10-20/` относятся к предыдущим этапам, а не к текущим количествам костей/клипов.
'''
    # Assertions finish before publishing either artifact.
    assert (sha(blend), sha(fbx)) == (blend_hash, fbx_hash), 'Models changed while building report'
    (OUT / 'REPORT_RU.md').write_text(text, encoding='utf-8')
    (OUT / 'deliverable_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps({'report': str(OUT / 'REPORT_RU.md'), 'bones': 241, 'clips': clip_count,
                      'changed_vertices': weights['changed_vertices'], 'files': len(manifest)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
