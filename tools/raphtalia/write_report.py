"""Generate one current report and manifest, refusing stale model evidence."""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'docs/raphtalia'
def read(path):return json.loads((OUT/path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    a=read('reaudit/after.json');b=read('reaudit/before.json');rt=read('roundtrip.json')
    ai=read('reaudit/assimp_after.json');ctrl=read('reaudit/controls.json');st=read('reaudit/structure.json')
    sw=read('joint_sweep.json');oldsw=read('reaudit/joint_sweep_before.json');patch=read('reaudit/weight_patch.json');clean=read('reaudit/cleanup.json')
    raw=read('reaudit/raw_prepared.json')
    positive_bones={w['bone']['name'] for m in raw['meshes'] for w in m['bone_weights'] if w['total_weight']>0}
    files=['assets/models/Raphtalia.fbx','assets/models/backups/Raphtalia.original.fbx',
      'assets/models/Raphtalia_prepared.fbx','assets/models/Raphtalia_prepared.blend',
      'assets/models/backups/Raphtalia_prepared.pre_reaudit_2026-10-21.zip']
    manifest=[{'path':n,'bytes':(ROOT/n).stat().st_size,'sha256':sha(ROOT/n).upper()} for n in files]
    assert a['sha256']==sha(ROOT/files[3])==rt['files']['prepared_blend']['sha256']==sw['sha256']
    assert rt['files']['prepared_fbx']['sha256']==sha(ROOT/files[2])
    assert st['result']==ctrl['result']=='PASS'
    assert a['bones']==241 and len(rt['animations_blend'])==10
    assert a['summary']['nonfinite']==0 and sw['summary']['nonfinite_vertex_total']==0
    assert rt['files_stable_during_check'] and rt['summary']['sampled_animation_max_delta_m']<2e-5
    assert rt['source_to_prepared_neutral']['surface_structure_preserved'] and rt['blend_to_fbx_neutral']['surface_structure_preserved']
    assert rt['weights_blend_to_fbx']['max_weight_delta']<1e-6
    for w in a['weights'].values():
        assert w['max_influences']<=4 and w['unweighted']==0 and w['max_sum_error']<1e-5 and not w['unknown_groups']
    for mode in ('source_validation','project_import'):
        data=ai[mode]
        assert data['loaded'] and data['max_positive_influences']<=4
        for key in ('unweighted_vertices','invalid_weight_entries','nonunit_weight_vertices_epsilon_1e_5','unresolved_bone_entries','nonfinite_global_bind_matrices','nonfinite_bind_deformed_vertices'):assert data[key]==0,key
        for clip in data['animations']:
            assert clip['nonfinite_key_entries']==clip['sampled_nonfinite_total']==0
            assert not clip['unresolved_or_ambiguous_channels']
        assert not any(n['name'].startswith(('CTRL_','_auto_twist','_curl_')) for n in data['nodes'])
    gpu=(OUT/'reaudit/gpu_after/gpu_audit.txt').read_text()
    assert all(line in gpu.splitlines() for line in ('animation_clips=10','rendered_images=100','gl_errors=0','bind_reset=EXACT','result=PASS'))
    assert manifest[0]['sha256']==manifest[1]['sha256']=='A602086AC9E580F2D265A489478922B75AAE00410313B17A89BD846AD4F3B34C'
    assert not rt['missing_actions'] and not rt['extra_actions'] and not rt['skeleton']['parent_mismatches']
    (OUT/'deliverable_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    tests=[('TEST_elbows_90','Прежний тест локтей +90°'),('elbow_-120','Локти −120°'),('TEST_knees_90','Колени 90°'),
      ('knee_120','Колени 120°'),('ankle_-35','Голеностопы −35°'),('ankle_35','Голеностопы +35°'),
      ('toes_-30','Носки стоп −30°'),('toes_30','Носки стоп +30°'),('TEST_fingers_35','Прежний тест фаланг 35°'),
      ('finger_curl_30','Фаланги +30° по поперечным осям'),('finger_curl_60','Фаланги +60°, стресс'),
      ('wrist_flex_60','Кисти +60°, стресс'),('wrist_roll_60','Поворот кистей +60°'),
      ('shoulder_raise_100','Плечи 100°, ключицы неподвижны'),('shoulder_coordinated','Ключицы 15° + плечи 80°'),
      ('torso_head','Корпус 30° + шея 15° + голова 60°'),('secondary_all','Все цепочки волос/ткани/хвоста совместно'),('combined','Комбинированная поза')]
    rows=[]
    for key,label in tests:
        x=b['poses'][key];y=a['poses'][key]
        rows.append(f"| {label} | {x['max_ratio']:.3f} / {x['over_2']} | {y['max_ratio']:.3f} / {y['over_2']} |")
    patch_rows='\n'.join(f"| `{p['mesh']}` | {len(p['changes'])} |" for p in patch)
    strict=sum(not x['positions_within_10_micrometres'] for x in rt['animation_samples'])
    text=f'''# Raphtalia — повторный аудит и состояние модели

Дата: **21 октября 2026**. Проверены фактические FBX/BLEND, исходники JULIA, прежние скрипты, JSON и изображения. Инструменты: bpy 4.2.0, Assimp {ai['assimp_version']}, MSVC, настоящие GLModelLoader/OpenGLRenderer проекта, NumPy/SciPy/Pillow.

**Модель улучшена и пригодна для разработки animation player и авторинга движений. Полностью готовым универсальным анимационным персонажем она не объявляется.** Остаются деформации глубоких поз, отдельные швы одежды, ручная работа над веками/ртом и проверка gait/contact. Основной C++-рендерер и desktop playback не изменены.

## Файлы и сохранность

| Файл | Назначение |
|---|---|
| [Raphtalia_prepared.blend](../../assets/models/Raphtalia_prepared.blend) | Редактируемая модель, Blender-контроллеры, 10 Actions |
| [Raphtalia_prepared.fbx](../../assets/models/Raphtalia_prepared.fbx) | 241 deform bone, 10 запечённых TEST-клипов, без управляющих объектов |
| [Оригинал](../../assets/models/Raphtalia.fbx) / [резервная копия](../../assets/models/backups/Raphtalia.original.fbx) | Не изменены; SHA256 A602086AC9E580F2D265A489478922B75AAE00410313B17A89BD846AD4F3B34C |
| [Архив до повторного аудита](../../assets/models/backups/Raphtalia_prepared.pre_reaudit_2026-10-21.zip) | Побайтовые prepared FBX/BLEND до изменений и manifest для отката |
| [Манифест](deliverable_manifest.json) / [иерархия CSV](skeleton_prepared.csv) | Текущие размеры, хэши, 241 head/tail/parent |
| [Воспроизведение и управление](../../tools/raphtalia/README.md) | Команды, контроллеры, ограничения экспорта |

`Raphtalia1.fbx` сохранён: это другой asset, используемый `tests/test_opengl.cpp`. Текстуры не удалялись и не переназначались. У оригинала нет texture references; JULIA использует существующий material/fallback-путь. Новые назначения неизвестных карт не выдумывались.

## 1. Независимо подтверждённое состояние

Оригинал и prepared повторно импортированы; выполнены собственные позовые испытания, Assimp/CPU, реальная GPU-отрисовка и сравнение поверхности. Прежний отчёт не был единственным доказательством.

- Все **60** записанных refit-положений суставов действительно присутствуют. **9** исправлений родителей лент ткани и **5** исправлений terminal tails подтверждены по данным модели. [structure.json](reaudit/structure.json).
- Один корень; вырожденных костей и невалидных rest matrices нет. Все 28 meshes используют LBS без Preserve Volume. Bind/rest проверены нейтральной деформацией и экспортным roundtrip.
- Плечи, таз, позвоночник, шея/голова сохранены. Локти, кисти, фаланги, колени и стопы следуют ранее восстановленной запечённой позе. В этом аудите pivots не перемещались. Вид [спереди](reaudit/joints_front.png) и [сбоку](reaudit/joints_side.png) проверен по реальной поверхности.
- Асимметричные ноги и согнутая левая кисть принадлежат исходной геометрии. Формальная симметризация не проводилась. Это ещё не стандартная T/A reference pose; внешние клипы требуют retargeting.
- Центры суставов остаются приближённой реконструкцией по геометрии/весам, а не восстановленной авторской анатомией. Практические позы проверены; строгая анатомическая сертификация не заявляется.
- Часть прежних diagnostic JSON фиксировала промежуточные веса перчатки/манжет. Их плохие назначения не приписываются окончательной модели; старые данные отделены от текущих.

## 2. Скелет и исправления весов

**241 deform bone:** 31 тело/корни/twist, 30 фаланг, 116 волос, 54 ткани, 4 хвоста, 6 лица. К прежним 239 добавлены только **brow.L/R**, по 17 реальных вершин брови на сторону. Остальные управляющие элементы — объекты Blender, не кости FBX. Сохраняются дерево root → pelvis/spine → limbs/head, ветви twist и независимые ribbons/цепочки волос и хвоста. Имена и rest matrices прежних костей сохранены.

Из них **{len(positive_bones)} имеют положительные vertex weights**, а 13 — необходимые корневые/родительские узлы, передающие движение потомкам. Assimp возвращает 241 имя `aiBone`/palette entry, включая пустые cluster entries; это не число напрямую взвешенных костей. Такие счётчики не используются как доказательство качества. Raw FBX содержит {len(raw['nonempty_clusters'])} непустых и {raw['empty_cluster_count']} пустых mesh/bone clusters, создаваемых экспортёром; дополнительная оптимизация FBX не выполнялась.

Blender Z up, 0.01 м на исходную единицу; FBX Y up / −Z forward. Высота около 4.16 единицы; автоматического назначения человеческого роста нет. Для C++ надо согласовать physical scale и root motion.

Помимо 34 вершин новых бровей, изменены веса **{sum(len(p['changes']) for p in patch)} вершин в пяти meshes**: области плеч, шеи, локтей, кистей/фаланг, коленей, голеностопов, носков стоп и пояса. Остальные meshes не перевзвешивались.

| Mesh | Изменено вершин |
|---|---:|
{patch_rows}

Метод: ограниченная диффузия по настоящим рёбрам и совпадающим UV-seam вершинам, обратный квадрат длины, компактные маски у суставов, шесть итераций с привязкой к исходным весам. Узкие связанные сгибы обуви/стопы/пояса согласованы по весам. Сохранены четыре сильнейших влияния и нормализация. Полного automatic/heat weighting нет. Полный before/after журнал: [weight_patch.json](reaudit/weight_patch.json).

Ранее не испытанный дефект носков стоп: rigid foot/toe assignments соседних вершин растягивали ребро 1.258 мм до 20.04 мм при −30°. Исправлен плавный локальный переход; в обоих тестах ±30° флагов >2× теперь нет. Обувь целиком жёсткой не делалась: голенище и носок сохраняют подвижность.

Малый sweep также выявил переход spine/cloth у пояса: ребро 2.13 мм достигало 18.16 мм при 5° spine yaw. Шов локально смягчён и короткие связанные складки согласованы; остатки требуют ручной проверки конструкции одежды. Веса у основания большого пальца кожи сохранены исходными там, где пробное сглаживание ухудшало отдельный тест 60°.

Нейтральная геометрия, UV, топология и материалы не редактировались. Новые brow bones имеют корректный bind и не меняют внешность в нейтрали. Исправные pivots/иерархии волос, хвоста, ушей, манжет и аксессуаров сохранены.

## 3. Управление в Blender

- Четыре двухзвенных IK рук/ног, hand/foot targets, pole targets, отключённый stretch, ориентация кисти/стопы от target. IK/FK influence в Object custom properties Armature; по умолчанию 0.
- Pole angles откалиброваны по асимметричной rest pose. В восьми перемещениях targets максимальная ошибка конца цепочки **{max(x['end_error_m'] for x in ctrl['ik'])*1000:.4f} мм**, изменение длины <0.1 мм. Переключение в rest: максимальный сдвиг головы кости **{max(x['rest_switch_head_jump_m'] for x in ctrl['ik'])*1000:.4f} мм**. Для произвольной FK-позы targets надо предварительно совмещать; автоматического snap/matching нет.
- Десять finger curl controls: добавочное сгибание трёх фаланг каждого пальца, с поперечной осью каждого сегмента. Левая кисть уже согнута; начинать с 0..30°, +60° — стресс.
- Два forearm helpers получают половину локального продольного twist кисти через swing/twist-разложение. Из 60° получено 30°. Ручной FK сохраняется. Upper-arm/leg twist остаются FK: полной автоматизации восьми helpers нет.
- Тело управляется существующим `root.x_01`; глаза/уши/брови доступны в коллекции Face. Новые фиктивные body/facial controls не добавлялись.

[control_layout.json](reaudit/control_layout.json) описывает связи, [controls.json](reaudit/controls.json) подтверждает каждый curl control, IK и twist. Невалидных drivers нет, возврат к нейтрали точный. Сложный heel/toe foot roll, полный набор joint limits, FK/IK snap и collision system не реализованы: они требуют проверки походки/контактов и не нужны для текущего FK-экспорта.

## 4. Лицо

Работают отдельные радужки/блики, уши и новые брови. Проверены горизонтальный gaze ±12°, вертикальный ±8°, уши ±20°, подъём бровей на 0.8 исходной единицы и сочетания с головой/волосами. Радужки — patch surfaces, не сферические eyeballs; большие углы не гарантируются. [Горизонтальный gaze](reaudit/after_gaze_horizontal_1.png), [вертикальный](reaudit/after_gaze_vertical_1.png), [брови](reaudit/face_brows_raised.png).

Пробный morph моргания сводил две 16-вершинные границы глаз в линию закрытия, плавно деформируя кожу и ресницы. Он закрывал проём, но создавал складки щеки/века и смещал нижние ресницы. **Кандидат отклонён, в модель не включён**: [rejected_blink.png](reaudit/rejected_blink.png), воспроизведение — [face_blink_probe.py](../../tools/raphtalia/face_blink_probe.py). Сжатие глаз не подменяло моргание.

Нужны вручную вылепленные верхние/нижние eyelid shapes с проверкой lash attachments, уголков и толщины века. Полная смена топологии заранее не требуется, но одной автоматической маски оказалось недостаточно. В ротовой области есть внутренняя поверхность, однако качественные jaw/lip/cheek/inner-mouth deformations и visemes не созданы. Нужны ручное авторство и визуальная QA; необходимость локального изменения геометрии определяется при ней. Открывание/закрывание рта, речь и полноценные эмоции **не готовы**. Фиктивных blink/lip controllers и morph targets нет. Будущие Shape Keys потребуют поддержки importer/buffer/shader в C++; bone-мимика использует нынешний skinning path.

## 5. Сравнение деформаций

«До» — реальная prepared-модель с 239 костями начала этого аудита, а не повреждённый исходный T-pose skeleton. Таблица: **максимальная длина ребра относительно rest / число рёбер >2×**; исключены rest-рёбра ≤1 мкм. Углы — добавки к асимметричной запечённой позе, не анатомические ROM. Метрика выявляет растяжение, но не оценивает объём, красоту и коллизии.

| Тест | До | После |
|---|---:|---:|
{chr(10).join(rows)}

Отдельно проверены все десять пальцев при +60°, обратные сгибания, кисти ±60°, плечи 60°/100°, таз ±45°, toes ±30°. Всего **{a['summary']['poses']} позы**, NaN/Inf = 0: [after.json](reaudit/after.json). Одинаковые базовые 50 поз: [before.json](reaudit/before.json). В них не выросли числа флагов по каждому mesh; это не гарантия отсутствия всех художественных ухудшений. Сохранены координаты/веса концов худших рёбер. Рендеры просмотрены: [сравнение](reaudit/comparison.png), [кисть](reaudit/after_hand_l.png), [локоть](reaudit/after_elbow_detail.png), [колено](reaudit/after_knee_detail.png), [обувь](reaudit/after_shoe_detail.png), [комбинация](reaudit/after_combined.png).

Малый отдельный sweep 241 костей по +5° local X/Y/Z: **723 позы, NaN/Inf = 0**; >2× в **{sw['summary']['poses_with_edges_stretched_over_2']}** позах против **{oldsw['summary']['poses_with_edges_stretched_over_2']}** в 717 позах прежнего скелета. [joint_sweep.json](joint_sweep.json). У всех восьми продольных local-Y twist-тестов флагов нет. Поперечные X/Z helpers — стресс, а не штатное сгибание второго локтя/колена. Максимум малого sweep **{sw['summary']['max_edge_ratio_requested']:.3f}×**; finite PASS не означает качества всех деформаций.

## 6. Экспорт и совместимость

Прежние семь Actions сохранены. Добавлены `TEST_practical_ranges` (8 секунд, пики −120° локти / 120° колени / curl 30° / комбинация), `TEST_brows_wrist`, `TEST_toes_30` (по 2 секунды). Это диагностические клипы, не idle/walk/run/речь. Все имеют нейтральные концы, 30 fps. Экспорт: meshes + deform Armature; constraints оцениваются/запекаются, управляющих empties в FBX нет.

Повторный импорт: 241/241 костей, 10/10 clips, parent mismatches = 0. Веса соответствующих вершин совпадают **точно**, max delta = {rt['weights_blend_to_fbx']['max_weight_delta']}. Source → prepared neutral: **{rt['summary']['source_neutral_max_delta_m']*1000:.6f} мм**. BLEND → FBX neutral: **{rt['summary']['roundtrip_neutral_max_delta_m']*1000:.6f} мм**. В **{rt['summary']['sample_count']}** образцах времени максимум **{rt['summary']['sampled_animation_max_delta_m']*1000:.6f} мм**. Строгий порог 0.01 мм превышен в {strict} образцах; все <0.02 мм. Топология, UV (delta 0), material slots/indices и параметры материалов сохранены. Corner normals после FBX отличаются до **{rt['blend_to_fbx_neutral']['max_normal_angle_degrees']:.3f}°**; побайтовое равенство shading не заявляется. [roundtrip.json](roundtrip.json).

Assimp source validation и точные postprocess flags JULIA: **28 meshes, 26 материалов, 241 имя aiBone/palette entry, 271 scene node**; положительные weights есть у {len(positive_bones)} костей. Оба режима: максимум 4 influences; нет invalid/nonunit/unweighted vertices, unresolved bones, NaN/Inf в клипах и CPU-деформации. После postprocess {ai['project_import']['vertices']:,} вершин и 62 342 треугольника; до него 187 032 / 62 344. Два degenerate triangles удаляет нынешний pipeline, как у оригинала. Bind residual **{ai['project_import']['max_bind_position_residual_scene_units']:.9g}** imported scene units. По девять CPU samples каждого клипа. [assimp_after.json](reaudit/assimp_after.json), [raw FBX](reaudit/raw_prepared.json).

Реальный OpenGL 3.3 renderer JULIA на RTX 4070 SUPER: **100 изображений, 10 движущихся клипов, 0 GL errors, точный bind reset**. Начало/конец клипов совпадают побайтово на четырёх видах. Проверены composition masks и пять точек local-TRS/quaternion crossfade. Для длинного range-клипа выбраны настоящие пики 12.5/37.5/62.5/87.5%; выборка четвертей попадала на нейтральные кадры и не подтверждала диапазон. Прежний 239-bone FBX также заново отрисован: 7 клипов / 74 снимка.

[GPU-протокол](reaudit/gpu_after/gpu_audit.txt), [front](reaudit/gpu_after/gpu_front_contact.png), [back](reaudit/gpu_after/gpu_back_contact.png), [face](reaudit/gpu_after/gpu_face_contact.png), [mix](reaudit/gpu_after/gpu_mix_contact.png), [endpoints](reaudit/gpu_after/gpu_endpoints.csv). Pixel change сам по себе не подтверждает отсутствие пересечений; использованы также численные данные и просмотр кадров.

## 7. Ограничения и дальнейшая работа

1. На глубоких плечевых позах, кистях +60°, фалангах +60°, коленях 120° и голеностопах ±35° остаются pinching/stretch. Полный кулак, захваты и произвольный mocap не сертифицированы. Нужны ручной cleanup и corrective формы вместо дальнейшего общего размытия.
2. Отдельные переходы spine/cloth/броня при скручивании ещё деформируются. Улучшение шва пояса не заменяет утверждение конструкции одежды и жёстких деталей; смотреть `worst_edge` и локальные рендеры.
3. Большие одновременные повороты всех волос/ткани дают накопленный изгиб и пересечения. Физики, collisions и защиты от проникновения в тело нет. Исправные rest pivots/цепочки сохранены.
4. Качественные blink, верхние/нижние веки, mouth/jaw/visemes и эмоции требуют ручного facial authoring. Подтверждены gaze, уши и брови.
5. Rest pose асимметрична. IK помогает авторингу, но walk/run, foot contact, heel roll, root-motion scale и переходы ещё надо подготовить.
6. Desktop JULIA остаётся статическим: GLModelLoader не сохраняет aiAnimation/morph channels для playback. Следующий этап C++: сохранить clips/TRS, сопоставить все 271 scene nodes, local TRS/slerp → `model.skinningMatrices(localPose)` → renderer. Palette уже поддерживает 241 матрицу. Формула проекта `globalNode × adjustedOffset`; повторное inverse(root) не добавлять. [runtime_audit.md](runtime_audit.md).

## 8. Очистка

Удалены два содержательных файла: `docs/raphtalia/glove_weight_patch.json` — точный дубль сохранённого `glove_transfer_applied.json`; `tools/raphtalia/deformation_diagnostics.py` — устаревший glove-transfer experiment, заменённый reaudit/control checks. Его очистка AnimationData отключала бы новые drivers. Внешних вызовов/импортов не найдено.

{len(clean['moves'])} исторических файлов перемещены с проверкой SHA256 в [history/2026-10-20](history/2026-10-20/README.md): старые prepared/GPU-рендеры, intermediate patches/diagnostics. Измерения оригинала, инструменты подготовки и текстуры сохранены. [cleanup.json](reaudit/cleanup.json) содержит точные пути и причины. Старые отчёты явно исторические; актуальный основной отчёт — этот файл. Дополнительно удалены созданные этим аудитом промежуточные GPU-кандидаты и точные временные копии final outputs. Raw GPU before/final, зависимости, логи и rollback snapshots сохранены; глобальной очистки JULIA не было.

Приоритет художественного прохода: плечи/пояс → кисти и глубокие колени → веки/губы → gait/contact и retarget. Для разработки animation player asset пригоден; для выпуска полноценного анимированного персонажа нужна указанная доработка.
'''
    (OUT/'REPORT_RU.md').write_text(text,encoding='utf-8')
    print('Report/manifest:',a['bones'],'bones;',len(rt['animations_blend']),'clips')

if __name__=='__main__':main()
