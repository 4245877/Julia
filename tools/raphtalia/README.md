# Raphtalia: подготовка, управление и воспроизводимые проверки

Рабочая папка — `C:\Julia`. Актуальный итог: [REPORT_RU.md](../../docs/raphtalia/REPORT_RU.md).
Текущая модель после этапа моргания: **429 deform bones, 15 TEST Actions**. Сохранены исходные 241 кость, 11 прежних клипов, IK/FK, управление глазами/бровями и исправленные веса тела/одежды. Добавлены 188 костей век и локальной поверхности, независимые параметры моргания и четыре диагностических клипа. Исходный `Raphtalia.fbx` и резервные копии сохраняются.

Основной desktop теперь воспроизводит эти 15 клипов. Сборка, управление, архитектура, результаты и ограничения материалов: [runtime_audit.md](../../docs/raphtalia/runtime_audit.md). Быстрая проверка: `julia_desktop/julia_desktop.exe`, клавиши 1–4 — моргание, Space — pause/play, PgUp/PgDn — clip, X — exact bind, F/B — лицо/общий вид. `--verify-animations output_directory` проверяет именно desktop. Исходные карты этой модели пока отсутствуют; импортированные однотонные материалы не являются восстановленной текстурированной внешностью.

## Runtime глаз — 22 октября 2026

Desktop запускается с автономным idle. G включает его повторно, H/J переключают auto blink/gaze, K/U/I вызывают оба/левый/правый глаз, numpad 1..9 и Home удерживают направление, T фиксирует позицию камеры как пространственную цель, [/] меняют средний интервал, F8 задаёт фиксированный seed. X/R отключают слой и восстанавливают точный bind. `--no-eyes` запускает без слоя. Четыре TEST blink clips сохраняют полный приоритет; body/head clips допускают независимое моргание. Это ограниченный eye/lid overlay.

```powershell
.\tools\build.ps1 -UseInstalledDependencies -Configuration Debug
.\tools\build.ps1 -UseInstalledDependencies -Configuration Release
.\julia_desktop\julia_desktop.exe --verify-eyes build/eyes_check
.\julia_desktop\julia_desktop.exe --review-eyes build/eyes_visible
python tools/raphtalia/runtime_eye_audit.py
python tools/raphtalia/eye_evidence.py
```

`runtime_eye_audit.py` только читает FBX/BLEND и сверяет семь сохранённых SHA256; использует установленные Assimp/MSVC и локальный bpy. `eye_evidence.py` собирает PNG/GIF из CTest `build/vs-release/verification/desktop_eyes`. C++ runtime не требует Blender, Pillow или новых зависимостей. API и границы проверки описаны в [runtime_audit.md](../../docs/raphtalia/runtime_audit.md).

## Управление `.blend`

В Object custom properties объекта `Raphtalia_Rig`:

- `IK_arm.L/R`, `IK_leg.L/R`: 0 — существующий FK, 1 — IK; промежуточные значения смешивают решения. По умолчанию 0.
- В коллекции `Animation controls (Blender only)` перемещайте `CTRL_hand.L/R`, `CTRL_foot.L/R` и соответствующие `CTRL_pole_*`. Вращение target управляет ориентацией кисти/стопы. Stretch выключен. Перед переходом из произвольной FK-позы совместите targets с текущими концами цепочек: автоматического matching нет.
- `CTRL_curl_index/middle/ring/pinky/thumb.L/R` → custom property `curl_degrees`: добавочное сгибание трёх фаланг. Начинайте с 0..30°, особенно для уже согнутой левой руки. +60° — стресс, не сертифицированный кулак.
- `auto_forearm_twist` (по умолчанию 1) добавляет половину локального Y-twist кисти на соответствующий forearm helper. Можно отключить для полностью ручного FK. Upper-arm/leg twist остаются ручными.
- `Blink_L`, `Blink_R`: независимое закрытие левого/правого глаза от 0 до 1. Значение 0 открывает глаз, 1 закрывает; одинаковые значения закрывают оба глаза. Стороны соответствуют персонажу. Параметры находятся на `Raphtalia_Rig`, а не на материалах; drivers действительно перемещают деформирующие кости век. Перед ручной настройкой отключите активный TEST Action, иначе его ключи переустановят значения при смене кадра.
- Для положения тела используйте существующий `root.x_01`. Коллекция Face содержит eye/ear/brow; `brow.L/R` перемещаются/поворачиваются в Pose Mode. Новые кости находятся в коллекции `Eyelids`. Риг рта/челюсти и речевая мимика в этот этап не входят.

`Constraint helpers (no export)` скрыта в viewport; helpers обеспечивают реальные ограничения. Коллекция исходных transform helpers остаётся архивной. Drivers используют стандартные выражения Blender и не требуют регистрации внешних Python-функций. [Документация swing/twist](https://docs.blender.org/api/4.2/bpy_types_enum_items/driver_target_rotation_mode_items.html).

Прежние 11 Actions сохранены без изменения ключей, включая `TEST_deep_body_cloth` с диапазоном 1..361. Новые клипы при 30 fps:

| Action | Кадры | Время между первым и последним кадром | Проверка |
| --- | --- | --- | --- |
| `TEST_blink_natural` | 1..31 | 1 с | Оба глаза; закрывание быстрее открывания |
| `TEST_blink_independent` | 1..61 | 2 с | Левый, затем правый глаз |
| `TEST_blink_gaze` | 1..121 | 4 с | Моргание при разных направлениях взгляда |
| `TEST_blink_head` | 1..121 | 4 с | Голова, взгляд, существующие брови и моргание |

Выберите нужный Action и его диапазон кадров: сохранённый timeline нового теста не ограничивает длину старых клипов. Для деформации используется linear blend skinning без Preserve Volume, максимум четыре влияния на вершину. Shape Keys не требуются.

Исходные глазницы имеют по 16 узлов, а ресницы включают отдельные ribbon, нижние акценты, складки и брови. `blink_local_surface.py` дважды делит только окологлазные треугольники и ресничные полосы; новые точки лежат на исходной поверхности. Исходные индексы/координаты вершин сохранены, UV интерполируются по исходным треугольникам. Provenance каждой вершины/грани записан в `docs/raphtalia/blink/patch.json`. Локальные custom normals и sharp edges скорректированы для век; остальная модель не сглаживается глобально. Дополнительные 188 костей состоят из 124 контурных и 64 костей двух внутренних рядов поверхности.

Небольшой pinch в уголках остаётся ограничением стилизованной исходной топологии. Численные смены знака проекции почти edge-on треугольников при полном закрытии следует отличать от видимых складок; учитывайте реальные площади, 3D dihedral, закрытие ресницами и контрольные изображения.

## Зависимости

Python 3.11, NumPy, SciPy, Pillow; официальный bpy 4.2.0 уже находится в `build/raphtalia_tools/bpy`. SciPy нужен инструментам генерации/QA, а не runtime модели. Системный Blender не устанавливался. Если локального bpy нет:

```powershell
python -m pip install --target build/raphtalia_tools/bpy --no-deps bpy==4.2.0
$env:PYTHONPATH = "$PWD/build/raphtalia_tools/bpy"
```

MSVC, Assimp и GL-зависимости используются из существующего `build/vs-release/vcpkg_installed/x64-windows`. GPU-аудит линкуется с готовыми Release-библиотеками JULIA; после изменения C++ сначала пересоберите их. Основное приложение не получает проигрыватель от запуска аудита.

## Проверить текущий этап моргания без изменения модели

Для сравнения сохранности один раз извлеките точный снимок 241/11 из `assets/models/backups/Raphtalia_prepared.pre_blink_2026-10-21.zip` в `build/raphtalia_tools/blink/base`. Архив содержит BLEND и FBX; не извлекайте его поверх актуальных assets.

```powershell
$env:PYTHONPATH = "$PWD/build/raphtalia_tools/bpy"
Expand-Archive -LiteralPath assets/models/backups/Raphtalia_prepared.pre_blink_2026-10-21.zip -DestinationPath build/raphtalia_tools/blink/base
python tools/raphtalia/blink_integrity.py --base build/raphtalia_tools/blink/base/Raphtalia_prepared.blend --candidate assets/models/Raphtalia_prepared.blend --patch docs/raphtalia/blink/patch.json --output docs/raphtalia/blink/integrity.json
python tools/raphtalia/blink_visibility.py --blend assets/models/Raphtalia_prepared.blend --output docs/raphtalia/blink/visibility.json
python tools/raphtalia/blink_render_suite.py --blend assets/models/Raphtalia_prepared.blend --output docs/raphtalia/blink
python tools/raphtalia/blink_corner_audit.py --blend assets/models/Raphtalia_prepared.blend --dense --winks --output docs/raphtalia/blink/corner_audit.json
python tools/raphtalia/roundtrip_check.py --blend assets/models/Raphtalia_prepared.blend --fbx assets/models/Raphtalia_prepared.fbx --output docs/raphtalia/blink/roundtrip.json
./tools/raphtalia/build_assimp_audit.ps1 -InputModel assets/models/Raphtalia_prepared.fbx -OutputJson docs/raphtalia/blink/assimp.json
python tools/raphtalia/blink_gpu_samples.py --blend assets/models/Raphtalia_prepared.blend --output build/raphtalia_tools/blink/gpu_samples.csv
./tools/raphtalia/build_gpu_audit.ps1 -InputModel assets/models/Raphtalia_prepared.fbx -OutputDirectory build/raphtalia_tools/blink/gpu -BlinkSamplesCsv build/raphtalia_tools/blink/gpu_samples.csv
```

Эти проверки не сохраняют открытый BLEND и не перезаписывают FBX; они записывают evidence и изображения. Повторную распаковку уже подготовленного baseline пропустите. `blink_integrity.py` проверяет исходные кости/клипы/контроллеры, provenance subdivision, UV, материалы и веса; отдельно сообщает bit-exact и сравнение с допуском float. Небольшая погрешность хранения новых midpoint и LBS не является изменением координат исходных вершин.

Для GPU before/after и компактных изображений нужен также аудит исходного FBX 241/11:

```powershell
./tools/raphtalia/build_gpu_audit.ps1 -InputModel build/raphtalia_tools/blink/base/Raphtalia_prepared.fbx -OutputDirectory build/raphtalia_tools/blink/gpu_before
python tools/raphtalia/convert_blink_gpu.py --source build/raphtalia_tools/blink/gpu --before build/raphtalia_tools/blink/gpu_before --samples build/raphtalia_tools/blink/gpu_samples.csv --output docs/raphtalia/blink/gpu
```

CSV выбирает реальные моменты 0/25/50/75/100% управляющих кривых, в том числе между целыми кадрами. Успешный импорт Assimp не заменяет воспроизведение этих состояний в GPU. Различайте Blender-деформацию, сохранность FBX/Assimp, независимый GPU-аудитор и новую проверку основного desktop. `gpu_pose_audit.cpp` дополнительно сравнивает свой независимый sampler с production `sampleAnimation`; отдельный запуск аудитора не заменяет desktop-проверку. Прямой runtime API `Blink_L/R` не добавлен.

GPU clay-материалы обусловлены прежним prepared FBX с нулём Assimp texture slots. Этап моргания не добавляет и не удаляет исходные текстуры/материалы. Workbench/GPU кадры служат диагностикой деформаций, а не подтверждением текстурированного beauty render.

`material_audit.py` повторно читает FBX/BLEND и доступные PNG без изменения модели. `desktop_evidence.py` конвертирует framebuffer PPM основного desktop в компактные листы и сравнивает их с независимым аудитором при одинаковых camera/MSAA. Их стандартные outputs — `docs/raphtalia/desktop/`; raw frames остаются в `build/`. Команды приведены в runtime audit, источники и backups сохраняются.

`write_report.py` — исторический генератор отчёта тела/одежды, рассчитанный на этап 241/11. **Не запускайте его для публикации этапа моргания: он перезаписывает новый `REPORT_RU.md` старым body-only содержимым.** `deep_integrity.py` и `joint_sweep.py` также рассчитаны на прежние снимки/число костей и не являются проверками актуального 429-костного файла. Их прежние метрики и assertions используйте только с соответствующими архивами. `face_blink_probe.py` сохраняет отклонённый эксперимент; это не реализация текущего моргания.

## Воспроизвести этап моргания отдельным кандидатом

Вход — только извлечённый pre-blink BLEND 241/11, выход — отдельный путь в `build`. Не применяйте генератор повторно к 429/15.

```powershell
$env:PYTHONPATH = "$PWD/build/raphtalia_tools/bpy"
python tools/raphtalia/build_blink.py --input build/raphtalia_tools/blink/base/Raphtalia_prepared.blend --output build/raphtalia_tools/blink/rebuilt.blend --export
python tools/raphtalia/blink_integrity.py --base build/raphtalia_tools/blink/base/Raphtalia_prepared.blend --candidate build/raphtalia_tools/blink/rebuilt.blend --patch build/raphtalia_tools/blink/rebuilt.patch.json --output build/raphtalia_tools/blink/rebuilt_integrity.json
python tools/raphtalia/roundtrip_check.py --blend build/raphtalia_tools/blink/rebuilt.blend --fbx build/raphtalia_tools/blink/rebuilt.fbx --output build/raphtalia_tools/blink/rebuilt_roundtrip.json
```

`build_blink.py` не меняет вход и пишет `*.blend`, `*.patch.json`, а с `--export` также `*.fbx`. Перед заменой prepared assets сравнивайте одинаковые позы, сохранность, roundtrip, Assimp и GPU. `blink_topology_audit.py` читает исходный снимок, `blink_lash_mapping.py` разделяет ресничные части, `blink_local_surface.py` передаёт поверхность с provenance. Ни один из этих помощников не заменяет визуальную оценку.

Итоговый blink-экспорт использует `bake_step=.25` (120 samples/sec при 30 fps), simplify 0. В 216 roundtrip-позах максимальная ошибка составила 0,03192 мм, против 1,006 мм при прежнем шаге 1 кадр. Это увеличивает размер FBX примерно до 170 МБ. Сами авторские Actions остаются при 30 fps. В `patch.json` также сохранены стандартные выражения всех drivers век и единицы движения для будущего переноса независимых параметров в JULIA.

## История: локальные изменения тела/одежды, этап 241/10 → 241/11

Распакуйте `assets/models/backups/Raphtalia_prepared.pre_deep_deformation_2026-10-21.zip` в отдельную папку `build/raphtalia_tools/deep/base`. Это точная исходная модель 241/10; не распаковывайте её поверх текущих assets.

```powershell
python tools/raphtalia/refine_deep_deformation.py --input build/raphtalia_tools/deep/base/Raphtalia_prepared.blend --output build/raphtalia_tools/deep/rebuilt.blend --export
python tools/raphtalia/deep_integrity.py --candidate build/raphtalia_tools/deep/rebuilt.blend --output build/raphtalia_tools/deep/rebuilt_integrity.json
python tools/raphtalia/deep_pose_audit.py --blend build/raphtalia_tools/deep/rebuilt.blend --output build/raphtalia_tools/deep/rebuilt.json --render
python tools/raphtalia/roundtrip_check.py --blend build/raphtalia_tools/deep/rebuilt.blend --fbx build/raphtalia_tools/deep/rebuilt.fbx --output build/raphtalia_tools/deep/rebuilt_roundtrip.json
python tools/raphtalia/limb_validation.py --output build/raphtalia_tools/deep/rebuilt_limb_validation.json
```

`refine_deep_deformation.py` принимает только исходную модель 241/10, запрещает совпадение входного/выходного пути и сохраняет patch рядом с кандидатом. `--baseline` создаёт контрольную копию с тем же новым клипом, но без исправления весов: она нужна для сопоставимого GPU before/after. Статические веса формируют `deep_body.py` и `deep_cloth.py`; runtime-решатель не требуется. Повторно применять скрипт к результатам 241/11 или 429/15 нельзя.

Числа 50 body/clothing поз, 723 отдельных вращения и 166 GPU-изображений относятся к этому историческому этапу. Evidence находится в `docs/raphtalia/deep_deformation/`; оно подтверждает прежние исправления тела и одежды и не заменяет текущие проверки моргания.

## История: уточнение рига, этап 239/7 → 241/10

В `assets/models/backups/Raphtalia_prepared.pre_reaudit_2026-10-21.zip` сохранены прежние FBX/BLEND с SHA256 и manifest. Извлеките архив в **отдельную** папку `build/raphtalia_tools/reaudit/base`; не поверх final assets. Затем:

```powershell
python tools/raphtalia/refine_rig.py --input build/raphtalia_tools/reaudit/base/Raphtalia_prepared.blend --output build/raphtalia_tools/reaudit/rebuilt.blend --controls --export
python tools/raphtalia/reaudit.py --blend build/raphtalia_tools/reaudit/rebuilt.blend --output build/raphtalia_tools/reaudit/rebuilt.json --render
python tools/raphtalia/control_check.py --blend build/raphtalia_tools/reaudit/rebuilt.blend --output build/raphtalia_tools/reaudit/rebuilt_controls.json
python tools/raphtalia/roundtrip_check.py --blend build/raphtalia_tools/reaudit/rebuilt.blend --fbx build/raphtalia_tools/reaudit/rebuilt.fbx --output build/raphtalia_tools/reaudit/rebuilt_roundtrip.json
```

`refine_rig.py` принимает только 239-костный результат первой стадии: повторное применение к 241-костной модели запрещено. Он не меняет входную модель. Точные локальные патчи и конфигурация контроллеров сохраняются рядом с кандидатом в `*.weights.json` / `*.controls.json`. Перед заменой prepared надо сравнить кандидата и повторить Assimp/GPU; оригинал и backup сохраняются всегда.

## История: первая стадия из оригинала

Инструменты сохранены для полного воспроизведения: `fbx_audit.py` → `blender_inspect.py` → `joint_geometry.py` → `face_geometry.py` → `prepare_rig.py`. Их исходные измерения — `raw_original.json`, `joint_geometry.json`, `face_geometry.json`. `glove_skinning.py` используется подготовкой, не является мусором.

**`prepare_rig.py` создаёт только старую первую стадию (239 костей / 7 Actions) и перезаписывает prepared assets.** Она не воспроизводит текущий результат. Полная цепочка: первая стадия → `refine_rig.py --controls --export` (241/10) → `refine_deep_deformation.py --export` (241/11) → `build_blink.py --export` (429/15), затем проверки отдельного кандидата. Существующий финальный rig нельзя заменять промежуточной стадией. Для воспроизведения моргания используйте точный архив pre-blink 241/11.

## Экспорт собственных анимаций

Выбирайте только Armature и meshes персонажа: типы ARMATURE/MESH, `use_armature_deform_only=True`, `add_leaf_bones=False`, `use_mesh_modifiers=False`, simplify 0. Для моргания `build_blink.py --export` вызывает `refine_rig.export_fbx(..., bake_step=.25)`: четверть кадра при 30 fps соответствует 120 Гц. Старый шаг 1 давал на быстрых переходах отклонения порядка 0.001 м между отсчётами. Итоговую точность экспорта с шагом .25 определяет свежий `roundtrip.json`; не переносите в него метрики старого bake.

Ключи `Blink_L/R` находятся на Armature; FBX получает вычисленные TRS костей, а не Blender drivers и интерфейс custom properties. Для нового клипа с такими параметрами проверяйте результат bake, включая промежуточные значения. Если ключи находятся на внешних управляющих объектах, экспортируйте активный клип со всеми зависимостями, `bake_anim_use_all_actions=False`, либо сделайте visual bake в копии сцены. После bake отключите уже внесённые аддитивные controls и сравните результат с исходным движением. Нынешние 15 TEST Actions сохраняются отдельными клипами.

Будущие morphs в JULIA требуют отдельной поддержки; их наличие в FBX не создаёт playback. Heel roll, авто IK/FK matching, collisions, gait/contact и полноценная мимика остаются следующими задачами.

## Структура evidence

- `docs/raphtalia/REPORT_RU.md`, `runtime_audit.md` — актуальное описание модели и границ runtime-поддержки.
- `docs/raphtalia/blink/` — текущие patch/provenance, integrity, visibility, roundtrip, Assimp/GPU и сравнительные изображения глаз. Сверяйте SHA256 evidence с проверяемыми файлами.
- `docs/raphtalia/deep_deformation/` — сохранённые before/after тела/одежды, weight patch, integrity и GPU предыдущего этапа 241/11. Старые `joint_sweep.json` и подобные корневые evidence также относятся к своим исходным снимкам.
- `docs/raphtalia/reaudit/` — сохранённые данные предыдущего этапа уточнения рига; количества и метрики оттуда не являются текущим QA.
- `docs/raphtalia/history/2026-10-20/` — явно исторические отчёты/кадры и промежуточные эксперименты. Они не являются текущим QA.
- `build/raphtalia_tools/` — зависимости, executables, PPM, логи и кандидаты; в доставку модели не входят. Они не удалялись вслепую.

Промежуточные пробы третьего этапа удалялись только по явному списку в `docs/raphtalia/deep_deformation/cleanup.json`. Сохранены исходные резервные копии, необходимые зависимости, raw GPU before/final, инструменты и итоговое evidence.

Удалён только устаревший `deformation_diagnostics.py`: он повторно переносил glove weights в памяти и очищал AnimationData, что отключило бы drivers. Его заменяют read-only `reaudit.py` и `control_check.py`. Полный протокол очистки находится в `docs/raphtalia/reaudit/cleanup.json`.
