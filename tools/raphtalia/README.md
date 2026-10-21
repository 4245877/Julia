# Raphtalia: подготовка, управление и воспроизводимые проверки

Рабочая папка — `C:\Julia`. Актуальный итог: [REPORT_RU.md](../../docs/raphtalia/REPORT_RU.md).
Текущая модель: **241 deform bone, 10 TEST Actions**. Исходный `Raphtalia.fbx` и его backup никогда не перезаписываются.

## Управление `.blend`

В Object custom properties объекта `Raphtalia_Rig`:

- `IK_arm.L/R`, `IK_leg.L/R`: 0 — существующий FK, 1 — IK; промежуточные значения смешивают решения. По умолчанию 0.
- В коллекции `Animation controls (Blender only)` перемещайте `CTRL_hand.L/R`, `CTRL_foot.L/R` и соответствующие `CTRL_pole_*`. Вращение target управляет ориентацией кисти/стопы. Stretch выключен. Перед переходом из произвольной FK-позы совместите targets с текущими концами цепочек: автоматического matching нет.
- `CTRL_curl_index/middle/ring/pinky/thumb.L/R` → custom property `curl_degrees`: добавочное сгибание трёх фаланг. Начинайте с 0..30°, особенно для уже согнутой левой руки. +60° — стресс, не сертифицированный кулак.
- `auto_forearm_twist` (по умолчанию 1) добавляет половину локального Y-twist кисти на соответствующий forearm helper. Можно отключить для полностью ручного FK. Upper-arm/leg twist остаются ручными.
- Для положения тела используйте существующий `root.x_01`. Коллекция костей Face содержит eye/ear/brow. `brow.L/R` можно перемещать/поворачивать в Pose Mode; нет фиктивных blink/jaw controllers.

`Constraint helpers (no export)` скрыта в viewport; helpers обеспечивают реальные ограничения. Коллекция исходных transform helpers остаётся архивной. Drivers используют стандартные выражения Blender и не требуют регистрации внешних Python-функций. [Документация swing/twist](https://docs.blender.org/api/4.2/bpy_types_enum_items/driver_target_rotation_mode_items.html).

Семь прежних Actions сохранены. Дополнительно `TEST_practical_ranges` имеет 241 кадр (8 секунд), `TEST_brows_wrist` и `TEST_toes_30` — 61 кадр. Timeline расширен до 241. Нейтральные начала/концы у всех клипов. Для тела используется linear blend skinning, Preserve Volume отключён, как в JULIA.

## Зависимости

Python 3.11, NumPy, SciPy, Pillow; официальный bpy 4.2.0 уже находится в `build/raphtalia_tools/bpy`. SciPy нужен инструментам генерации/QA, а не runtime модели. Системный Blender не устанавливался. Если локального bpy нет:

```powershell
python -m pip install --target build/raphtalia_tools/bpy --no-deps bpy==4.2.0
$env:PYTHONPATH = "$PWD/build/raphtalia_tools/bpy"
```

MSVC, Assimp и GL-зависимости используются из существующего `build/vs-release/vcpkg_installed/x64-windows`. GPU-аудит линкуется с готовыми Release-библиотеками JULIA; после изменения C++ сначала пересоберите их. Основное приложение не получает проигрыватель от запуска аудита.

## Проверить текущие файлы без изменения модели

```powershell
$env:PYTHONPATH = "$PWD/build/raphtalia_tools/bpy"
python tools/raphtalia/verify_prepared.py --render
python tools/raphtalia/reaudit.py --render
python tools/raphtalia/control_check.py
python tools/raphtalia/joint_sweep.py
python tools/raphtalia/roundtrip_check.py
./tools/raphtalia/build_assimp_audit.ps1 -InputModel assets/models/Raphtalia_prepared.fbx -OutputJson docs/raphtalia/reaudit/assimp_after.json
./tools/raphtalia/build_gpu_audit.ps1 -OutputDirectory build/raphtalia_tools/reaudit/gpu_final
python tools/raphtalia/convert_gpu_evidence.py build/raphtalia_tools/reaudit/gpu_final docs/raphtalia/reaudit/gpu_after
python tools/raphtalia/fbx_audit.py assets/models/Raphtalia_prepared.fbx --output docs/raphtalia/reaudit/raw_prepared.json
python tools/raphtalia/create_blender_comparison.py
python tools/raphtalia/write_report.py
```

Проверки читают модель и пишут только evidence/CSV/отчёт. `write_report.py` проверяет хэши моделей против результатов roundtrip/sweep, чтобы не публиковать старые количества костей. Результат finite/импорта не означает отсутствие коллизий; смотрите локальные изображения и `worst`/`worst_edge` в JSON.

`face_blink_probe.py` отдельно воспроизводит **отклонённый** procedural blink в памяти и пишет только `build/raphtalia_tools/reaudit/rejected_blink.png`. Известные складки века/щеки и смещение ресниц — причина не включать его в rig. Скрипт сохранён для воспроизводимости лицевого аудита и не сохраняет BLEND/FBX.

`reaudit.py --blend ... --output ... [--render]`, `control_check.py --blend ... --output ...`, `verify_prepared.py --blend ... --output ...`, `joint_sweep.py --blend ... --output ...` позволяют проверять кандидата. `roundtrip_check.py` принимает `--blend`, `--fbx`, `--output`. Длинный клип сравнивается в 17 точках; остальные в пяти, включая endpoints. `verify_prepared.py` также обновляет текущий skeleton CSV.

## Воспроизвести уточнение из точной модели до аудита

В `assets/models/backups/Raphtalia_prepared.pre_reaudit_2026-10-21.zip` сохранены прежние FBX/BLEND с SHA256 и manifest. Извлеките архив в **отдельную** папку `build/raphtalia_tools/reaudit/base`; не поверх final assets. Затем:

```powershell
python tools/raphtalia/refine_rig.py --input build/raphtalia_tools/reaudit/base/Raphtalia_prepared.blend --output build/raphtalia_tools/reaudit/rebuilt.blend --controls --export
python tools/raphtalia/reaudit.py --blend build/raphtalia_tools/reaudit/rebuilt.blend --output build/raphtalia_tools/reaudit/rebuilt.json --render
python tools/raphtalia/control_check.py --blend build/raphtalia_tools/reaudit/rebuilt.blend --output build/raphtalia_tools/reaudit/rebuilt_controls.json
python tools/raphtalia/roundtrip_check.py --blend build/raphtalia_tools/reaudit/rebuilt.blend --fbx build/raphtalia_tools/reaudit/rebuilt.fbx --output build/raphtalia_tools/reaudit/rebuilt_roundtrip.json
```

`refine_rig.py` принимает только 239-костный результат первой стадии: повторное применение к 241-костной модели запрещено. Он не меняет входную модель. Точные локальные патчи и конфигурация контроллеров сохраняются рядом с кандидатом в `*.weights.json` / `*.controls.json`. Перед заменой prepared надо сравнить кандидата и повторить Assimp/GPU; оригинал и backup сохраняются всегда.

## Первая стадия из оригинала

Инструменты сохранены для полного воспроизведения: `fbx_audit.py` → `blender_inspect.py` → `joint_geometry.py` → `face_geometry.py` → `prepare_rig.py`. Их исходные измерения — `raw_original.json`, `joint_geometry.json`, `face_geometry.json`. `glove_skinning.py` используется подготовкой, не является мусором.

**`prepare_rig.py` создаёт только старую первую стадию (239 костей / 7 Actions) и перезаписывает prepared assets.** Для текущего результата после него обязательно запустить `refine_rig.py --controls --export` на этом результате и проверить отдельного кандидата. Существующий финальный rig нельзя заменять одной первой стадией. Удобнее использовать точный архив выше.

## Экспорт собственных анимаций

Выбирайте только Armature и meshes персонажа: типы ARMATURE/MESH, `use_armature_deform_only=True`, `add_leaf_bones=False`, `use_mesh_modifiers=False`, bake animations с шагом 1 и simplify 0. Общие параметры — в `refine_rig.export_fbx`. Функция для текущих TEST Actions проверена roundtrip.

Если ключи находятся на управляющих объектах, а не только на Armature, экспортируйте **активный** клип со всеми его зависимостями, `bake_anim_use_all_actions=False`, либо сначала сделайте visual bake в копии сцены. Экспорт всех несовместимых object Actions не заменяет сборку клипа. После visual bake отключите внесённые в него аддитивные controls, чтобы не применить их второй раз, и сравните экспорт с исходным control-движением. Нынешние 10 TEST Actions хранят ключи на Armature и экспортированы отдельно.

Будущие morphs в JULIA требуют отдельной поддержки; их наличие в FBX не создаёт playback. Heel roll, авто IK/FK matching, collisions, gait/contact и полноценная мимика остаются следующими задачами.

## Структура evidence

- `docs/raphtalia/REPORT_RU.md`, `runtime_audit.md`, `skeleton_prepared.csv`, `roundtrip.json`, `joint_sweep.json`, manifest — текущие данные.
- `docs/raphtalia/reaudit/` — новые before/after, локальные кадры, контроллеры, weight patch, cleanup и отдельные GPU before/after.
- `docs/raphtalia/history/2026-10-20/` — явно исторические отчёты/кадры и промежуточные эксперименты. Они не являются текущим QA.
- `build/raphtalia_tools/` — зависимости, executables, PPM, логи и кандидаты; в доставку модели не входят. Они не удалялись вслепую.

Промежуточные GPU-кандидаты этого аудита и точные копии финальных outputs удалены по явному списку. Сохранены raw GPU before/final, необходимые зависимости, логи и оба before snapshot; исходные резервные копии не удалялись.

Удалён только устаревший `deformation_diagnostics.py`: он повторно переносил glove weights в памяти и очищал AnimationData, что отключило бы drivers. Его заменяют read-only `reaudit.py` и `control_check.py`. Полный протокол очистки находится в `docs/raphtalia/reaudit/cleanup.json`.
