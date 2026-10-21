# Raphtalia: текущий C++/OpenGL contract

Проверка: **21 октября 2026**. Текущий asset — `Raphtalia_prepared.fbx`, **241 deform bone / 11 TEST-клипов**. Размеры и SHA256: [deliverable_manifest.json](deliverable_manifest.json). Изменения тела/одежды и результаты визуального сравнения: [REPORT_RU.md](REPORT_RU.md). Данные текущего этапа находятся в `deep_deformation/`; `reaudit/` и [history/2026-10-20/](history/2026-10-20/runtime_audit.md) относятся к предыдущим этапам.

## Что поддерживает JULIA

`plugins/render_opengl/src/GLModelLoader.cpp` импортирует meshes, материалы, vertex weights, nodes, global bind и inverse bind. Вершины приводятся к imported scene space; offsets учитывают inverse(meshGlobal). `GLModel::skinningMatrices(localPose)` ожидает локальные трансформации **всех nodes** в порядке модели, вычисляет глобальные parent → child и выдаёт `globalNode × adjustedOffset`. Второй inverse(root) добавлять нельзя. Пустой массив позы использует imported bind.

`OpenGLRenderer.cpp` принимает palette через `renderModel(model, modelMatrix, palette)`. `samplerBuffer`/`GL_RGBA32F` хранит четыре texel на mat4; лимит проверяется через `GL_MAX_TEXTURE_BUFFER_SIZE`. Текущий asset требует 241 матрицу, 15424 байта. NVIDIA RTX 4070 SUPER сообщила 268435456 texels; это характеристика одного устройства, не результат performance benchmark.

Vertex contract: четыре bone indices / weights, **LBS без Preserve Volume**. Исправления этого этапа — статические веса, поэтому специальный runtime solver не нужен. Fitting/камера — Y up; `fitToHeightTransform(2.2f)` отдельно центрирует/масштабирует imported bounds. Physical scale и root motion требуют согласованных единиц.

**Основное приложение остаётся статическим.** Loader не сохраняет `aiAnimation`/morph channels после уничтожения Assimp scene. Animation player, state machine, masks и morph shader не добавлены. Standalone auditor проверяет существующий путь skinning и не внедряет playback в desktop. C++-рендерер на этом этапе не менялся.

## Проверка текущего файла

Assimp **6.0.4** проверяет два режима: source validation без сокращения весов и точные postprocess flags загрузчика. [assimp_after.json](deep_deformation/assimp_after.json) содержит оба результата и SHA256 входного FBX.

| Проверка | Результат |
|---|---|
| Узлы / meshes / материалы | 271 / 28 / 26 в обоих режимах |
| Palette entries / skeletal clips | 241 / 11 в обоих режимах |
| Максимум influences | 4 |
| Invalid / nonunit / unweighted vertices | 0 / 0 / 0 |
| Unresolved bones/channels и NaN/Inf | 0 |
| CPU-деформации | По девять samples каждого клипа в каждом режиме |
| Blender → FBX → Blender | 87 samples, 241/241 костей, 11/11 Actions; родители и веса сохранены |

Palette entries включают пустые FBX clusters; 241 entry не означает 241 непосредственно взвешенную кость. Полные количества vertices/triangles и положительных назначений приведены в отчёте и JSON. Существующий project postprocess удаляет два вырожденных треугольника. Численные проверки не доказывают художественное качество или отсутствие пересечений.

[roundtrip.json](roundtrip.json) привязан к SHA256 текущих BLEND/FBX. Новый `TEST_deep_body_cloth` проверяется во всех ключевых положениях и серединах переходов. Максимальная ошибка анимационной поверхности <0.02 мм; строгий порог 0.01 мм проходит не везде. UV, топология и материалы сохранены; небольшая разница corner normals после FBX измерена, shading не объявлен побайтово одинаковым. [integrity.json](deep_deformation/integrity.json) отдельно подтверждает точную сохранность исходной геометрии, скелета, controls, прежних Actions и лица до экспорта.

## Реальный GPU

[gpu_pose_audit.cpp](../../tools/raphtalia/gpu_pose_audit.cpp) создаёт скрытый OpenGL 3.3 context и использует **настоящие GLModelLoader/OpenGLRenderer**. Независимый Assimp sampler интерполирует local position/scale линейно, rotation — quaternion slerp, затем передаёт palette через существующий API. Проверяются channels, конечность матриц и GL errors; изображения читаются с GPU.

Итог: **166 изображений, все 11 клипов меняют rendering, GL errors = 0, bind reset = EXACT**. Все шесть пиков нового body/clothing-клипа показаны с близкими видами плеча, талии, колена и обуви. Для `TEST_practical_ranges` выбраны фактические пики 12.5/37.5/62.5/87.5%; четверти этого клипа нейтральны. `TEST_toes_30` проверяет оба направления на 25/75%. Сохранены 38-node masked composition и пять TRS/quaternion crossfade points.

Начало и конец каждого клипа совпадают побайтово на всех десяти видах. Animation-neutral немного отличается от imported bind вследствие экспорта; точные RGB/palette дельты находятся в endpoints. Отдельная повторная отрисовка bind точна. Существующие лицевые клипы участвуют только в проверке сохранности: новые лицевые движения не создавались.

Доказательства: [протокол и SHA256](deep_deformation/gpu_after/gpu_audit.txt), [samples](deep_deformation/gpu_after/gpu_samples.csv), [endpoints](deep_deformation/gpu_after/gpu_endpoints.csv), [mix](deep_deformation/gpu_after/gpu_mix.csv), [body/clothing closeups](deep_deformation/gpu_after/gpu_body_cloth_contact.png). Для сопоставимого [GPU baseline](deep_deformation/gpu_before/gpu_body_cloth_contact.png) исходная модель 241/10 получила только тот же новый диагностический клип, без исправления весов.

Эти проверки подтверждают выполнение исправленных экспортированных деформаций. Сохраняются LBS-уплощение глубоких сгибов и пересечения юбки с ногами в некоторых глубоких позах; автоматических коллизий нет. Полный OpenGL unit suite и gameplay/performance benchmark не запускались.

## Следующий этап runtime

1. Сохранить clips, duration, ticks-per-second и TRS channels до уничтожения importer; явно обработать неизвестный ticks-per-second.
2. Привязать channels ко всем node indices, включая helpers без weights. Для незаданных компонентов использовать bind TRS.
3. Смешивать локальные TRS, rotation — slerp; затем вычислять globals. Matrix lerp для переходов не использовать.
4. Передавать palette из `model.skinningMatrices(localPose)` в `renderer.renderModel`.
5. После базового playback отдельно реализовать masks/layers, transitions, root motion и foot contacts; согласовать рост/scale. Для юбки авторить контакты ключами существующих цепочек.
6. Если понадобятся Shape Keys, добавить importer/buffers/shader и повторить визуальный roundtrip/GPU. Текущие исправления morph-поддержки не требуют. Лицевая анимация остаётся отдельной задачей.

Команды: [tools/raphtalia/README.md](../../tools/raphtalia/README.md). GPU-инструмент линкуется с готовыми Release-библиотеками JULIA; после будущих изменений C++ их нужно пересобрать. Текущий генератор отчёта отклоняет evidence, SHA256 которого не соответствует опубликованным assets.
