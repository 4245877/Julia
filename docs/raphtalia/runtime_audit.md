# Raphtalia: текущий C++/OpenGL contract

Проверка: **21 октября 2026**. Актуальный asset — `Raphtalia_prepared.fbx`, **241 deform bone / 10 TEST-клипов**. SHA256 и размер: [deliverable_manifest.json](deliverable_manifest.json). Подробные изменения и ограничения: [REPORT_RU.md](REPORT_RU.md). Исторический аудит оригинала и 239-костной версии: [history/2026-10-20/runtime_audit.md](history/2026-10-20/runtime_audit.md).

## Что поддерживает JULIA

`plugins/render_opengl/src/GLModelLoader.cpp` импортирует meshes, материалы, vertex weights, nodes, global bind и inverse bind. Вершины приводятся к imported scene space; offsets учитывают inverse(meshGlobal). `GLModel::skinningMatrices(localPose)` из `GLModel.cpp` ожидает полный массив локальных трансформаций **всех nodes**, в порядке модели, вычисляет глобальные parent → child и выдаёт `globalNode × adjustedOffset`. Второй inverse(root) добавлять нельзя.

`OpenGLRenderer.cpp` принимает palette через `renderModel(model, modelMatrix, palette)`. `samplerBuffer`/`GL_RGBA32F` хранит четыре texel на mat4; лимит проверяется через `GL_MAX_TEXTURE_BUFFER_SIZE`. Текущий asset требует 241 матрицу, около 15.4 KB. NVIDIA RTX 4070 SUPER сообщила 268435456 texels. Это результат одного устройства, не гарантия выделения всей теоретической памяти или производительности на других GPU.

Vertex contract: четыре bone indices / weights, LBS, без Preserve Volume. Fitting/камера — Y up; `fitToHeightTransform(2.2f)` отдельно центрирует/масштабирует imported bounds. Physical scale и root motion должны использовать согласованные единицы.

**Основное приложение остаётся статическим.** Loader не сохраняет `aiAnimation`/morph channels после уничтожения Assimp scene. Desktop не получил animation player, state machine, masks или morph shader. Дальнейшее воспроизведение — отдельный этап; standalone auditor его не внедряет.

## Фактическая проверка финального файла

Assimp **6.0.4**, два независимых режима — source validation без сокращения весов и точные postprocess flags loader. Полные данные: [assimp_after.json](reaudit/assimp_after.json), [raw_prepared.json](reaudit/raw_prepared.json).

| Показатель | Source validation | Project postprocess |
|---|---:|---:|
| Scene nodes / meshes | 271 / 28 | 271 / 28 |
| Имена aiBone / palette entries | 241 | 241 |
| Материалы / texture references / morphs | 26 / 0 / 0 | 26 / 0 / 0 |
| Вершины | 187032 | 167327 |
| Треугольники | 62344 | 62342 |
| Максимум influences | 4 | 4 |
| Invalid / nonunit / unweighted vertices | 0 / 0 / 0 | 0 / 0 / 0 |
| Unresolved bone entries | 0 | 0 |
| Skeletal clips | 10 | 10 |

Положительные vertex weights имеют **228 костей**. Остальные 13 — необходимые корневые/родительские узлы цепочек; `aiBone` включает и пустые mesh/bone clusters. FBX содержит 369 непустых и 6379 пустых clusters после стандартного экспорта. Поэтому 241 palette entry нельзя трактовать как 241 непосредственно взвешенную кость или как доказательство качества рига.

Два вырожденных треугольника удаляет существующий pipeline, как у оригинала. Bind residual около 3.7e-6 imported scene units. По девять CPU samples каждого клипа: NaN/Inf в ключах, globals и деформированных вершинах нет. Original FBX дополнительно перепроверен собственным binary parser, исходный SHA256 сохранён.

Blender → FBX: 241/241 костей, совпадающие родители, 10/10 Actions, веса соответствующих вершин совпадают точно. В 62 sampled poses максимальная ошибка поверхности <0.02 мм. Строгий порог 0.01 мм проходит не везде; точные значения и сохранность UV/топологии/материалов — [roundtrip.json](roundtrip.json). Небольшая разница corner normals измерена, shading не объявлен побайтово одинаковым.

## Реальный GPU

[gpu_pose_audit.cpp](../../tools/raphtalia/gpu_pose_audit.cpp) создаёт скрытый OpenGL 3.3 context и использует **настоящие GLModelLoader/OpenGLRenderer**. Независимый Assimp sampler интерполирует local position/scale линейно, rotation через quaternion slerp, затем передаёт palette через существующий API. Неразрешённые channels, NaN и GL errors проверяются. Рендеры читаются с GPU.

Итог: **100 изображений, 10 клипов меняют rendering, GL errors = 0, bind reset = EXACT**. Нейтральные endpoints каждого клипа совпадают побайтово на front/back/face views. Проверены 38-node masked composition и пять TRS/quaternion crossfade points. Для `TEST_practical_ranges` выбраны фактические пики 12.5/37.5/62.5/87.5%; четверти длинного клипа нейтральны и как range-проверка не используются. `TEST_toes_30` проверяет оба направления носка стопы на 25/75%.

Доказательства: [протокол](reaudit/gpu_after/gpu_audit.txt), [samples](reaudit/gpu_after/gpu_samples.csv), [endpoints](reaudit/gpu_after/gpu_endpoints.csv), [mix](reaudit/gpu_after/gpu_mix.csv), [front](reaudit/gpu_after/gpu_front_contact.png), [back](reaudit/gpu_after/gpu_back_contact.png), [face](reaudit/gpu_after/gpu_face_contact.png), [crossfade](reaudit/gpu_after/gpu_mix_contact.png).

239-костный prepared baseline заново прошёл тот же GPU path: [gpu_before](reaudit/gpu_before/gpu_audit.txt), 7 клипов / 74 изображения. Новый результат сравнивается с реальной предыдущей моделью. Эти проверки доказывают совместимость и выполнение skinning, но не отсутствие collisions или художественную готовность. Полный OpenGL unit suite и gameplay/performance benchmark не запускались; standalone проверки нельзя выдавать за них.

## Следующий этап runtime

1. Сохранить clips, duration, ticks-per-second и TRS channels до уничтожения importer; явно обработать неизвестный ticks-per-second.
2. Привязать channels к node indices, включая helpers без weights. Для незаданных компонентов использовать bind TRS.
3. Смешивать локальные TRS, rotation — slerp; затем вычислять globals. Не делать matrix lerp для переходов.
4. Передавать palette из `model.skinningMatrices(localPose)` в `renderer.renderModel`.
5. После базового playback — masks/layers, transitions, root motion и foot contacts. Согласовать рост/scale; готовые gait clips не приложены.
6. Для будущих Shape Keys добавить importer/buffers/shader. Нынешние eye/ear/brow bones совместимы с существующим skeletal path.

Команды сборки и запуска: [tools/raphtalia/README.md](../../tools/raphtalia/README.md). GPU-инструмент линкуется с уже собранными Release-библиотеками, поэтому после изменения C++ их необходимо пересобрать. Существующие внешние PNG и material fallback не менялись.
