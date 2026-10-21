# Raphtalia: текущий C++/OpenGL contract

Проверка: **21 октября 2026**. Актуальный `Raphtalia_prepared.fbx`: **429 deform bones, 15 TEST-клипов**, включая четыре клипа моргания. SHA256 и размеры — [deliverable_manifest.json](deliverable_manifest.json). Реализация, визуальные сравнения и ограничения — [REPORT_RU.md](REPORT_RU.md). Текущие evidence находятся в `blink/`; `deep_deformation/` относится к предыдущему снимку 241/11.

## Что фактически поддерживает JULIA

`plugins/render_opengl/src/GLModelLoader.cpp` импортирует meshes, материалы, четыре vertex weights, nodes и inverse bind. `GLModel::skinningMatrices(localPose)` принимает локальные трансформации всех nodes в порядке модели, вычисляет globals parent→child и palette `globalNode × adjustedOffset`. Второй inverse(root) не нужен. Пустая поза использует imported bind.

`OpenGLRenderer::renderModel(model, modelMatrix, palette)` выполняет LBS без Preserve Volume. Palette хранится в `samplerBuffer`/`GL_RGBA32F`: четыре texel на mat4. Фиксированного предела 256 костей нет; доступный предел проверяется через `GL_MAX_TEXTURE_BUFFER_SIZE / 4`. Итоговые 429 матриц занимают 27456 байт. В аудите RTX 4070 SUPER сообщил 268435456 texels, или 67108864 матрицы; это характеристика проверенного GPU, не performance benchmark.

**Основной desktop остаётся статическим.** Loader не сохраняет `aiAnimation`/morph channels после уничтожения Assimp scene. Animation player, state machine и runtime UI для `Blink_L/R` не добавлены. Standalone auditor использует готовый renderer API и независимый sampler; он не внедряет playback в desktop. C++-рендерер не изменён.

## Моргание и экспортный контракт

В BLEND объект `Raphtalia_Rig` содержит `Blink_L` и `Blink_R` от 0 до 1. Они управляют реальной геометрией через стандартные drivers новых костей век и корректирующей поверхности. Все новые кости находятся под существующей головой. Исходные 241 кость, их rest/parents, 11 Actions, IK/FK и прежние лицевые controls сохранены.

FBX переносит **запечённые TRS костей**, не Python или зависимости от Blender. Авторские Actions имеют 30 fps; экспортный шаг 0,25 кадра даёт 120 samples/sec, simplify 0. Длительности четырёх клипов — 1/2/4/4 с. Это увеличивает файл примерно до 170 МБ. Morph targets/channels/keys = 0, что отдельно проверено [morph-аудитором](blink/morph.json). Morph shader для этого решения не требуется.

Контроллеры Blender сами не становятся интерфейсом приложения. Для непрерывных независимых скаляров необходимо перенести выражения из [patch.json](blink/patch.json), соблюдая исходные координаты и масштаб 0,01, либо проигрывать запечённые клипы с масками лицевых костей. Матрицы поз для смешивания следует строить из локальных TRS, quaternion — slerp; matrix lerp не использовать.

## Практические проверки

| Проверка | Результат |
|---|---|
| Blender → FBX → Blender | 216 sampled poses, 429/429 костей, 15/15 Actions, родители и веса сохранены |
| Максимальная ошибка нейтрали / анимации | 0,00196 / 0,03192 мм; включены дробные состояния 25/50/75% |
| Assimp 6.0.4: source/project flags | 459 nodes, 28 meshes, 26 materials, 15 clips, 429 palette entries, max4 weights |
| Invalid / unweighted / nonunit / unresolved / nonfinite | 0 в обоих режимах |
| Source/project vertices | 206832 / 171523 |
| Source/project triangles | 68944 / 68942; два прежних degenerate удаляются существующим pipeline |
| Реальный OpenGL | 307 snapshots, 15/15 clips изменяют rendering, GL errors 0, bind reset EXACT |
| Начало/конец каждого клипа | RGB совпадают; отдельная повторная отрисовка bind точна |

[Roundtrip](blink/roundtrip.json), [Assimp](blink/assimp.json), [GPU-протокол](blink/gpu/gpu_audit.txt) содержат SHA256 проверенных байтов. Кандидат был проверен до записи в prepared assets; итоговые файлы имеют те же SHA256. После локального subdivision расстояние между ближайшими вершинами исходной и новой сетки не является расстоянием между поверхностями. [Integrity](blink/integrity.json) проверяет исходные точки, barycentric provenance, площади и UV; эта проверка сохраняет смысл при изменении числа вершин.

GPU-аудитор вызывает настоящие `GLModelLoader`/`OpenGLRenderer` в скрытом GL 3.3 context. Независимый Assimp sampler интерполирует position/scale линейно, rotations — slerp. Для blink выбраны реальные закрытые ключи и дробные уровни, а не только 25/50/75% длительности. [35 выбранных состояний / 70 PNG](blink/gpu/manifest.json) показывают общие уровни, независимые wink, шесть направлений взгляда и голову. Сохранены прежние masked composition и TRS crossfade checks.

Нулевое количество Assimp texture slots у исходного prepared FBX — прежнее ограничение; текущие GPU изображения используют clay-материалы. Исходные materials, UV и файлы текстур сохранены. Эти кадры подтверждают deformation/render path, а не textured beauty render. Небольшой canthus pinch при полном закрытии описан в основном отчёте. Проверки конечны и не доказывают отсутствие всех коллизий для любых поз/камер.

## Следующие изменения runtime

1. Сохранить clips, duration, ticks-per-second и TRS channels до уничтожения importer; определить обработку неизвестного ticks-per-second.
2. Привязать channels ко всем node indices, включая helpers; незаданные компоненты брать из bind TRS.
3. Смешивать локальные TRS и quaternion slerp; затем вычислять globals и palette через существующий API.
4. Добавить playback и лицевые masks; для независимых `Blink_L/R` перенести документированные выражения или авторить соответствующие клипы.
5. Проверять размер palette через уже существующий GL texture-buffer limit. Увеличение skeleton не требует нового shader contract.
6. Речевую мимику и возможные morphs исследовать отдельным этапом. При выборе morphs понадобятся importer/buffers/shader и отдельная проверка morph deltas/channels; нынешний успешный skeletal playback этого не доказывает.

Команды и ограничения исторических генераторов: [tools/raphtalia/README.md](../../tools/raphtalia/README.md).
