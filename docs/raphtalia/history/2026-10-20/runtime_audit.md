> ???????????? ????? ?? 20.10.2026. ?????????? ?????????: [REPORT_RU.md](../../REPORT_RU.md).

# Raphtalia: совместимость с текущим C++/OpenGL runtime

Дата проверки: 20 октября 2026. Проверены исходники проекта и выполнен самостоятельный CPU-импорт исходного FBX установленной библиотекой **Assimp 6.0.4**. Приложение и его runtime не изменялись.

## Вывод

Проект поддерживает импорт скелета, bone weights, inverse bind и отображение заданной скелетной позы на GPU. **Воспроизведение FBX-клипов, смешивание анимаций, переходы состояний и morph targets пока отсутствуют.** Исправление самого FBX не создаёт эти возможности автоматически.

Полный машинный результат практического импорта: [assimp_original.json](../../assimp_original.json). Он содержит два независимых импорта: `source_validation` без сокращения весов и `project_import` с postprocess-флагами загрузчика. Данные Blender и Assimp могут различаться: Assimp делит геометрию по материалам и считает только weighted bones как `aiBone`; число узлов сцены не равно количеству костей Armature.

| Проверка исходного FBX | Source validation | Project postprocess |
|---|---:|---:|
| Импорт через Assimp | Успешно | Успешно |
| Узлы сцены | 372 | 372 |
| Меши / экземпляры мешей | 28 / 28 | 28 / 28 |
| Вершины | 187 032 | 186 601 |
| Треугольники | 62 344 | 62 342 |
| Уникальные weighted bones / nodes | 235 / 235 | 235 / 235 |
| Bone entries, суммарно по мешам | 6 580 | 6 348 |
| Максимум положительных weights / vertex | 4 | 4 |
| Вершины без weights | 0 | 0 |
| Сумма weights отличается от 1 более чем на 1e-5 | 0 | 0 |
| Невалидные weights | 0 | 0 |
| Bone без соответствующего node | 0 | 0 |
| FBX animation clips | 0 | 0 |
| Morph targets | 0 | 0 |
| Embedded textures | 0 | 0 |
| Материалы / texture references | 26 / 0 | 26 / 0 |
| NaN/Inf в деформированных bind vertices | 0 | 0 |

Postprocess удаляет два вырожденных треугольника и объединяет некоторые одинаковые вершины. Исходный файл на диске при этом не меняется. Это реальные результаты Assimp, а не предположение о возможностях формата.

Максимальная ошибка CPU-реконструкции bind pose — **0.0227748435 scene units**. Высота imported bounds — **423.0130615 scene units**, относительная ошибка около **5.38e-5**. Это маленькая, но ненулевая ошибка: точное совпадение bind pose не подтверждено. При fitting до высоты 2.2 ошибка соответствует приблизительно 0.0001184 runtime units. Часть расхождения может быть связана с float-арифметикой и экспортированными bind matrices; инструмент не устанавливает причину автоматически.

Исходные bounds в imported scene space: min `(-88.9279633, -320.2386475, -437.9806519)`, max `(66.4183044, 102.7744141, -242.7533875)`. Это не положение персонажа в нормированном окне desktop: приложение отдельно центрирует и масштабирует asset.

## Финальная подготовленная модель: практическое подтверждение

Ниже — результаты **повторного запуска после финальных корректировок weights перчаток и манжет, восстановления длины terminal pinky, исправления теста twist вокруг продольной local Y и коррекции концов четырёх terminal hair bones и head bone**. Проверен `assets/models/Raphtalia_prepared.fbx`, размер **23422156 байт**, SHA256 **`E9D5E45AED9C8B05BA5C9B9FA04B3508D936D003FACE8E409BF5D34B622217F9`**. В промежуточном первом экспорте clips отсутствовали; все приведённые далее evidence относятся к финальному экспорту с семью клипами.

| Проверка prepared FBX | Source validation | Project postprocess |
|---|---:|---:|
| Загрузка / validation | Успешно | Успешно |
| Nodes / meshes | 269 / 28 | 269 / 28 |
| Вершины | 187 032 | 167 327 |
| Треугольники | 62 344 | 62 342 |
| Уникальные weighted bones / nodes | 239 / 239 | 239 / 239 |
| Максимум influences | 4 | 4 |
| Вершины без weights / invalid weights | 0 / 0 | 0 / 0 |
| Сумма weights отличается от 1 более чем на 1e-5 | 0 | 0 |
| Bone-node resolution errors | 0 | 0 |
| Bind vertex max residual, scene units | 3.7850416e-6 | 3.7850416e-6 |
| Skeletal animation clips | 7 | 7 |
| Материалы / texture refs / embedded images | 26 / 0 / 0 | 26 / 0 / 0 |
| Morph targets | 0 | 0 |

Исходные 235 weighted bone names сохранены, добавлены `eye.L`, `eye.R`, `ear.L`, `ear.R`. Все 26 material names совпадают с original. Отсутствие texture refs сохраняется из исходного FBX и не означает удаления текстур инструментом проверки. Количество raw source vertices и triangles не изменилось; уменьшение vertices после project postprocess связано с `JoinIdenticalVertices` и данными нового экспорта. Изменённые units/root transforms не позволяют сравнивать original/prepared scene-space coordinates простым вычитанием; проверка внешности/нейтральной формы производится отдельно в Blender.

Каждый clip длится **60 ticks / 30 ticks per second = 2 секунды**, содержит **240 node channels**. Имена: `Raphtalia_Rig|TEST_elbows_90`, `TEST_eyes_ears`, `TEST_fingers_35`, `TEST_hair_tail_cloth`, `TEST_head_torso`, `TEST_knees_90`, `TEST_shoulders_hips_ankles` (для последних шести также присутствует префикс `Raphtalia_Rig|`). Во всех клипах — 0 ambiguous/unresolved channels, 0 nonfinite key entries, 0 nonfinite sampled global matrices и deformed vertices. CPU-проверка включает девять равномерных времён каждого clip, включая начало, midpoint и конец. Полные данные: [assimp_prepared.json](assimp_prepared.json).

Практический GPU-тест через **существующие** `GLModelLoader`, `GLModel::skinningMatrices()` и `OpenGLRenderer` сохранил **74 PNG**: 4 neutral views, 56 кадров отдельных клипов и 14 кадров комбинации/переходов. Во всех семи clips зарегистрированы изменения pixels при подаче анимированной palette. Ошибок OpenGL — **0**, восстановление явной bind palette — **точное по RGB**. На этом export runtime использует **239 palette entries**. См. [gpu_audit.txt](gpu_audit.txt), [gpu_samples.csv](gpu_samples.csv), [neutral contact](gpu_neutral_contact.png), [front contact](gpu_front_contact.png), [back contact](gpu_back_contact.png), [face contact](gpu_face_contact.png).

Дополнительно начало и конец каждого clip отрисованы в четырёх views: во всех случаях **end ↔ start: 0 изменённых RGB bytes, 0 matrix delta**. Начало clip и явный bind не являются bit-exact: максимум CPU vertex displacement около **2.97e-6 scene units**, palette element delta **2.12193e-5**, различаются **91–743 RGB bytes** изображения 768×768×3. Это следует отличать от точного возвращения start ↔ end. Данные: [gpu_endpoints.csv](gpu_endpoints.csv).

Отдельный пример совместной анимации накладывает local pose по **38 node masks**: локти, голова/корпус, пальцы, выбранные цепи волос, хвоста и одежды. Отдельный crossfade смешивает peak локтей и peak коленей при весах **0, 0.25, 0.5, 0.75, 1**: position/scale интерполируются линейно, rotation — quaternion slerp; смешивание matrix elements не используется. Все local/global/palette matrices конечны, GPU-отображение меняется. [gpu_mix.csv](gpu_mix.csv), [mix/contact](gpu_mix_contact.png). Этот код находится в диагностическом инструменте; полноценный playback, state machine и blending не добавлены в desktop application.

Contact sheets просмотрены: в отобранных позах видны движения рук/ног/головы и комбинированной позы, не обнаружено разлёта mesh. Это ограниченная визуальная проверка. Она **не подтверждает** идеальное сохранение объёма суставов, качество каждого пальца в экстремальной позе, отсутствие всех пересечений или готовые циклы ходьбы/бега. `eyes_ears` проверяет только добавленные skeletal controls и не является тестом моргания, губ или полного facial rig. Исходные материалы без texture refs и faceted normals также видны в GPU images.

## Контракт скелетной деформации

| Компонент | Реализация и местоположение |
|---|---|
| Иерархия | Все узлы сохраняются в порядке parent-before-child: `plugins/render_opengl/src/GLModelLoader.cpp:784`, `include/julia/render_opengl/GLModel.hpp:20`. |
| Bone → node | Сначала `aiBone::mNode`; при отсутствии — поиск единственного node с данным именем. Неоднозначное имя вызывает ошибку: `GLModelLoader.cpp:709`. |
| Вершины | Mesh-node global transform запекается в geometry: `GLModelLoader.cpp:638`. |
| Inverse bind | `adjustedOffset = AssimpOffset * inverse(meshGlobal)` — `GLModelLoader.cpp:724`. |
| Palette | `globalNode * adjustedOffset` — `plugins/render_opengl/src/GLModel.cpp:122`. |
| Вход позы | Одна **локальная матрица на каждый node**, включая не являющиеся костями helpers: `GLModel.cpp:95`, `GLModel.hpp:61`. |
| Weights | Четыре наиболее сильных веса, нормализуемые после импорта: `GLMesh.hpp:23`, `GLModelLoader.cpp:113`, `:158`, `:857`. |
| GPU skinning | Linear blend skinning, GLSL 330, `assets/shaders/model.vert:47`. ID передаются как integer attribute: `GLMesh.cpp:287`. |
| Normals | Учитываются cofactor нормалей и знак skin matrix: `model.vert:82`. |

**Не добавлять стандартный `inverse(root)` повторно:** этот проект использует импортированное scene space и для geometry, и для bone palette. Root transform уже включён в обе стороны контракта. Обычная формула из tutorial с дополнительным inverse root здесь даст другой результат.

Palette entries не равны уникальным костям: для одного skeletal node допустимы разные adjusted inverse binds у разных mesh instances. Загрузчик сохраняет отдельные записи при различающихся матрицах (`GLModelLoader.cpp:727–752`). Все записи нужно заполнить при передаче palette. Bone name сам по себе не является уникальным ключом palette.

Невзвешенные вершины остаются в baked bind position. `GLMesh` не сохраняет исходный mesh-node index; анимация одного невзвешенного mesh-node не двигает такую геометрию. Для подвижных аксессуаров подходят weights 1.0 к соответствующей кости либо отдельная доработка runtime mesh transforms.

## Чего в проигрывателе ещё нет

- `GLModelLoader::loadFromFile()` не копирует `aiScene::mAnimations`, channels и keys до уничтожения локального `Assimp::Importer` (`GLModelLoader.cpp:835–901`).
- `GLModel::skinningMatrices()` не является sampler: вызывающий код должен вычислить локальную позу.
- В `apps/julia_desktop/main.cpp:402` вызывается `renderModel(model, modelMatrix)` без palette. Это статическая geometry, даже если в FBX имеются animation clips.
- Morph/blendshape geometry и morph channels не импортируются в runtime. Shader не содержит morph deformation.
- Отсутствуют state machine, crossfade, additive layers, bone masks, IK, foot locking и вторичная физика волос/хвоста/одежды.
- Blender constraints, drivers, control bones, pose libraries не вычисляются в C++ автоматически. Результат constraints/IK следует bake в deform bone keys.
- Shader использует LBS; Blender Preserve Volume / dual quaternion deformation не повторяется. Качество следует проверять с отключённым Preserve Volume и максимум четырьмя весами.

## OpenGL и производительность

Фиксированного ограничения 100/128 костей в shader нет. Матрицы передаются в `samplerBuffer`, формат `GL_RGBA32F`, четыре texel на mat4. Runtime запрашивает `GL_MAX_TEXTURE_BUFFER_SIZE` и проверяет palette count against `limit / 4` (`OpenGLRenderer.cpp:349`, `:513–516`, `:572`). Дополнительный самостоятельный GPU-тест на NVIDIA GeForce RTX 4070 SUPER получил `GL_MAX_TEXTURE_BUFFER_SIZE=268435456`, то есть 67108864 mat4 в пределах этого API-limit. Это не гарантия возможности выделить столь большой буфер; реальная память ограничивается отдельно. Импортированный original использовал лишь 235 palette entries.

Palette занимает 64 байта на запись. Сейчас `glBufferData(..., GL_STREAM_DRAW)` вызывается при каждом skinned draw (`OpenGLRenderer.cpp:562–565`). Оптимизацию следует выбирать по измерениям, учитывая не только уникальные кости, но и palette entries, количество draw calls и passes. Количество костей само по себе не подтверждает проблемы производительности.

Камера и fitting используют Y-up (`OpenGLRenderer.hpp:18`, `GLModel.cpp:141`). Физические единицы не нормализуются самим loader; приложение применяет `fitToHeightTransform(2.2f)` (`main.cpp:322`). Root motion, physical scale и animation displacement должны использовать согласованный масштаб.

## Практическое тестирование и его пределы

Самостоятельный инструмент [assimp_audit.cpp](../../../../tools/raphtalia/assimp_audit.cpp) выполнен на исходном `assets/models/Raphtalia.fbx`. Он проверяет два варианта импорта, finite matrices, weights, bone-node resolution, hierarchy, material texture references, morph counts, CPU bind deformation и imported bounds. Source validation также использует `aiProcess_PopulateArmatureData`, но не ограничивает веса.

Если в asset присутствуют clips, инструмент семплирует каждый в девяти равномерных точках, используя linear interpolation position/scale, quaternion slerp rotation, endpoint clamp, затем вычисляет всю иерархию и деформирует вершины CPU. Результаты включают finite checks и displacement относительно bind geometry. Mesh/morph animation channels только подсчитываются. Поведение вне диапазона keys и нестандартная интерполяция не воспроизводятся; этот sampler предназначен для диагностики экспортированных baked skeletal clips.

В исходном файле clips отсутствуют, поэтому **практическое тестирование временной анимации исходного FBX через Assimp не состоялось**. Численная корректность skinning не означает естественность суставов или отсутствие пересечений: это требует визуальных позовых тестов.

Дополнительно собран и выполнен [gpu_pose_audit.cpp](../../../../tools/raphtalia/gpu_pose_audit.cpp). Для original он создаёт скрытый OpenGL 3.3 context, загружает модель **действительными GLModelLoader/OpenGLRenderer**, передаёт bind palette, сохраняет четыре PPM (front/back/face front/face back), повторно возвращается в bind pose и сравнивает RGB точно. Результат: 4 снимка, 0 GL errors, exact bind reset. Драйвер `3.3.0 NVIDIA 617.42`, GLSL `3.30`, 768×768, 4×MSAA. Contact sheet просмотрен: framing персонажа и лица корректен, отсутствие texture refs и исходная faceted shading видны. Это подтверждение GPU skinning path в neutral pose; в original нет clips для проверки движения.

GPU-инструмент для файлов с clips отдельно загружает Assimp channels, интерполирует локальные TRS, вызывает `model.skinningMatrices(localPose)` и передаёт реальную palette renderer. Семплы — 25%, 50%, 75% duration; front/back для всех, closeup лица для midpoint. `gpu_samples.csv` содержит число mapped/unresolved channels, delta palette и изменённых RGB bytes. Pixel difference доказывает влияние поданной позы на rendering, но не устанавливает естественность суставов или отсутствие пересечений. Инструмент не изменяет desktop playback.

Существующий `tests/test_opengl.cpp:125–149` загружает Raphtalia, рисует bind palette и затем сдвигает всю palette. Это тест загрузки GPU weights, но не сгибания суставов. В рамках данного runtime-аудита этот тест не запускался; готовые test EXE в build/tests не найдены. Старый LastTest.log не использовался как доказательство текущей работоспособности.

## Воспроизводимый запуск

Из PowerShell в `C:\Julia`:

```powershell
& .\tools\raphtalia\build_assimp_audit.ps1
& .\tools\raphtalia\build_assimp_audit.ps1 `
  -InputModel 'path/to/prepared.fbx' `
  -OutputJson 'docs/raphtalia/assimp_prepared.json' -SkipBuild

& .\tools\raphtalia\build_gpu_audit.ps1 `
  -InputModel 'path/to/prepared.fbx' `
  -OutputDirectory 'build/raphtalia_tools/gpu_prepared'
python .\tools\raphtalia\convert_gpu_evidence.py `
  build/raphtalia_tools/gpu_prepared docs/raphtalia
```

Сырые PPM остаются в ignored `build`; в `docs/raphtalia` находятся PNG, contact sheets, CSV и текстовый отчёт. Скрипт GPU-сборки использует уже собранные Release static libraries проекта, поэтому перед повторением на изменённом runtime нужно сначала пересобрать проект. Pillow требуется только для преобразования evidence в PNG.

Скрипт создаёт только build artifacts в `build/raphtalia_tools` и указанный JSON. Модель не перезаписывается. Инструмент завершает процесс с ненулевым кодом при неудачном импорте; конкретные численные diagnostics, warnings и ограничения остаются в JSON и требуют интерпретации.

Используемое окружение:

- MSVC: `C:/Program Files/Microsoft Visual Studio/18/Professional/VC/Tools/MSVC/14.50.35717/bin/Hostx64/x64/cl.exe`.
- MSVC setup: `C:/Program Files/Microsoft Visual Studio/18/Professional/VC/Auxiliary/Build/vcvars64.bat`.
- Assimp headers/library/DLL: `build/vs-release/vcpkg_installed/x64-windows/{include/assimp,lib/assimp-vc143-mt.lib,bin/assimp-vc143-mt.dll}`.
- CMake/CTest: `C:/Program Files/Microsoft Visual Studio/18/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/`.
- Ninja: соседний `CommonExtensions/Microsoft/CMake/Ninja/ninja.exe`.
- Presets `vs-release`, `build-release`, `test-release`; C++20. Dependency baseline — `vcpkg.json:4`.

## Следующие изменения в animation runtime

1. Скопировать clips, TRS channels, duration и ticks-per-second в независимые структуры до уничтожения Assimp scene. Явно обработать неизвестный ticks-per-second.
2. Сопоставлять channel с node index, сохранять bind TRS для незаданных компонентов и helpers. Не отбрасывать узлы, которые не имеют weights.
3. Смешивать локальные position/rotation/scale, применять quaternion slerp и вычислять global transforms после blending. Не выполнять matrix lerp для переходов.
4. Передавать результат `model.skinningMatrices(localPose)` в `renderer.renderModel(model, modelMatrix, palette)`.
5. Добавить bone masks для совместной ходьбы и жестов, отдельную facial layer, затем crossfade/state machine и root motion.
6. Для morph-мимики расширить importer, buffers и shader; bone-мимика использует уже имеющийся skinning path.
7. Проверить экспортированные clips на GPU по нескольким выбранным суставам и сравнить CPU/GPU-деформацию с Blender. После этого тестировать художественное качество циклов, переходов и пересечений.
