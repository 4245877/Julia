# CPU-ресурс модели и GPU-загрузка: второй этап

Рабочий каталог: `C:\Julia`. Исследование и проверка: 2026-11-08.
Развиты находившиеся в рабочем дереве изменения первого этапа; сброса не было.

## Исходное состояние и уточнённый план

До переноса штатный `tools/build.ps1 -UseInstalledDependencies -KeepDiagnostics`
подтвердил **19/19**, отдельная `cpu-release` с warnings-as-errors — **5/5**.
Логи: `build/diagnostics/stage2/baseline-build.log`, `baseline-cpu.log`.
В sandbox Ninja зависал до компиляции; сборки выполнены с расширенным доступом.

Исследование подтвердило: `GLModelLoader::processMesh` запекал mesh transforms,
строил adjusted offsets и сразу создавал GLMesh; material loader сразу создавал
GLTexture. `GLModel::addNode` декомпозировал bind TRS через Assimp. EyePoseLayer
читал GLModel, хотя сам не вызывал GL. Поэтому план уточнён до отдельных модулей
ресурса, импорта и CPU-адаптера персонажа, с совместимым графическим фасадом.

## Получившийся путь и владение

`Файл → ModelImporter → shared_ptr<const ModelAsset> → GLModelLoader::upload → GLModel → renderer`.

- `julia_animation`: прежнее ядро позы, зависимость только GLM; Assimp/GL не добавлены.
- `julia_model`: геометрия, bounds, параметры материалов, RGBA8-изображения,
  Skeleton/bindings/clips; общий CPU-код восстановления нормалей. stb используется
  здесь, в единственном декодере изображений.
- `julia_model_import`: единственный поддерживаемый Assimp-импорт. Importer и scene
  локальны функции и уничтожаются до её возврата. Строки, ключи, вершины и пиксели
  скопированы; указателей на Assimp в ресурсе нет. Assimp — закрытая зависимость.
- `julia_embodiment`: прежний AnimationPlayer и специфичный для Raphtalia
  EyePoseLayer, принимающие CPU ModelAsset. Графических зависимостей нет.
- OpenGL-плагин создаёт меши, текстуры и sampler-объекты из готового ресурса.
  `loadFromFile` остался коротким совместимым вызовом import + upload; приложение
  явно вызывает эти операции отдельно. GLModel удерживает **тот же** CPU-ресурс.

ModelAsset строится до публикации и затем читается через const. Изображения
разделяются материалами через `shared_ptr<const Image>`; sampling принадлежит
материалу. Player/eyes ссылаются на asset, который должен их пережить, и хранят
собственные Pose, время и поведение. Несколько экземпляров не разделяют позу.
Совместимый procedural-builder у GLModel предназначен для ручных GPU-сцен;
узлам нужен явный bind TRS. Мутации скелета импортированного GLModel запрещены.

GPU upload не читает файлы. Создание, перемещение с освобождением прежних ресурсов
и уничтожение GPU-владельцев требуют текущего инициализированного GL-контекста.
В приложении и тестах владельцы уничтожаются внутри области до закрытия окна;
CPU-ресурс может пережить контекст и продолжать вычисления.

## Сохранённые контракты

Сохранены bake позиций, inverse-transpose нормалей, коррекция winding при
отражениях, порядок parent-before-child и точные bind-матрицы. Bind TRS по-прежнему
получается через Assimp на границе импорта. Bindings объединяются только при
совпадении узла и **точном** совпадении adjusted offset; палитра остаётся
`global * (assimpOffset * inverse(meshGlobal))`. IDs и нормализация четырёх
влияний, ключи, fallback 25 ticks/s и отсутствующие компоненты не изменены.

Сохранён порядок разрешения внешних текстур: исходный абсолютный путь либо
model-relative/textures-relative путь, затем basename возле модели и в соседнем
textures. Legacy atlas fallback остаётся opt-in. Сохранены UV channel selection,
embedded compressed/raw images, top-down RGBA без второго flip, alpha policies,
цвета, toon profiles и отдельный sampling при общей картинке.

Глаза применяются после sampling и до окончательных FK/palette. Авторские
eye-клипы и собственная gaze-анимация сохраняют приоритет. Tracking, bake 188 век,
pause/stop/seek и прежнее кадровое время перенесены без изменения. Добавлено
отклонение eye-layer от другого asset вместо применения неверных индексов.

## Проверки и независимый эталон

**Основная конфигурация: 21/21. CPU-only: 10/10. Обе собраны с warnings-as-errors.**
После последнего защитного изменения проверки импортёра дополнительно повторены.

- `model_asset`: настоящий Raphtalia_prepared.fbx импортируется без GLFW/GL;
  после уничтожения импортёра работают поза, tracking и моргание. Геометрия,
  индексы, normals/UV/weights, материалы, полные RGBA, skeleton/bind TRS/offsets,
  все ключи и результаты eyes совпали с **снятым до переноса** результатом старого
  пути для обоих режимов нормалей. Это не сравнение нового импортёра с самим собой.
  Эталон: `tests/data/raphtalia_stage2_reference.txt`; формат и происхождение рядом
  в README. 28 мешей; 171 523 исходных / 171 550 repaired вершин, 206 826 индексов,
  459 узлов, 429 bindings, 15 клипов.
- `pose_parity`: **17 791 поза**, все 15 клипов, максимальная разница
  locals/globals/palette **0** относительно сохранённого прежнего алгоритма.
- CPU fixtures: instanced skin с двумя offsets одного узла, отражение и
  неравномерный scale, UV1, compressed/raw embedded и внешние картинки, Unicode,
  shared images с разным sampling, fallback policy, default ticks, неверные
  индексы/веса/UV/размеры и независимые экземпляры. Удаление source/image-файлов
  после импорта не ломает данные; GPU upload также проверен после удаления source.
- `opengl`: импорт до glfwInit, отдельный upload и повторный upload одного asset,
  затем вычисления CPU после уничтожения GPU и контекста. Также сохранены старые
  model_import, GPU resource, material, shader, outline и toon-проверки.
- Desktop: запуск/камера/режимы окна, **15 клипов / 334 кадра**, глаза / **65 кадров**,
  итог `PASS`, GL errors **0**. Выборочно просмотрены neutral и full blink.
  Рендер проверен на NVIDIA GeForce RTX 4070 SUPER, OpenGL 3.3.
- `dumpbin /dependents` headless EXE и поставляемых Assimp DLL, а также команды
  CPU-сборки: OpenGL/GLFW/GLAD отсутствуют. Проверка не ограничена отсутствием окна.

Итоговые логи и копии кадров: `build/diagnostics/stage2/` (`final-tests.log`,
`final-import-checks.log`, `final-cpu-tests.log`, `cpu-dependents.log`,
`cpu-build-commands.log`, `gpu/`). Штатные CTest также сохраняют свои результаты
в `build/vs-release/verification/`. В первом расширенном прогоне была исправлена
ошибка нового теста: у Raphtalia1 нет skin bindings, поэтому пустая палитра корректна.

## Воспроизведение

Из Developer PowerShell for VS x64 в `C:\Julia`, с установленными зависимостями:

```powershell
# Штатная сборка и полный тестовый набор:
.\tools\build.ps1 -UseInstalledDependencies -KeepDiagnostics

# Усиленная основная конфигурация, использованная для итоговой проверки:
cmake --preset vs-release -DVCPKG_MANIFEST_INSTALL=OFF -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build --preset build-release --parallel 4
ctest --preset test-release -V

# Полностью отдельный CPU-путь:
cmake --preset cpu-release -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build --preset build-cpu-release --parallel 4
ctest --preset test-cpu-release -V
dumpbin /dependents build/cpu-release/tests/julia_test_model_asset.exe

# Основной headless-критерий отдельно:
.\build\cpu-release\tests\julia_test_model_asset.exe build/diagnostics/stage2/manual
```

`cpu-release` использует зависимости из `build/vs-release/vcpkg_installed`.
Нужны GLM, stb и Assimp; `julia_animation` отдельно по-прежнему требует только GLM.
Для обычного PowerShell подготовка MSVC приведена в отчёте первого этапа.

## Ограничения и следующий этап

Это CPU-ресурс визуального рига; загружаются все mesh/image/clip данные, без streaming
или сериализованного cache. RGBA и CPU-геометрия сохраняются также при наличии GPU,
что увеличивает память. Эталон привязан к модели, Assimp и численной среде MSVC;
он не доказывает идентичность на другом GPU/компиляторе. Проверена Release, не Debug.
Глаза остаются адаптером Raphtalia, а общий EyeBehavior пока размещён в CPU-модуле
render. Семантика времени тела не изменена. Морфы, IK, физика и ИИ не добавлялись.

Следующий ограниченный этап — **CPU-профиль калибровки тела Raphtalia**: стабильные
узлы/цепи, оси и нулевые рамки, единицы и body/world/rig transforms, управляемые DoF
и пределы. Критерий — проверяемые положения кистей/стоп и знаки каждой оси;
без добавления scheduler, IK, физики или ИИ в этот этап.
