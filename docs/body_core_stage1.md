# CPU-ядро позы: первый этап

Рабочий каталог: `C:\Julia`. Исследование и реализация: 2026-11-08.
Предыдущий `build/diagnostics/body_architecture_2026-11-08/report.md` использован
как справочный материал; выводы ниже проверяются по действующему коду.

## Последовательность работ

1. Проследить импорт → данные модели → sampling → глаза → FK → palette → GPU.
2. До изменения вычислений выполнить конфигурацию, сборку и существующий CTest;
   сохранить результат отдельно от результата новой реализации.
3. Добавить `julia_animation` (`libs/animation`, C++20 + GLM): описание узлов,
   привязок скиннинга и клипов, состояние позы и отдельные вычислительные функции.
4. Перенести владение CPU-данными из полей `GLModel` в CPU-типы без дублирования;
   оставить совместимые методы-адаптеры. Сохранить Assimp-декомпозицию на границе
   импорта и время/команды проигрывания в `AnimationPlayer`.
5. Подключить CPU-позу к проигрывателю; глаза применять к локальным матрицам
   после sampling и до FK/palette. Сохранить существующий контракт рендерера.
6. Проверить математические контракты без GL, сравнить с сохранённым прежним
   алгоритмом, затем выполнить существующие проверки приложения и глаз.
7. Записать фактические результаты, ограничения и этапы дальнейшей реализации.

## Проверенный исходный путь и точки связи

- Корневой CMake подключает `core`, `diagnostics`, `events`, `runtime`, `render`,
  OpenGL-плагин, desktop, tools и tests напрямую. `libs/CMakeLists.txt` не входит
  в этот путь; `embodiment`, `physics`, `world`, `cognition` — INTERFACE-заготовки.
- `GLModelLoader.cpp::registerNodes` сохраняет все Assimp-узлы в порядке
  parent-before-child; `GLModel::addNode` использует `aiMatrix4x4::Decompose`.
  `copyAnimations` копирует ключи и разрешает каналы в индексы узлов; fallback
  нулевой исходной частоты — 25 ticks/s.
- `processMesh` запекает `meshGlobal` в вершины и сохраняет
  `assimpOffset * inverse(meshGlobal)`; совпадение имени недостаточно для
  объединения skin bindings. `loadMaterial` загружает `GLTexture`, а конструктор
  `GLMesh` создаёт GPU-буферы прямо внутри импорта. Это действительная связь
  импорта с GL-контекстом.
- До переноса `GLModel.hpp` включал `GLMesh.hpp` вместе с CPU-узлами/клипами;
  `GLModel::skinningMatrices` вычислял FK/palette. `Animation.cpp` содержал
  sampling и проигрыватель, а цель `julia_render_opengl` публично линковала GL,
  GLFW, GLAD, Assimp, GLM и spdlog. Поэтому даже вычислительные тесты линковались
  с графическим плагином.
- `EyeBehavior` в `libs/render` задаёт взгляд и моргание; `EyePoseLayer`
  в плагине адаптирует это к Raphtalia (188 деформаторов век и исходный bake).
  Авторские eye-клипы имеют приоритет. При target gaze нужен FK базовой позы.
- `main.cpp` создаёт модель, player и eyes; передаёт frame delta игроку,
  `player.palette()` — в `OpenGLRenderer::renderModel`. Камера использует
  `player.globalTransforms()`. Рендерер загружает готовую палитру в texture buffer;
  источник движения ему не нужен.

## Границы первого этапа

Неизменяемые после построения данные: полная иерархия и точные bind-матрицы,
импортированные bind TRS для sparse-каналов, отдельные skin bindings и клипы.
Изменяемые данные экземпляра: локальные/глобальные матрицы и палитра; время и
playback state пока принадлежат переходному `AnimationPlayer`. CPU-ядро не
владеет часами, окном, GL-объектами, импортёром или поведением глаз.

Математика: column vectors; локальный animated transform — `T * R * S`;
`global = parentGlobal * local`; root включён; `palette = global * adjustedOffset`.
Неанимированные узлы и пустые каналы сохраняют **точную** исходную матрицу,
включая shear/reflection. Sparse-компоненты используют прежний Assimp bind TRS.
Обратное разложение/сборка всех bind-матриц недопустимы.

Полное отделение FBX-импорта и перенос специфичного eye-adapter из плагина
не входят в этот этап. Новые solver, физика, управление суставами и ИИ пока
не реализуются.

Реализованное API:

- `Skeleton.hpp`: `SkeletonNode`, `SkinBinding`, `Skeleton` с проверкой
  parent-before-child и вычислением global bind; без Assimp-декомпозиции.
- `Animation.hpp`: собственные ключи/каналы/клипы; `prepareAnimationClip`
  проверяет данные и нормализует quaternion при построении asset;
  `sampleAnimation` читает подготовленный клип и пишет caller-owned locals.
- `Pose.hpp`: `Pose` экземпляра и независимые `setBindPose`,
  `computeGlobalTransforms`, `buildSkinningPalette`. После создания Pose
  эти операции не выделяют память. Результаты FK также доступны камере.
- `GLModel` владеет одним `Skeleton` и CPU-клипами, не второй копией иерархии.
  `GLModelNode`/`GLBoneInfo` — совместимые aliases, `skinningMatrices` — адаптер.
  `AnimationPlayer` владеет одним `Pose` и прежним временем, включая pause,
  seek, loop и stop. Изменение asset после привязки player/layer запрещено
  контрактом времени жизни; слой глаз хранит ссылки на клипы.
- `EyePoseLayer` по-прежнему является адаптером Raphtalia в плагине и владеет
  `EyeBehavior`; его targeted-gaze FK теперь тоже вызывает CPU-функцию.
  Порядок: sampling/bind → eye override → FK → palette → renderer.
  Авторские eye-клипы обходят overlay; exact bind требует отключения глаз.

Модель собирается до создания персонажей, затем используется read-only;
несколько `Pose` могут независимо читать один скелет/набор клипов. Массивы
`Pose` принадлежат экземпляру, размер должен соответствовать asset. Частичное
перекрытие буферов не поддерживается; globals/palette должны быть раздельными.
FK допускает точное in-place преобразование locals в globals.

## Следующие этапы (план, не реализованная функциональность)

| Этап | Цель и зависимости | Границы работ | Наблюдаемый критерий готовности |
| --- | --- | --- | --- |
| 2. CPU-импорт и GPU-upload | После ядра получить asset без окна | `ModelAsset` с геометрией, материалами/ссылками на изображения, skeleton, bindings, clips; Assimp CPU-import отдельно от `GLModel` upload; сохранить bake, offsets, normals, веса, texture policy; перенести eye rig-adapter на CPU asset | FBX импортируется без GLFW/GL; сопоставлены узлы, ключи, веса, bounds и палитры; GPU-upload этого asset проходит текущие pixel/eye-тесты; lifetime importer не влияет на данные |
| 3. Калибровка тела | Нужны CPU asset и стабильные идентификаторы | Профиль Raphtalia: цепи, оси/нулевые рамки, метры, body/world/rig transforms, управляемые DoF, ограничения q/v, визуальные helper/twist/lid отдельно; `fitToHeightTransform` остаётся viewer transform | Контрольные точки кистей/стоп совпадают с визуальным ригом; движение каждой DoF имеет ожидаемый знак; пределы и отражённые рамки проверены; отсутствующие/неоднозначные узлы отвергаются |
| 4. Фиксированный шаг | После калибровки, до интерактивного IK | Scheduler в runtime, отдельный BodyInstance, команда с ID/временем/frame, приоритет clip/gaze/reach; dt, catch-up limit, suspend policy и диагностируемый backlog; render snapshots/interpolation; время глаз отдельно от playback pause | Один поток команд при 30/60/144 FPS и без окна даёт одинаковую историю тиков в установленном допуске; pause/seek просмотра не останавливает тело; suspend не даёт большого шага/вечного catch-up |
| 5. Reach/IK кисти | Профиль суставов + фиксированный тик | Положение/ориентация цели, двухзвенная рука или ограниченный iterative solver, pole/elbow continuity, достижимость, ограничения, плавная траектория и отмена; visual binding для helper/twist; сначала без коллизий | Набор заранее не записанных достижимых целей достигается с измеренной ошибкой; недостижимые дают конечную ограниченную позу и статус; смена цели не переворачивает локоть; длины и лимиты сохраняются |
| 6. Сцена, коллизии и опоры | Reach и world queries; физический backend ещё необязателен | Стабильные object handles/frame, ray/sweep/distance queries, floor и препятствия, foot targets и moving supports; планирование пути с учётом объёма руки; root motion согласовать с фактическим перемещением; простой захват обозначать как кинематическое прикрепление | Рука не проходит сквозь тонкую стену между тиками; стопа удерживается в frame движущейся опоры с измеренным drift; удаление цели/опоры и отмена имеют явный результат; grasp success проверяется удержанием |
| 7. Динамическое исполнение | Калиброванные единицы/рамки и scene/contact контракты | Выбор physics backend по отдельному прототипу; массы/инерции/COM, joint drives, q/v, ограниченные усилия, контакты/трение/CCD; reference pose отделить от фактического состояния; один контактный solver | Изменение массы, трения и доступного момента закономерно меняет результат; толчок вызывает наблюдаемое отклонение/восстановление; нет скрытой телепортации к reference pose; воспроизводимая нагрузочная сцена и измеренная стоимость тика |
| 8. Подключение ИИ | Стабильные команды, outcomes и наблюдения тела; кинематический режим возможен раньше динамического | Асинхронные цели и отмена, дедлайны, confidence/reason, proprioception/contact snapshots, запись/воспроизведение экспериментов; выбранная политика не владеет GL и не блокирует тик | Один сценарий управляется скриптом и будущим агентом через один API; успех/неуспех вытекает из состояния; replay воспроизводим; задержка агента не останавливает тело |

Численные критерии IK/опор и частота тика выбираются после калибровки и
измерений, затем становятся фиксированными acceptance thresholds. Наличие
этого ядра не подтверждает биологическую эквивалентность мозга на x86.

## Выполненные проверки

Исходный HEAD: `4fe6444`; исходное рабочее дерево было чистым. Все результаты
ниже относятся к Release на этой машине, MSVC 19.51.36260, Ninja 1.13.2.
GPU-проверки выполнялись на NVIDIA GeForce RTX 4070 SUPER, OpenGL 3.3.

| Проверка | Фактический результат | Свидетельство |
| --- | --- | --- |
| Исходная конфигурация, сборка, CTest до переноса | Успех, **17/17**, 60.18 s | `build/diagnostics/body_architecture_2026-11-08/baseline_build.log` |
| Сборка после выделения и интеграции | Успех; приложение обновлено в `julia_desktop/julia_desktop.exe` | `stage1_build.log`, затем `final_build.log` в той же диагностической папке |
| Полный CTest после интеграции | **19/19**, 67.69 s, включая desktop, анимации, глаза, import/model math, GPU/shaders | `stage1_ctest.log` |
| Отдельная новая CPU-конфигурация `cpu-release` | Сборка с `JULIA_WARNINGS_AS_ERRORS=ON`; **5/5**, 0.23 s; OpenGL-плагин/приложение/Assimp не конфигурируются | `cpu_build.log` |
| Усиленные окончательные pose-тесты после расширения sweep | **2/2**, 12.33 s; 459 узлов, 429 bindings, 15 клипов, **17 791** поз, максимальная абсолютная разница locals/globals/palette **0** | `final_pose_tests.log` |
| Окончательные расширенные edge cases в CPU-only сборке | **1/1** `pose_core`, /WX | `final_cpu_tests.log` |
| DLL-зависимости `julia_test_pose_core.exe` | Только Windows/C++ runtime; GL/GLFW/GLAD/Assimp отсутствуют | `cpu_dependencies.log`, `dumpbin /dependents` |
| Отображение клипов в приложении | Все 15 меняют пиксели, 334 сохранённых кадра, exact bind reset, GL errors = 0 | `build/vs-release/verification/desktop_animation/desktop_animation.txt` |
| Глаза в приложении | 188 lid nodes; не-глазные locals не меняются; bypass авторских blink-клипов имеет palette delta = 0; tracking, pause/stop/resume, 600 s simulated soak и 1000 переключений прошли; GL errors = 0 | `build/vs-release/verification/desktop_eyes/desktop_eyes.txt` |

`pose_core` проверяет root с отражением/неравномерным масштабом, helper с shear,
лес узлов, точные bind-матрицы и пустые каналы, translation/rotation/scale-only
каналы с bind-fallback, несколько offsets одного узла, независимость двух
экземпляров и процедурную правку locals, in-place FK, duplicate timestamps,
одиночные ключи, antipodal quaternions, нулевую длительность, endpoint clamp,
пустой скелет и отклонение неверных размеров/индексов/нечисловых значений/ключей.
Существующий `animation` дополнительно сохраняет playback-регрессии.

`legacy_pose_reference.hpp` — тестовая копия алгоритмов sampling и
`GLModel::skinningMatrices` до переноса; она не вызывает новое ядро.
`pose_parity` сначала проверяет интеграцию player на 3 временах каждого клипа,
затем **уничтожает GPU-модель, окно и GL-контекст**, завершает GLFW и сравнивает
CPU-данные с эталоном: bind, 7 выбранных времён каждого клипа и все различные
времена исходных ключей всех каналов с соседними значениями ±1e-8 s.
Допуск теста 1e-5, наблюдаемая разница 0 относится к этому компилятору/asset.

Просмотрены четыре кадра действительного рендера глаз (neutral, gaze left/up,
asymmetric blink): в `build/diagnostics/body_architecture_2026-11-08/eyes_review.png`
сохранён обзор. Это выборочный визуальный просмотр, не полный ручной аудит
всех движений. Полные автоматические отчёты и кадры остаются под
`build/vs-release/verification/` до следующей очистки.

Первый запуск Ninja в sandbox завис до старта компилятора. Тот же штатный
сценарий вне sandbox завершил исходную сборку и тесты; блокирующих проблем
исходников не обнаружено. Точная системная причина зависания внутри sandbox
не локализована. Первая конфигурация существующего cache также выдала
нефатальное предупреждение о неиспользованном повторно переданном toolchain;
новая CPU-конфигурация и последующие сборки успешны.

## Точные команды

Из PowerShell в `C:\Julia` (установленные зависимости уже присутствуют):

```powershell
# Именно этим сценарием подтверждено исходное состояние (до правок):
.\tools\build.ps1 -UseInstalledDependencies -KeepDiagnostics

# Сборка изменённого приложения; затем полный набор проверок:
.\tools\build.ps1 -UseInstalledDependencies -SkipTests
# Следующие cmake/ctest-команды выполняются в Developer PowerShell for VS x64:
ctest --preset test-release

# Усиленные окончательные проверки переноса с выводом числа поз и max delta:
ctest --preset test-release -R '^(pose_core|pose_parity)$' -V

# Отдельное дерево CPU-only, без OpenGL-целей:
cmake --preset cpu-release -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build --preset build-cpu-release --parallel 4
ctest --preset test-cpu-release
ctest --preset test-cpu-release -R '^pose_core$' -V
dumpbin /dependents build/cpu-release/tests/julia_test_pose_core.exe
```

Для обычного PowerShell перед прямыми CMake/CTest-командами настройка MSVC
эквивалентна используемой в `tools/build.ps1`:

```powershell
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$juliaVs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
Import-Module (Join-Path $juliaVs 'Common7/Tools/Microsoft.VisualStudio.DevShell.dll')
Enter-VsDevShell -VsInstallPath $juliaVs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
if (-not $env:VCPKG_ROOT) { $env:VCPKG_ROOT = Join-Path $juliaVs 'VC/vcpkg' }
```

`cpu-release` переиспользует `build/vs-release/vcpkg_installed` и отключает
manifest install; зависимости сначала устанавливаются обычной сборкой
`.\tools\build.ps1`. Для отдельной среды CPU-ядру нужна только GLM:
можно конфигурировать без vcpkg-presets с `-DJULIA_BUILD_DESKTOP=OFF` и путём
к пакету GLM через `CMAKE_PREFIX_PATH`. Этот альтернативный способ здесь не
выполнялся.

## Оставшиеся ограничения

FBX loader по-прежнему создаёт GPU-меши/текстуры, поэтому первичная загрузка
FBX всё ещё требует GL. Сам CPU-скелет, клипы и их вычисления этого не требуют.
Eye rig-adapter пока ссылается на `GLModel`; общие поведение/приоритеты/время
тела будут переноситься следующими этапами. Время сейчас остаётся кадровым,
а pause клипа сохраняет прежнее поведение глаз.

Скелет описывает визуальный риг, а не анатомические суставы; IK, суставные
ограничения, опоры, коллизии, динамика, root-motion extraction, crossfade и ИИ
не добавлены. Численная эквивалентность рендера на другом GPU/компиляторе,
Debug-конфигурация и полный ручной аудит всех поз не проверялись. Успех
тестов не является доказательством физической корректности будущего тела.
