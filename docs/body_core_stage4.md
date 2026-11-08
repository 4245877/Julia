# Фиксированное время и состояние экземпляра: этап 4

Дата: 2026-11-08. Фактический каталог: `C:\Julia`.
Прочитаны отчёты этапов 1–3 и действующие исходники. Незакоммиченные изменения
этапа 3 сохранены; сброса рабочего дерева не было.

## Исходная точка и уточнение плана

До правок штатный `tools/build.ps1 -UseInstalledDependencies -KeepDiagnostics`
подтвердил **23/23** теста, отдельная CPU-конфигурация с warnings-as-errors —
**11/11**. `pose_parity`: **17 791** поза, максимальная разница locals/globals/palette
**0**. Обе конфигурации используют `/WX`. Логи: `build/diagnostics/stage4/baseline-*.log`.
Ninja внутри sandbox не начинал сборку; штатные сборки выполнены вне sandbox.

Исследование подтвердило:

- `AnimationPlayer::update(frameDelta)` продвигал клип и EyePoseLayer; pause клипа
  останавливал глаза. Target gaze потреблял pending delta в `EyePoseLayer::apply`.
- `DirectBodyControl` не имел часов: callbacks меняли q, mode и reset немедленно,
  затем вызывали `player.update(0)`. Остальные клавиши прямо меняли player/eyes.
- Desktop передавал `glfwGetTime()` delta в player. При сворачивании обновление
  пропускалось; clip/eyes имели разные собственные правила больших задержек.
- AvatarCamera читала текущие globals player. OrbitCamera имеет отдельное кадровое
  сглаживание пользовательского обзора; оно не является состоянием тела.
- ModelAsset уже const/shared; binding ссылался на asset, control — на binding,
  player — на asset/control/eyes. Эти ссылки требовали правильного внешнего lifetime.

План уточнён до композиции существующих CPU-компонентов в одного владельца,
последовательного scheduler в runtime, очереди с границами тика и отдельного
восстановления отображаемой позы из параметров слоёв. Потоки не добавлены.

## Владение и один тик

`BodyInstance` удерживает `shared_ptr<const ModelAsset>`, владеет binding,
DirectBodyControl, EyePoseLayer, AnimationPlayer, корневой rigid-позой, очередью,
эпохой и номером тика. Внутренние необладающие ссылки замкнуты на этого владельца;
копирование/перемещение экземпляра запрещено. Публичный доступ к компонентам const.
Неизменяемый asset можно разделять; позы, q, глаза и RNG у экземпляров отдельные.

Единственная изменяемая рабочая поза остаётся внутри вложенного player. Копии
в `BodySnapshot` — опубликованные завершённые состояния, а не второй контроллер.
Snapshot хранит tick/time/epoch, root, playback, diagnostic q, выход глаз,
компоненты базового клипа и готовую Pose. `results()` содержит исходы последнего
тика; результат submission возвращается сразу. Для длительного хранения снимок
или результаты нужно скопировать до следующего `step()`.

Порядок `step()`:

1. Сохранить предыдущий завершённый снимок, увеличить tick.
2. Применить команды этого тика по возрастанию ID.
3. Продвинуть клип и независимые часы глаз ровно на `1/120` s.
4. Выполнить clip/bind → diagnostic override → eyes → FK → palette.
5. Опубликовать снимок; при разрыве приравнять предыдущий снимок текущему.

Команда тика N применяется в начале интервала, заканчивающегося N. Поэтому Play
на N даёт первый продвинутый sample в снимке N, Pause на N уже запрещает его
клиповое продвижение. `simulationTime = tick * (1/120)`; накопления body time нет.

q — только диагностические координаты относительно точного импортного bind-zero.
Они сохраняются при показе клипа, но не описывают анатомические углы этого клипа.
Скоростей и исполнения с ограниченной скоростью нет: SetCoordinate мгновенно
меняет q на границе тика. SetRoot — явная телепортация rigid-позы в метрах.
Clip root motion остаётся внутри визуальной позы; его извлечение не добавлено.

## Время и нагрузка

`runtime::FixedStepScheduler` не владеет персонажем: callback последовательно
выполняет `body.step()` (или несколько экземпляров). Начальная частота **120 Гц**
даёт два существующих внутренних eye-subtick по 1/240 s, 4 тика при 30 FPS и
2 при 60 FPS, при небольшой задержке интерполяции. Частота задана одной константой,
на лету не меняется.

| Ситуация | Правило |
| --- | --- |
| Обычный кадр | Накопление времени, максимум 8 тиков за кадр; сохраняется дробный остаток |
| Перегрузка до 0.5 s | Лишние целые шаги времени отбрасываются, backlog не переносится бесконечно; `overloadFrames`, `discardedSeconds` |
| Gap >0.5 s или свёрнутое окно | Тело заморожено, gap и остаток отброшены; `suspendedFrames`, `discardedSeconds`; desktop сбрасывает историю показа |
| Полная пауза | Отдельное состояние scheduler; тики не выполняются, elapsed учитывается в `pausedSeconds`, очередь ждёт возобновления |
| `runTicks(N, callback)` | Ровно N вызовов, без wall clock, бюджета кадра и потерь; при полной паузе явная ошибка |

Номера тиков никогда не перескакивают через отброшенное wall time. Симуляция
при перегрузке отстаёт от реального времени, будущие команды ждут свои тики.
Диагностика доступна через scheduler и заголовок desktop. Пользовательская камера
продолжает обрабатываться кадрово; body delta от неё не зависит.

Pause/Seek/Stop клипа не замораживают и не перематывают RNG/время глаз.
Процедурное поведение развивается и под авторским eye-клипом; видимая авторская
поза имеет прежний приоритет. Обычные clips с собственной gaze-анимацией сохраняют
её; при direct-mode базой глаз становится bind/direct независимо от скрытого клипа.
Отключённый eye-layer не развивается. Старое правило EyeBehavior для delta >0.5 s
сохранено в низкоуровневом API, но BodyInstance никогда не передаёт ему большой шаг.

## Команды и reset

Протокол: `{id, target BodyId, tick, epoch, variant operation}`. ID ненулевой и
уникальный в эпохе; порядок одного тика задаёт ID, а не порядок доставки.
Поддержаны абсолютная/относительная установка q с Reject/Clamp, обнуление q,
mode, root teleport, полный reset, select/play/pause/stop/seek/loop/step/next клипа,
natural/manual/target gaze, blink/wink, eye enable, auto toggles, seed и blink interval.
Углы тела — радианы; gaze-команды сохраняют единицы существующего eye API — градусы;
target gaze задаётся в resource space. Desktop переводит туда точку камеры.

Неверный адресат/эпоха/нулевой или повторный ID и `tick <= completedTick`
отклоняются при submission. Неверный payload отклоняется на целевом тике с reason,
остальные команды продолжаются; пределы дают Applied/Clamped/Rejected. Результат
содержит ID, эпоху, тик обработки и статус. `enqueue` назначает ID и следующий тик.
Для воспроизводимого replay команды должны иметь одинаковые целевые тики;
физическое время прихода клавиши само по себе этого не гарантирует.

ResetBody сбрасывает q/mode, root, selection=0, stop/time=0, loop=true,
eye settings/toggles/overrides/target/RNG с исходным seed, enabled в начальную
настройку экземпляра; отменяет остаток этого тика и всю будущую очередь со
статусом Cancelled. Начинается новая эпоха и пространство ID; старые команды
не принимаются. Tick остаётся монотонным. Состояние после reset проходит обычный
один шаг глаз этого тика; повторный reset даёт тот же будущий путь от этой границы.
История отображения схлопывается. Scheduler clock/полная пауза отдельно не меняются.

## Снимки и desktop

`display(alpha)` — const CPU-операция. Она интерполирует два завершённых снимка:
root translation + quaternion slerp, direct q линейно с повторным применением
калибровки, компоненты анимированных узлов — translation/scale lerp и rotation
slerp, gaze/blink — по выходным параметрам с повторным применением eye bake.
Затем заново вычисляются FK и palette. Матрицы палитры не смешиваются.
Bind/helper/empty-channel матрицы копируются точно, без декомпозиции; direct
адаптер сохраняет исходные affine factors, включая reflection/shear/nonuniform scale.
В endpoints используются точные готовые снимки.

Обычный показ отстаёт от принятого времени на один шаг: время показа
`(N-1+alpha)/120`, `alpha = остаток/step`. При reset, root teleport, смене mode,
select/stop/seek/restart, loop seam и других явных разрывах история схлопывается;
также она схлопывается при полной паузе и suspend. Искусственного пролёта между
несвязанными позами нет. Снимки не являются потокобезопасной очередью: весь путь
пока исполняется последовательно.

Основной desktop-цикл читает один BodyDisplay для renderer, AvatarCamera anchor
и суставного overlay. Никакой render-вызов не продвигает глаза, q или часы.
`Pause` — полная пауза симуляции; `F6` — полный reset на следующем тике;
Space/comma/period управляют только клипом; прежние F2/F3/F4/+/-/Backspace и
eye-клавиши идут через `enqueue`. При полной паузе body-команды ждут следующего
тика. Для точного bind по-прежнему нужно отключить глаза и direct mode.

Старые animation/eye/display verification fixtures изолированно используют
standalone player API. Действительный производственный путь дополнительно
проверяют `desktop_launch` (обычный цикл) и обновлённый `desktop_body`.
Эталон этапа 2 не изменён: его harness явно воспроизводит прежние часы поверх
нового CPU sampling/eye adapter; новый контракт проверяется отдельными тестами.

## Проверки и воспроизведение

Итоговая штатная конфигурация: **24/24**, CPU-only: **12/12**, обе с
`JULIA_WARNINGS_AS_ERRORS=ON`. Логи: `final-build.log`, `final-cpu.log`.

Артефакты и логи находятся в `build/diagnostics/stage4/`:

- `body_instance`: 1200 точных тиков / 10 s, сравнение **каждого** тика при
  30/60/144 FPS и irregular intervals; max locals/globals/palette delta **0**,
  совпадают q/root/playback/eyes/epoch и результаты команд, допуск поз 1e-6.
- Проверены порядок/поздние/ошибочные команды, reset/cancellation/эпохи,
  независимость экземпляров, Pause/Seek, authored eyes/direct, продолжение скрытых
  eye-часов, точный bind, pure display и affine fixture, root/loop discontinuity,
  полная пауза, suspend и bounded overload.
- `desktop/runtime.txt`: 120 кадров, 116 тиков, 4 кадра полной паузы, queue-only
  клавиатура, clip/gaze/target/direct/root/reset, один источник camera/render.
- `gpu/desktop_body.txt`: **63** кадра, queued callbacks/relative increments/clamp,
  интерполированные снимки и повторные настоящие GPU draws; GL errors **0**.
- Headless CLI: **1237** тиков, `10.308333333333334` s, dropped=0;
  `headless.csv` содержит строку на каждый завершённый тик. DLL-зависимости
  CPU EXE: Assimp + Windows/C++ runtime, без OpenGL/GLFW/GLAD.
- Сохранены импортный эталон обоих normal modes, 17 791 поза с delta=0,
  профиль 20 joints/44 DoF, существующие shaders/GPU/camera/display/clip/eye checks.
- Выборочно просмотрены zero, elbow60, shoulder45 side и overlay; обзор
  `visual-review.png`. Это не полный ручной аудит всех движений.

Команды из Developer PowerShell VS x64 в `C:\Julia`:

```powershell
.\tools\build.ps1 -UseInstalledDependencies -KeepDiagnostics
cmake --preset cpu-release -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build --preset build-cpu-release --parallel 4
ctest --preset test-cpu-release

# Новая проверка replay без окна:
.\build\cpu-release\tests\julia_test_body_instance.exe build/diagnostics/stage4/manual
# CPU-приложение, произвольное точное число тиков; --elbow включает direct mode:
.\build\cpu-release\apps\julia_body_sim\julia_body_sim.exe --ticks 1237 --clip 10 --elbow 0.3 --output build/diagnostics/stage4/headless.csv
dumpbin /dependents build/cpu-release/apps/julia_body_sim/julia_body_sim.exe
# Снимки в настоящем приложении:
.\julia_desktop\julia_desktop.exe --verify-body build/diagnostics/stage4/manual-gpu
.\julia_desktop\julia_desktop.exe --window --no-save
```

## Ограничения и следующий этап

Детерминизм подтверждён для текущих asset/MSVC Release и одинаковых tick-addressed
команд, не для произвольных платформ или физического времени ввода. Интерполяция
кинематическая; она не создаёт физического исполнения. Snapshot/display сейчас
копируют CPU-массивы и могут выделять память; ID history растёт до reset. Нет
сериализованного command replay, streaming, потоков, root-motion extraction,
collision, dynamics, trajectory planning или ИИ.

Следующий отдельный этап — **движение кисти к произвольной цели через IK**
с существующими метрическими рамками, суставными ограничениями, проверкой
достижимости и устойчивым выбором локтя. Он здесь не реализован.
