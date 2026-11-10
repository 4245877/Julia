# Управляемый контакт одной руки: этап 7Д / stage7e

10 ноября 2026, `C:\Julia`, Windows x64, MSVC Release `/WX`, MuJoCo **3.3.7**.
Этап завершён. [Карта окончательных доказательств](../build/diagnostics/stage7e/README.md) —
`build/diagnostics/stage7e/`. C++20 исполняет физику и управление;
Python запускает процессы, проверяет CSV и строит необязательные графики.

## Исходная точка и воспроизводимость

Начальный commit `9bef309b42b3722bb5052b8d692c50bad905e020` записан в
`initial-commit.txt`; исходники сохранены в `baseline-source.zip`. Чистота исходного
дерева отмечена предыдущим исполнителем; отдельный `initial-status.txt` отсутствует
и самостоятельным доказательством не служит. Архитектурный контекст — отчёты 7А–7Г,
backend, rig adapters, JointDrive, приложения и scheduler. Исходные три сборки с `/WX`
прошли **59/59 desktop, 26/26 CPU, 22/22 physics OFF**; логи — в `baseline/`.

`tools/stage7d_ctest.py` читает generated `tests/CTestTestfile.cmake`, заменяет
абсолютные назначения `build/diagnostics/` на `<output>/artifacts/` и build
`verification` на `<output>/verification`, затем запускает CTest в копии файла.
EXE, аргументы сценариев и working directory сохраняются. Это перенаправление
путей, не изменение порогов. Скрипт сам **не запрещает повторный output**: для
этого этапа каждый запуск получил отдельное имя. Старые материалы не удалялись.

Критерии записаны до первого нового опыта в `criteria.txt`. В истории описаны
неудачные опыты и исправления: первоначальная проекция только по нормали мешала
поиску; после полной позиционной проекции входной пик 10,856 Н в одном варианте
потребовал замедления поиска с 12 до 8 мм/с. Подтверждение получило предусмотренный
гистерезис 0,6/0,2 Н. Пороги силы, ошибки, проникновения и времени не ослаблялись.
Сохранены также ошибки начальной геометрии fixtures и первый длительный Disable,
дошедший до прежнего ограничения soft-limit 0,04 рад. Позднее добавлен отдельный
опыт с отключённой силовой обратной связью; его критерии выбраны до запуска.

Промежуточный полный набор `final/` прошёл 66/66, 27/27, 22/22 и матрицу
`parity/` 54/54. `publication/`, `publication-parity/` и `measurements-review2/`
также относятся к версии **до заключительного исправления Confirm**. Они сохранены
для истории и не используются как окончательное доказательство новой версии.

Заключительная версия прошла **66/66 desktop, 27/27 CPU, 22/22 physics OFF**:
[desktop](../build/diagnostics/stage7e/accepted/vs-release/ctest.log),
[CPU](../build/diagnostics/stage7e/accepted/cpu-release/ctest.log),
[OFF](../build/diagnostics/stage7e/accepted/cpu-no-physics/ctest.log).
Время наборов — 803,97 / 407,30 / 242,24 с соответственно. При возобновлении работы
desktop и матрица приложений ещё выполнялись; наблюдалось продвижение логов,
дубли не запускались, EXE не пересобирались.

Все **62/62** файла совпали с `accepted-source-sha256.json`. Логи configure/build —
`accepted-build/`; обе physics-сборки используют MuJoCo 3.3.7, все три — Release и `/WX`.
Проверены точные подстановки только выходных путей в финальных CTest definitions.
Read-only Ninja dry run не требует компиляции/линковки: для CPU/OFF `no work to do`,
для desktop остаётся только штатное копирование runtime assets. Текущие хеши и mtime
302 source/tool/EXE/DLL/build файлов записаны и повторно проверены в `accepted-audit/`.
Это фиксация текущих артефактов после сборки, а не ранее отсутствовавший хеш EXE
на момент запуска. В ходе завершающего аудита исходники реализации не менялись;
прежние допуски и эталоны сохранены.

## Область и геометрия

Одна **Full-рука, шесть исходных координат**, закреплённая в рамке плеча.
Общая реализация работает с Left и Right; вторая рука, корпус и ноги остаются
визуальным bind-фоном. Это стенд, не физически стоящий персонаж.
`BodyInstance`, клипы, IK, anchors и hand layers не исполняются совместно с ним.

Новое приложение заменяет **только collider кисти** на сферу `g2`, r=18 мм,
в прежнем distal capsuleEnd. Это контактный proxy существующей кисти, без нового
тела и добавленной массы. `inertiafromgeom=false` сохранён. Его рамка имеет оси
hand joint frame, origin в центре сферы:

| Параметр | Left | Right |
| --- | ---: | ---: |
| Origin в wrist joint, м, примерно | (0; 0,0838320514; 0) | (0; 0,0832694517; 0) |
| Масса кисти, кг | 0,3 | 0,3 |
| COM в segment frame, м | (0; 0,0455608975; 0) | (0; 0,0452551368; 0) |
| Ixx=Izz в COM/segment, кг·м² | 0,000284379538 | 0,000281602741 |
| Iyy в COM/segment, кг·м² | 0,0001536 | 0,0001536 |

Это прежние цилиндрические инженерные инерции кисти; замена proxy на сферу
не превращает физическую кисть в однородный шар. Нанометрические компоненты origin
и все точные рамки, COM, инерции записаны в `contact-setup.txt`, `calibration.txt`
и `model.xml`. Payload 0,5 кг — отдельная прежняя welded-сфера Grip r=40 мм,
со своей массой и I=0,4mr²; это нагрузка, не захват.

Цель — передняя плоская грань неподвижного `wall` box, half-size
(0,12; 0,12; 0,02) м. Выбранная наружная нормаль n=(0;0;−1), подход идёт по +Z.
Стартовый зазор сфера–грань 28 мм, варианты 38 и 23 мм. Box центр вычисляется
**один раз из начальной FK**, независимо от цели силы. Для 3 и 6 Н его глубина
не подбирается. API наблюдений поддерживает axis-aligned грань box или Y-up floor;
управляемая демонстрация проверена для указанной грани −Z.

Две подготовленные q, рад:

```text
(.45, .18, -.10, .65, .12, .08)
(.35, .22,  .08, .55, .05,-.04)
```

Обе стороны проверены отдельно на этих позах и трёх смещениях поверхности.
Сохранены scale **0,4 м/resource unit**, fingerprint `8613155383294238335`, точные
bind-данные и разные калибровки L/R. Проверяются прежние 185 FK-конфигураций на
сторону и начальная позиция нового tool proxy с допуском 30 мкм.
Новый аргумент конструктора PhysicsWorld позволяет выполнить его обычный явный
reset сразу в подготовленную q; старый вызов по умолчанию по-прежнему начинает с нуля.
Это устраняет промежуточную неподходящую bind-zero позу для fixture чужого контакта.

## Архитектура и владение

`ContactControl.hpp/.cpp` — чистый ограниченный регулятор/автомат без MuJoCo,
без записи q/v и без applyWrench. `ContactObservation.cpp` агрегирует наблюдения
и пишет новые CSV. PhysicsWorld получает Jacobian через `mj_jac`, передаёт
завершённое измерение регулятору и является единственным владельцем live q/v.
`ContactBench.hpp` задаёт воспроизводимые условия и явные внешние возмущения;
`julia_contact_physics` и `julia_contact_desktop` используют один сценарный код.

Добавлен `DriveMode::Contact` (значение 4). Все шесть координат имеют одного
владельца. Start требует Disabled и явно объявленную сферическую пару на
закреплённой шестикоординатной модели. Target, SmoothTarget, Torque и configureServo
при владении Contact атомарно отклоняются; сначала нужен Disable.
ForceTarget разрешён только в Ramp/Hold, конечное число в [1;8] Н. Проверенный
рабочий диапазон этого этапа — **3–6 Н**, не весь допустимый входной диапазон.
Capabilities объявляет `normalForceControl`; свободное тело и Two отвергаются.

`JointReference`/SmoothTarget не используется как частый поток контактных целей.
Его прежняя семантика торможения перед новой конечной целью сохранена. Contact
передаёт непрерывно вычисляемый внутренний момент через тот же `evaluateDrive`:

```text
tau_requested = tau_contact + tau_gravity
tau_limited   = clamp(tau_requested, -maxTorque, +maxTorque)
```

Gravity — прежняя scratch RNE при v=0. Все её составляющие входят в общий cap.
Gear=1, motor ctrlrange/forcerange также ограничивают суммарное усилие.
Actual torque и работа учитываются прежним общим путём JointDrive; отдельного
неучтённого силового канала нет. Контакт создают joint motors и solver, без
attachment/equality, телепортации кисти, коррекции q/v или внешнего wrench регулятора.
Сценарные applyWrench — только явно записанные возмущения, в отдельном external work.

## Выбор регулятора

Admittance по нормали потребовал бы интегрировать ограниченную позиционную опору
и решать обратную кинематику/скорость с ограничениями. Здесь доступен motor torque
и точный Jacobian, поэтому выбран Jᵀ force feedback с касательной импедансной
стабилизацией и мягкой постурой. Это ограниченное гибридное управление одной
нормальной силой, не шестимерное wrench control.
Различие прямого и косвенного силового управления и несовместимость независимой
жёсткой позиции и силы вдоль одной нормали описаны в
[MIT Manipulator Control](https://manipulation.csail.mit.edu/force.html).

Пусть v — скорость центра tool, e=Fref−Ffeedback, Pt=I−nnᵀ. В Ramp/Hold:

```text
I_F <- bounded integral(e), |I_F| <= 2 N s
f_n  = -Fref - .35*e - 3*I_F - 35*(n dot v)
f_t  = -100*Pt*(p-p_start) - 12*Pt*v
f    = n*clamp(f_n,-15,15) + f_t
P_J  = I - J^T*(J*J^T + 1e-8 I)^-1*J
tau_contact = J^T*f + P_J*[2*(q_start-q) - .3*v_joint]
```

Здесь нет normal position spring в фазе удержания. Нормальное смещение определяется
физикой и силовой обратной связью. Касательная жёсткость 100 Н/м и постура 2 Н·м/рад
не являются настройкой мягкости contact solver. Проекция по всему позиционному J
не даёт постуре навязать скрытую нормальную позицию. Это кинематическая регуляризованная
проекция, не динамически согласованное whole-body управление; ориентация удерживается
лишь мягким постуральным предпочтением. Все касательные/постуральные воздействия
также ограничены общими joint effort caps.

Интегратор условный: не продвигается при предыдущем насыщении; при насыщении
текущего суммарного момента предложенный шаг интегратора откатывается. При потере
контакта/отказе/Disable интегратор обнуляется. При разгрузке дополнительно затухает
с постоянной 0,1 с. Нормальная цель, её смена и разгрузка ограничены **3 Н/с**.
Emergency Fault и Disable снимают активную силовую задачу сразу; физическая сила
при ударе не обещается ограниченной скоростью изменения.

## Автомат и критерий успешного удержания

| Фаза | Условие/действие |
| --- | --- |
| FreeMotion | Мягкий подход по нормали на 8 мм за 1 с, cubic position/velocity reference |
| Search | Скоростная опора 8 мм/с, normal effort clamp ±3 Н; максимум 60 мм фактического продвижения или 5 с поиска |
| Confirm | Вход при ≥0,6 Н; непрерывные 40 мс при ≥0,6 Н; ниже 0,6 Н таймер обнуляется, при ≤0,2 Н возврат в Search |
| Ramp | От начальной малой опоры 0,6 Н к цели, максимум 3 Н/с |
| Hold | Измеренная сила в пределах ±0,6 Н, |normal tool speed|≤0,01 м/с, penetration≤2 мм и отсутствие насыщения непрерывно 0,5 с |
| Unload | Fref → 0 с 3 Н/с; затем отключение силового интегратора |
| Retreat | Выход на 25 мм за 1,5 с; Complete требует фактического отхода ≥20 мм и силы <0,2 Н |

Первый Hold длится 2,2 с до сценарной смены 3→6 Н; второй — 3,5 с, затем разгрузка.
В Confirm малая inward опора 0,8 Н даёт запас над порогом датчика 0,6 Н;
нахождение в гистерезисной полосе само по себе время подтверждения не накапливает.
Независимый синтетический fixture сначала проверяет сброс таймера при 0,4 Н,
затем требует 39 выборок по 1/960 с для подтверждения (40,625 мс) и 77 нулевых
выборок для потери (80,208 мс); на 38 и 76 выборках перехода ещё нет.
`holdSucceeded` — защёлка выполненной выдержки для текущей цели; смена цели и Fault
её очищают. `good_hold_s` показывает текущую непрерывную выдержку. Флаг не заменяет
силу/скорость/проникновение и не является гарантией отсутствия последующего срыва.
`referenceReached` у Contact остаётся false и не подтверждает прижим.

После подтверждения сила ≤0,2 Н в течение 80 мс означает ContactLost. Любой
положительный solver force на другой паре считается ForeignContact; он не включается
в целевую силу. Непрерывное насыщение любой координаты 0,5 с — Saturation.
Отсутствие хорошей выдержки к 2 с Hold — HoldTimeout. Penetration>2 мм — GeometryLimit.
Fault защёлкивает причину, снимает inward force и интегратор, оставляет ограниченные
gravity/damping и касательную/постуральную стабилизацию. Нет автоматического повторного
прижима. Исходный runtime-envelope PhysicsWorld остаётся дополнительной защитой.

Cancel при контакте ведёт через Unload/Retreat; до подтверждения — прямо в Retreat.
Disable даёт нулевой активный момент на следующем подшаге, оставляя физическую гравитацию,
пассивное damping, constraints и реальные движения. Отрицательный Disable-опыт
смотрит последние 25 тиков после выключения в Hold. Длительное пассивное падение
из такой позы может нарушить прежний soft-limit envelope; оно не исправляется clamp.

Reset очищает live/scratch, ctrl/warmstart, pending wrench, все режимы, фильтр,
очередь задержки, подтверждение/потерю, интегратор, фазы, насыщения, contact work,
slip distance и peaks; epoch растёт. Новый Start после Disable очищает состояние
регулятора/фильтров, но сохраняет накопленные контактные work/slip/peaks до reset.
Прежние drive work тоже сохраняются при смене режима. Desktop reset создаёт
отдельный CSV-каталог и сбрасывает scheduler.

## Время и контактные наблюдения

Внешний тик 120 Гц, номинально **8×1/960 с**. Контур вызывается каждый физический
подшаг. Порядок:

```text
завершённое измерение на t -> фильтр/очередь -> управление следующего подшага
-> общий cap -> один mj_step -> mj_forward на фактическом endpoint q/v
-> endpoint силы, скорости, работа, фазовые условия -> снимок
```

Фильтр: Ff += (1−exp(−h/0,01))*(Fsolver−Ff). Номинально управление использует
последнее завершённое измерение; дополнительная задержка 4 samples добавляет
4h к этому причинному порядку. Поля `sample_feedback_*` показывают силу/время,
реально использованные последним управляющим расчётом; `feedback_*` — значение,
готовое для следующего. Контактный индекс внутри mjData.contact нигде не сохраняется
как идентификатор. Статическая пара разрешается по именам форм в compiled model.

Разделены signed sphere-to-face `geometric_gap_m` и band `near` ±2 мм в границах
выбранной грани, количество solver records, силовых точек active и подтверждение
по силе/времени. Отсутствие записи не подменяет геометрический зазор, а запись
сама по себе не подтверждает прижим. Полная рамка tool (world quaternion), нормаль,
world силы, обе скорости точек и их относительная касательная скорость публикуются.

[mj_contactForce 3.3.7](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-contactforce)
возвращает force:torque в контактных осях. Три строки `contact.frame` сохраняются
как **три столбца** worldFromContact. Нормаль направлена geom1→geom2:

```text
F_on_geom2_world = worldFromContact * (fn,ft1,ft2)
F_on_geom1_world = -F_on_geom2_world
v_point = v_COM_world + omega_world cross (p_contact - COM_world)
```

То же преобразование применяется к контактному моменту. Нормальная и касательная
работа в выбранном condim=3 моментном вкладе не нуждаются: torque компоненты нулевые.
Нормальную силу, world resultant и касательную силу суммируют **только для желаемой
неупорядоченной пары**, по всем её точкам. Перестановка выбранных сторон меняет
знак сил, а не работу пары. Для Fn<0,6 Н отношение Ft/Fn публикуется `NA`.

## Solver, трение и работа

Номинальные настройки Full сохранены: implicitfast, Newton50, tolerance1e−10,
elliptic cone, contact solref=(0,006 с;1), solimp=(0,9;0,95;0,001;0,5;2),
joint damping0,02 и solreflimit0,005 с. Новая явная pair имеет condim=3,
friction=(μ;μ;0,005;0,0001;0,0001), μ=0,6 или 0,05. Global override выключен.

Для динамического смешивания MuJoCo учитывает priority/solmix; при равном priority
friction берётся покомпонентным максимумом. Поэтому уменьшение коэффициента одной
формы не гарантирует низкое трение пары. Здесь параметры заданы явной pair и
проверяются через `contactMaterial()` из compiled model и фактические contact.friction.
Правила и параметры сверены с
[MuJoCo modeling 3.3.7](https://mujoco.readthedocs.io/en/3.3.7/modeling.html#solver-parameters).

На каждом завершённом подшаге для каждой выбранной точки один раз считаются
Pn=Fn_on2·(v2−v1), Pt=Ft_on2·(v2−v1). Силы и скорости относятся к одному endpoint,
учитывают вращение и движение обеих сторон; неподвижная сторона даёт нулевую скорость.
Wn/Wt интегрируются правыми прямоугольниками h*P. Это first-order диагностическая
квадратура solver estimate, не точный импульсный баланс. Work включает обе стороны
пары один раз; в этом стенде равна работе на движущемся инструменте.

Прежние active delivered/absorbed, passive damping и external wrench work сохраняются
отдельно. `constraintPower` — мощность **всех** generalized constraints, не контактная
сила. Только в независимом fixture без иных ограничений она сопоставляется с
контактной мощностью. Энергетический остаток не приписывается целиком трению:
мягкий solver, soft limits, дискретизация и выбранная пара не дают полного баланса.
В переходе около stiction встречаются положительные endpoint Pt порядка микроватт
при скоростях около 0,03 мм/с; они сохраняются в сырых данных. Проверка знака
мощности применяется к явному скольжению ≥0,01 м/с, а весь net tangent work
выбранных опытов неположителен. Эта запись не объявляется точной теплотой трения.

Независимые fixtures: свободный box 2 кг на floor даёт четыре точки и world force
(−1;19,62;−2) Н при известной внешней (1;0;2) Н; обратная выбранная сторона даёт
противоположную силу. Проверены все оси базиса, несколько точек, нулевой power без
касательной нагрузки, omega×r и обобщённая virtual power. Ошибка мощности/скорости
в скользящем sphere fixture — порядка 1e−15 Вт / 1e−15 м/с. Тангенциальная работа
при 8/16/32 substeps −0,1448755/−0,1480495/−0,1492610 Дж, последовательные разности
уменьшаются. Отдельный offset-COM fixture проверил 10 контактных точек с максимальной
ошибкой скорости 1,762e−16 м/с. Заключительные fixtures —
`accepted-fixtures/contact-control.txt`, `failures=0`; они также повторены полными
CPU/desktop-наборами. Soft-friction creep при малой нагрузке измеряется, не скрывается.

## Измеренные результаты

Окончательные таблицы —
[summary.csv](../build/diagnostics/stage7e/measurements-accepted/summary.csv),
[telemetry-audit.txt](../build/diagnostics/stage7e/measurements-accepted/telemetry-audit.txt) и
[legacy-comparison.txt](../build/diagnostics/stage7e/measurements-accepted/legacy-comparison.txt).
Сырые 40 вариантов — `accepted/cpu-release/artifacts/stage7e/cpu/`.
В каждом nominal cycle измеряются обе цели; hold RMS — endpoint выборки после
1,5 с соответствующего Hold, условие выдержки проверяется на всех 960 Гц samples.
Дополнительный аудит каждой цели отдельно — `accepted-audit/independent-1/hold-by-goal.csv`:
20 полных циклов × цели 3/6 Н, в том числе обе позы, смещения, payload и sensitivity.
Точный текст всех детерминированных полей **80 прежних физических CSV** совпал с
сохранённым baseline. Аудит 60 CSV-наборов (40 CPU + 20 настоящих CLI) проверил
причинное время обратной связи, ramp, общий cap момента, мощность, работу и ratio/NA.
Для 26 финальных CLI/GPU-прогонов независимо восстановлены **20328 контактных точек**:
базисы, world силы обеих сторон, выбор пары, нормальная/касательная мощность,
скольжение, penetration и геометрический зазор согласованы с aggregate CSV.

| Опыт | Left | Right |
| --- | ---: | ---: |
| Nominal p0 RMS, Н | 0,000274 | 0,000277 |
| Nominal p0 peak за цикл, Н | 6,0292 | 6,0292 |
| Nominal p0 peak penetration, мм | 0,04124 | 0,04035 |
| Две позы × три положения поверхности | все циклы Complete, обе цели | все циклы Complete, обе цели |
| Payload0,5 кг peak, Н | 7,0489 | 7,0098 |
| +1 Н normal load: RMS feedback on/off, Н | 0,07414 / 0,97213 | 0,07998 / 1,05612 |
| μ0,6 / 0,5 Н tangent: peak slip, мм/с | 0,493 | 0,707 |
| μ0,05 / 4 Н tangent: peak slip, м/с | 0,1878 | 0,1794 |
| μ0,05: max X displacement, мм | 40,15 | 38,12 |
| μ0,05: tangent contact work, Дж | −0,03589 | −0,03452 |

Малая идеальная статическая ошибка не обещает такой точности реального датчика
или неизвестной модели. Причинный опыт +1 Н передаёт регулятору только solver force,
не величину возмущения. При одинаковой геометрии и остальных параметрах отключение
только force P/I оставляет около 1 Н ошибки; обратная связь уменьшает её более чем
на 92%. Это свидетельство регулирования силы, а не подобранной глубины проникновения.

При большой касательной нагрузке сила временно отклоняется: в `left-slip` пик
10,0193 Н, в `right-slip` максимальная ошибка Hold 3 Н; есть настоящие скольжения
и краткие размыкания. Это отрицательный/возмущённый опыт, не обещание
сохранить nominal force envelope при произвольном воздействии. Отдельный −Z12 Н
pull вызывает ContactLost через выдержку 80 мс. При отсутствующей стенке поиск
завершается за заданное время с продвижением около 39 мм; чужой контакт и 0,35 Н·м
приводы получают свои причины отказа без ложного успеха.

Проверены cancel, Disable, reset/replay с точным совпадением детерминированных
записей, атомарные неверные команды и отказ free-body/Two. На L/R прошли варианты
16 substeps, solref0,012 с и дополнительная задержка4 samples с теми же nominal
порогами. Мягкость0,012 с увеличивает penetration примерно до 0,16 мм, оставаясь
в пределах выбранных 2 мм. Номинальные настройки и шаг прежних сценариев сохранены.

## Публикация, desktop и стоимость

Форматы и прежний смысл `state.csv`, `joints.csv`, `contacts.csv`, `drive-v1.csv`
не изменены. Новые `contact-v1.csv` (одна строка на завершённый тик) и
`contact-points-v1.csv` (фактические точки данного endpoint) имеют schema=1.
В режиме Contact q_ref в drive-v1 — постуральная исходная q, не жёсткая normal
позиция; прямой controller effort равен requested−gravity FF. Старое поле
gravity_compensation остаётся настройкой **Servo**, а Contact использует свою
обязательную fixed-base gravity compensation. Суммировать повторяющиеся global
work поля drive-v1 по суставам нельзя.

Desktop читает тот же согласованный RenderSnapshot, что CSV: actual skin,
инструмент cyan, wall orange, normal yellow, actual force red, цель magenta,
slip green. HUD показывает фазу/отказ, goal/reference/solver/filter force,
геометрический зазор, penetration, трение, подтверждение, насыщение, моменты и work.
Повторный draw одного снимка проверяется по pixels и неизменности физики/контроля.
Сохранены общие и крупные контрольные кадры в `accepted-parity/<side>-<case>-<fps>/`
и `accepted/vs-release/artifacts/stage7e/gpu/<side>-<fps>/`. При завершающем аудите
просмотрены четыре финальных кадра nominal/slip L/R и графики из
`figures-accepted-agg/`; PNG-копии кадров без изменения содержимого — `accepted-audit/visuals/`.

`accepted-parity/` сравнивает настоящие CPU CLI и GLFW EXE для обеих сторон,
10 сценариев ×30/60/144 Гц: **60/60**,
[comparison.txt](../build/diagnostics/stage7e/accepted-parity/comparison.txt).
Исполнены 20 CLI и 60 desktop-процессов. Все старые детерминированные CSV
поля и обе новые записи сравниваются текстово, timings исключены. Заданные интервалы
проверяют scheduler/draw count, не фактический FPS. GPU errors и internal callbacks
проверяются отдельно: во всех 60 отчётах `GL_errors=0`, `callback_checks=PASS`,
1920 тиков и 480/960/2304 draw соответственно. Наблюдаемый GPU — NVIDIA GeForce
RTX 4070 SUPER, OpenGL 3.3. Callbacks включают pause/reset/view, Start/Disable,
изменение force goal и Cancel до/после контакта.

OS input **не подтверждён**: Computer Use launch завершился `app approval timed out`,
после обновления targetable window отсутствовало; клавиши/мышь не отправлялись.
Подробности — `native-input.txt`. Успешные OpenGL и callback tests это не заменяют.

Производительность — 10000 последовательных тиков на вариант, окончательные raw CSV —
`accepted/cpu-release/artifacts/stage7e/cpu/performance-{0,1,2}.csv`,
[сводка](../build/diagnostics/stage7e/accepted-audit/independent-1/performance.csv).
Измеряется весь world.step отдельно от import,
CSV IO, pose publication и GPU. `contact_control_us` — Jacobian/controller без
gravity scratch; `contact_us` — frames/point velocities, pair aggregation,
filter/history/work/status copy. Эти времена входят соответственно в control/read,
повторно складывать их с total нельзя. Вариант active contact меняет цели 3/6 Н
в течение всего длительного опыта. Сравнительные Servo варианты держат свободную
позу; разность total включает также работу solver и не является чистым overhead.

Каждая ячейка ниже — **mean / p95 / max**, мкс; источник — финальный CPU CTest.

| Вариант | world.step | control (включая gravity) | contact observations | Jacobian/controller |
| --- | ---: | ---: | ---: | ---: |
| Servo, без выбранной пары | 68,761 / 104,5 / 1293,9 | 20,034 / 24,2 / 834,9 | 0 / 0 / 0 | 0 / 0 / 0 |
| Servo, наблюдение пары | 70,795 / 117,5 / 1174,3 | 19,903 / 22,8 / 731,8 | 1,462 / 1,9 / 625,3 | 0 / 0 / 0 |
| Contact, активный прижим | 119,929 / 230,6 / 2666,0 | 25,357 / 31,5 / 1513,8 | 3,121 / 3,8 / 654,8 | 3,062 / 3,6 / 923,4 |

Все три p95 world.step ниже заранее заданных **8333 мкс**. Суммарные 30000 строк
проверены на последовательность тиков 1..10000 и конечные неотрицательные timings.

P95 — sorted[floor(0,95N)]. Фоновая нагрузка и планировщик Windows не фиксированы;
hard realtime и точный причинный выигрыш от разности отдельных прогонов не заявляются.

## Команды воспроизведения

Из `C:\Julia`; прямые CMake/CTest — в VS x64 Developer PowerShell. Каждый output
выбирается новым, чтобы сохранить опубликованные материалы.

```powershell
.\tools\build.ps1 -UseInstalledDependencies -KeepDiagnostics -SkipTests
cmake --preset cpu-release -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build --preset build-cpu-release --parallel 4
cmake --preset cpu-release -B build/cpu-no-physics -DJULIA_BUILD_PHYSICS=OFF -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build build/cpu-no-physics --parallel 4
# Эквивалент всех трёх configure/build с раздельными логами:
.\tools\stage7e_build.ps1 -Label my-build

python tools/stage7d_ctest.py --build build/vs-release --output build/diagnostics/stage7e/my-desktop
python tools/stage7d_ctest.py --build build/cpu-release --output build/diagnostics/stage7e/my-cpu
python tools/stage7d_ctest.py --build build/cpu-no-physics --output build/diagnostics/stage7e/my-off

.\build\cpu-release\apps\julia_arm_physics\julia_contact_physics.exe --side left --case cycle --output build/diagnostics/stage7e/my-left
.\build\cpu-release\apps\julia_arm_physics\julia_contact_physics.exe --side right --pose 1 --offset 0.01 --case cycle --output build/diagnostics/stage7e/my-right
.\build\cpu-release\apps\julia_arm_physics\julia_contact_physics.exe --side right --case slip --output build/diagnostics/stage7e/my-slip
.\build\cpu-release\apps\julia_arm_physics\julia_contact_physics.exe --case normal-disturbance --feedback off --output build/diagnostics/stage7e/my-no-force-feedback
.\build\vs-release\apps\julia_arm_physics\julia_contact_desktop.exe --side left --case cycle --output build/diagnostics/stage7e/my-window
.\build\vs-release\apps\julia_arm_physics\julia_contact_desktop.exe --side right --case slip --fps 144 --verify build/diagnostics/stage7e/my-gpu
.\build\cpu-release\tests\julia_test_contact_control.exe build/diagnostics/stage7e/my-contact-tests
.\build\cpu-release\tests\julia_test_contact_control.exe build/diagnostics/stage7e/my-fixtures --fixtures-only
.\build\cpu-release\tests\julia_test_contact_control.exe build/diagnostics/stage7e/my-performance --performance-only
python tools/verify_stage7e.py --output build/diagnostics/stage7e/my-parity

# Аудит уже завершённых наборов; my-measurements должен быть новым каталогом:
python tools/analyze_stage7e.py --cpu build/diagnostics/stage7e/accepted/cpu-release/artifacts/stage7e/cpu --baseline build/diagnostics/stage7e/baseline/cpu-release/artifacts --current build/diagnostics/stage7e/accepted/cpu-release/artifacts --applications build/diagnostics/stage7e/accepted-parity --output build/diagnostics/stage7e/my-measurements
$env:MPLBACKEND='Agg'
python tools/plot_stage7e.py --cpu build/diagnostics/stage7e/accepted/cpu-release/artifacts/stage7e/cpu --output build/diagnostics/stage7e/my-figures
```

Сценарии: `cycle`, `normal-disturbance`, `small-tangent`, `slip`, `lost`, `absent`,
`foreign`, `weak`, `cancel`, `disable`. Все по умолчанию 1920 тиков /16 с.
Аргументы: `--side left|right`, `--pose 0|1`, `--payload 0..1`, `--offset -0.01..0.02`,
`--friction 0.01..1` (slip явно использует0,05), `--softness 0.004..0.02`,
`--substeps 8|16|32`, `--delay 0..32`, `--feedback on|off`, `--fps 30|60|144`.
`--feedback off` — явно отрицательное сравнение, не силовой регулятор.
Без --verify/--ticks desktop работает до закрытия. Space pause, C Cancel, D Disable,
S Start, R reset, V close view, Up/Down меняют goal на ±1 Н в Ramp/Hold, Esc закрывает.

## Ограничения и отдельные будущие задачи

Доказана ограниченная нормальная задача на закреплённой руке с инженерной моделью,
подготовленными начальными позами, известной неподвижной гранью и solver force estimate.
Нет универсального планирования, идентификации неизвестной нормали, датчика с шумом,
6D wrench/orientation control, движущейся поверхности, физического хвата/пальцев,
мышц, перехода владения с BodyInstance или гарантии устойчивости вне проверенного набора.
Condim3 не удерживает torsional/rolling contact moments; низкое трение и потеря
контакта сохраняются. Мягкий solver и податливость регулятора — разные механизмы.

Для исследования опор теперь готовы именованные пары и их реальные world силы,
нормальные/касательные компоненты, базисы, скорости точек, slip, penetration,
подтверждение/потеря, saturation, фактически ограниченные моменты и раздельная работа.
Это позволяет строить и независимо проверять будущие опорные наблюдатели.

Полное тело по-прежнему имеет 20 сегментов, 41 hinge и свободный таз,
nq=48/nv=47/nu=41; root без приводов и скрытой поддержки. Для баланса потребуется
отдельное управление COM и импульсом, допустимое распределение реакций нескольких
опор с cones/CoP/моментными ограничениями, whole-body inverse dynamics/QP с пределами
приводов, смена контактов, оценка устойчивости/падения и проверки возмущений на
свободном основании. Контроллер закреплённой руки этого не решает и на free body
не включается. Ходьба и физический хват остаются отдельными задачами.
Следующие этапы не начинались. Проверка ввода через ОС, работа с неизвестной или
движущейся поверхностью и расширение силового управления требуют отдельных задач
и собственных критериев; к завершению данного стенда они не приписываются.
