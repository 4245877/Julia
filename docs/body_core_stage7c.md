# Сочленённое физическое тело со свободным основанием: этап 7В

10 ноября 2026, `C:\Julia`, Windows x64, MSVC Release `/WX`, MuJoCo **3.3.7**.
Рабочее дерево перед работой было чистым. Сбросов Git и удаления пользовательских
изменений не было. Проверены действующие отчёты, профиль, CPU FK, affine adapters,
`BodyInstance`, scheduler, backend, стенды рук и OpenGL renderer. Независимая
исходная проверка: **49/49 основных, 24/24 CPU**. Логи и материалы этапа лежат
в `build/diagnostics/stage7c/`; исходные логи — `baseline-{desktop,cpu}.log`.

## Результат и владение

`julia_physics` теперь компилирует одно ветвящееся дерево: **20 массивных тел,
свободный таз и 41 hinge**. Мир и solver остались общими со стендами 7А/7Б.
`Link::parent` задаёт индекс ранее объявленного родителя; `-1` означает world,
`-2` сохраняет прежний последовательный формат. MJCF строится рекурсивно.
Адреса qpos, qvel и actuators разрешаются по именам после компиляции, поэтому
порядок обхода MuJoCo может отличаться от порядка CPU-описания.

`BodyRig` в `julia_physics_rig` строит дерево по реальному ancestry controlled
nodes ресурса, использует существующие калибровки и возвращает измеренную CPU
позу. `julia_body_physics` и `julia_body_physics_desktop` — отдельные исполнители
эксперимента. Кинематический `BodyInstance`, IK, support anchors, hand layers
и проигрыватель клипов в них не запускаются.

Единственный писатель live q/v — `PhysicsWorld`. Команды меняют target/effort,
затем ограниченные моменты передаются solver; skin получает фактические q.
Live q/v записываются только при явном reset. Root не имеет приводов, damping,
equality constraints, опоры на world или исправления после шага. Wrench —
явный экспериментальный вход, удерживаемый один внешний тик.

## Соответствие профилю

Индексы в следующей таблице — индексы скалярных физических координат, **не**
индексы qpos/qvel. Порядок каждой группы — исходный `R0*R1*R2` / `R0*R1`.

| Body | Сегмент / joint профиля | Parent | Физические координаты |
| ---: | --- | ---: | --- |
| 0 | Pelvis | world | free: 3 переноса + 3 вращения |
| 1 | Lumbar | 0 | 0–2: flexion, left_bend, left_turn |
| 2 | Thorax | 1 | 3–5: flexion, left_bend, left_turn |
| 3 | Chest | 2 | 6–8: flexion, left_bend, left_turn |
| 4 | Neck | 3 | 9–11: flexion, left_bend, left_turn |
| 5 | Head | 4 | 12–14: flexion, left_bend, left_turn |
| 6 | LeftClavicle | 3 | 15: elevation |
| 7 | LeftUpperArm / LeftShoulder | 6 | 16–18: flexion, abduction, axial_roll |
| 8 | LeftForearm / LeftElbow | 7 | 19: flexion |
| 9 | LeftHand / LeftWrist | 8 | 20–21: palm_flexion, radial_deviation |
| 10 | RightClavicle | 3 | 22: elevation |
| 11 | RightUpperArm / RightShoulder | 10 | 23–25: flexion, abduction, axial_roll |
| 12 | RightForearm / RightElbow | 11 | 26: flexion |
| 13 | RightHand / RightWrist | 12 | 27–28: palm_flexion, radial_deviation |
| 14 | LeftThigh / LeftHip | 0 | 29–31: flexion, abduction, axial_roll |
| 15 | LeftShank / LeftKnee | 14 | 32: flexion |
| 16 | LeftFoot / LeftAnkle | 15 | 33–34: dorsiflexion, inversion |
| 17 | RightThigh / RightHip | 0 | 35–37: flexion, abduction, axial_roll |
| 18 | RightShank / RightKnee | 17 | 38: flexion |
| 19 | RightFoot / RightAnkle | 18 | 39–40: dorsiflexion, inversion |

Три прежних угла Pelvis были вращением персонажа относительно кинематического
body frame. Здесь их заменяет свободная ориентация таза; дополнительный набор
hinges на free body был бы избыточным. Поэтому measured Pelvis coordinates
визуального адаптера равны нулю, а его мировое движение задаёт free root.
Модель не ограничивает мировую ориентацию прежними кинематическими пределами таза.

Остальные 41 координата и их пределы взяты из профиля без переопределения.
Для корпуса оси +X / −Z / +Y; Lumbar/Thorax/Chest ±15/±10/±15°, Neck
±15/±15/±25°, Head ±20/±15/±35°. Ключицы: elevation вокруг +Z слева / −Z
справа, −10…15°. Руки сохраняют шесть координат, оси, порядок и пределы 7Б;
дополнительной пронации предплечья нет.

Hip: flexion вокруг −X, −15…50°; abduction вокруг +Z слева / −Z справа,
−10…30°; roll вдоль собственного bind-вектора hip→knee, ±20°.
Knee: +X собственной откалиброванной рамки, −5…80°. Ankle: dorsiflexion
вокруг −X, −20…25°, inversion вокруг −Z слева / +Z справа, ±10°.
Асимметричные bind-позиции обеих ног сохранены; ноги не выпрямляются для старта.

Toes и пальцы жёстко наследуют соответствующие foot/hand transforms при bind
своих локальных узлов; их масса включена в конечный сегмент. Wrist/palm/grip/sole —
рамки эффекторов, без отдельной массы или сустава. Neck, head и clavicles имеют
собственную массу, а не повторную массу корпуса. Skin/helper/eye/hair/clothing
nodes остаются визуальными потомками. Существующие arm/thigh twist-followers
с долей roll 0,5 и elbow/knee followers сохранены как skin-деформация;
это не дополнительные физические сегменты. Ничего не закреплено в мировых
координатах для устойчивого изображения.

## Free root, единицы и рамки

Контракт сверен именно с [MuJoCo 3.3.7: floating objects](https://mujoco.readthedocs.io/en/3.3.7/overview.html#floating-objects),
[freejoint](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#body-freejoint)
и [jnt_qposadr / jnt_dofadr](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APItypes.html#mjmodel).
`alignfree=false`, freejoint `align=false` сохраняют выбранный origin и оси.

| Данные | Контракт |
| --- | --- |
| root qpos | 7: world position XYZ в м; quaternion WXYZ, body→world |
| root qvel | 6: linear XYZ в world, м/с; angular XYZ в local root, рад/с |
| RootState | `position`, `glm::dquat(w,x,y,z)`, `linearVelocityWorld`, `angularVelocityLocal` |
| Quaternion conversion | явное чтение/присваивание компонентов; память GLM не копируется |
| Текущая compiled модель | nq=48, nv=47, nu=41 |
| Текущий layout | root qpos[0:7], qvel[0:6]; hinge k: qpos=7+k, qvel=6+k, actuator=k |

Последняя строка — измеренный layout текущего дерева, а не предположение backend.
Отдельный искусственный разветвлённый тест объявляет тела не в DFS-порядке и
проверяет адреса и усилия при переставленных compiled координатах.

Сохраняются **0,4 м/resource unit**, `bodyFromResource`, fingerprint
`8613155383294238335`, Y-up, column vectors и точные affine bind-данные.
Импортный `.01` не применяется повторно. Из float proper-калибровок создаются
нормированные double rigid-рамки физики; исходные матрицы skin не пересобираются.

```text
worldFromBody = worldFromPhysicalPelvis * inverse(bodyFromPelvisBind)
worldFromResource = worldFromBody * bodyFromResource
```

Палитра остаётся в resource frame. Модельная матрица применяется один раз при
рисовании; камера не меняет состояние. До интеграции проверяются joint/segment
frames всех 20 тел и все восемь эффекторов: bind, одиночные нижние/верхние
значения 41 координаты и 47 совместных конфигураций, всего **130**. Половина
конфигураций использует перенос (1,2; 0,4; −0,7) м и поворот 0,73 рад вокруг
нормированного (0,2; 0,8; −0,4). Допуски 30 мкм / 3e−5; получено
**0,674 мкм / 7,915e−7**. Scratch FK и mass audit не меняют live состояние.

## Массы, COM и инерции

Выбраны инженерные **55 кг**, без претензии на подтверждённую биомеханику.
32 кг приходятся на таз/корпус/шею/голову, 0,6 кг на ключицы, 5,6 кг на руки,
10 кг на бёдра, 5,2 кг на голени, 1,6 кг на стопы. Это явная распределённая
модель для проверки динамики; одежда и волосы отдельной массы не добавляют.

Для всех capsule-сегментов масса моделируется цилиндром длины полного
откалиброванного сегмента L, радиуса r, с COM в середине. Segment Y направлен
к segmentEnd. Главные моменты: поперечные `m*(3*r²+L²)/12`, продольный
`m*r²/2`. Collision capsule занимает 8–92% сегмента; масса не выводится из
её объёма. У головы сфера r=0,08 м, COM (0;0,09;0) м в head frame,
`I=0,4*m*r²`. У стоп — box с half-size (0,045;0,025;0,085) м, его COM
в центре, `Ixx=m*(hy²+hz²)/3` и циклически.

Полные параметры всех 20 сегментов — `cpu/drop/model.xml` и
`cpu/drop/calibration.txt`: compiled массы, COM в joint frame, девять
элементов тензора в joint frame, оси, пределы, PD и compiled адреса.
`numerical-details.txt` дополнительно содержит все principal inertia и
положения proxy-форм. Таблица округлённых параметров приведена ниже.

| Segment | Масса, кг | Геометрия/расчётная L, м | COM joint XYZ, м | Principal I, кг·м² |
| --- | ---: | --- | --- | --- |
| Pelvis | 8 | r=0.07; L=0.132230 | -0.000000 / 0.066104 / -0.001207 | 0.02145646 / 0.01960000 / 0.02145646 |
| Lumbar | 4 | r=0.06; L=0.132228 | 0.000000 / 0.066103 / -0.001206 | 0.00942809 / 0.00720000 / 0.00942809 |
| Thorax | 7 | r=0.065; L=0.132228 | 0.000000 / 0.066103 / -0.001206 | 0.01759289 / 0.01478750 / 0.01759289 |
| Chest | 8 | r=0.065; L=0.132230 | -0.000000 / 0.066104 / -0.001206 | 0.02010643 / 0.01690000 / 0.02010643 |
| Neck | 1 | r=0.025; L=0.058178 | -0.000000 / 0.029029 / 0.001869 | 0.00043830 / 0.00031250 / 0.00043830 |
| Head | 4 | sphere r=0.08 | 0.000000 / 0.090000 / 0.000000 | 0.01024000 / 0.01024000 / 0.01024000 |
| LeftClavicle | 0.3 | r=0.022; L=0.084195 | 0.040542 / -0.000000 / -0.011339 | 0.00021352 / 0.00007260 / 0.00021352 |
| LeftUpperArm | 1.6 | r=0.032; L=0.264514 | 0.021623 / -0.128307 / 0.023699 | 0.00973859 / 0.00081920 / 0.00973859 |
| LeftForearm | 0.9 | r=0.027; L=0.245795 | 0.000000 / 0.122898 / -0.000000 | 0.00469518 / 0.00032805 / 0.00469518 |
| LeftHand | 0.3 | r=0.032; L=0.091122 | 0.000000 / 0.045561 / 0.000000 | 0.00028438 / 0.00015360 / 0.00028438 |
| RightClavicle | 0.3 | r=0.022; L=0.084195 | -0.040542 / -0.000000 / -0.011339 | 0.00021352 / 0.00007260 / 0.00021352 |
| RightUpperArm | 1.6 | r=0.032; L=0.253742 | -0.045160 / -0.117092 / -0.018610 | 0.00899425 / 0.00081920 / 0.00899425 |
| RightForearm | 0.9 | r=0.027; L=0.245394 | 0.000000 / 0.122697 / 0.000000 | 0.00468040 / 0.00032805 / 0.00468040 |
| RightHand | 0.3 | r=0.032; L=0.090510 | -0.000000 / 0.045255 / -0.000000 | 0.00028160 / 0.00015360 / 0.00028160 |
| LeftThigh | 5 | r=0.045; L=0.388613 | 0.018800 / -0.192148 / -0.021922 | 0.06545629 / 0.00506250 / 0.06545629 |
| LeftShank | 2.6 | r=0.037; L=0.383025 | -0.000000 / 0.191512 / 0.000000 | 0.03267657 / 0.00177970 / 0.03267657 |
| LeftFoot | 0.8 | box half 0,045 / 0,025 / 0,085 | 0.003709 / -0.110198 / 0.032878 | 0.00209333 / 0.00246667 / 0.00070667 |
| RightThigh | 5 | r=0.045; L=0.387963 | -0.007055 / -0.190255 / 0.037176 | 0.06524597 / 0.00506250 / 0.06524597 |
| RightShank | 2.6 | r=0.037; L=0.380473 | -0.000000 / 0.190237 / -0.000000 | 0.03225447 / 0.00177970 / 0.03225447 |
| RightFoot | 0.8 | box half 0,045 / 0,025 / 0,085 | -0.008230 / -0.075834 / 0.081398 | 0.00209333 / 0.00246667 / 0.00070667 |

`jointFromSegment` переносит COM и вращает principal tensor в joint frame;
MuJoCo получает явные `<inertial pos quat mass diaginertia>`,
`inertiafromgeom=false`, `fusestatic=false`. Отдельной массы от geoms или
добавочных массивных pivot bodies нет.

Аудит читает compiled model, независимо восстанавливает COM, тензоры и
общий тензор с теоремой переноса осей. В четырёх конфигурациях независимо
собрана матрица `M=sum(m*Jvᵀ*Jv + Jwᵀ*I_world*Jw)` по центральным разностям
CPU-дерева, включая мировые переносы и локальные вращения free root.
Максимальная ошибка M — **9,54e−10** при допуске 2e−6; все Cholesky pivots
>2,98e−4. Масса проверена с допуском 1e−10 кг, COM/тензоры — 1e−8.
Для отдельной 2-кг сферы независимо проверены диагональная free mass matrix,
поступательное движение, вращение при неединичной начальной ориентации и
изменение импульса от известной мировой силы.

## Контакты и геометрические упрощения

В мире есть неподвижная Y-up plane на y=0. У тела 17 капсул, сфера головы и
два foot boxes. Box стопы ориентирован по существующему Sole; его центр —
`sole * Translate(0,.025,-.035)`, нижняя грань лежит в plane существующего Sole.
Левая/правая Sole не принудительно выравниваются. Гарантия относится к этим
proxy, не ко всей skin-оболочке, одежде, волосам или ушам.

Все 20 сегментов могут контактировать с полом. Все ненепосредственно соседние
сегменты могут контактировать друг с другом, включая разные конечности,
руку/корпус, ногу/корпус, голову/конечности и несоседние звенья одной ветви.
Masks тела: contype=1, conaffinity=3; пола: 2/1. Дополнительных `<exclude>`
у полного тела **нет**. Сохранён стандартный parent/weld filter MuJoCo:
19 непосредственных parent-child пар исключены, поскольку proxy сочленений
примыкают/перекрываются в pivot. Это все пары, указанные столбцом Parent
таблицы дерева. Статический world-parent к этому исключению не относится.
Правила сверены с [collision filtering 3.3.7](https://mujoco.readthedocs.io/en/3.3.7/computation/index.html#selection).

Таким образом, потенциально доступны 171 несоседняя self-пара и 20 floor-пар;
broadphase дополнительно отсеивает удалённые формы. Искусственный тест siblings
показывает положительную силу при включённом selfCollision и отсутствие
контакта при выключенном. Полное тело имеет фактические самоконтакты во всех
четырёх floor-сценариях; это не только проверка masks.

`condim=3`, elliptic cone, friction=(0,6;0,005;0,0001), contact solref=(0,01 с;1),
solimp=(0,9;0,95;0,001 м;0,5;2), defaults SDK 3.3.7. При condim=3 используются две компоненты
скользящего трения; torsional/rolling компоненты не добавляют contact DOF.
Начальная поза floor-сценариев не имеет проникновений proxy; Drop/Passive/Pulse
начинаются с worldFromBody translation y=0,25 м. Tilt — y=0,5 м и roll=0,65 рад.
Допуск динамического проникновения заранее выбран 20 мм. Solver определяет
силы и перемещения; anchors, IK и геометрическое отклонение target не участвуют.

## Приводы, время и диагностика

`tau=kp*(target-q)-kd*v`, saturation до ctrl и независимые MuJoCo ctrl/force
limits, gear=1. Компенсации гравитации, интегратора и root stabilization нет.

| Группа | kp, Н·м/рад | kd, Н·м·с/рад | max, Н·м |
| --- | ---: | ---: | ---: |
| Lumbar/Thorax/Chest | 50 | 2 | 20 |
| Neck/Head | 12 | 0,5 | 5 |
| Clavicle | 20 | 0,5 | 5 |
| Shoulder flexion / abduction / roll | 80 / 60 / 12 | 8 / 6 / 0,4 | 12 / 12 / 3 |
| Elbow | 40 | 4 | 8 |
| Wrist flexion / deviation | 8 / 6 | 0,25 / 0,2 | 2 / 1,5 |
| Hip flexion / abduction | 100 | 5 | 40 |
| Hip roll | 25 | 1 | 12 |
| Knee | 80 | 4 | 30 |
| Ankle, обе координаты | 30 | 1 | 12 |

В полном теле passive damping: torso/neck/head 0,5; clavicle/arm 0,15;
legs 0,1 Н·м·с/рад. У рук ограниченные PD сохранены из 7Б; дополнительное
демпфирование именно full-body стенда сдерживает ударные скорости малых
сегментов. Новые более массивные сегменты получили большие effort limits;
это настройка запуска, не оптимизация двигательной функции.

Начали с прежних **8×1/960 с**, `implicitfast`, Newton 50, tolerance=1e−10.
Начальная сфера головы r=90 мм пересекалась с chest; исправлена геометрия
(r=80 мм, COM +90 мм), без collision exclusion. При ударе головой на 6-мс
contact solref скорость достигла 84,97 рад/с. На 10 мс и явном damping
скорость уменьшилась, но прежний 5-мс joint limit дал превышение 0,1248 рад.
Full-body solreflimit стал **2,5 мс**. При наклонном ударе кистью измерены
83,50 рад/с; damping рук увеличен до 0,15. Пороги не ослаблялись.
`experiment1`–`experiment9` и `manual*` сохраняют эти неуспешные измерения.

После этих изменений 8 подшагов дали ошибку импульса 0,12244 кг·м/с,
выше исходного допуска 0,1. Выполнено сравнение сходимости при том же controller:

| Подшагов/тик | max COM error, м | max ΔP, кг·м/с | max ΔL, кг·м²/с |
| ---: | ---: | ---: | ---: |
| 8 | 0,00142838 | 0,122441 | 0,0264559 |
| **16** | **0,000457746** | **0,0613075** | **0,0128576** |
| 32 | 0,000229154 | 0,0306749 | 0,00633374 |

Выбраны **16×1/1920 с только для полного тела**. Численное сохранение импульса
имеет измеренную ошибку, не объявляется точным. Стенды 7А/7Б сохраняют восемь
подшагов, integrator, contact/limit параметры и приводную динамику. Все регрессии
повторены. PD обновляется каждый подшаг; ровно один `mj_step` на подшаг для
всего мира. `mj_forward` после него обновляет endpoint transforms/telemetry,
не продвигая время. За 720 тиков полный мир делает **11520** шагов.

Runtime-envelope полного тела: |joint v|≤80 рад/с, soft-limit error≤0,12 рад,
penetration≤0,02 м, contact force norm≤30000 Н, KE≤5000 Дж, contacts≤128;
|root linear v|≤15 м/с, |root angular v|≤40 рад/с, quaternion norm error≤1e−8.
Пороги root имеют собственные единицы и не скопированы с руки. Все q/v/qacc,
effort, энергия и root проверяются на конечность, любые MuJoCo warnings
(включая autoreset) или discontinuity времени завершают исполнение с latch.
Отказ содержит tick/steps/q/root и контакты. Никакого визуального clamp нет.

Snapshot содержит root, target/q/v/effort/saturation/limit errors всех hinges,
world frames/COM каждого сегмента, общий COM, линейный и угловой импульсы,
контакты с geom/link IDs, solver iterations/warnings, энергию, wrench и timings.
Импульсы рассчитаны из compiled инерций и COM velocities; angular momentum —
относительно общего COM в world. [mj_objectVelocity и mj_contactForce](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-objectvelocity)
используются с явными соглашениями рамок.

Root/q/v/COM/contact estimates относятся к концу тика. Calculated torque —
начало последнего подшага; applied torque — endpoint qfrc_actuator при том же
held ctrl. Peak limit/contact/force/penetration — максимумы наблюдений после
каждого подшага, не непрерывный максимум. Normal/tangent force — Н в contact
frame, не импульс и не датчик. Нормаль направлена от geom1 к geom2; для floor→body
это +Y. CSV и render читают один `BodyRenderSnapshot` без интерполяции.

Reset — явная граница эксперимента: `mj_resetData` для live/scratch, очистка
q/v/ctrl/warmstart, pending wrench, targets/mode, failure latch, счётчиков,
timings/peaks и снимка; epoch увеличивается. Desktop также пересоздаёт scheduler
и пишет reset в отдельный каталог. Переход из клипа с восстановлением скоростей
не реализован и не требуется для этого стенда.

## Проверки и наблюдаемое поведение

Критерии сохранены до динамических прогонов в `criteria.txt` каждого эксперимента
и в тесте `test_body_physics.cpp`. Допуски между опытами не повышались.

| Проверка | Результат |
| --- | --- |
| Free sphere, 0,5 с, zero-g | Ошибка переноса 1,01e−13 м; вращения 3,28e−15 на элемент |
| Sphere под gravity до контакта | Ошибка положения 2,555 мм <3 мм, скорости 2,04e−14 м/с <1e−9 |
| Sphere удар о floor | Фактический contact solver; отдельный solref 4 мс для падения с 3 м |
| Internal, 4 с zero-g | COM/P/L в пределах таблицы 16 подшагов; все 41 PD меняют внутреннюю позу |
| Drop, 6 с | Обе стопы касаются пола; затем тело падает, контактируют голова и корпус |
| Tilt, 6 с | Опрокидывание, ранние контакты кисти и головы, последующие контакты корпуса |
| Pulse | 200 Н world X в COM Chest на тиках 91–102: 20 Н·с; root и другие сегменты меняют движение |
| Passive | Без приводов тело падает и взаимодействует с floor/self proxies |
| Reset/replay | Каждый из пяти сценариев повторён; состояние и контактная telemetry совпадают точно |
| Scheduler | Каждый сценарий, 30/60/144 Гц: 180/360/864 кадров, 720 тиков, точное совпадение history |
| Ошибки | Invalid root quaternion отклонён без изменения live state; чрезмерная скорость даёт latch, reset восстанавливает мир |

В Drop обе стопы впервые наблюдаются в tick 28: L point=(0,040109;−0,003253;
−0,279257) м, normal=(0;1;0), normal force=322,944 Н; R point=(−0,158926;
−0,004058;0,107255) м, normal=(0;1;0), normal force=1257,576 Н. Разная сила
соответствует асимметричному bind и решению solver, не принудительному делению
веса пополам. Все точки, силы и distance обеих сторон находятся в `contacts.csv`.

| Сценарий | peak penetration, мм | peak normal, Н | peak joint violation, рад | max contacts / solver iterations |
| --- | ---: | ---: | ---: | --- |
| Drop | 15,440 | 12999,10 | 0,052963 | 13 / 8 |
| Tilt | 17,711 | 11605,53 | 0,087161 | 14 / 8 |
| Pulse | 14,841 | 14592,12 | 0,073673 | 12 / 9 |
| Passive | 11,704 | 13132,44 | 0,063902 | 11 / 9 |

Во всех запусках — без NaN, warning, hidden reset и переписывания root.
Падение является ожидаемым результатом; способность стоять не заявляется.

## Desktop, производительность и сборки

Desktop показывает floor grid, фактический skin, cyan proxy-формы, дерево,
локальные COM оранжевым, общий COM жёлтой сферой, root пурпурным,
contact points/normals жёлтым и normal forces красным. Численные q/effort,
root speed, нарушения, penetration и solver отображаются в заголовке окна.
V переключает передний, обратный и верхний ракурсы. Камера следует XZ общего
COM только для кадрирования; её transforms не попадают в мир.

`--verify` создаёт настоящий скрытый OpenGL 3.3 context, рисует каждый заданный
scheduler frame, сохраняет контрольные PPM и два дополнительных ракурса.
Повторный draw одного снимка проверяется по равенству pixels и неизменности
физического состояния. Internal callbacks отдельно проверяют все 41 выбранные
координаты, pause/reset/camera/disable/wrench.

OS input **не проверен**: Computer Use `launch_app` не получил targetable window,
обновлённый поиск также не нашёл окна; клавиши/мышь не отправлялись.
`native-input.txt` фиксирует этот результат. Это не подменяется callback-тестом.

CPU measurement: AMD Ryzen 5 7500F, Release `/WX`, 10000 последовательных тиков
Drop, 16 подшагов; учтён весь переходный процесс, импорт, запись timing CSV и
работа GPU не входят в таймеры. Windows background scheduling не контролируется.
`step()` включает PD, integration и endpoint reads ×16; pose publication — одну
actual pose, FK, palette и копию physics snapshot.

| Работа | mean | p95 | max |
| --- | ---: | ---: | ---: |
| Tick, мкс | 945,38 | 1155,50 | 7243,10 |
| Pose publication, мкс | 56,29 | 82,00 | 269,80 |
| Endpoint contacts | 10,8023 | 11 | 13 |

Raw значения — `cpu/performance.csv`, итог — `cpu/body-physics.txt`.
Типичная стоимость ниже бюджета 8333 мкс; это не hard real-time гарантия.
Compiled model: **76604 байт**; один mjData с arena **2331624**, live+scratch
**4663248**, arena high-water **118924**. Snapshot struct **2968**, живые
vector elements в конце замера **4184**, одна pose **86208 байт**. Это размеры
буферов без allocator overhead/capacity, asset/GL heap и VRAM.

GPU — RTX 4070 SUPER. Per-run mean/p95/max CPU submission, GPU elapsed и palette
upload записаны в `gpu.txt`; query synchronized. Заданные scheduler intervals
30/60/144 Гц проверяют одинаковую симуляцию и draw count, **не измеряют реальный FPS**.

`headless-dependencies.txt` содержит dumpbin EXE и всех поставляемых DLL:
OpenGL/GLFW/GLAD imports отсутствуют. `JULIA_BUILD_DESKTOP=OFF` сохраняет CPU-only
путь. `JULIA_BUILD_PHYSICS=OFF` исключает SDK, обе группы физических приложений
и физические тесты; отдельные configure/build/test и dumpbin сохранены.

Итоговый штатный набор — **54/54**, CPU — **25/25**, OFF — **22/22**
(`final-desktop.log`, `final-cpu.log`, `no-physics.log`). После расширения
текста ошибки root и теста переставленных адресов повторены все три CPU physics
suite: **3/3**, `final-physics-tests.log`. Численные критерии старых этапов
сохранены; изменения их тестов ограничены instance capabilities и явной константой
шести координат руки вместо общей увеличенной ёмкости.

`tools/verify_stage7c.py` выполнил **15 реальных headless/desktop сравнений**:
пять сценариев × 30/60/144 Гц. Точно совпали 721 строка ×37 детерминированных
полей состояния, 29561 строка ×11 полей суставов и все строки ×16 полей контактов;
исключён только mapping time. `parity/parity.txt` хранит результаты. Все 15 GPU
запусков имеют GL_errors=0 и callback_checks=PASS. Дополнительно сохранены
четыре штатных GPU-прогона в `gpu/`. Кадры Drop/Tilt с переднего, обратного и
верхнего ракурсов просмотрены (`gpu/review-drop.png`, `gpu/review-tilt.png`,
`visual-review.txt`); физические сегменты следуют skin-ригу, включая падение.

Обычный видимый desktop также выполнил 720 тиков при реальных wall-clock
интервалах (`manual-desktop.log`); это отдельный прогон без синтетического
кадрового шага. OS-input статус остаётся not-tested. Прежний CSV 7Б дополнительно
сравнён между right/full/spatial CLI и штатным desktop: 720 строк ×76 полей
совпали (`legacy-parity.txt`).

## Проверенные команды

Из корня проекта; прямые CMake/CTest команды — в VS x64 Developer PowerShell:

```powershell
.\tools\build.ps1 -UseInstalledDependencies -KeepDiagnostics
cmake --preset cpu-release -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build --preset build-cpu-release --parallel 4
ctest --preset test-cpu-release

# Headless: точные 720 тиков / 6 с
.\build\cpu-release\apps\julia_body_physics\julia_body_physics.exe --case drop --ticks 720 --output build/diagnostics/stage7c/manual-drop
.\build\cpu-release\apps\julia_body_physics\julia_body_physics.exe --case internal --ticks 720 --output build/diagnostics/stage7c/manual-internal

# Интерактивный стенд и отдельная GPU-проверка
.\build\vs-release\apps\julia_body_physics\julia_body_physics_desktop.exe --case tilt --output build/diagnostics/stage7c/manual-desktop
.\build\vs-release\apps\julia_body_physics\julia_body_physics_desktop.exe --case drop --fps 144 --verify build/diagnostics/stage7c/manual-gpu

# Полный физический аудит и сравнение реальных приложений
.\build\cpu-release\tests\julia_test_body_physics.exe build/diagnostics/stage7c/manual-cpu
python tools/verify_stage7c.py

# Прежние стенды и отдельная сборка без backend
.\build\cpu-release\apps\julia_arm_physics\julia_arm_physics.exe --chain two --case impulse --output build/diagnostics/stage7c/regression-7a
.\build\cpu-release\apps\julia_arm_physics\julia_arm_physics.exe --chain full --side right --case contact --output build/diagnostics/stage7c/regression-7b
cmake --preset cpu-release -B build/cpu-no-physics -DJULIA_BUILD_PHYSICS=OFF -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build build/cpu-no-physics --parallel 4
ctest --test-dir build/cpu-no-physics --output-on-failure
```

Cases: `drop`, `tilt`, `pulse`, `internal`, `passive`. В окне 1–5 выбирают их,
R делает reset, Space pause, V camera, Tab coordinate, Up/Down меняют target
на 0,02 рад, D выключает drives, I задаёт 200 Н в Chest на один тик, Esc exit.
При смене mass/environment description создаётся новый мир. В `internal`
цели генерируются сценарием каждый тик; ручные targets будут заменены им.

## Границы и дальнейшее развитие

Модель готова как проверяемая основа для дальнейшего исследования приводов и
контактного управления. Для стояния потребуется отдельный контроллер с собственными
критериями COM, опорных сил, устойчивости и падений; этот стенд его не заменяет.
Контакты мягкие, с измеренными проникновениями до 17,8 мм в выбранных опытах;
нет гарантии отсутствия tunneling при произвольных скоростях. Массы и proxy-формы
инженерные, не идентифицированные по реальному персонажу. Пальцы, силовой захват,
одежда/волосы/мышцы, баланс, локомоция и переход из клипа со скоростями остаются
вне этапа. Межплатформенная побитовая детерминированность не заявляется.
