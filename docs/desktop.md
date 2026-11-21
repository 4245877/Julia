# Запуск и разработка Julia

## Запуск и настройки

```powershell
.\julia_desktop\julia_desktop.exe
.\julia_desktop\julia_desktop.exe --desktop
.\julia_desktop\julia_desktop.exe --window
```

При отсутствии настроек используется Desktop Avatar. Приложение ищет
`assets/` рядом с EXE. Сборка для разработки при отсутствии каталога
использует исходные ресурсы. Модель по умолчанию `Raphtalia_prepared.fbx`
содержит 429 костей, 15 клипов и 24 цветовые карты. Можно передать путь к
другой модели позиционным аргументом. Относительный путь отсчитывается
от текущего каталога.

`display.ini` рядом с EXE хранит режим, закрепление, размер аватара,
монитор и геометрию Full Window. `--settings path` выбирает другой файл,
`--no-save` запрещает запись. Сохраняйте этот файл при обновлении приложения.
Дополнительные параметры: `--avatar-height 180..720`, `--topmost`,
`--no-topmost`, `--no-eyes`; `--help` выводит справку.

Аватар расположен у нижнего края рабочей области Windows без нижнего отступа;
правый отступ задаёт `avatar_margin_right_dip` (по умолчанию 12 DIP).
DPI и панель задач учитываются автоматически. Для выбора монитора переместите
туда Full Window и нажмите F10. ПКМ по аватару открывает меню режима,
закрепления, размера и выхода. Клавиши работают при фокусе окна.

## Управление

| Действие | Клавиши / мышь |
| --- | --- |
| Desktop Avatar / Full Window | F10 |
| Закрепление аватара поверх окон | F9 |
| Меню аватара | ПКМ по силуэту |
| Вращение камеры в Full Window | Перетаскивание ПКМ |
| Панорамирование | Средняя кнопка или Shift + ПКМ |
| Зум | Колёсико, W / S |
| Смещение кадра | A / D, Q / E или стрелки |
| Фокус на лице / общий фронтальный вид | F / B |
| Предыдущий / следующий клип | PgUp / PgDn |
| Play / pause | Space |
| Stop, exact bind и отключение eye overlay | X или R |
| Loop | L |
| Шаг с паузой на 1/30 с | `,` / `.` |
| Клип blink: natural / independent / gaze / head | 1 / 2 / 3 / 4 |
| Список клипов в консоли | F1 |
| Автономный idle глаз | G |
| Автоматические blink / gaze | H / J |
| Blink обоими / левым / правым глазом | K / U / I |
| Направление взгляда / нейтраль | Numpad 1–9 / Home |
| Камера как цель взгляда | T |
| Средний интервал blink | `[` / `]` |
| Фиксированный seed поведения глаз | F8 |
| Direct mode / полный reset тела | F2 / F6 |
| Предыдущий / следующий сустав, координата, ±5° | Shift+F3 / F3, F4, `-` / `+` |
| Reach/retarget выбранной руки в direct mode | F7 или Shift+F7 для другой руки |
| Кинематические сцены коллизий / обход | F11 / F12 |
| Пауза всей симуляции тела | Pause |
| Выход | Esc |

ЛКМ зарезервирована для взаимодействия и не двигает камеру.
В Desktop Avatar камера автоматическая; ракурс Full Window сохраняется при F10.

## Сборка и автоматические проверки

```powershell
.\tools\setup-physics.ps1
.\tools\build.ps1 -UseInstalledDependencies
```

`setup-physics.ps1` устанавливает проверенный по SHA-256 MuJoCo 3.3.7. Для
первой установки vcpkg-зависимостей запустите `build.ps1` без
`-UseInstalledDependencies`. `-Clean` заново настраивает CMake и перекомпилирует
проект. Release и Debug по умолчанию обновляют `julia_desktop/` и используют
общий каталог `build/vs-release/vcpkg_installed`. Для самостоятельной
desktop-сборки задайте абсолютный `-DJULIA_DESKTOP_RUNTIME_DIR=...` при
конфигурации другого дерева. Её EXE, DLL и ресурсы попадут в этот каталог.

Команды для Visual Studio Developer PowerShell с заданным `VCPKG_ROOT`.

```powershell
cmake --preset vs-release -DVCPKG_MANIFEST_INSTALL=OFF
cmake --build --preset build-release --parallel 4
ctest --preset test-release -N

cmake --preset cpu-release -DJULIA_BUILD_PHYSICS=ON
cmake --build --preset build-cpu-release --parallel 4
cmake --preset cpu-release -B build/cpu-no-physics -DJULIA_BUILD_PHYSICS=OFF
cmake --build build/cpu-no-physics --parallel 4
```

CPU-конфигурации собирают импорт, анимацию, кинематику и свои тесты без OpenGL.
`JULIA_BUILD_PHYSICS=OFF` отключает MuJoCo, физические стенды и их тесты.
Для desktop-тестов нужна Windows-сессия с рабочим OpenGL.

Обычный CTest пишет в `build/<сборка>/verification`. `build.ps1` очищает этот
каталог перед тестами и после успеха. `-KeepDiagnostics` сохраняет результаты,
при ошибке они сохраняются автоматически. Журнал CTest доступен в
`build/<сборка>/Testing/Temporary/LastTest.log`.

Для проверки без записи в стандартные каталоги используйте отдельный запуск.
Скрипт пишет журнал, JUnit и результат наблюдения за фактическими записями.
Тесты используют собственные настройки окна.

```powershell
$run = 'build/verification-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
python -B tools/stage7d_ctest.py --build build/vs-release --output $run
```

Для CPU укажите `--build build/cpu-release` или `--build build/cpu-no-physics`.

## Ручная диагностика

Физические стенды требуют явный каталог в `--output` или `--verify`. Для
короткой проверки свободного тела после Release-сборки используйте новый
каталог. При прямом запуске тестового EXE с артефактами передайте каталог первым
позиционным аргументом.

```powershell
$run = 'build/diagnostics/manual-drop-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
.\build\vs-release\apps\julia_body_physics\julia_body_physics.exe --case drop --ticks 10 --output $run
```

Полные опыты могут создавать сотни мегабайт трасс. [Границы модели](architecture.md).
[Исторические команды и сценарии Stage 7K](body_core_stage7k.md).

```powershell
$run = 'build/diagnostics/manual-camera-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
.\julia_desktop\julia_desktop.exe --verify $run
```

`--verify` проверяет callbacks и 120 кадров, `--verify-display` проверяет альфу,
композицию, клипы, F9/F10 и сохранение сцены. Передавайте новый каталог и для
`--verify-display`, `--verify-animations` и `--verify-eyes`. Режимы
`--review-animations` и `--review-eyes` выполняют проверки в видимом окне.
[Очистка и восстановление истории](maintenance_audit.md).
