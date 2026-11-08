# Запуск и разработка Julia

## Запуск и настройки

```powershell
.\julia_desktop\julia_desktop.exe
.\julia_desktop\julia_desktop.exe --desktop
.\julia_desktop\julia_desktop.exe --window
```

При отсутствии настроек используется Desktop Avatar. Ресурсы загружаются
относительно EXE, независимо от рабочей папки. Модель по умолчанию —
`Raphtalia_prepared.fbx`: 429 костей, 15 клипов, 24 цветовые карты.
Другую модель можно передать позиционным аргументом.

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
| Выход | Esc |

ЛКМ зарезервирована для взаимодействия и не двигает камеру.
В Desktop Avatar камера автоматическая; ракурс Full Window сохраняется при F10.

## Сборка и автоматические проверки

```powershell
.\tools\build.ps1                         # установка зависимостей при необходимости
.\tools\build.ps1 -UseInstalledDependencies -Clean
.\tools\build.ps1 -UseInstalledDependencies -Configuration Debug
```

`-Clean` пересоздаёт CMake cache и полностью перекомпилирует проект.
Зависимости остаются в `build/vs-release/vcpkg_installed`; Debug использует
тот же каталог. Обе конфигурации обновляют `julia_desktop/`.
BLEND, backups и метаданные в комплект запуска не копируются.

В уже настроенной Visual Studio Developer PowerShell:

```powershell
cmake --preset vs-release --fresh -DVCPKG_MANIFEST_INSTALL=OFF
cmake --build --preset build-release --clean-first --parallel 4
ctest --preset test-release
```

Все 17 наборов постоянные. Core/events проверяют базовые контракты;
orbit_camera и desktop_launch — математику и callbacks камеры;
model_math/model_import — transforms, skinning, FBX/glTF и материалы;
gpu_resources/shader_pipeline/opengl/outline_normals/toon_render — реальные
GPU-ресурсы, GLSL, alpha, нормали и тени; animation/eye_behavior и desktop
animation/eyes — sampling, playback и gaze/blink. Два display-набора проверяют
DPI/layout, настройки, native hit testing и настоящее прозрачное окно.
Для GPU-проверок нужна доступная Windows-сессия с рабочим OpenGL-драйвером.

CTest пишет в `build/<preset>/verification`, использует отдельные настройки
и не изменяет пользовательский `display.ini`. `build.ps1` очищает прежние кадры
перед тестами, удаляет результаты успешного прогона и сохраняет их при ошибке.
`-KeepDiagnostics` оставляет кадры для просмотра. Последний журнал CTest остаётся
в `build/<preset>/Testing/Temporary/LastTest.log`.

## Ручная диагностика и очистка

```powershell
.\julia_desktop\julia_desktop.exe --verify build/diagnostics/camera
.\julia_desktop\julia_desktop.exe --verify-display build/diagnostics/display
.\julia_desktop\julia_desktop.exe --verify-animations build/diagnostics/animation
.\julia_desktop\julia_desktop.exe --verify-eyes build/diagnostics/eyes
.\tools\clean.ps1 -WhatIf
.\tools\clean.ps1
```

`--verify` проверяет callbacks и 120 кадров; `--verify-display` — alpha,
композицию, все клипы, F9/F10 и сохранение сцены. При отсутствии output-пути
эти две команды используют `%TEMP%/Julia/verification/`. Для animation/eyes
output-путь обязателен. `--review-animations` и `--review-eyes` выполняют
те же проверки в видимом окне. Сохраняйте результаты ручных проверок в `build/`.

`clean.ps1` удаляет только `build/vs-*/verification`, `build/diagnostics`,
`build/packages` и старый `build/anime-audit`. Параметр `-Configuration Release`
ограничивает выбор конфигурации; `-VerificationOnly` оставляет ручную диагностику
и пакеты. Каталоги с symlink/junction сценарий удалять отказывается.
Не удаляйте `julia_desktop/display.ini`, исходные ресурсы и installed dependencies.
