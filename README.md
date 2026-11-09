# Julia

C++20-приложение для отображения анимированного 3D-персонажа на Windows.
Работают загрузка FBX, skeletal animation, автономный взгляд/моргание,
OpenGL toon-рендерер и два режима одного окна:

- **Desktop Avatar** — прозрачный персонаж по пояс у нижнего правого края рабочей области.
- **Full Window** — обычное окно с орбитальной камерой.

Запуск: `julia_desktop/julia_desktop.exe`. **F10** переключает режим,
**F9** — закрепление аватара поверх окон. Переносите всю папку `julia_desktop`,
включая DLL и `assets`. Настройки находятся в `julia_desktop/display.ini`.

## Сборка

Нужны Visual Studio с Desktop development with C++, CMake 3.25+, Ninja и vcpkg.
Из PowerShell в корне проекта:

```powershell
.\tools\setup-physics.ps1 # Закреплённый MuJoCo 3.3.7, SHA-256 проверяется
.\tools\build.ps1
# Уже установленные зависимости, чистая Release-сборка и CTest:
.\tools\build.ps1 -UseInstalledDependencies -Clean
```

Сценарий находит MSVC, обновляет папку запуска и выполняет CTest.
`-Configuration Debug` выбирает Debug; обе конфигурации обновляют один EXE.
После Debug верните Release перед распространением приложения.
`-SkipTests` пропускает проверки, `-KeepDiagnostics` сохраняет их кадры.

Физический стенд руки запускается отдельно:

```powershell
.\build\vs-release\apps\julia_arm_physics\julia_arm_physics.exe --case contact --ticks 720 --output build/diagnostics/stage7a/run
.\build\vs-release\apps\julia_arm_physics\julia_physics_desktop.exe --case hold
```

В окне: 1–7 — нагрузочные случаи, R — reset, Space — пауза, стрелки — цель,
D — выключить приводы, I — толчок, Esc — выход. Основание руки фиксировано;
физическая устойчивость персонажа не моделируется. Параметры, все команды и
результаты — в [отчёте 7А](docs/body_core_stage7a.md).
Для прежней сборки без SDK: `cmake --preset cpu-release -DJULIA_BUILD_PHYSICS=OFF`;
для возвращения стенда задайте `-DJULIA_BUILD_PHYSICS=ON`.

## Разработка

| Каталог | Назначение |
| --- | --- |
| `apps/julia_desktop/` | Окно, настройки, ввод, интеграция сцены и проверки приложения |
| `libs/` | Core, events, камера, поведение глаз и заготовки будущих модулей |
| `libs/animation/` | CPU-скелет, клипы, поза, семплирование, иерархия и палитра; только GLM |
| `libs/model/`, `libs/model_import/` | Самостоятельный CPU-ресурс модели, изображения и Assimp-импорт без GL |
| `libs/embodiment/` | CPU-проигрыватель и адаптер глаз Raphtalia |
| `libs/physics/`, `apps/julia_arm_physics/` | MuJoCo backend, фиксированная динамическая рука, headless/desktop диагностика |
| `plugins/render_opengl/` | GPU-загрузка готового CPU-ресурса и рендерер |
| `assets/` | Оригиналы, рабочие ресурсы, BLEND, backups и метаданные происхождения |
| `tests/` | Постоянные регрессионные проверки |
| `tools/`, `cmake/` | Сборка, очистка, упаковка и явно запускаемая диагностика |
| `build/` | Восстанавливаемые сборки, зависимости и временные результаты |
| `julia_desktop/` | Готовый комплект запуска; `display.ini` — пользовательские данные |

[Запуск, управление и проверки](docs/desktop.md) ·
[Архитектура и ограничения](docs/architecture.md) ·
[CPU-ядро позы: проверки и дальнейшие этапы](docs/body_core_stage1.md) ·
[CPU-импорт и отдельная GPU-загрузка](docs/body_core_stage2.md) ·
[Диагностика моделей](tools/raphtalia/README.md) ·
[Визуальный аудит шейдеров](tools/shader_visual_audit.md)

Временные кадры, логи и пакеты сохраняйте под `build/`.
`tools/clean.ps1 -WhatIf` показывает доступную очистку;
`tools/clean.ps1` удаляет только диагностические результаты и пакеты,
сохраняя сборку, зависимости, исходники и пользовательские настройки.

Долгосрочное направление Julia — модульный цифровой персонаж с памятью,
поведением, голосом и взаимодействием со сценой. Эти будущие системы пока
представлены заготовками модулей; текущая реализация сосредоточена на визуальном runtime.
