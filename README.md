# Julia

C++20-проект виртуального 3D-персонажа для Windows x64. Работают Assimp-импорт
FBX, CPU-анимация и глаза, OpenGL toon-рендерер, фиксированный шаг тела,
ограниченная IK и кинематическое взаимодействие.

Отдельные MuJoCo-стенды содержат динамическую руку и свободное тело из 20 сегментов
с 41 суставной координатой. Физика ещё не интегрирована в `BodyInstance`
основного desktop-приложения. Память, речь, обучение и когнитивные модули — заготовки.

## Запуск

```powershell
.\julia_desktop\julia_desktop.exe
.\julia_desktop\julia_desktop.exe --window --no-save
```

F10 переключает Desktop Avatar / Full Window, F9 — закрепление аватара.
Переносите всю папку `julia_desktop`, включая DLL и `assets`.
`display.ini` — пользовательские настройки; сборка и очистка их сохраняют.
Модель по умолчанию — `Raphtalia_prepared.fbx`.

## Сборка и тесты

Нужны Visual Studio с Desktop development with C++, Windows SDK, CMake 3.25+,
Ninja и vcpkg. Сценарий сам подключает MSVC через `vswhere`.
Зависимости зафиксированы в `vcpkg.json`; физический SDK — MuJoCo 3.3.7.

```powershell
.\tools\setup-physics.ps1
.\tools\build.ps1
# Полная перекомпиляция Release с установленными зависимостями и CTest:
.\tools\build.ps1 -UseInstalledDependencies -Clean
```

`-SkipTests` пропускает CTest, `-KeepDiagnostics` сохраняет результаты успешных
тестов. При ошибке результаты остаются для разбора. Debug выбирается через
`-Configuration Debug` и обновляет тот же комплект запуска. Перед
распространением верните Release.

В Developer PowerShell доступны `cmake --preset vs-release`,
`cmake --build --preset build-release`, `ctest --preset test-release`.
CPU-конфигурация `cpu-release` исключает OpenGL, но по умолчанию включает
MuJoCo; без физики явно задайте `-DJULIA_BUILD_PHYSICS=OFF`.
Installed dependencies общие: `build/vs-release/vcpkg_installed`.

## Навигация для разработчика и ИИ-помощника

1. [Архитектура и границы компонентов](docs/architecture.md).
2. [Сборка, запуск, управление и диагностика](docs/desktop.md).
3. [Аудит, очистка и хранение исследований](docs/maintenance_audit.md).
4. [Исходные ресурсы](tools/raphtalia/README.md) и
   [визуальный аудит шейдеров](tools/shader_visual_audit.md).

`docs/body_core_stage*.md` — исторические инженерные отчёты. Их числа тестов,
планы следующего этапа и прежние ограничения не определяют текущее API.
Последние направления: [COM и QP](docs/body_core_stage7h.md),
[оптимизация решателя](docs/body_core_stage7i.md),
[нагрузка стоп](docs/body_core_stage7j.md),
[согласованный COM/нагрузка](docs/body_core_stage7k.md).
Фиксированная историческая приёмка 7К осталась незавершённой; это отдельно
от новых регрессионных проверок проекта.

| Каталог | Назначение |
| --- | --- |
| `apps/julia_desktop/` | Окно, ввод, настройки, кинематическое тело |
| `apps/julia_body_sim/` | CPU-стенд `BodyInstance` |
| `apps/julia_arm_physics/`, `apps/julia_body_physics/` | Headless/desktop-стенды MuJoCo |
| `libs/animation/`, `libs/model/`, `libs/model_import/` | CPU-поза, ресурс модели, импорт |
| `libs/embodiment/`, `libs/world/`, `libs/runtime/` | Управление телом, геометрия, планировщик |
| `libs/physics/` | MuJoCo, приводы, контакты, контроллеры стойки |
| `plugins/render_opengl/` | GPU-ресурсы и рендерер |
| `assets/` | Исходные модели, BLEND, текстуры, backups и provenance |
| `tests/` | Исходники тестов и независимые эталоны |
| `tools/`, `cmake/` | Сборка, обслуживание и исследования |
| `build/<preset>/verification/` | Воспроизводимые результаты текущих тестов |
| `build/diagnostics/` | Исторические параметры, сертификаты, снимки, ручные опыты |
| `build/research-archive/` | Сжатые исторические трассы и кадры |
| `julia_desktop/` | Комплект запуска и пользовательские настройки |

## Очистка и исследования

```powershell
.\tools\clean.ps1 -WhatIf
.\tools\clean.ps1
python tools/archive-diagnostics.py plan
python tools/archive-diagnostics.py restore --stage stage7k
```

Штатная очистка удаляет результаты тестов и временные пакеты. Исторические
исследования, архивы, SDK, сборочные деревья и пользовательские данные сохраняются.
Архиватор (Python 3.11+) сжимает CSV/PPM/PAM/BIN без потерь и проверяет SHA-256 каждого файла
перед удалением несжатого экземпляра. Восстановление нужно перед запуском
исторических readers и проверок evidence-манифестов; оно может потребовать
десятки гигабайт. Исторические runtime-манифесты относятся к прежним бинарникам.

Для исследований сохраняйте параметры, версию исходников, команды и итоговые
сертификаты. После разбора удаляйте воспроизводимые тестовые выходы или сжимайте
ценные трассы. Новые результаты CTest изолированы по сборкам.
