# Julia

Julia — виртуальный 3D-персонаж на C++20 для Windows x64. Основной desktop
запускает кинематическое тело, анимацию, глаза и OpenGL-рендерер. У руки и
свободного тела есть отдельные стенды MuJoCo 3.3.7. Внедрение ИИ отложено.

Stage 7K принят 21.11.2026. Согласованный малый перенос центра масс и нагрузки
двух опор подтверждён в 33 шагах и 25 проверках. Итоговые наборы дали 76/76
desktop, 37/37 CPU и 22/22 без физики. Изоляция итогового прогона чистая.
[Отчёт Stage 7K](docs/body_core_stage7k.md).

## Сборка и запуск

Из корня проекта в PowerShell. Нужны Visual Studio с C++ Desktop workload,
Windows SDK, CMake 3.25+, Ninja и vcpkg. На новой установке загрузите ресурсы
Git LFS и выполните команды ниже.

```powershell
.\tools\setup-physics.ps1
.\tools\build.ps1
.\julia_desktop\julia_desktop.exe --window --no-save
```

Для демонстрации Idle запустите `.\julia_desktop\julia_desktop.exe --idle --window --no-save`.
Пробел переводит Рафталию в стойку Ready и возвращает в Idle, O раскрывает
и расслабляет кисти. F10 переключает Full Window и Desktop Avatar.

В этом рабочем дереве зависимости уже установлены. Для локальной сборки
используйте `.\tools\build.ps1 -UseInstalledDependencies`. Запуск EXE без
аргументов читает `julia_desktop/display.ini`. F10 переключает режим окна,
F9 переключает закрепление аватара. Комплект запуска переносится целиком.
[Конфигурации Release, команды проверки и управление](docs/desktop.md).

## Где искать

| Путь | Назначение |
| --- | --- |
| `apps/julia_desktop/`, `apps/julia_body_sim/` | Приложение и CPU-стенд кинематического тела |
| `apps/julia_arm_physics/`, `apps/julia_body_physics/`, `libs/physics/` | Стенды и контроллеры MuJoCo |
| `libs/`, `plugins/render_opengl/` | Импорт, анимация, кинематика, сцена и рендерер |
| `assets/`, `tests/` | Авторские ресурсы, тесты и эталоны |
| `cmake/`, `tools/` | Сборка, проверки и обслуживание |

- [Архитектура](docs/architecture.md)
- [Очистка, проверка архивов и история аудитов](docs/maintenance_audit.md)
- [Подготовка ресурсов Raphtalia](tools/raphtalia/README.md)
