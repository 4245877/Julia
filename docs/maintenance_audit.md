# Обслуживание Julia

Актуальные команды сборки и запуска находятся в [README](../README.md) и
[руководстве desktop](desktop.md). Итог приёмки Stage 7K датирован 21.11.2026.
Отчёты `body_core_stage*.md` относятся к указанному в заголовке этапу.
[Отчёт Stage 7K](body_core_stage7k.md).
[Исторический аудит от 12.11.2026](maintenance_audit_2026-11-12.md).

## Рабочие данные

| Путь | Содержимое |
| --- | --- |
| `assets/`, `tests/data/` | Ресурсы и тестовые эталоны |
| `julia_desktop/` | Готовый комплект запуска с пользовательским `display.ini` |
| `build/dependencies/`, `build/vs-release/vcpkg_installed/` | MuJoCo SDK и установленные зависимости |
| `build/<сборка>/verification/` | Воспроизводимые выходы текущих тестов |
| `build/diagnostics/` | Журналы исследований, графики, отобранные кадры и сертификаты |
| `build/research-archive/` | ZIP с историческими данными и индексами восстановления |

В штатной очистке сначала проверьте список. `-VerificationOnly` ограничивает
действие каталогом `verification` выбранных VS-конфигураций. При ошибке теста
скрипт сборки оставляет материалы для разбора.

```powershell
.\tools\clean.ps1 -WhatIf
.\tools\clean.ps1
python -B tools/archive-diagnostics.py plan
```

Для ручного опыта выбирайте новый каталог. При уборке старых прогонов
проверьте входы тестов, ссылки и манифесты. Закройте активный прогон перед
архивированием. Индекс `_manifest.json` хранит исходный путь, размер, mtime
и SHA-256. Для точных совпадений `stored_path` указывает на общий ZIP-элемент.

## Приёмка Stage 7K и восстановление

Оригинальные пути отчётов и сертификатов сохранены.
`build/diagnostics/stage7k/README.md` содержит карту доказательств.
`final-validation-08/summary.json` и `delivery-certificate.json` содержат
итог. В `final-validation-08/postprocess/visuals/` доступны контрольные
кадры, а в `postprocess/plots/` — графики.

- `stage7k-raw.zip` — принятый архив первого набора. Его SHA-256 входит в принятую поставку.
- `stage7k-validation-raw.zip` — CSV, сырые кадры и другие данные завершённых и прерванных прогонов, включая `final-validation-08`, `-06` и `-05`.
- `stage7k-resumption-raw.zip` — два исходных снимка для возобновления прогона от 17.11.2026.
- `stage7k-runtime-v7-snapshot.zip` — снимок всех 535 файлов поставки. Пути его индекса заданы от корня проекта.

Архивы диагностик используют пути от `build/diagnostics`.
Проверка манифестов сверяет фактические байты с принятой длиной и SHA-256.
Она читает архивные копии по исходным путям при отсутствии или изменении
рабочих экземпляров. Запускайте из корня проекта.

```powershell
$stage = 'build/diagnostics/stage7k'
$archive = 'build/research-archive'
python -B tools/stage7k_manifest.py check --path "$stage/evidence-manifest-v7-final.json" --diagnostic-archive "$archive/stage7k-validation-raw.zip" --diagnostic-archive "$archive/stage7k-resumption-raw.zip"
python -B tools/stage7k_manifest.py check --path "$stage/frozen-runtime-v7.json" --runtime-snapshot "$archive/stage7k-runtime-v7-snapshot.zip"
python -B tools/stage7k_manifest.py check --path "$stage/runtime-manifest-v7-final.json" --runtime-snapshot "$archive/stage7k-runtime-v7-snapshot.zip"
```

Историческому reader может понадобиться распакованный исходный путь.
Восстановите только нужный файл в отдельный каталог и передайте этот путь reader.

```powershell
python -B tools/archive-diagnostics.py verify --stage stage7k --part validation
python -B tools/archive-diagnostics.py restore --stage stage7k --part validation --destination build/restored/stage7k --file stage7k/final-validation-08/legacy-load-parity/pulse-30/frame_0.ppm
```

Для раннего `stage7k-raw.zip` опустите `--part`. Без `--file` команда
восстанавливает всю часть, для которой может понадобиться несколько десятков
гигабайт. Восстановление отказывается перезаписывать файл с другим хешем.
Уплотнение исходных ZIP, хеши которых заверены в evidence-манифесте,
заблокировано.

Отчёт и список действий аудита от 21.11.2026 находятся в
`build/maintenance-audit/julia-2026-11-21/`.
