# Диагностика и упаковка Raphtalia

Рабочий editable источник — `assets/models/Raphtalia_prepared.blend`;
приложение загружает соседний FBX и `assets/textures/raphtalia`.
Исходные модели и backups сохранены. Происхождение текстур и правок рига:
`assets/metadata/raphtalia`. Старые сценарии одноразового ремонта и evidence
доступны в истории Git; текущий runtime от них не зависит.

## Read-only аудит

В Visual Studio Developer PowerShell после конфигурации Release:

```powershell
cmake --build --preset build-release --target julia_assimp_audit julia_assimp_morph_audit julia_gpu_pose_audit
.\build\vs-release\tools\julia_assimp_audit.exe assets/models/Raphtalia_prepared.fbx build/diagnostics/model/assimp.json
.\build\vs-release\tools\julia_assimp_morph_audit.exe assets/models/Raphtalia_prepared.fbx build/diagnostics/model/morph.json
.\build\vs-release\tools\julia_gpu_pose_audit.exe assets/models/Raphtalia_prepared.fbx assets build/diagnostics/model/gpu
```

Цели исключены из обычной сборки. Используются общие CMake/vcpkg-зависимости;
привязки к конкретной версии Visual Studio нет. GPU-аудитор сэмплирует клипы
независимо от desktop и принимает необязательный четвёртый аргумент с CSV
выбранных blink-сэмплов. Аудиторы читают модель и создают только указанные outputs.

`fbx_audit.py input.fbx --output build/diagnostics/model/raw.json` анализирует
бинарный FBX без Blender; нужны Python, NumPy и SciPy. Для проверки основной
интеграции используйте CTest или `--verify-animations` / `--verify-eyes`,
описанные в [desktop.md](../../docs/desktop.md).

## Упаковка и восстановление текстур

```powershell
python tools/raphtalia/package_textured.py
.\tools\clean.ps1
```

Упаковка использует Python standard library, создаёт
`build/packages/Raphtalia_textured_4K.zip` и проверяет CRC/SHA256 каждого ресурса.
В архив входят актуальные FBX, BLEND, карты, сведения об авторстве и manifest;
исторические отчёты и диагностические кадры не требуются.

`restore_reference_textures.py` — явно запускаемое восстановление цветовых PNG
по сохранённому манифесту источника. Требуются requests, NumPy, Pillow и сеть;
сценарий **перезаписывает** `assets/textures/raphtalia`. Промежуточные загрузки
и отчёт остаются в `build/diagnostics/texturing`. Для обычной сборки,
тестирования и запуска этот сценарий и Blender не нужны. Перед правками рига
работайте с копией BLEND и сохраняйте оригиналы.
