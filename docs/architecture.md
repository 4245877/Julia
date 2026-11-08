# Архитектура Julia

## Модули и ресурсы

Корневой CMake подключает `core`, `diagnostics`, `events`, `runtime`, `render`,
плагин `render_opengl`, desktop-приложение, тесты и необязательные инструменты.
Остальные каталоги `libs/` — заготовки расширения, не активные подсистемы.
Core содержит типы/время/result, events — очередь и шину событий;
diagnostics/runtime пока представлены interface-targets.

`GLModelLoader` через Assimp переносит геометрию, иерархию, материалы,
текстуры и клипы в собственные структуры до уничтожения importer.
`GLModel`, `GLMesh`, `GLTexture`, `GLShader` владеют соответствующими ресурсами.
Все GPU-объекты уничтожаются до закрытия текущего GL-контекста.
Рендерер читает оба рабочих шейдера: `assets/shaders/model.vert` и `model.frag`.

Оригинальные FBX, editable `Raphtalia_prepared.blend` и архивы в
`assets/models/backups` сохранены для разработки. Метаданные происхождения
текстур и правок рига находятся в `assets/metadata/`; это не runtime-ресурсы.
В `julia_desktop/assets` копируются модели, шейдеры и текстуры, без BLEND,
backups и метаданных. `display.ini` принадлежит пользователю.

## Координаты и анимация

GLM использует column-major матрицы; GLSL получает их с `transpose=GL_FALSE`.
Контракт: `Projection * View * Model * Skin * bakedPosition`.
Импорт запекает `meshGlobal` в позиции, а offset палитры равен
`assimpOffset * inverse(meshGlobal)`. Матрица палитры:
`posedBoneGlobal * adjustedOffset`; дополнительный inverse-root не применяется.
Палитра передаётся через texture buffer RGBA32F, четыре texel на матрицу.

`AnimationPlayer` сэмплирует TRS-каналы, вычисляет глобальную иерархию и палитру.
Неанимированные узлы сохраняют точные bind-матрицы. Поддерживаются выбор клипа,
play/pause/stop, seek и loop. `EyeBehavior` задаёт фиксации, саккады, tracking
и blink; специфичный для Raphtalia `EyePoseLayer` накладывает глаза/веки перед
расчётом иерархии. Авторские eye-клипы имеют приоритет.
Exact bind требует отключённого eye overlay. Crossfade, общий animation graph,
morph/lip-sync и симуляция физики пока не реализованы.

## Рендерер

OpenGL 3.3 / GLSL 330 forward pipeline работает в линейном пространстве;
текстуры и framebuffer используют sRGB. Материалы имеют Opaque/Mask/Blend,
alpha cutoff и собственные sampler-параметры. Toon-профили разделяют ткань,
кожу, волосы, металл, кожу одежды, глаза и ink. Их параметры находятся в
`GLMeshMaterial`. Профиль определяется именем mesh/material и может быть переопределён.

Нормали двух известных rigged FBX восстанавливаются явно при загрузке:
connected surface fans, angle weights, соединение UV-разрезов и разделение
граней от 45°. Внешние модели сохраняют авторские нормали по умолчанию.
Outline использует отдельные нормали, не меняя surface shading.
Освещение включает directional light, hemisphere fill, material highlights,
inverted-hull outline и alpha-aware self-shadow map 2048×2048.
Для кожи используется более мягкая фильтрация теней и receiver offset.

Blend-meshes сортируются по статическим центрам в одном `renderModel`;
OIT нет. Прозрачные поверхности не отбрасывают тени. Normal/PBR maps и
reflection probes не реализованы. `Raphtalia.fbx` не имеет подходящих texture
assignments; `Albedo.fbx` ссылается на отсутствующие внешние карты. Оригиналы
сохранены без подстановки чужих UV-текстур. Рабочая prepared-модель использует
24 связанные карты; материал бликов имеет постоянный цвет.

## Два режима окна

`DisplayController` меняет оформление и геометрию одного GLFW-окна.
`GLModel`, `AnimationPlayer`, `EyePoseLayer`, WGL-контекст и GPU-ресурсы общие.
`OrbitCamera` обслуживает Full Window; фон — непрозрачный sRGB `#808080`.
`AvatarCamera` — фронтальная ортографическая камера по пояс с опорой на spine,
fallback по габаритам и запасом над волосами/ушами. Движение spine в анимации
переносит камеру вместе с телом, без покадровых скачков масштаба.

Окно создаётся с transparent framebuffer, alpha, sRGB и MSAA. В Desktop Avatar
`AvatarSurface` рисует в multisampled RGBA16F FBO, разрешает покрытие и переводит
premultiplied linear RGB в premultiplied sRGB для DWM:

```text
straight_linear = alpha > 0 ? linear_rgb / alpha : 0
desktop_rgb = linear_to_srgb(straight_linear) * alpha
desktop_alpha = alpha
```

`WindowPlatform` — адаптер Windows. Он строит `HRGN` по ненулевому alpha:
пустые области пропускают мышь к чужим окнам, силуэт остаётся интерактивным.
Порядок кадра: offscreen render → resolve → alpha readback → region → present.
Region сбрасывается перед сменой режима/размера. Чтение alpha синхронное;
при существенном увеличении окна его стоимость нужно измерять отдельно.

Layout использует work area выбранного монитора в физических пикселях;
только размеры DIP и правый отступ умножаются на DPI. Нижний край:
`windowY + avatarHeight = workBottom`, без нижнего отступа. Размер ограничен
половиной высоты и 60% доступной ширины; на больших логических экранах рост
не превышает 25%. Учитываются панель задач, отрицательные координаты и
отключение выбранного монитора. Настройки валидируются и записываются атомарно.

На Windows нужны DWM и рабочий OpenGL-драйвер. При недоступной прозрачности
используется Full Window. Физическое переключение DPI/мониторов и другие GPU
нуждаются в отдельной проверке; математические тесты не заменяют оборудование.
Анимации, выходящие далеко за bind-габариты, могут выходить за портретный кадр.

Практические команды и тесты: [desktop.md](desktop.md).
