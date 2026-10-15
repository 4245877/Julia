# Технический аудит OpenGL, 15 октября 2026

Аудит охватил оба рабочих GLSL-шейдера, весь `plugins/render_opengl`,
создание контекста и жизненный цикл ресурсов в desktop-приложении,
камеру, обе исходные FBX и существующие тесты. Исправления внесены в исходники.
Используемый API остался OpenGL 3.3 / GLSL 330 core, стилизованное cel-освещение сохранено.

## Критические ошибки и результат исправления

| Ошибка | Исправление и проверка |
| --- | --- |
| `mat4[128]` в обычных vertex uniforms требует 2048 компонент ещё до MVP; это превышает минимальную гарантию GL 3.3. Индексы >=128 молча отбрасывались. | Палитра перенесена в `samplerBuffer` / RGBA32F: четыре texel на матрицу. Размер ограничивается фактическим `GL_MAX_TEXTURE_BUFFER_SIZE`, проверяется до загрузки. GPU-тест использует индекс 256; поставляемый `Raphtalia.fbx` отрисован с 235 элементами палитры. |
| Короткая палитра позволяла читать нулевые или оставшиеся от предыдущего draw матрицы. | C++ отвергает неполную непустую палитру; GLSL проверяет индекс относительно `uBoneCount`. Нулевые/недопустимые влияния дают identity, оставшиеся веса нормализуются. |
| Кости объединялись по одному имени, хотя после baking transforms inverse bind различается между meshes/instances. Иерархия терялась после завершения Assimp importer. | Сохранены узлы, родители, локальные и глобальные bind transforms. Палитра объединяется только при совпадении узла и adjusted offset. `skinningMatrices()` строит позу с учётом helper nodes и root transform. Проверены разные offsets одной кости в двух экземплярах меша. |
| Нормали skinning умножались на прямую матрицу: освещение неверно при неравномерном масштабе. Зеркальная кость также ломала facing и contour culling. | Направление inverse-transpose вычисляется кофакторами с учётом знака determinant. Передаётся handedness примитива; skinning outline отсекает нужную сторону в fragment shader. GPU transform feedback сравнивает нормали с CPU-эталоном; изображения отражения через Model и через кость совпадают. |
| Альфа текстур и opacity игнорировались, результат всегда имел alpha=1. | Добавлены Opaque/Mask/Blend, opacity/cutoff, discard в заливке и контуре, отдельный blend-pass с сортировкой meshes и выключенной записью depth. FBO-тесты численно проверяют цвет, alpha и отсутствие записи глубины. |
| Автоматический fallback по словам body/hair подставлял текстуры от другой модели с несовместимыми UV. | По умолчанию используются назначенные/embedded maps. Прежний fallback сохранён как явная опция `GLModelLoadOptions::useLegacyMaterialTextureFallback`. Проверены default и opt-in. |

## Другие устранённые дефекты

- Model normal matrix вычисляется один раз на CPU, с double intermediates и проверкой
  вырожденности; дорогое `inverse(uModel)` больше не выполняется для каждой вершины.
- Контур использует размер viewport в пикселях и производную перспективного деления.
  Его ширина проверяется на трёх размерах/пропорциях viewport. Opaque/masked depth
  записывается до оболочки, чтобы скрытые оболочки не затирали видимые поверхности.
- Материальные цветовые множители остаются линейными. sRGB декодирует текстура,
  framebuffer выполняет обратное преобразование. Удалены повторная gamma-конверсия
  материального цвета и искусственный нижний предел albedo 0.02.
- Камера отвергает NaN/Inf, совпадение position/target, вырожденный up, неверные FOV
  и clipping planes. View/projection рассчитываются в double и проверяются до изменения
  состояния камеры. Проверены также крайние конечные значения и `resize`.
- `beginFrame()` восстанавливает viewport, color/depth write masks и отключает scissor
  перед очисткой. `renderModel()` восстанавливает program, VAO, используемые texture
  units/bindings/samplers, TBO binding, depth, culling/front-face и blend state.
- Проверяются текущий GL-контекст, обязательные uniforms, их типы/размеры и расположение
  vertex attributes. Diffuse sampler2D и bone samplerBuffer используют разные units.
- При ошибке compile/link сохраняется прежняя программа. Диагностика указывает stage
  и driver log. Исправлены перенос/очистка uniform cache, поведение bound program при
  successful reload и проверка размера/типа `setMat4Array`.
- Mesh upload проверяет индексы, треугольники, размеры и выделение buffers; неверные
  vertex/material значения отвергаются до GPU. Пустые/moved-from meshes не рисуют VAO 0.
  Upload восстанавливает чужие VAO/VBO/EBO; выделенные объекты освобождаются при исключениях.
- Texture loader поддерживает 1/2/3/4 канала, grayscale/alpha и сжатые/raw embedded maps.
  Grayscale корректно расширяется в RGB с sRGB decoding; alpha остаётся линейной.
  Проверяются размеры, storage и mipmaps. Пути файлов поддерживают Unicode.
- Upload текстур сохраняет чужой PBO, texture binding и unpack row/skip/alignment state.
  Flip картинки не дублирует преобразование UV импортера.
- Wrap/filter принадлежат sampler материала, а не общей cached texture. Проверены
  Repeat/Clamp/Mirror и разные фильтры двух материалов, использующих один image.
- Assimp сохраняет маленькие невырожденные треугольники и постоянные UV; прежние
  postprocess defaults могли удалить их. Поддержаны выбранный UV channel,
  UV transformations, glTF base color/alpha/samplers и перенос texture paths.
- При baking отражения переставляются индексы треугольника. Проверяются finite
  матрицы/геометрия/UV, веса и ссылки на вершины/узлы. Bounds center не переполняется
  при сложении больших координат, model fitting отвергает некорректные размеры.
- `find_package(Stb)` теперь выполняется до использования его include directory,
  что устраняет зависимость первой конфигурации от уже заполненного CMake cache.

## Математический контракт

Матрицы GLM передаются column-major с `transpose=GL_FALSE`.
Порядок координат: `Projection * View * Model * Skin * bakedPosition`.
При импорте `bakedPosition = meshGlobal * meshLocalPosition`, поэтому offset палитры
равен `assimpOffset * inverse(meshGlobal)`. Итоговая матрица элемента палитры:
`posedBoneGlobal * adjustedOffset`. Дополнительный inverse-root здесь не применяется:
root уже входит в scene-space геометрию и узлы. Порядок MVP и нормали проверяются
transform feedback реального vertex shader, а не копией его формул в отдельном shader.

```cpp
auto model = GLModelLoader::loadFromFile(path);
auto palette = model.skinningMatrices();       // импортированная bind pose
renderer.renderModel(model, modelMatrix, palette);

// Для внешней анимации: одна локальная матрица на узел в порядке model.nodes().
auto animatedPalette = model.skinningMatrices(localNodeTransforms);
renderer.renderModel(model, modelMatrix, animatedPalette);
```

Пустая палитра сохраняет прежний режим статической baked-геометрии.
GPU wrappers и renderer нужно уничтожать до уничтожения текущего контекста;
desktop-приложение уже выполняло это правило, поэтому его lifecycle не переписывался.

## Проверки

Изменённые файлы:

| Файлы | Изменения |
| --- | --- |
| `assets/shaders/model.vert`, `model.frag` | TBO skinning, нормали, handedness, pixel outline, linear albedo и alpha |
| `plugins/render_opengl/src/OpenGLRenderer.cpp` и соответствующий `.hpp` | GL state, palette storage, uniform contract, камера и sRGB probe |
| `plugins/render_opengl/src/GLShader.cpp` и `.hpp` | безопасная compile/link замена, cache/moves, mat3 и array validation |
| `plugins/render_opengl/src/GLModel.cpp`, `GLModelLoader.cpp` и соответствующие `.hpp` | иерархия/палитра, импорт/материалы, прозрачность, явный legacy fallback |
| `plugins/render_opengl/src/GLMesh.cpp`, `GLTexture.cpp` и соответствующие `.hpp` | GPU resources, channels/embedded data, samplers и validation |
| `plugins/render_opengl/include/julia/render_opengl/Bounds.hpp` | finite bounds и устойчивый центр |
| `plugins/render_opengl/CMakeLists.txt`, `tests/CMakeLists.txt` | dependency discovery и регистрация новых suites |
| `tests/test_opengl.cpp` | обе модели, skeletal render, stress/timing |
| `tests/test_model_math.cpp`, `test_model_import.cpp`, `test_gpu_resources.cpp`, `test_shader_pipeline.cpp` | четыре новых regression suites |

Release x64 собран MSVC 14.50 с `/W4 /WX /permissive-`, C++20.
Проверка выполнена на настоящем OpenGL 3.3 NVIDIA 617.42,
NVIDIA GeForce RTX 4070 SUPER, оконный MSAA 8.

| CTest | Что проверено | Результат |
| --- | --- | --- |
| core, events, orbit_camera | существующие библиотеки и движение камеры | Passed |
| opengl | обе FBX, фон RGB128, camera motion/resize, фактическая skeletal pose, stress/timing | Passed |
| desktop_launch | bundled EXE, resources относительно EXE, 120 кадров и завершение | Passed |
| model_math | иерархия, root/helper transforms, offsets, palette, Bounds и неверные входы | Passed |
| model_import | Assimp/glTF fixtures, embedded maps, Unicode, UV1, sampler/alpha, tiny geometry и reflection | Passed |
| gpu_resources | реальные texels, PBO/unpack state, buffers/VAO/samplers, moves/destruction и invalid inputs | Passed |
| shader_pipeline | actual GLSL compile/link, failed/successful reload, MVP/skinning/normals, alpha и GL state | Passed |

Дополнительно выполнен `git diff --check`. Рендеры сохранены и просмотрены:
`build/vs-release/verification/stage-0.png` … `stage-3.png`, `rig-bind.png`.
Исходные PPM также сохранены. Журнал: `build/vs-release/Testing/Temporary/LastTest.log`.

Поставляемые модели: `Raphtalia1.fbx` — 19 meshes, без костей, три embedded maps;
`Raphtalia.fbx` — 28 meshes, 235 элементов палитры, 372 nodes.
Нагрузочный прогон — 360 кадров 640×480 с вращением второй модели,
без GL errors. В последнем прогоне: среднее GL_TIME_ELAPSED **0.093 мс/кадр**,
среднее wall time **0.093 мс/кадр**; весь CTest — **9/9 Passed за 2.20 с**.
GL_TIME_ELAPSED измеряет среднее время последовательности;
результат зависит от текущей нагрузки GPU и не является сравнением с прежней версией.

## Ограничения и нерешённые вопросы

1. У `Raphtalia.fbx` отсутствуют texture assignments и embedded images: 26 материалов
   содержат только diffuse factor 0.8. Это проверено через Assimp и исходные записи FBX.
   Текстуры/назначения исходной модели необходимо получить из источника; три атласа
   соседней `Raphtalia1.fbx` не подходят её UV. Геометрия и skinning проверены,
   но восстановить исходную текстурированную внешность из этих файлов нельзя.
2. Прозрачные meshes сортируются по центрам, внутри одного `renderModel`. Для
   пересекающихся прозрачных поверхностей, крупных meshes или нескольких моделей
   это не точная сортировка по пикселям. Нужен отдельный scene queue/OIT, если такие
   сцены необходимы. Вызывающая сторона должна рисовать opaque модели раньше blend.
   Inverted-hull контур применяется к Opaque/Mask; для Blend он не добавляет второй
   слой полупрозрачности. Треугольники с вырожденной skinning-деформацией имеют
   устойчивую fallback-нормаль; единственной математически определённой нормали
   у коллапсировавшей поверхности нет.
3. Проигрывателя animation clips, morph targets и PBR/normal-map pipeline в исходной
   реализации не было. Аудит исправляет существующий toon pipeline и skeletal pose
   interface; добавленный расчёт позы не заявляет поддержку sampling animation clips.
4. Проверена одна связка Windows/NVIDIA. AMD/Intel/macOS/Linux, длительный soak test,
   искусственное GPU OOM и внешний профилировщик утечек VRAM не проверялись.
   Регрессионные тесты удаления GL-объектов выполнены.

## Воспроизведение

В Visual Studio Developer PowerShell, с установленными vcpkg dependencies:

```powershell
cmake --preset vs-release -DVCPKG_MANIFEST_INSTALL=OFF -DJULIA_WARNINGS_AS_ERRORS=ON
cmake --build --preset build-release --parallel 4
ctest --preset test-release
```

Первоначальная автоматическая установка vcpkg не смогла записать загрузку в
внешний `D:/libraries/vcpkg_downloads`; проверка выполнена с уже установленными
зависимостями. Компилятор через sandbox зависал при запуске дочерних процессов;
итоговая сборка выполнена разрешённым запуском Visual Studio вне этого ограничения.

Нормативные источники: [лимиты shader resources Khronos](https://wikis.khronos.org/opengl/Shader_Resource_Limit),
[linear base-color factors и alpha glTF](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html),
[sRGB framebuffer GLFW](https://www.glfw.org/docs/3.3/window_guide.html),
[Assimp Windows UTF-8 IO](https://github.com/assimp/assimp/blob/master/code/Common/DefaultIOSystem.cpp).
На проверенном драйвере encoding-query default framebuffer возвращает GL_LINEAR,
хотя фактическая конверсия .5→188 работает; initialize проверяет именно конверсию
пикселя с восстановлением затронутого GL state.
