# Общие настройки компиляции для всего проекта.
#
# Создаёт INTERFACE-таргет julia_compiler_options со стандартом языка и
# базовыми флагами, не зависящими от конкретной цели.

if(TARGET julia_compiler_options)
    return()
endif()

add_library(julia_compiler_options INTERFACE)
add_library(julia::compiler_options ALIAS julia_compiler_options)

target_compile_features(julia_compiler_options INTERFACE cxx_std_20)

# Единый стандарт C++ без расширений компилятора.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Генерируем compile_commands.json для IDE / clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    target_compile_options(julia_compiler_options INTERFACE
        /utf-8          # исходники и литералы в UTF-8
        /EHsc           # стандартная модель исключений C++
        /Zc:__cplusplus # корректное значение макроса __cplusplus
        /MP             # параллельная компиляция
    )
    target_compile_definitions(julia_compiler_options INTERFACE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        _CRT_SECURE_NO_WARNINGS
    )
endif()
