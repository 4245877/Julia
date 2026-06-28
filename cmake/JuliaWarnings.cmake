# Общая политика предупреждений компилятора.
#
# Создаёт INTERFACE-таргет julia_warnings, который можно линковать к любой
# цели проекта (но не к сторонним зависимостям), чтобы получить единый
# строгий набор предупреждений на MSVC, GCC и Clang.

if(TARGET julia_warnings)
    return()
endif()

add_library(julia_warnings INTERFACE)
add_library(julia::warnings ALIAS julia_warnings)

option(JULIA_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

set(JULIA_MSVC_WARNINGS
    /W4
    /permissive-
    /w14242 # conversion, possible loss of data
    /w14254 # operator conversion, possible loss of data
    /w14263 # member function does not override a base class virtual
    /w14265 # class has virtual functions but non-virtual destructor
    /w14287 # unsigned/negative constant mismatch
    /w14296 # expression is always true/false
    /w14545 # expression before comma has no effect
    /w14619 # unknown #pragma warning
    /w14640 # thread-unsafe static member initialization
    /w14826 # conversion is sign-extended
    /w14905 # wide string literal cast to LPSTR
    /w14906 # string literal cast to LPWSTR
)

set(JULIA_GCC_CLANG_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wsign-conversion
    -Wdouble-promotion
    -Wnull-dereference
    -Wformat=2
)

if(MSVC)
    set(JULIA_PROJECT_WARNINGS ${JULIA_MSVC_WARNINGS})
    if(JULIA_WARNINGS_AS_ERRORS)
        list(APPEND JULIA_PROJECT_WARNINGS /WX)
    endif()
else()
    set(JULIA_PROJECT_WARNINGS ${JULIA_GCC_CLANG_WARNINGS})
    if(JULIA_WARNINGS_AS_ERRORS)
        list(APPEND JULIA_PROJECT_WARNINGS -Werror)
    endif()
endif()

target_compile_options(julia_warnings INTERFACE ${JULIA_PROJECT_WARNINGS})

# Любая цель, использующая общую политику предупреждений, должна также
# получать общие опции компилятора (например /utf-8 для исходников с
# кириллицей в комментариях). Подключаем их транзитивно.
if(TARGET julia_compiler_options)
    target_link_libraries(julia_warnings INTERFACE julia_compiler_options)
endif()
