cmake_minimum_required(VERSION 3.25)

project(Julia
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(libs/core)
add_subdirectory(libs/diagnostics)
add_subdirectory(libs/events)
add_subdirectory(libs/runtime)

add_subdirectory(libs/render)
add_subdirectory(plugins/render_opengl)

add_subdirectory(apps/julia_desktop)