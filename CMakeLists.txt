cmake_minimum_required(VERSION 3.24)

project(Julia
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(JuliaCompilerOptions OPTIONAL)
include(JuliaWarnings OPTIONAL)
include(JuliaDependencies OPTIONAL)
include(JuliaHelpers OPTIONAL)

add_subdirectory(libs)
add_subdirectory(apps)
add_subdirectory(plugins) 
