add_library(julia_render STATIC)

target_sources(julia_render
    PRIVATE
        src/RenderSystem.cpp
)

target_include_directories(julia_render
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_render
    PUBLIC
        julia_core
)