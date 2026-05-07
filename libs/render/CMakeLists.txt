add_library(julia_render INTERFACE)

target_include_directories(julia_render INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_render INTERFACE
    julia_core
    julia_events
)
