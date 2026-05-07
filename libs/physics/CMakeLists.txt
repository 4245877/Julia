add_library(julia_physics INTERFACE)

target_include_directories(julia_physics INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_physics INTERFACE
    julia_core
    julia_events
)
