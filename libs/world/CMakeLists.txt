add_library(julia_world INTERFACE)

target_include_directories(julia_world INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_world INTERFACE
    julia_core
    julia_events
)
