add_library(julia_safety INTERFACE)

target_include_directories(julia_safety INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_safety INTERFACE
    julia_core
    julia_events
)
