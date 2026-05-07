add_library(julia_diagnostics INTERFACE)

target_include_directories(julia_diagnostics INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_diagnostics INTERFACE
    julia_core
    julia_events
)
