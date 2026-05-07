add_library(julia_runtime INTERFACE)

target_include_directories(julia_runtime INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_runtime INTERFACE
    julia_core
    julia_events
)
