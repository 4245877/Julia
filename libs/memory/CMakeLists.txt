add_library(julia_memory INTERFACE)

target_include_directories(julia_memory INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_memory INTERFACE
    julia_core
    julia_events
)
