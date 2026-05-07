add_library(julia_generation INTERFACE)

target_include_directories(julia_generation INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_generation INTERFACE
    julia_core
    julia_events
)
