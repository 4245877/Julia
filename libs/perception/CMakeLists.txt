add_library(julia_perception INTERFACE)

target_include_directories(julia_perception INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_perception INTERFACE
    julia_core
    julia_events
)
