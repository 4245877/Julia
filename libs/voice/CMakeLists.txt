add_library(julia_voice INTERFACE)

target_include_directories(julia_voice INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_voice INTERFACE
    julia_core
    julia_events
)
