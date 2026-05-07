add_library(julia_cognition INTERFACE)

target_include_directories(julia_cognition INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_cognition INTERFACE
    julia_core
    julia_events
)
