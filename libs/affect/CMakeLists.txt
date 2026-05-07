add_library(julia_affect INTERFACE)

target_include_directories(julia_affect INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_affect INTERFACE
    julia_core
    julia_events
)
