add_library(julia_embodiment INTERFACE)

target_include_directories(julia_embodiment INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_embodiment INTERFACE
    julia_core
    julia_events
)
