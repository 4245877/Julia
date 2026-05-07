add_library(julia_communication INTERFACE)

target_include_directories(julia_communication INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_communication INTERFACE
    julia_core
    julia_events
)
