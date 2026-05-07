add_library(julia_platform INTERFACE)

target_include_directories(julia_platform INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_platform INTERFACE
    julia_core
    julia_events
)
