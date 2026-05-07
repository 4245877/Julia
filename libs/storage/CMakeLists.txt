add_library(julia_storage INTERFACE)

target_include_directories(julia_storage INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_storage INTERFACE
    julia_core
    julia_events
)
