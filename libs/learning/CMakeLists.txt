add_library(julia_learning INTERFACE)

target_include_directories(julia_learning INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_learning INTERFACE
    julia_core
    julia_events
)
