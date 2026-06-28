add_library(julia_events
    src/EventBus.cpp
    src/EventQueue.cpp
)

target_include_directories(julia_events PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_events
    PUBLIC
        julia_core
    PRIVATE
        julia::warnings
)

target_compile_features(julia_events PUBLIC cxx_std_20)
