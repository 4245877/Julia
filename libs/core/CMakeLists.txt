add_library(julia_core
    src/Time.cpp
)

target_include_directories(julia_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(julia_core PUBLIC cxx_std_20)

target_link_libraries(julia_core PRIVATE julia::warnings)
