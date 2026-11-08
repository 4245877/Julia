find_package(glm CONFIG REQUIRED)

add_library(julia_animation STATIC
    src/Skeleton.cpp
    src/Animation.cpp
    src/Pose.cpp
)
target_include_directories(julia_animation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(julia_animation PUBLIC cxx_std_20)
target_link_libraries(julia_animation PUBLIC glm::glm PRIVATE julia::warnings)
