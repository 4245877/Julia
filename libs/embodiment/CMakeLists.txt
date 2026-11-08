add_library(julia_embodiment STATIC src/Animation.cpp src/EyePoseLayer.cpp)
target_include_directories(julia_embodiment PUBLIC include)
target_link_libraries(julia_embodiment PUBLIC julia_model julia_render PRIVATE julia::warnings)
target_compile_features(julia_embodiment PUBLIC cxx_std_20)
