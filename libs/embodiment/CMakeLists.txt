add_library(julia_embodiment STATIC src/Animation.cpp src/EyePoseLayer.cpp
    src/BodyProfile.cpp src/RaphtaliaBodyProfile.cpp src/BodyInstance.cpp src/ArmReach.cpp src/BodyCollision.cpp src/HandInteraction.cpp src/LegSupport.cpp)
target_include_directories(julia_embodiment PUBLIC include)
target_link_libraries(julia_embodiment PUBLIC julia_model julia_render julia_runtime julia_world PRIVATE julia::warnings)
target_compile_features(julia_embodiment PUBLIC cxx_std_20)
