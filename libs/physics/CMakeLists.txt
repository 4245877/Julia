include(JuliaMuJoCo)
find_package(glm CONFIG REQUIRED)
add_library(julia_physics STATIC src/PhysicsWorld.cpp)

target_include_directories(julia_physics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(julia_physics PUBLIC glm::glm PRIVATE
    julia_mujoco julia::warnings
)
add_library(julia_physics_rig STATIC src/ArmRig.cpp src/BodyRig.cpp)
target_include_directories(julia_physics_rig PUBLIC include)
target_link_libraries(julia_physics_rig PUBLIC julia_physics julia_embodiment PRIVATE julia::warnings)
