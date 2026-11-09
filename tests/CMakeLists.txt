# Unit-тесты. Регистрируются в CTest через add_test.
# Каждый тест — отдельный исполняемый файл, возвращающий число провалов.

add_executable(julia_test_core test_core.cpp)
target_include_directories(julia_test_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_core PRIVATE julia_core julia::warnings)
add_test(NAME core COMMAND julia_test_core)

add_executable(julia_test_events test_events.cpp)
target_include_directories(julia_test_events PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_events PRIVATE julia_events julia::warnings)
add_test(NAME events COMMAND julia_test_events)

add_executable(julia_test_orbit_camera test_orbit_camera.cpp)
target_include_directories(julia_test_orbit_camera PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_orbit_camera PRIVATE julia_render julia::warnings)
add_test(NAME orbit_camera COMMAND julia_test_orbit_camera)

add_executable(julia_test_pose_core test_pose_core.cpp)
target_include_directories(julia_test_pose_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_pose_core PRIVATE julia_animation julia::warnings)
add_test(NAME pose_core COMMAND julia_test_pose_core)

add_executable(julia_test_eye_behavior test_eye_behavior.cpp)
target_include_directories(julia_test_eye_behavior PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_eye_behavior PRIVATE julia_render julia::warnings)
add_test(NAME eye_behavior COMMAND julia_test_eye_behavior)

add_executable(julia_test_pose_parity test_pose_parity.cpp)
target_include_directories(julia_test_pose_parity PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(julia_test_pose_parity PRIVATE
    JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_pose_parity PRIVATE julia_model_import julia_embodiment julia::warnings)
add_test(NAME pose_parity COMMAND julia_test_pose_parity)
set_tests_properties(pose_parity PROPERTIES TIMEOUT 60)

add_executable(julia_test_animation test_animation.cpp)
target_include_directories(julia_test_animation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_animation PRIVATE julia_embodiment julia::warnings)
add_test(NAME animation COMMAND julia_test_animation)
add_executable(julia_test_model_math test_model_math.cpp)
target_link_libraries(julia_test_model_math PRIVATE julia_embodiment julia::warnings)
add_test(NAME model_math COMMAND julia_test_model_math)

foreach(suite model_asset model_asset_import)
    add_executable(julia_test_${suite} test_${suite}.cpp)
    target_compile_definitions(julia_test_${suite} PRIVATE
        JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
        JULIA_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(julia_test_${suite} PRIVATE julia_model_import julia_embodiment julia::warnings)
    add_test(NAME ${suite} COMMAND julia_test_${suite} "${CMAKE_SOURCE_DIR}/build/diagnostics/stage2/${suite}")
    set_tests_properties(${suite} PROPERTIES TIMEOUT 90)
endforeach()
find_package(assimp CONFIG REQUIRED)
target_link_libraries(julia_test_model_asset_import PRIVATE assimp::assimp)

add_executable(julia_test_body_profile test_body_profile.cpp)
target_compile_definitions(julia_test_body_profile PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_profile PRIVATE julia_model_import julia_embodiment julia::warnings)
add_test(NAME body_profile COMMAND julia_test_body_profile "${CMAKE_SOURCE_DIR}/build/diagnostics/stage3/cpu")
set_tests_properties(body_profile PROPERTIES TIMEOUT 90)

add_executable(julia_test_body_instance test_body_instance.cpp)
target_compile_definitions(julia_test_body_instance PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_instance PRIVATE julia_model_import julia_embodiment julia::warnings)
add_test(NAME body_instance COMMAND julia_test_body_instance "${CMAKE_SOURCE_DIR}/build/diagnostics/stage4/cpu")
set_tests_properties(body_instance PROPERTIES TIMEOUT 90)

add_executable(julia_test_arm_reach test_arm_reach.cpp)
target_compile_definitions(julia_test_arm_reach PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
    JULIA_TEST_STAGE5_DIR="${CMAKE_SOURCE_DIR}/build/diagnostics/stage5/cpu")
target_link_libraries(julia_test_arm_reach PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_arm_reach PRIVATE psapi)
endif()
add_test(NAME arm_reach COMMAND julia_test_arm_reach "${CMAKE_SOURCE_DIR}/build/diagnostics/stage5/cpu")
set_tests_properties(arm_reach PROPERTIES TIMEOUT 180)

add_executable(julia_test_body_spatial test_body_spatial.cpp)
target_compile_definitions(julia_test_body_spatial PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_spatial PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_body_spatial PRIVATE psapi)
endif()
add_test(NAME body_spatial COMMAND julia_test_body_spatial "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6e/cpu")
set_tests_properties(body_spatial PROPERTIES TIMEOUT 240)

add_executable(julia_test_body_collision test_body_collision.cpp)
target_compile_definitions(julia_test_body_collision PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_collision PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_body_collision PRIVATE psapi)
endif()
add_test(NAME body_collision COMMAND julia_test_body_collision "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6a/cpu")
set_tests_properties(body_collision PROPERTIES TIMEOUT 180)

add_executable(julia_test_body_multik test_body_multik.cpp)
target_compile_definitions(julia_test_body_multik PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_multik PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_body_multik PRIVATE psapi)
endif()
add_test(NAME body_multik COMMAND julia_test_body_multik "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6c/cpu")
set_tests_properties(body_multik PROPERTIES TIMEOUT 240)

add_executable(julia_test_body_detour test_body_detour.cpp)
target_compile_definitions(julia_test_body_detour PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_detour PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_body_detour PRIVATE psapi)
endif()
add_test(NAME body_detour COMMAND julia_test_body_detour "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6b/cpu")
set_tests_properties(body_detour PROPERTIES TIMEOUT 240)

add_executable(julia_test_body_ranked test_body_ranked.cpp)
target_compile_definitions(julia_test_body_ranked PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_ranked PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_body_ranked PRIVATE psapi)
endif()
add_test(NAME body_ranked COMMAND julia_test_body_ranked "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6d/cpu")
set_tests_properties(body_ranked PROPERTIES TIMEOUT 240)

add_executable(julia_test_body_interaction test_body_interaction.cpp)
target_compile_definitions(julia_test_body_interaction PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_body_interaction PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_body_interaction PRIVATE psapi)
endif()
add_test(NAME body_interaction COMMAND julia_test_body_interaction "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6f/cpu")
set_tests_properties(body_interaction PROPERTIES TIMEOUT 240)

add_executable(julia_test_leg_support test_leg_support.cpp)
target_compile_definitions(julia_test_leg_support PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_leg_support PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_leg_support PRIVATE psapi)
endif()
add_test(NAME leg_support COMMAND julia_test_leg_support "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6g/cpu")
set_tests_properties(leg_support PROPERTIES TIMEOUT 240)

add_executable(julia_test_stance_reach test_stance_reach.cpp)
target_compile_definitions(julia_test_stance_reach PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_stance_reach PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_stance_reach PRIVATE psapi)
endif()
add_test(NAME stance_reach COMMAND julia_test_stance_reach "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6h/cpu")
set_tests_properties(stance_reach PROPERTIES TIMEOUT 240)

add_executable(julia_test_stance_preparation test_stance_preparation.cpp)
target_compile_definitions(julia_test_stance_preparation PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_stance_preparation PRIVATE julia_model_import julia_embodiment julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_stance_preparation PRIVATE psapi)
endif()
add_test(NAME stance_preparation COMMAND julia_test_stance_preparation "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6i/cpu")
set_tests_properties(stance_preparation PROPERTIES TIMEOUT 240)

if(JULIA_BUILD_PHYSICS)
    add_executable(julia_test_arm_physics test_arm_physics.cpp)
    target_compile_definitions(julia_test_arm_physics PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
    target_link_libraries(julia_test_arm_physics PRIVATE julia_physics_rig julia_model_import julia::warnings)
    if(WIN32)
        target_link_libraries(julia_test_arm_physics PRIVATE psapi)
    endif()
    julia_copy_mujoco(julia_test_arm_physics)
    add_test(NAME arm_physics COMMAND julia_test_arm_physics "${CMAKE_SOURCE_DIR}/build/diagnostics/stage7a/cpu")
    set_tests_properties(arm_physics PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
    add_executable(julia_test_arm_physics_full test_arm_physics_full.cpp)
    target_compile_definitions(julia_test_arm_physics_full PRIVATE JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
    target_link_libraries(julia_test_arm_physics_full PRIVATE julia_physics_rig julia_model_import julia::warnings)
    julia_copy_mujoco(julia_test_arm_physics_full)
    add_test(NAME arm_physics_full COMMAND julia_test_arm_physics_full "${CMAKE_SOURCE_DIR}/build/diagnostics/stage7b/cpu")
    set_tests_properties(arm_physics_full PROPERTIES TIMEOUT 180 RUN_SERIAL TRUE)
endif()

if(NOT JULIA_BUILD_DESKTOP)
    return()
endif()

if(JULIA_BUILD_PHYSICS)
    add_test(NAME desktop_physics COMMAND $<TARGET_FILE:julia_physics_desktop>
        --case impulse --verify "${CMAKE_SOURCE_DIR}/build/diagnostics/stage7a/gpu/impulse")
    set_tests_properties(desktop_physics PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    foreach(side left right)
        add_test(NAME desktop_physics_full_${side} COMMAND $<TARGET_FILE:julia_physics_desktop>
            --chain full --side ${side} --case spatial --verify "${CMAKE_SOURCE_DIR}/build/diagnostics/stage7b/gpu/${side}-spatial")
        set_tests_properties(desktop_physics_full_${side} PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    endforeach()
endif()

# A real hidden OpenGL window verifies shaders, FBX loading and rendered pixels.
add_executable(julia_test_opengl test_opengl.cpp)
target_include_directories(julia_test_opengl PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(julia_test_opengl PRIVATE
    JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_opengl PRIVATE julia_render_opengl julia::warnings)
add_test(NAME opengl COMMAND julia_test_opengl "${CMAKE_BINARY_DIR}/verification")
set_tests_properties(opengl PROPERTIES TIMEOUT 60)
add_test(NAME desktop_launch COMMAND $<TARGET_FILE:julia_desktop>
    --verify "${CMAKE_BINARY_DIR}/verification/desktop_camera")
set_tests_properties(desktop_launch PROPERTIES
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 60)

add_executable(julia_test_desktop_display test_desktop_display.cpp)
target_include_directories(julia_test_desktop_display PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_desktop_display PRIVATE julia_desktop_display julia::warnings)
if(WIN32)
    target_link_libraries(julia_test_desktop_display PRIVATE user32 gdi32)
endif()
add_test(NAME desktop_display_math_alpha_input COMMAND julia_test_desktop_display "${CMAKE_BINARY_DIR}/verification")
set_tests_properties(desktop_display_math_alpha_input PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
if(WIN32)
    add_test(NAME desktop_display_modes COMMAND $<TARGET_FILE:julia_desktop>
        --verify-display "${CMAKE_BINARY_DIR}/verification/desktop_display")
    set_tests_properties(desktop_display_modes PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 60 RUN_SERIAL TRUE)
endif()

add_test(NAME desktop_animation COMMAND $<TARGET_FILE:julia_desktop>
    --verify-animations "${CMAKE_BINARY_DIR}/verification/desktop_animation")
set_tests_properties(desktop_animation PROPERTIES
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_eyes COMMAND $<TARGET_FILE:julia_desktop>
    --verify-eyes "${CMAKE_BINARY_DIR}/verification/desktop_eyes")
set_tests_properties(desktop_eyes PROPERTIES
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)

foreach(suite model_import gpu_resources shader_pipeline)
    add_executable(julia_test_${suite} test_${suite}.cpp)
    target_include_directories(julia_test_${suite} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_definitions(julia_test_${suite} PRIVATE
        JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
    target_link_libraries(julia_test_${suite} PRIVATE julia_render_opengl julia::warnings)
    add_test(NAME ${suite} COMMAND julia_test_${suite} "${CMAKE_BINARY_DIR}/verification")
    set_tests_properties(${suite} PROPERTIES TIMEOUT 60)
endforeach()

# CPU geometry regression: separate hull normals leave surface shading intact.
add_executable(julia_test_outline_normals test_outline_normals.cpp)
target_include_directories(julia_test_outline_normals PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_outline_normals PRIVATE julia_render_opengl julia::warnings)
add_test(NAME outline_normals COMMAND julia_test_outline_normals)

# Actual GLSL shadows, masked casters, directional lighting and material response.
add_executable(julia_test_toon_render test_toon_render.cpp)
target_include_directories(julia_test_toon_render PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_test_toon_render PRIVATE julia_render_opengl julia::warnings)
add_test(NAME toon_render COMMAND julia_test_toon_render)
set_tests_properties(toon_render PROPERTIES TIMEOUT 60)

# The GPU fixture exporter is a test-only Assimp dependency.
target_link_libraries(julia_test_model_import PRIVATE assimp::assimp)

add_test(NAME desktop_body COMMAND $<TARGET_FILE:julia_desktop>
    --verify-body "${CMAKE_SOURCE_DIR}/build/diagnostics/stage4/gpu")
set_tests_properties(desktop_body PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_ik COMMAND $<TARGET_FILE:julia_desktop>
    --verify-ik "${CMAKE_SOURCE_DIR}/build/diagnostics/stage5/gpu")
set_tests_properties(desktop_ik PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_collision COMMAND $<TARGET_FILE:julia_desktop>
    --verify-collision "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6a/gpu")
set_tests_properties(desktop_collision PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_multik COMMAND $<TARGET_FILE:julia_desktop>
    --verify-multik "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6c/gpu")
set_tests_properties(desktop_multik PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_ranked COMMAND $<TARGET_FILE:julia_desktop>
    --verify-ranked "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6d/gpu")
set_tests_properties(desktop_ranked PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_spatial COMMAND $<TARGET_FILE:julia_desktop>
    --verify-spatial "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6e/gpu")
set_tests_properties(desktop_spatial PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_detour COMMAND $<TARGET_FILE:julia_desktop>
    --verify-detour "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6b/gpu")
set_tests_properties(desktop_detour PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)

add_test(NAME desktop_interaction COMMAND $<TARGET_FILE:julia_desktop>
    --verify-interaction "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6f/gpu")
set_tests_properties(desktop_interaction PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)

add_test(NAME desktop_support COMMAND $<TARGET_FILE:julia_desktop>
    --verify-support "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6g/gpu")
set_tests_properties(desktop_support PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_stance_reach COMMAND $<TARGET_FILE:julia_desktop>
    --verify-stance-reach "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6h/gpu")
set_tests_properties(desktop_stance_reach PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
add_test(NAME desktop_stance_preparation COMMAND $<TARGET_FILE:julia_desktop>
    --verify-stance-preparation "${CMAKE_SOURCE_DIR}/build/diagnostics/stage6i/gpu")
set_tests_properties(desktop_stance_preparation PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 180 RUN_SERIAL TRUE)
