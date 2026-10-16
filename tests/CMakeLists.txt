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

# A real hidden OpenGL window verifies shaders, FBX loading and rendered pixels.
add_executable(julia_test_opengl test_opengl.cpp)
target_include_directories(julia_test_opengl PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(julia_test_opengl PRIVATE
    JULIA_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_test_opengl PRIVATE julia_render_opengl julia::warnings)
add_test(NAME opengl COMMAND julia_test_opengl "${CMAKE_BINARY_DIR}/verification")
set_tests_properties(opengl PROPERTIES TIMEOUT 60)
add_test(NAME desktop_launch COMMAND $<TARGET_FILE:julia_desktop> --verify)
set_tests_properties(desktop_launch PROPERTIES
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}" TIMEOUT 60)

foreach(suite model_math model_import gpu_resources shader_pipeline)
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

# Developer audit tool: explicit invocation, since images/timings require review.
add_executable(julia_shader_visual_audit ../tools/shader_visual_audit.cpp)
target_compile_definitions(julia_shader_visual_audit PRIVATE
    JULIA_AUDIT_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_shader_visual_audit PRIVATE julia_render_opengl julia::warnings)
