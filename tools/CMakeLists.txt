# Diagnostic tools are opt-in targets, independent of the regular test suite.
find_package(assimp CONFIG REQUIRED)
add_executable(julia_shader_visual_audit EXCLUDE_FROM_ALL shader_visual_audit.cpp)
target_compile_definitions(julia_shader_visual_audit PRIVATE
    JULIA_AUDIT_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
target_link_libraries(julia_shader_visual_audit PRIVATE julia_render_opengl julia::warnings)

foreach(audit assimp_audit assimp_morph_audit gpu_pose_audit)
    add_executable(julia_${audit} EXCLUDE_FROM_ALL raphtalia/${audit}.cpp)
    target_link_libraries(julia_${audit} PRIVATE julia::compiler_options assimp::assimp)
endforeach()
target_link_libraries(julia_gpu_pose_audit PRIVATE julia_render_opengl)
