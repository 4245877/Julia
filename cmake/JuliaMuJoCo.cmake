# Separate, pinned binary SDK. No dependency on the MuJoCo viewer or Python.
set(JULIA_MUJOCO_VERSION "3.3.7")
set(JULIA_MUJOCO_ROOT "${CMAKE_SOURCE_DIR}/build/dependencies/mujoco-${JULIA_MUJOCO_VERSION}"
    CACHE PATH "MuJoCo 3.3.7 SDK root (bin/include/lib)")
if(NOT WIN32 OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "The pinned physics SDK currently supports Windows x64; use JULIA_BUILD_PHYSICS=OFF elsewhere")
endif()
foreach(required_file include/mujoco/mujoco.h lib/mujoco.lib bin/mujoco.dll)
    if(NOT EXISTS "${JULIA_MUJOCO_ROOT}/${required_file}" OR
       IS_DIRECTORY "${JULIA_MUJOCO_ROOT}/${required_file}")
        message(FATAL_ERROR
            "Incomplete MuJoCo SDK: missing ${JULIA_MUJOCO_ROOT}/${required_file}. "
            "Run tools/setup-physics.ps1, point JULIA_MUJOCO_ROOT at the installed SDK, "
            "or set JULIA_BUILD_PHYSICS=OFF.")
    endif()
endforeach()
add_library(julia_mujoco SHARED IMPORTED GLOBAL)
set_target_properties(julia_mujoco PROPERTIES
    IMPORTED_IMPLIB "${JULIA_MUJOCO_ROOT}/lib/mujoco.lib"
    IMPORTED_LOCATION "${JULIA_MUJOCO_ROOT}/bin/mujoco.dll"
    INTERFACE_INCLUDE_DIRECTORIES "${JULIA_MUJOCO_ROOT}/include")
function(julia_copy_mujoco target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:julia_mujoco>" "$<TARGET_FILE_DIR:${target}>"
        VERBATIM)
endfunction()
