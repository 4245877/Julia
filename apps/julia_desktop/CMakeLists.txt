add_executable(julia_desktop
    main.cpp
    AnimationVerification.cpp
)

# The only runnable application bundle, shared by all build configurations.
set_target_properties(julia_desktop PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/julia_desktop"
)
foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(julia_desktop PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${config} "${CMAKE_SOURCE_DIR}/julia_desktop"
    )
endforeach()

# Always refresh resources, even if only a shader/texture changed.
add_custom_target(julia_desktop_assets ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets" "${CMAKE_SOURCE_DIR}/julia_desktop/assets"
    COMMENT "Updating JULIA runtime assets"
)
add_dependencies(julia_desktop julia_desktop_assets)

target_compile_features(julia_desktop
    PRIVATE
        cxx_std_20
)

target_compile_definitions(julia_desktop
    PRIVATE
        JULIA_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
)

set_property(TARGET julia_desktop PROPERTY
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(julia_desktop
    PRIVATE
        julia_core
        julia_render
        julia_render_opengl
        julia::warnings
)
