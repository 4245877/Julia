add_library(julia_desktop_display STATIC
    DisplaySettings.cpp
    DisplayController.cpp
    WindowPlatform.cpp
)
target_include_directories(julia_desktop_display PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(julia_desktop_display PUBLIC julia_render_opengl PRIVATE julia::warnings)
if(WIN32)
    target_link_libraries(julia_desktop_display PRIVATE user32 gdi32)
endif()

add_executable(julia_desktop
    main.cpp
    AnimationVerification.cpp
    EyeVerification.cpp
    BodyDiagnostics.cpp
    ReadySphereVerification.cpp
)

# Keep the usual launch location while allowing isolated build trees to own their bundles.
set(JULIA_DESKTOP_RUNTIME_DIR "${CMAKE_SOURCE_DIR}/julia_desktop"
    CACHE PATH "Directory for the desktop executable, DLLs and assets")
get_filename_component(julia_desktop_runtime_dir "${JULIA_DESKTOP_RUNTIME_DIR}"
    ABSOLUTE BASE_DIR "${CMAKE_BINARY_DIR}")
set_target_properties(julia_desktop PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${julia_desktop_runtime_dir}"
)
foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(julia_desktop PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${config} "${julia_desktop_runtime_dir}"
    )
endforeach()

# Always refresh resources, even if only a shader/texture changed.
add_custom_target(julia_desktop_assets ALL
    COMMAND ${CMAKE_COMMAND}
        "-DJULIA_ASSET_SOURCE=${CMAKE_SOURCE_DIR}/assets"
        "-DJULIA_ASSET_DESTINATION=${julia_desktop_runtime_dir}/assets"
        -P "${CMAKE_SOURCE_DIR}/cmake/JuliaRuntimeAssets.cmake"
    COMMENT "Updating JULIA runtime assets"
    VERBATIM
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
        julia_desktop_display
        julia::warnings
)
