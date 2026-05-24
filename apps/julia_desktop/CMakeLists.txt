add_executable(julia_desktop
    main.cpp
)

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
)