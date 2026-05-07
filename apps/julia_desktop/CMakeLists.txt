add_executable(julia_desktop
    main.cpp
)

target_link_libraries(julia_desktop
    PRIVATE
        julia_core
        julia_render
        julia_render_opengl
)