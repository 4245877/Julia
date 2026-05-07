add_library(julia_render_opengl STATIC)

target_sources(julia_render_opengl
    PRIVATE
        src/OpenGLRenderer.cpp
        src/GLShader.cpp
        src/GLMesh.cpp
        src/GLModel.cpp
        src/GLTexture.cpp
        src/GLModelLoader.cpp
)

target_include_directories(julia_render_opengl
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(julia_render_opengl
    PUBLIC
        cxx_std_20
)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

target_link_libraries(julia_render_opengl
    PUBLIC
        julia_render
        OpenGL::GL
        glfw
        glad::glad
        glm::glm
        assimp::assimp
        spdlog::spdlog
)