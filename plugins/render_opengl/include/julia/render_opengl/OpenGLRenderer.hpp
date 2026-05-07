#pragma once

#include "julia/render_opengl/GLModel.hpp"
#include "julia/render_opengl/GLShader.hpp"

#include <glm/glm.hpp>

#include <span>

namespace julia::render_opengl
{
    class OpenGLRenderer
    {
    public:
        OpenGLRenderer() = default;
        ~OpenGLRenderer();

        OpenGLRenderer(const OpenGLRenderer&) = delete;
        OpenGLRenderer& operator=(const OpenGLRenderer&) = delete;

        OpenGLRenderer(OpenGLRenderer&&) = delete;
        OpenGLRenderer& operator=(OpenGLRenderer&&) = delete;

        void initialize(int width, int height);
        void resize(int width, int height);

        void beginFrame(const glm::vec3& clearColor = glm::vec3{0.08f, 0.08f, 0.10f});

        // Рендер получает готовую позу в виде boneMatrices.
        // Если boneMatrices пустой — модель рисуется в bind/static pose.
        void renderModel(
            const GLModel& model,
            const glm::mat4& modelMatrix,
            std::span<const glm::mat4> boneMatrices = {}
        );

    private:
        GLShader shader_;

        // Шейдер и VAO для фонового fullscreen triangle.
        GLShader backgroundShader_;
        unsigned int backgroundVao_ = 0;

        int width_ = 1280;
        int height_ = 720;

        glm::mat4 view_{1.0f};
        glm::mat4 projection_{1.0f};

        void updateCamera();
        void releaseBackground();
    };
}