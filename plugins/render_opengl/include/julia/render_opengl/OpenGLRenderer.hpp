#pragma once

#include "julia/render_opengl/GLModel.hpp"
#include "julia/render_opengl/GLShader.hpp"

#include <glm/glm.hpp>

#include <span>

namespace julia::render_opengl
{
    // Параметры камеры. Раньше были захардкожены в updateCamera().
    struct CameraConfig
    {
        glm::vec3 position{0.0f, 1.25f, 3.6f};
        glm::vec3 target{0.0f, 1.15f, 0.0f};
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        float fovDegrees = 45.0f;
        float nearPlane = 0.01f;
        float farPlane = 100.0f;
    };

    // Палитра процедурного фона. Раньше восемь цветов задавались прямо
    // в beginFrame(); теперь это настраиваемое состояние рендерера.
    struct BackgroundPalette
    {
        glm::vec3 primary{0.761f, 0.541f, 0.353f};
        glm::vec3 forest{0.290f, 0.180f, 0.141f};
        glm::vec3 gold{0.651f, 0.416f, 0.247f};
        glm::vec3 accent{0.482f, 0.247f, 0.196f};
        glm::vec3 sky{0.749f, 0.639f, 0.541f};
        glm::vec3 background{0.910f, 0.847f, 0.769f};
        glm::vec3 surface{0.420f, 0.275f, 0.196f};
        glm::vec3 border{0.169f, 0.106f, 0.082f};
    };

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

        void setCamera(const CameraConfig& camera);
        void setBackgroundPalette(const BackgroundPalette& palette);

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

        CameraConfig camera_{};
        BackgroundPalette palette_{};

        glm::mat4 view_{1.0f};
        glm::mat4 projection_{1.0f};

        void updateCamera();
        void releaseBackground();
    };
}