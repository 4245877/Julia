#pragma once

#include "julia/render_opengl/GLModel.hpp"
#include "julia/render_opengl/GLShader.hpp"

#include <glm/glm.hpp>

#include <span>
#include <filesystem>

namespace julia::render_opengl
{
    // Параметры камеры. Раньше были захардкожены в updateCamera().
    struct CameraConfig
    {
        glm::vec3 position{0.0f, 1.25f, 3.6f};
        glm::vec3 target{0.0f, 1.15f, 0.0f};
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        float fovDegrees = 45.0f;
        // Kept well away from far to preserve depth-buffer precision; the
        // subject sits a few metres from the camera, so 0.1 clips nothing.
        float nearPlane = 0.1f;
        float farPlane = 100.0f;
    };

    class OpenGLRenderer
    {
    public:
        OpenGLRenderer() = default;
        ~OpenGLRenderer() = default;

        OpenGLRenderer(const OpenGLRenderer&) = delete;
        OpenGLRenderer& operator=(const OpenGLRenderer&) = delete;

        OpenGLRenderer(OpenGLRenderer&&) = delete;
        OpenGLRenderer& operator=(OpenGLRenderer&&) = delete;

        void initialize(int width, int height,
                        const std::filesystem::path& assetDirectory = {});
        void resize(int width, int height);

        void setCamera(const CameraConfig& camera);
        void beginFrame();

        // Рендер получает готовую позу в виде boneMatrices.
        // Если boneMatrices пустой — модель рисуется в bind/static pose.
        void renderModel(
            const GLModel& model,
            const glm::mat4& modelMatrix,
            std::span<const glm::mat4> boneMatrices = {}
        );

    private:
        GLShader shader_;

        int width_ = 1280;
        int height_ = 720;

        CameraConfig camera_{};

        glm::mat4 view_{1.0f};
        glm::mat4 projection_{1.0f};

        void updateCamera();
    };
}
