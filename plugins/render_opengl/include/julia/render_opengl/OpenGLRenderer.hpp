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
        ~OpenGLRenderer();

        OpenGLRenderer(const OpenGLRenderer&) = delete;
        OpenGLRenderer& operator=(const OpenGLRenderer&) = delete;

        OpenGLRenderer(OpenGLRenderer&&) = delete;
        OpenGLRenderer& operator=(OpenGLRenderer&&) = delete;

        void initialize(int width, int height,
                        const std::filesystem::path& assetDirectory = {});
        void resize(int width, int height);

        void setCamera(const CameraConfig& camera);
        void beginFrame();

        // Requires the initialized context to remain current through destruction.
        // Empty palette draws baked static geometry. A nonempty palette contains
        // final scene-space skinning transforms for every entry in model.bones()
        // (e.g. model.skinningMatrices()). The device TBO limit is checked.
        // Draw opaque models before models containing blended surfaces; sorting
        // within one model is by mesh center, not per pixel or across models.
        void renderModel(
            const GLModel& model,
            const glm::mat4& modelMatrix,
            std::span<const glm::mat4> boneMatrices = {}
        );

    private:
        GLShader shader_;
        GLuint boneBuffer_ = 0;
        GLuint boneTexture_ = 0;
        GLint maxBoneTexels_ = 0;
        bool boneStorageInitialized_ = false;

        int width_ = 1280;
        int height_ = 720;

        CameraConfig camera_{};

        glm::mat4 view_{1.0f};
        glm::mat4 projection_{1.0f};

        void updateCamera();
    };
}
