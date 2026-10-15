#include "julia/render_opengl/OpenGLRenderer.hpp"

#include <glad/glad.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <glm/gtc/matrix_transform.hpp>

namespace julia::render_opengl
{
    namespace
    {
        std::filesystem::path getAssetDir(const std::filesystem::path& requested)
        {
            if (!requested.empty())
                return requested;
#ifdef JULIA_ASSET_DIR
            const std::filesystem::path assetDir = JULIA_ASSET_DIR;

            if (std::filesystem::exists(assetDir / "shaders"))
            {
                return assetDir;
            }

            throw std::runtime_error(
                "Invalid JULIA_ASSET_DIR: " + assetDir.string() +
                ". Expected directory: shaders"
            );
#else
            throw std::runtime_error(
                "JULIA_ASSET_DIR is not defined. Cannot locate assets directory."
            );
#endif
        }

        std::string readTextFile(const std::filesystem::path& path)
        {
            std::ifstream file(path, std::ios::in);

            if (!file.is_open())
            {
                throw std::runtime_error("Failed to open file: " + path.string());
            }

            std::stringstream buffer;
            buffer << file.rdbuf();

            return buffer.str();
        }

        std::string readShaderFile(const std::filesystem::path& assetDir,
                                  const std::filesystem::path& shaderFileName)
        {
            const std::filesystem::path fullPath =
                assetDir / "shaders" / shaderFileName;

            return readTextFile(fullPath);
        }
    }

    void OpenGLRenderer::initialize(int width, int height,
                                   const std::filesystem::path& assetDirectory)
    {
        width_ = width;
        height_ = height;

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        glEnable(GL_MULTISAMPLE);
        glEnable(GL_DITHER);
        glEnable(GL_FRAMEBUFFER_SRGB);

        // Пока culling лучше не включать:
        // у некоторых FBX бывают неожиданные winding/axis.
        glDisable(GL_CULL_FACE);

        const auto assetDir = getAssetDir(assetDirectory);
        const std::string modelVertexShaderSource =
            readShaderFile(assetDir, "model.vert");

        const std::string modelFragmentShaderSource =
            readShaderFile(assetDir, "model.frag");

        shader_.createFromSource(
            modelVertexShaderSource.c_str(),
            modelFragmentShaderSource.c_str()
        );

        resize(width_, height_);
    }

    void OpenGLRenderer::setCamera(const CameraConfig& camera)
    {
        camera_ = camera;
        updateCamera();
    }

    void OpenGLRenderer::resize(int width, int height)
    {
        width_ = std::max(width, 1);
        height_ = std::max(height, 1);

        glViewport(0, 0, width_, height_);

        updateCamera();
    }

    void OpenGLRenderer::beginFrame()
    {
        // Linear 0.21586 maps to neutral sRGB #808080. Clearing a buffer needs
        // no geometry, texture, shader, camera transform, or visual effects.
        glClearColor(0.21586f, 0.21586f, 0.21586f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void OpenGLRenderer::renderModel(
        const GLModel& model,
        const glm::mat4& modelMatrix,
        std::span<const glm::mat4> boneMatrices
    )
    {
        shader_.bind();

        shader_.setMat4("uModel", modelMatrix);
        shader_.setMat4("uView", view_);
        shader_.setMat4("uProjection", projection_);
        shader_.setVec3("uViewPos", camera_.position);

        const bool useSkinning = !boneMatrices.empty();

        shader_.setBool("uUseSkinning", useSkinning);

        if (useSkinning)
        {
            const std::size_t count = std::min<std::size_t>(boneMatrices.size(), MaxBones);
            shader_.setMat4Array("uBoneMatrices", boneMatrices.subspan(0, count));
        }

        // 1) Тонкий контур: рисуем слегка расширенную оболочку.
        shader_.setBool("uOutlinePass", true);

        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        glDepthMask(GL_FALSE);

        // The outline fragment shader always writes alpha = 1.0, so blending
        // would be a no-op; the inverted hull is a plain opaque draw.
        model.draw(shader_);

        glDepthMask(GL_TRUE);
        glCullFace(GL_BACK);
        glDisable(GL_CULL_FACE);

        // 2) Основная мягкая anime-заливка.
        shader_.setBool("uOutlinePass", false);

        model.draw(shader_);
    }

    void OpenGLRenderer::updateCamera()
    {
        const float aspect =
            static_cast<float>(width_) /
            static_cast<float>(std::max(height_, 1));

        projection_ = glm::perspective(
            glm::radians(camera_.fovDegrees),
            aspect,
            camera_.nearPlane,
            camera_.farPlane
        );

        view_ = glm::lookAt(camera_.position, camera_.target, camera_.up);
    }

}
