#include "julia/render_opengl/OpenGLRenderer.hpp"

#include <glad/glad.h>

#include <algorithm>
#include <array>
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
        constexpr int MaxBones = 128;

        std::filesystem::path getAssetDir()
        {
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

        std::string readShaderFile(const std::filesystem::path& shaderFileName)
        {
            const std::filesystem::path fullPath =
                getAssetDir() / "shaders" / shaderFileName;

            return readTextFile(fullPath);
        }
    }

    OpenGLRenderer::~OpenGLRenderer()
    {
        releaseBackground();
    }

    void OpenGLRenderer::initialize(int width, int height)
    {
        width_ = width;
        height_ = height;

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        glEnable(GL_MULTISAMPLE);
        glEnable(GL_DITHER);

        // Пока culling лучше не включать:
        // у некоторых FBX бывают неожиданные winding/axis.
        glDisable(GL_CULL_FACE);

        const std::string modelVertexShaderSource =
            readShaderFile("model.vert");

        const std::string modelFragmentShaderSource =
            readShaderFile("model.frag");

        const std::string backgroundVertexShaderSource =
            readShaderFile("background.vert");

        const std::string backgroundFragmentShaderSource =
            readShaderFile("background.frag");

        shader_.createFromSource(
            modelVertexShaderSource.c_str(),
            modelFragmentShaderSource.c_str()
        );

        backgroundShader_.createFromSource(
            backgroundVertexShaderSource.c_str(),
            backgroundFragmentShaderSource.c_str()
        );

        glGenVertexArrays(1, &backgroundVao_);

        resize(width_, height_);
    }

    void OpenGLRenderer::resize(int width, int height)
    {
        width_ = std::max(width, 1);
        height_ = std::max(height, 1);

        glViewport(0, 0, width_, height_);

        updateCamera();
    }

    void OpenGLRenderer::beginFrame(const glm::vec3& clearColor)
    {
        glClearColor(clearColor.r, clearColor.g, clearColor.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);

        backgroundShader_.bind();

        backgroundShader_.setVec3("uPrimaryColor", glm::vec3{ 0.761f, 0.541f, 0.353f }); // Amber Hair      #C28A5A
        backgroundShader_.setVec3("uForestColor", glm::vec3{ 0.290f, 0.180f, 0.141f }); // Shadow Bark     #4A2E24
        backgroundShader_.setVec3("uGoldColor", glm::vec3{ 0.651f, 0.416f, 0.247f }); // Earth Leather   #A66A3F
        backgroundShader_.setVec3("uAccentColor", glm::vec3{ 0.482f, 0.247f, 0.196f }); // Heart Chestnut  #7B3F32
        backgroundShader_.setVec3("uSkyColor", glm::vec3{ 0.749f, 0.639f, 0.541f }); // Warm Sand       #BFA38A
        backgroundShader_.setVec3("uBgColor", glm::vec3{ 0.910f, 0.847f, 0.769f }); // Soft Linen      #E8D8C4
        backgroundShader_.setVec3("uSurfaceColor", glm::vec3{ 0.420f, 0.275f, 0.196f }); // Deep Bark       #6B4632
        backgroundShader_.setVec3("uBorderColor", glm::vec3{ 0.169f, 0.106f, 0.082f }); // Dark Umber      #2B1B15



        glBindVertexArray(backgroundVao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
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

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        model.draw(shader_);

        glDisable(GL_BLEND);

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
            glm::radians(45.0f),
            aspect,
            0.01f,
            100.0f
        );

        const glm::vec3 cameraPosition{ 0.0f, 1.25f, 3.6f };
        const glm::vec3 cameraTarget{ 0.0f, 1.15f, 0.0f };
        const glm::vec3 cameraUp{ 0.0f, 1.0f, 0.0f };

        view_ = glm::lookAt(cameraPosition, cameraTarget, cameraUp);
    }

    void OpenGLRenderer::releaseBackground()
    {
        if (backgroundVao_ != 0)
        {
            glDeleteVertexArrays(1, &backgroundVao_);
            backgroundVao_ = 0;
        }
    }
}