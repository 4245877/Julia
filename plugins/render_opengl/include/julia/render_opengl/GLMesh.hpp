#pragma once

#include "julia/render_opengl/Bounds.hpp"
#include "julia/render_opengl/GLTexture.hpp"

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace julia::render_opengl
{
    class GLShader;

    struct GLVertex
    {
        glm::vec3 position{0.0f};
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        glm::vec2 texCoord{0.0f};

        // Уже сейчас храним влияние костей.
        // Рендер не управляет телом, он только принимает готовую позу.
        glm::ivec4 boneIds{-1, -1, -1, -1};
        glm::vec4 boneWeights{0.0f};
    };

    struct GLMeshMaterial
    {
        glm::vec3 diffuseColor{0.8f, 0.8f, 0.8f};
        std::shared_ptr<GLTexture> diffuseTexture;
    };

    class GLMesh
    {
    public:
        GLMesh() = default;

        GLMesh(
            std::vector<GLVertex> vertices,
            std::vector<std::uint32_t> indices,
            GLMeshMaterial material
        );

        ~GLMesh();

        GLMesh(const GLMesh&) = delete;
        GLMesh& operator=(const GLMesh&) = delete;

        GLMesh(GLMesh&& other) noexcept;
        GLMesh& operator=(GLMesh&& other) noexcept;

        void draw(const GLShader& shader) const;

        const Bounds& bounds() const
        {
            return bounds_;
        }

        std::size_t indexCount() const
        {
            return indexCount_;
        }

    private:
        GLuint vao_ = 0;
        GLuint vbo_ = 0;
        GLuint ebo_ = 0;

        std::size_t indexCount_ = 0;
        GLMeshMaterial material_{};
        Bounds bounds_{};

        void upload(
            const std::vector<GLVertex>& vertices,
            const std::vector<std::uint32_t>& indices
        );

        void release();
    };
}