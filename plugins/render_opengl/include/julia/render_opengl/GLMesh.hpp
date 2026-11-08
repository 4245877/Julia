#pragma once

#include "julia/render_opengl/Bounds.hpp"
#include "julia/model/Geometry.hpp"
#include "julia/render_opengl/GLTexture.hpp"

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace julia::render_opengl
{
    class GLShader;

    using GLVertex = model::Vertex;
    using model::AlphaMode;
    using model::ToonMaterialKind;
    enum class MeshPass { Fill, Shadow, Outline };
    struct GLMeshMaterial : model::MaterialProperties
    {
        std::shared_ptr<GLTexture> diffuseTexture;
        GLenum wrapS = GL_REPEAT, wrapT = GL_REPEAT;
        GLenum minFilter = GL_LINEAR_MIPMAP_LINEAR, magFilter = GL_LINEAR;
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

        void draw(const GLShader& shader, MeshPass pass = MeshPass::Fill) const;

        const Bounds& bounds() const
        {
            return bounds_;
        }

        std::size_t indexCount() const
        {
            return indexCount_;
        }

        const GLMeshMaterial& material() const
        {
            return material_;
        }

        bool transparent() const
        {
            return material_.alphaMode == AlphaMode::Blend;
        }

    private:
        GLuint vao_ = 0;
        GLuint vbo_ = 0;
        GLuint ebo_ = 0;
        GLuint sampler_ = 0;

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
