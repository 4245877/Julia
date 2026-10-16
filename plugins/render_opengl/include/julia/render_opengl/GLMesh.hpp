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

        // Four strongest normalized influences, shared with model.vert.
        glm::ivec4 boneIds{-1, -1, -1, -1};
        glm::vec4 boneWeights{0.0f};
        // Geometric outline normal, welded across UV and shading-normal seams.
        // Filled once by GLMesh; authored lighting normals remain untouched.
        glm::vec3 outlineNormal{0.0f};
    };

    enum class AlphaMode { Opaque, Mask, Blend };

    // Shader contract: Generic=0, Skin=1, Hair=2, Eye=3, Metal=4,
    // Fabric=5, Leather=6, Ink=7. Profiles affect lighting, never base color.
    enum class ToonMaterialKind
    {
        Generic = 0, Skin = 1, Hair = 2, Eye = 3,
        Metal = 4, Fabric = 5, Leather = 6, Ink = 7
    };

    struct GLMeshMaterial
    {
        glm::vec3 diffuseColor{0.8f, 0.8f, 0.8f};
        std::shared_ptr<GLTexture> diffuseTexture;
        float lightResponse = 0.80f;
        float specularStrength = 0.004f;
        float rimStrength = 0.032f;
        float opacity = 1.0f;
        float alphaCutoff = 0.5f;
        AlphaMode alphaMode = AlphaMode::Opaque;
        // Sampling belongs to the material: one cached image can be shared by
        // materials with different wrap/filter settings.
        GLenum wrapS = GL_REPEAT;
        GLenum wrapT = GL_REPEAT;
        GLenum minFilter = GL_LINEAR_MIPMAP_LINEAR;
        GLenum magFilter = GL_LINEAR;
        ToonMaterialKind toonKind = ToonMaterialKind::Generic;
        float specularPower = 80.0f;
        // Inverted hull is unsuitable for thin eye/decal/ink layers.
        float outlineScale = 1.0f;
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
