#include "julia/render_opengl/GLMesh.hpp"

#include "julia/render_opengl/GLShader.hpp"

#include <cstddef>
#include <utility>

namespace julia::render_opengl
{
    GLMesh::GLMesh(
        std::vector<GLVertex> vertices,
        std::vector<std::uint32_t> indices,
        GLMeshMaterial material
    )
        : indexCount_(indices.size()),
          material_(material)
    {
        for (const auto& vertex : vertices)
        {
            bounds_.expand(vertex.position);
        }

        upload(vertices, indices);
    }

    GLMesh::~GLMesh()
    {
        release();
    }

    GLMesh::GLMesh(GLMesh&& other) noexcept
        : vao_(other.vao_),
          vbo_(other.vbo_),
          ebo_(other.ebo_),
          indexCount_(other.indexCount_),
          material_(other.material_),
          bounds_(other.bounds_)
    {
        other.vao_ = 0;
        other.vbo_ = 0;
        other.ebo_ = 0;
        other.indexCount_ = 0;
    }

    GLMesh& GLMesh::operator=(GLMesh&& other) noexcept
    {
        if (this != &other)
        {
            release();

            vao_ = other.vao_;
            vbo_ = other.vbo_;
            ebo_ = other.ebo_;
            indexCount_ = other.indexCount_;
            material_ = other.material_;
            bounds_ = other.bounds_;

            other.vao_ = 0;
            other.vbo_ = 0;
            other.ebo_ = 0;
            other.indexCount_ = 0;
        }

        return *this;
    }

    void GLMesh::draw(const GLShader& shader) const
    {
        shader.setVec3("uDiffuseColor", material_.diffuseColor);

        glBindVertexArray(vao_);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indexCount_), GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    void GLMesh::upload(
        const std::vector<GLVertex>& vertices,
        const std::vector<std::uint32_t>& indices
    )
    {
        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &ebo_);

        glBindVertexArray(vao_);

        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(vertices.size() * sizeof(GLVertex)),
            vertices.data(),
            GL_STATIC_DRAW
        );

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(indices.size() * sizeof(std::uint32_t)),
            indices.data(),
            GL_STATIC_DRAW
        );

        // position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
            0,
            3,
            GL_FLOAT,
            GL_FALSE,
            sizeof(GLVertex),
            reinterpret_cast<void*>(offsetof(GLVertex, position))
        );

        // normal
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(
            1,
            3,
            GL_FLOAT,
            GL_FALSE,
            sizeof(GLVertex),
            reinterpret_cast<void*>(offsetof(GLVertex, normal))
        );

        // texCoord
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(
            2,
            2,
            GL_FLOAT,
            GL_FALSE,
            sizeof(GLVertex),
            reinterpret_cast<void*>(offsetof(GLVertex, texCoord))
        );

        // boneIds: integer attribute
        glEnableVertexAttribArray(3);
        glVertexAttribIPointer(
            3,
            4,
            GL_INT,
            sizeof(GLVertex),
            reinterpret_cast<void*>(offsetof(GLVertex, boneIds))
        );

        // boneWeights
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(
            4,
            4,
            GL_FLOAT,
            GL_FALSE,
            sizeof(GLVertex),
            reinterpret_cast<void*>(offsetof(GLVertex, boneWeights))
        );

        glBindVertexArray(0);
    }

    void GLMesh::release()
    {
        if (ebo_ != 0)
        {
            glDeleteBuffers(1, &ebo_);
            ebo_ = 0;
        }

        if (vbo_ != 0)
        {
            glDeleteBuffers(1, &vbo_);
            vbo_ = 0;
        }

        if (vao_ != 0)
        {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }
    }
}