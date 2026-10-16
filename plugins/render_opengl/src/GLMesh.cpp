#include "julia/render_opengl/GLMesh.hpp"

#include "julia/render_opengl/GLShader.hpp"
#include "julia/render_opengl/OutlineNormals.hpp"

#include <cstddef>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace julia::render_opengl
{
    static_assert(std::is_standard_layout_v<GLVertex>);
    static_assert(sizeof(glm::ivec4::value_type) == sizeof(GLint));
    static_assert(sizeof(GLVertex) <= static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()));

    namespace
    {
        bool validWrap(GLenum wrap)
        {
            return wrap == GL_REPEAT || wrap == GL_MIRRORED_REPEAT ||
                wrap == GL_CLAMP_TO_EDGE || wrap == GL_CLAMP_TO_BORDER;
        }

        bool validMinFilter(GLenum filter)
        {
            return filter == GL_NEAREST || filter == GL_LINEAR ||
                filter == GL_NEAREST_MIPMAP_NEAREST || filter == GL_LINEAR_MIPMAP_NEAREST ||
                filter == GL_NEAREST_MIPMAP_LINEAR || filter == GL_LINEAR_MIPMAP_LINEAR;
        }

        class MeshUploadState
        {
        public:
            MeshUploadState()
            {
                glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao_);
                glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &arrayBuffer_);
            }

            ~MeshUploadState()
            {
                // The element buffer is owned by its VAO; rebinding the previous
                // VAO restores it without modifying any other mesh's index data.
                glBindVertexArray(static_cast<GLuint>(vao_));
                glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(arrayBuffer_));
            }

        private:
            GLint vao_ = 0;
            GLint arrayBuffer_ = 0;
        };
    }

    GLMesh::GLMesh(
        std::vector<GLVertex> vertices,
        std::vector<std::uint32_t> indices,
        GLMeshMaterial material
    )
        : indexCount_(indices.size()),
          material_(std::move(material))
    {
        for (const auto& vertex : vertices)
        {
            for (int component = 0; component < 3; ++component)
                if (!std::isfinite(vertex.position[component]) || !std::isfinite(vertex.normal[component]))
                    throw std::invalid_argument("Mesh position/normal contains non-finite values");
            for (int component = 0; component < 2; ++component)
                if (!std::isfinite(vertex.texCoord[component]))
                    throw std::invalid_argument("Mesh UV contains non-finite values");
            for (int component = 0; component < 4; ++component)
                if (!std::isfinite(vertex.boneWeights[component]) || vertex.boneWeights[component] < 0.0f)
                    throw std::invalid_argument("Mesh has a non-finite/negative bone weight");
            bounds_.expand(vertex.position);
        }
        for (int component = 0; component < 3; ++component)
            if (!std::isfinite(material_.diffuseColor[component]))
                throw std::invalid_argument("Mesh material has a non-finite diffuse color");
        if (!std::isfinite(material_.opacity) || !std::isfinite(material_.alphaCutoff) ||
            !std::isfinite(material_.lightResponse) || !std::isfinite(material_.specularStrength) ||
            !std::isfinite(material_.rimStrength) || !std::isfinite(material_.specularPower) ||
            !std::isfinite(material_.outlineScale))
            throw std::invalid_argument("Mesh material has non-finite parameters");
        if (material_.specularPower <= 0.0f || material_.outlineScale < 0.0f ||
            static_cast<int>(material_.toonKind) < static_cast<int>(ToonMaterialKind::Generic) ||
            static_cast<int>(material_.toonKind) > static_cast<int>(ToonMaterialKind::Ink))
            throw std::invalid_argument("Mesh material has invalid toon parameters");

        const auto outlineNormals = buildOutlineNormals(vertices, indices);
        for (std::size_t i = 0; i < vertices.size(); ++i)
            vertices[i].outlineNormal = outlineNormals[i];
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
          sampler_(std::exchange(other.sampler_, 0)),
          indexCount_(other.indexCount_),
          material_(std::move(other.material_)),
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
            sampler_ = std::exchange(other.sampler_, 0);
            indexCount_ = other.indexCount_;
            material_ = std::move(other.material_);
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
        if (vao_ == 0 || indexCount_ == 0)
            return;

        shader.setVec3("uDiffuseColor", material_.diffuseColor);
        shader.setFloat("uLightResponse", material_.lightResponse);
        shader.setFloat("uSpecularStrength", material_.specularStrength);
        shader.setFloat("uRimStrength", material_.rimStrength);
        shader.setInt("uMaterialKind", static_cast<int>(material_.toonKind));
        shader.setFloat("uSpecularPower", material_.specularPower);
        shader.setFloat("uOutlineScale", material_.outlineScale);
        shader.setFloat("uOpacity", material_.opacity);
        shader.setFloat("uAlphaCutoff", material_.alphaCutoff);
        shader.setInt("uAlphaMode", static_cast<int>(material_.alphaMode));

        const bool hasDiffuseTexture =
            material_.diffuseTexture &&
            material_.diffuseTexture->valid();

        shader.setBool("uHasDiffuseTexture", hasDiffuseTexture);

        if (hasDiffuseTexture)
        {
            shader.setInt("uDiffuseTexture", 0);
            // Unit zero is guaranteed by OpenGL 3.3. Avoid a device-limit query
            // for every mesh and pass; GLTexture::bind checks arbitrary units.
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, material_.diffuseTexture->id());
            glBindSampler(0, sampler_);
        }

        glBindVertexArray(vao_);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indexCount_), GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);

        if (hasDiffuseTexture)
        {
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }

    void GLMesh::upload(
        const std::vector<GLVertex>& vertices,
        const std::vector<std::uint32_t>& indices
    )
    {
        if (indices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) ||
            vertices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(GLVertex) ||
            indices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(std::uint32_t))
            throw std::length_error("Mesh exceeds OpenGL buffer or draw count limits");
        if (indices.size() % 3 != 0)
            throw std::invalid_argument("Triangle mesh index count must be divisible by three");
        for (const auto index : indices)
            if (index >= vertices.size())
                throw std::out_of_range("Mesh index references a missing vertex");
        if (indices.empty())
            return;
        const bool textured = material_.diffuseTexture && material_.diffuseTexture->valid();
        if (textured && (!validWrap(material_.wrapS) || !validWrap(material_.wrapT) ||
            !validMinFilter(material_.minFilter) ||
            (material_.magFilter != GL_NEAREST && material_.magFilter != GL_LINEAR)))
            throw std::invalid_argument("Invalid material texture sampler settings");

        MeshUploadState state;
        try
        {
            glGenVertexArrays(1, &vao_);
            glGenBuffers(1, &vbo_);
            glGenBuffers(1, &ebo_);
            if (vao_ == 0 || vbo_ == 0 || ebo_ == 0)
                throw std::runtime_error("Failed to allocate OpenGL mesh objects");
            if (textured)
            {
                glGenSamplers(1, &sampler_);
                if (sampler_ == 0)
                    throw std::runtime_error("Failed to allocate OpenGL material sampler");
                // Sampler-object parameter calls do not change any unit binding.
                glSamplerParameteri(sampler_, GL_TEXTURE_WRAP_S, static_cast<GLint>(material_.wrapS));
                glSamplerParameteri(sampler_, GL_TEXTURE_WRAP_T, static_cast<GLint>(material_.wrapT));
                glSamplerParameteri(sampler_, GL_TEXTURE_MIN_FILTER, static_cast<GLint>(material_.minFilter));
                glSamplerParameteri(sampler_, GL_TEXTURE_MAG_FILTER, static_cast<GLint>(material_.magFilter));
                // Assimp's Decal mode leaves the diffuse factor unchanged outside
                // the texture rectangle, which requires white in a multiply shader.
                constexpr GLfloat border[]{1.0f, 1.0f, 1.0f, 1.0f};
                glSamplerParameterfv(sampler_, GL_TEXTURE_BORDER_COLOR, border);
            }

            glBindVertexArray(vao_);

            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
            glBufferData(
                GL_ARRAY_BUFFER,
                static_cast<GLsizeiptr>(vertices.size() * sizeof(GLVertex)),
                vertices.data(),
                GL_STATIC_DRAW
            );
            GLint64 vertexBufferSize = 0;
            glGetBufferParameteri64v(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &vertexBufferSize);
            if (vertexBufferSize != static_cast<GLint64>(vertices.size() * sizeof(GLVertex)))
                throw std::runtime_error("Failed to allocate OpenGL vertex buffer storage");

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
            glBufferData(
                GL_ELEMENT_ARRAY_BUFFER,
                static_cast<GLsizeiptr>(indices.size() * sizeof(std::uint32_t)),
                indices.data(),
                GL_STATIC_DRAW
            );
            GLint64 indexBufferSize = 0;
            glGetBufferParameteri64v(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &indexBufferSize);
            if (indexBufferSize != static_cast<GLint64>(indices.size() * sizeof(std::uint32_t)))
                throw std::runtime_error("Failed to allocate OpenGL index buffer storage");

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

            glEnableVertexAttribArray(5);
            glVertexAttribPointer(
                5,
                3,
                GL_FLOAT,
                GL_FALSE,
                sizeof(GLVertex),
                reinterpret_cast<void*>(offsetof(GLVertex, outlineNormal))
            );

        }
        catch (...)
        {
            // A constructor that throws will not run ~GLMesh.
            release();
            throw;
        }
    }

    void GLMesh::release()
    {
        if (sampler_ != 0)
        {
            glDeleteSamplers(1, &sampler_);
            sampler_ = 0;
        }
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
