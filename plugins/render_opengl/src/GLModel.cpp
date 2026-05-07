#include "julia/render_opengl/GLModel.hpp"

#include "julia/render_opengl/GLShader.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

namespace julia::render_opengl
{
    void GLModel::addMesh(GLMesh&& mesh)
    {
        bounds_.expand(mesh.bounds());
        meshes_.push_back(std::move(mesh));
    }

    std::uint32_t GLModel::addBone(GLBoneInfo bone)
    {
        bones_.push_back(std::move(bone));
        return static_cast<std::uint32_t>(bones_.size() - 1);
    }

    void GLModel::draw(const GLShader& shader) const
    {
        for (const auto& mesh : meshes_)
        {
            mesh.draw(shader);
        }
    }

    glm::mat4 GLModel::fitToHeightTransform(float targetHeight) const
    {
        if (!bounds_.valid)
        {
            return glm::mat4{1.0f};
        }

        const glm::vec3 size = bounds_.size();

        float sourceHeight = size.y;
        if (sourceHeight <= 0.0001f)
        {
            sourceHeight = std::max({size.x, size.y, size.z});
        }

        if (sourceHeight <= 0.0001f)
        {
            return glm::mat4{1.0f};
        }

        const float scale = targetHeight / sourceHeight;
        const glm::vec3 center = bounds_.center();

        // Центрируем по X/Z, а нижнюю точку ставим на Y=0.
        const glm::vec3 offset{
            -center.x,
            -bounds_.min.y,
            -center.z
        };

        glm::mat4 transform{1.0f};
        transform = glm::scale(transform, glm::vec3{scale});
        transform = glm::translate(transform, offset);

        return transform;
    }
}