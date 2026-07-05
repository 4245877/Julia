#pragma once

#include "julia/render_opengl/Bounds.hpp"
#include "julia/render_opengl/GLMesh.hpp"

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace julia::render_opengl
{
    class GLShader;

    // Upper bound on skinning palette size. Must stay in sync with MAX_BONES in
    // assets/shaders/model.vert; influences referencing a bone at or above this
    // index are dropped by the vertex shader.
    inline constexpr std::uint32_t MaxBones = 128;

    struct GLBoneInfo
    {
        std::string name;

        // Inverse bind matrix / offset matrix из Assimp.
        // Body/animation-модуль позже будет использовать это для skinning.
        glm::mat4 offsetMatrix{1.0f};
    };

    class GLModel
    {
    public:
        GLModel() = default;
        ~GLModel() = default;

        GLModel(const GLModel&) = delete;
        GLModel& operator=(const GLModel&) = delete;

        GLModel(GLModel&&) noexcept = default;
        GLModel& operator=(GLModel&&) noexcept = default;

        void addMesh(GLMesh&& mesh);
        std::uint32_t addBone(GLBoneInfo bone);

        void draw(const GLShader& shader) const;

        bool empty() const
        {
            return meshes_.empty();
        }

        std::size_t meshCount() const
        {
            return meshes_.size();
        }

        std::size_t boneCount() const
        {
            return bones_.size();
        }

        const Bounds& bounds() const
        {
            return bounds_;
        }

        const std::vector<GLBoneInfo>& bones() const
        {
            return bones_;
        }

        // Удобно для первого запуска: модель центрируется и масштабируется.
        glm::mat4 fitToHeightTransform(float targetHeight = 2.2f) const;

    private:
        std::vector<GLMesh> meshes_;
        std::vector<GLBoneInfo> bones_;
        Bounds bounds_{};
    };
}