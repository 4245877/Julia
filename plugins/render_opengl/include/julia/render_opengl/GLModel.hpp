#pragma once

#include "julia/render_opengl/Bounds.hpp"
#include "julia/render_opengl/GLMesh.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace julia::render_opengl
{
    class GLShader;

    inline constexpr std::uint32_t NoModelNode = std::numeric_limits<std::uint32_t>::max();

    // Nodes are stored in parent-before-child order. Transforms include the
    // imported root transform: vertices and skinning palettes share scene space.
    struct GLModelNode
    {
        std::string name;
        std::uint32_t parent = NoModelNode;
        glm::mat4 localTransform{1.0f};
        glm::mat4 globalBindTransform{1.0f};
    };

    struct GLBoneInfo
    {
        std::string name;

        // Assimp mesh-local inverse bind, adjusted for the node transform
        // already baked into vertices. A node can have multiple mesh offsets.
        glm::mat4 offsetMatrix{1.0f};
        std::uint32_t nodeIndex = NoModelNode;
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
        std::uint32_t addNode(GLModelNode node);
        std::uint32_t addBone(GLBoneInfo bone);

        void draw(const GLShader& shader) const;
        void drawOpaque(const GLShader& shader) const;
        void drawOutline(const GLShader& shader) const;
        void drawTransparent(const GLShader& shader, const glm::mat4& viewModel) const;

        // Produce final matrices expected by renderModel(). An empty argument
        // uses the bind pose; otherwise provide one local matrix for every node.
        // Animation sampling belongs to the caller. No inverse-root transform is
        // applied because mesh positions already contain the imported root.
        std::vector<glm::mat4> skinningMatrices(
            std::span<const glm::mat4> localNodeTransforms = {}) const;

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

        const std::vector<GLMesh>& meshes() const
        {
            return meshes_;
        }

        const std::vector<GLBoneInfo>& bones() const
        {
            return bones_;
        }

        const std::vector<GLModelNode>& nodes() const
        {
            return nodes_;
        }

        // Удобно для первого запуска: модель центрируется и масштабируется.
        glm::mat4 fitToHeightTransform(float targetHeight = 2.2f) const;

    private:
        std::vector<GLMesh> meshes_;
        std::vector<GLModelNode> nodes_;
        std::vector<GLBoneInfo> bones_;
        Bounds bounds_{};
    };
}
