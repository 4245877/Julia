#pragma once

#include "julia/render_opengl/Bounds.hpp"
#include "julia/render_opengl/GLMesh.hpp"
#include "julia/render_opengl/Animation.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace julia::render_opengl
{
    class GLShader;

    inline constexpr std::uint32_t NoModelNode = animation::NoNode;
    using GLModelNode = animation::SkeletonNode;
    using GLBoneInfo = animation::SkinBinding;

    class GLModel
    {
    public:
        GLModel();
        explicit GLModel(std::shared_ptr<const model::ModelAsset> asset);
        const model::ModelAsset& asset() const { return *asset_; }
        const auto& assetHandle() const { return asset_; }
        operator const model::ModelAsset&() const { return *asset_; }
        ~GLModel() = default;

        GLModel(const GLModel&) = delete;
        GLModel& operator=(const GLModel&) = delete;

        GLModel(GLModel&&) noexcept = default;
        GLModel& operator=(GLModel&&) noexcept = default;

        // Procedural construction only; imported assets are immutable.
        // Nodes supplied here must include bindTRS for sparse animation.
        void addMesh(GLMesh&& mesh);
        std::uint32_t addNode(GLModelNode node);
        std::uint32_t addBone(GLBoneInfo bone);
        void addAnimation(AnimationClip clip);
        const std::vector<AnimationClip>& animations() const { return asset_->animations(); }

        const animation::Skeleton& skeleton() const { return asset_->skeleton(); }

        void draw(const GLShader& shader) const;
        void drawOpaque(const GLShader& shader, MeshPass pass = MeshPass::Fill) const;
        void drawOutline(const GLShader& shader) const;
        void drawTransparent(const GLShader& shader, const glm::mat4& viewModel) const;

        // Produce final matrices expected by renderModel(). An empty argument
        // uses the bind pose; otherwise provide one local matrix for every node.
        // Animation sampling belongs to the caller. No inverse-root transform is
        // applied because mesh positions already contain the imported root.
        std::vector<glm::mat4> skinningMatrices(
            std::span<const glm::mat4> localNodeTransforms = {}) const;
        // Allocation-free variant; scratch and output must be distinct and sized
        // to nodes()/bones(). Retains the same scene-space palette convention.
        void skinningMatrices(std::span<const glm::mat4> localNodeTransforms,
            std::span<glm::mat4> globals, std::span<glm::mat4> palette) const;

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
            return asset_->bones().size();
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
            return asset_->bones();
        }

        const std::vector<GLModelNode>& nodes() const
        {
            return asset_->nodes();
        }

        // Удобно для первого запуска: модель центрируется и масштабируется.
        glm::mat4 fitToHeightTransform(float targetHeight = 2.2f) const;

    private:
        std::vector<GLMesh> meshes_;
        std::shared_ptr<model::ModelAsset> builder_;
        std::shared_ptr<const model::ModelAsset> asset_;
        model::ModelAsset& builder();
        Bounds bounds_{};
    };
}
