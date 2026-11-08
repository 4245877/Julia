#pragma once
#include "julia/model/Geometry.hpp"
#include "julia/model/Image.hpp"
#include "julia/animation/Animation.hpp"
#include "julia/animation/Pose.hpp"
#include <memory>
#include <utility>

namespace julia::model
{
    // glTF sampler wire values, independent of graphics headers or context.
    namespace sampler
    {
        inline constexpr std::uint32_t Repeat = 10497, ClampToEdge = 33071,
            MirroredRepeat = 33648, ClampToBorder = 33069;
        inline constexpr std::uint32_t Nearest = 9728, Linear = 9729,
            NearestMipmapNearest = 9984, LinearMipmapNearest = 9985,
            NearestMipmapLinear = 9986, LinearMipmapLinear = 9987;
    }
    struct Material : MaterialProperties
    {
        std::shared_ptr<const Image> diffuseTexture;
        std::uint32_t wrapS = sampler::Repeat, wrapT = sampler::Repeat;
        std::uint32_t minFilter = sampler::LinearMipmapLinear, magFilter = sampler::Linear;
    };
    struct Mesh
    {
        std::vector<Vertex> vertices;
        std::vector<std::uint32_t> indices;
        Material material;
    };
    // Build before publishing as shared_ptr<const ModelAsset>. All data is owned.
    // Instances own Pose/clocks. addNode requires importer-provided bindTRS;
    // it never decomposes/recomposes the exact bind matrix.
    class ModelAsset
    {
    public:
        void addMesh(Mesh mesh);
        std::uint32_t addNode(animation::SkeletonNode node) { return skeleton_.addNode(std::move(node)); }
        std::uint32_t addBone(animation::SkinBinding binding) { return skeleton_.addBinding(std::move(binding)); }
        void addAnimation(animation::AnimationClip clip);
        const animation::Skeleton& skeleton() const { return skeleton_; }
        const auto& nodes() const { return skeleton_.nodes(); }
        const auto& bones() const { return skeleton_.bindings(); }
        const auto& animations() const { return clips_; }
        const auto& meshes() const { return meshes_; }
        const Bounds& bounds() const { return bounds_; }
        bool empty() const { return meshes_.empty(); }
        std::size_t meshCount() const { return meshes_.size(); }
        std::size_t boneCount() const { return bones().size(); }
        std::vector<glm::mat4> skinningMatrices(std::span<const glm::mat4> locals = {}) const;
        void skinningMatrices(std::span<const glm::mat4> locals,
            std::span<glm::mat4> globals, std::span<glm::mat4> palette) const;
        glm::mat4 fitToHeightTransform(float targetHeight = 2.2f) const;
    private:
        animation::Skeleton skeleton_;
        std::vector<animation::AnimationClip> clips_;
        std::vector<Mesh> meshes_;
        Bounds bounds_;
    };
}
