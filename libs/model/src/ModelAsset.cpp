#include "julia/model/ModelAsset.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::model
{
    namespace
    {
        bool finite(const glm::mat4& matrix)
        {
            for (int column = 0; column < 4; ++column)
                for (int row = 0; row < 4; ++row)
                    if (!std::isfinite(matrix[column][row])) return false;
            return true;
        }
    }
    void ModelAsset::addAnimation(animation::AnimationClip clip)
    {
        animation::prepareAnimationClip(clip, skeleton_);
        clips_.push_back(std::move(clip));
    }
    void ModelAsset::addMesh(Mesh mesh)
    {
        if (mesh.indices.size() % 3 != 0) throw std::invalid_argument("Mesh indices must be triangles");
        for (auto index : mesh.indices)
            if (index >= mesh.vertices.size()) throw std::invalid_argument("Invalid mesh vertex index");
        Bounds bounds;
        for (const auto& v : mesh.vertices)
        {
            for (int c = 0; c < 3; ++c)
                if (!std::isfinite(v.position[c]) || !std::isfinite(v.normal[c]))
                    throw std::invalid_argument("Non-finite mesh position/normal");
            for (int c = 0; c < 2; ++c)
                if (!std::isfinite(v.texCoord[c]))
                    throw std::invalid_argument("Non-finite mesh UV");
            for (int c = 0; c < 4; ++c)
                if (!std::isfinite(v.boneWeights[c]) || v.boneWeights[c] < 0 ||
                    (v.boneWeights[c] > 0 && (v.boneIds[c] < 0 ||
                        static_cast<std::size_t>(v.boneIds[c]) >= bones().size())))
                    throw std::invalid_argument("Invalid mesh skin influence");
            bounds.expand(v.position);
        }
        meshes_.push_back(std::move(mesh));
        bounds_.expand(bounds);
    }
    std::vector<glm::mat4> ModelAsset::skinningMatrices(
        std::span<const glm::mat4> localNodeTransforms) const
    {
        std::vector<glm::mat4> globals(nodes().size()), palette(bones().size());
        skinningMatrices(localNodeTransforms, globals, palette);
        return palette;
    }

    void ModelAsset::skinningMatrices(std::span<const glm::mat4> localNodeTransforms,
        std::span<glm::mat4> globalTransforms, std::span<glm::mat4> palette) const
    {
        animation::computeGlobalTransforms(skeleton_, localNodeTransforms, globalTransforms);
        animation::buildSkinningPalette(skeleton_, globalTransforms, palette);
    }

    glm::mat4 ModelAsset::fitToHeightTransform(float targetHeight) const
    {
        if (!std::isfinite(targetHeight) || targetHeight <= 0.0f)
            throw std::invalid_argument("Target model height must be finite and positive");
        if (!bounds_.valid)
        {
            return glm::mat4{1.0f};
        }

        const glm::vec3 size = bounds_.size();

        float sourceHeight = size.y;
        if (sourceHeight <= 0.0f)
        {
            sourceHeight = std::max({size.x, size.y, size.z});
        }

        if (sourceHeight <= 0.0f)
        {
            return glm::mat4{1.0f};
        }

        const float scale = targetHeight / sourceHeight;
        if (!std::isfinite(scale) || scale <= 0.0f)
            throw std::invalid_argument("Model bounds cannot be fitted to the requested height");
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

        if (!finite(transform))
            throw std::invalid_argument("Fitted model transform is not finite");

        return transform;
    }
}
