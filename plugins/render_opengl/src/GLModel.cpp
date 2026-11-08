#include "julia/render_opengl/GLModel.hpp"

#include "julia/render_opengl/GLShader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>


namespace julia::render_opengl
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

    void GLModel::addMesh(GLMesh&& mesh)
    {
        meshes_.push_back(std::move(mesh));
        bounds_.expand(meshes_.back().bounds());
    }

    GLModel::GLModel() : builder_(std::make_shared<model::ModelAsset>()), asset_(builder_) {}

    GLModel::GLModel(std::shared_ptr<const model::ModelAsset> asset) : asset_(std::move(asset))
    {
        if (!asset_) throw std::invalid_argument("GPU model requires a CPU asset");
    }

    model::ModelAsset& GLModel::builder()
    {
        if (!builder_) throw std::logic_error("Imported GPU model asset is immutable");
        return *builder_;
    }
    std::uint32_t GLModel::addNode(GLModelNode node) { return builder().addNode(std::move(node)); }
    std::uint32_t GLModel::addBone(GLBoneInfo bone) { return builder().addBone(std::move(bone)); }
    void GLModel::addAnimation(AnimationClip clip) { builder().addAnimation(std::move(clip)); }

    void GLModel::draw(const GLShader& shader) const
    {
        for (const auto& mesh : meshes_)
        {
            mesh.draw(shader);
        }
    }

    void GLModel::drawOpaque(const GLShader& shader, MeshPass pass) const
    {
        for (const auto& mesh : meshes_)
            if (!mesh.transparent()) mesh.draw(shader, pass);
    }

    void GLModel::drawOutline(const GLShader& shader) const
    {
        for (const auto& mesh : meshes_)
            if (!mesh.transparent() && mesh.material().outlineScale > 0.0f)
                mesh.draw(shader, MeshPass::Outline);
    }

    void GLModel::drawTransparent(const GLShader& shader, const glm::mat4& viewModel) const
    {
        struct Entry { const GLMesh* mesh; float depth; };
        std::vector<Entry> sorted;
        for (const auto& mesh : meshes_)
            if (mesh.transparent())
                sorted.push_back({&mesh, (viewModel * glm::vec4{mesh.bounds().center(), 1.0f}).z});
        // Camera forward is -Z. Sorting centers is the conventional mesh-level
        // approximation; intersecting transparent triangles need per-pixel OIT.
        std::stable_sort(sorted.begin(), sorted.end(), [](const Entry& a, const Entry& b) {
            return a.depth < b.depth;
        });
        for (const auto& entry : sorted) entry.mesh->draw(shader);
    }

    std::vector<glm::mat4> GLModel::skinningMatrices(
        std::span<const glm::mat4> localNodeTransforms) const
    {
        std::vector<glm::mat4> globals(nodes().size()), palette(bones().size());
        skinningMatrices(localNodeTransforms, globals, palette);
        return palette;
    }

    void GLModel::skinningMatrices(std::span<const glm::mat4> localNodeTransforms,
        std::span<glm::mat4> globalTransforms, std::span<glm::mat4> palette) const
    {
        animation::computeGlobalTransforms(skeleton(), localNodeTransforms, globalTransforms);
        animation::buildSkinningPalette(skeleton(), globalTransforms, palette);
    }

    glm::mat4 GLModel::fitToHeightTransform(float targetHeight) const
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
