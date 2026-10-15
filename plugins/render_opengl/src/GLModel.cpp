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

    std::uint32_t GLModel::addNode(GLModelNode node)
    {
        if (nodes_.size() >= NoModelNode)
            throw std::length_error("Model contains too many nodes");
        if (node.parent != NoModelNode && node.parent >= nodes_.size())
            throw std::invalid_argument("Model nodes must precede their children");
        if (!finite(node.localTransform))
            throw std::invalid_argument("Model node transform is not finite");
        node.globalBindTransform = node.parent == NoModelNode
            ? node.localTransform
            : nodes_[node.parent].globalBindTransform * node.localTransform;
        if (!finite(node.globalBindTransform))
            throw std::invalid_argument("Model global node transform is not finite");
        nodes_.push_back(std::move(node));
        return static_cast<std::uint32_t>(nodes_.size() - 1);
    }

    std::uint32_t GLModel::addBone(GLBoneInfo bone)
    {
        if (bones_.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::length_error("Model contains too many bone indices");
        if (!finite(bone.offsetMatrix))
            throw std::invalid_argument("Bone offset matrix is not finite");
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

    void GLModel::drawOpaque(const GLShader& shader) const
    {
        for (const auto& mesh : meshes_)
            if (!mesh.transparent()) mesh.draw(shader);
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
        if (!localNodeTransforms.empty() && localNodeTransforms.size() != nodes_.size())
            throw std::invalid_argument("Pose must provide one local transform per model node");

        std::vector<glm::mat4> globalTransforms;
        globalTransforms.reserve(nodes_.size());
        for (std::size_t i = 0; i < nodes_.size(); ++i)
        {
            const auto& local = localNodeTransforms.empty()
                ? nodes_[i].localTransform : localNodeTransforms[i];
            if (!finite(local))
                throw std::invalid_argument("Pose contains a non-finite transform");
            const auto global = nodes_[i].parent == NoModelNode
                ? local : globalTransforms[nodes_[i].parent] * local;
            if (!finite(global))
                throw std::invalid_argument("Pose global transform is not finite");
            globalTransforms.push_back(global);
        }

        std::vector<glm::mat4> palette;
        palette.reserve(bones_.size());
        for (const auto& bone : bones_)
        {
            if (bone.nodeIndex >= globalTransforms.size())
                throw std::logic_error("Bone is not linked to a model node: " + bone.name);
            const auto matrix = globalTransforms[bone.nodeIndex] * bone.offsetMatrix;
            if (!finite(matrix))
                throw std::invalid_argument("Skinning matrix is not finite: " + bone.name);
            palette.push_back(matrix);
        }
        return palette;
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
