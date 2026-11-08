#include "julia/animation/Skeleton.hpp"
#include "MatrixValidation.hpp"
#include <stdexcept>
#include <utility>

namespace julia::animation
{
    std::uint32_t Skeleton::addNode(SkeletonNode node)
    {
        if (nodes_.size() >= NoNode)
            throw std::length_error("Skeleton contains too many nodes");
        if (node.parent != NoNode && node.parent >= nodes_.size())
            throw std::invalid_argument("Skeleton nodes must precede their children");
        if (!detail::finite(node.localTransform))
            throw std::invalid_argument("Skeleton node transform is not finite");
        for (glm::length_t i = 0; i < 3; ++i)
            if (!std::isfinite(node.bindTRS.translation[i]) || !std::isfinite(node.bindTRS.scale[i]))
                throw std::invalid_argument("Skeleton bind TRS is not finite");
        const auto rotationLength = glm::length(node.bindTRS.rotation);
        if (!std::isfinite(rotationLength) || rotationLength < 1e-12f)
            throw std::invalid_argument("Skeleton bind rotation is invalid");
        node.globalBindTransform = node.parent == NoNode ? node.localTransform
            : nodes_[node.parent].globalBindTransform * node.localTransform;
        if (!detail::finite(node.globalBindTransform))
            throw std::invalid_argument("Skeleton global bind transform is not finite");
        nodes_.push_back(std::move(node));
        return static_cast<std::uint32_t>(nodes_.size() - 1);
    }

    std::uint32_t Skeleton::addBinding(SkinBinding binding)
    {
        if (bindings_.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::length_error("Skeleton contains too many skinning indices");
        if (binding.nodeIndex >= nodes_.size())
            throw std::invalid_argument("Skin binding references an invalid node");
        if (!detail::finite(binding.offsetMatrix))
            throw std::invalid_argument("Skin binding offset is not finite");
        bindings_.push_back(std::move(binding));
        return static_cast<std::uint32_t>(bindings_.size() - 1);
    }
}
