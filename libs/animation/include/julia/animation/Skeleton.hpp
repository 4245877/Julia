#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace julia::animation
{
    inline constexpr std::uint32_t NoNode = std::numeric_limits<std::uint32_t>::max();

    struct NodeTRS
    {
        glm::vec3 translation{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f};
    };

    struct SkeletonNode
    {
        std::string name;
        std::uint32_t parent = NoNode;
        // Preserve the exact imported matrix, even when not representable as TRS.
        glm::mat4 localTransform{1.0f};
        glm::mat4 globalBindTransform{1.0f}; // Computed by addNode, including root.
        NodeTRS bindTRS{}; // Importer-provided fallback for missing clip components.
    };

    struct SkinBinding
    {
        std::string name;
        // Mesh-local inverse bind adjusted for transforms already baked in vertices.
        glm::mat4 offsetMatrix{1.0f};
        std::uint32_t nodeIndex = NoNode;
    };

    // Asset data, shared read-only by characters. Build before creating instances;
    // do not mutate while players/layers refer to it. No pose, clock or GPU state.
    // All nodes (including helpers) are stored in parent-before-child order.
    class Skeleton
    {
    public:
        std::uint32_t addNode(SkeletonNode node);
        std::uint32_t addBinding(SkinBinding binding);
        const std::vector<SkeletonNode>& nodes() const { return nodes_; }
        const std::vector<SkinBinding>& bindings() const { return bindings_; }

    private:
        std::vector<SkeletonNode> nodes_;
        // Several bindings may reference the same node with different offsets.
        std::vector<SkinBinding> bindings_;
    };
}
