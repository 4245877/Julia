#pragma once

#include "julia/animation/Skeleton.hpp"
#include <span>

namespace julia::animation
{
    // Mutable state of one character. May be edited by any pose source/layer.
    // Vectors are allocated once; evaluation functions below allocate nothing.
    struct Pose
    {
        explicit Pose(const Skeleton& skeleton);
        std::vector<glm::mat4> locals, globals, palette;
    };

    void setBindPose(const Skeleton& skeleton, std::span<glm::mat4> locals);

    // Empty locals selects exact bind matrices. globals may alias locals exactly;
    // other partial overlap is unsupported. Root is included, no inverse-root.
    void computeGlobalTransforms(const Skeleton& skeleton,
        std::span<const glm::mat4> locals, std::span<glm::mat4> globals);

    // palette[i] = globals[binding.nodeIndex] * binding.offsetMatrix.
    // Globals and palette must be distinct buffers, sized to nodes/bindings.
    void buildSkinningPalette(const Skeleton& skeleton,
        std::span<const glm::mat4> globals, std::span<glm::mat4> palette);
}
