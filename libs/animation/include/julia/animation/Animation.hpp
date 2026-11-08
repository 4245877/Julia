#pragma once

#include "julia/animation/Skeleton.hpp"
#include <span>

namespace julia::animation
{
    template<class T> struct AnimationKey
    {
        double ticks = 0.0;
        T value{};
    };

    struct AnimationChannel
    {
        std::uint32_t nodeIndex = 0;
        std::vector<AnimationKey<glm::vec3>> translations;
        std::vector<AnimationKey<glm::quat>> rotations;
        std::vector<AnimationKey<glm::vec3>> scales;
    };

    struct AnimationClip
    {
        std::string name;
        double durationTicks = 0.0;
        double ticksPerSecond = 25.0;
        bool usedDefaultTicksPerSecond = false;
        std::vector<AnimationChannel> channels;
        double durationSeconds() const { return durationTicks / ticksPerSecond; }
    };

    // Asset-build boundary: validate metadata/keys/node mapping and normalize
    // quaternions once. Clips must remain unchanged during sampling.
    void prepareAnimationClip(AnimationClip& clip, const Skeleton& skeleton);

    // Stateless, endpoint-clamped sampling. Unanimated/empty channels retain exact
    // bind matrices; sparse components use imported bind TRS. Animated local = T*R*S.
    // No looping or clock ownership here. Output is caller-owned, one per node.
    // Optional components receive TRS for nonempty animated channels only;
    // callers retain exact matrices for every other node.
    void sampleAnimation(const Skeleton& skeleton, const AnimationClip& clip,
        double seconds, std::span<glm::mat4> localTransforms,
        std::span<NodeTRS> components = {});
}
