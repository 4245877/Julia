#pragma once

#include "legacy_pose_reference.hpp"
#include "julia/animation/Pose.hpp"
#include "julia_test.hpp"

namespace pose_test
{
    inline float delta(std::span<const glm::mat4> a, std::span<const glm::mat4> b)
    {
        if (a.size() != b.size()) return std::numeric_limits<float>::infinity();
        float maximum = 0;
        for (std::size_t i = 0; i < a.size(); ++i)
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                {
                    if (!std::isfinite(a[i][c][r]) || !std::isfinite(b[i][c][r]))
                        return std::numeric_limits<float>::infinity();
                    maximum = std::max(maximum, std::abs(a[i][c][r] - b[i][c][r]));
                }
        return maximum;
    }

    // Checks both locals and every global/palette element against the old path.
    inline float compare(const julia::animation::Skeleton& skeleton,
        const julia::animation::AnimationClip* clip = nullptr, double seconds = 0)
    {
        using namespace julia::animation;
        Pose current(skeleton);
        std::vector<glm::mat4> legacyLocals(skeleton.nodes().size()), legacyGlobals(legacyLocals.size()),
            legacyPalette(skeleton.bindings().size());
        if (clip)
        {
            sampleAnimation(skeleton, *clip, seconds, current.locals);
            legacy_pose_reference::sampleAnimation(skeleton, *clip, seconds, legacyLocals);
        }
        else
            for (std::size_t i = 0; i < legacyLocals.size(); ++i)
                legacyLocals[i] = skeleton.nodes()[i].localTransform;
        computeGlobalTransforms(skeleton, current.locals, current.globals);
        buildSkinningPalette(skeleton, current.globals, current.palette);
        legacy_pose_reference::skinningMatrices(skeleton, legacyLocals, legacyGlobals, legacyPalette);
        const auto maximum = std::max({delta(current.locals, legacyLocals),
            delta(current.globals, legacyGlobals), delta(current.palette, legacyPalette)});
        JULIA_CHECK(maximum <= 1e-5f);
        return maximum;
    }
}
