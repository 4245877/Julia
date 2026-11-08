#pragma once
// Frozen from GLModel.cpp and Animation.cpp before CPU extraction (2026-11-08).
// Test-only compatibility oracle, intentionally independent of the new functions.
#include "julia/animation/Animation.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legacy_pose_reference
{
    using namespace julia::animation;
    bool finite(const glm::mat4& matrix)
    {
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
                if (!std::isfinite(matrix[column][row])) return false;
        return true;
    }
    namespace
    {
        template<class T> T interpolate(const T& a, const T& b, float fraction)
        {
            return glm::mix(a, b, fraction);
        }

        template<> glm::quat interpolate(const glm::quat& a, const glm::quat& b, float fraction)
        {
            return glm::normalize(glm::slerp(a, b, fraction));
        }

        template<class T> T sample(const std::vector<AnimationKey<T>>& keys,
            double ticks, const T& rest)
        {
            if (keys.empty()) return rest;
            if (ticks <= keys.front().ticks) return keys.front().value;
            const auto next = std::upper_bound(keys.begin(), keys.end(), ticks,
                [](double t, const auto& key) { return t < key.ticks; });
            if (next == keys.end()) return keys.back().value;
            const auto& previous = *(next - 1);
            const auto fraction = static_cast<float>((ticks - previous.ticks) /
                (next->ticks - previous.ticks));
            return interpolate(previous.value, next->value, fraction);
        }
    }

    void sampleAnimation(const Skeleton& model, const AnimationClip& clip,
        double seconds, std::span<glm::mat4> localTransforms)
    {
        if (!std::isfinite(seconds) || localTransforms.size() != model.nodes().size())
            throw std::invalid_argument("Invalid animation sample time/output size");
        const double ticks = std::clamp(seconds, 0.0, clip.durationSeconds()) * clip.ticksPerSecond;
        for (std::size_t i = 0; i < model.nodes().size(); ++i)
            localTransforms[i] = model.nodes()[i].localTransform;
        for (const auto& channel : clip.channels)
        {
            if (channel.nodeIndex >= model.nodes().size())
                throw std::invalid_argument("Animation channel references an invalid node");
            if (channel.translations.empty() && channel.rotations.empty() && channel.scales.empty())
                continue;
            const auto& rest = model.nodes()[channel.nodeIndex].bindTRS;
            localTransforms[channel.nodeIndex] =
                glm::translate(glm::mat4{1.0f}, sample(channel.translations, ticks, rest.translation)) *
                glm::mat4_cast(sample(channel.rotations, ticks, rest.rotation)) *
                glm::scale(glm::mat4{1.0f}, sample(channel.scales, ticks, rest.scale));
        }
    }

    void skinningMatrices(const Skeleton& model, std::span<const glm::mat4> localNodeTransforms,
        std::span<glm::mat4> globalTransforms, std::span<glm::mat4> palette)
    {
        if (!localNodeTransforms.empty() && localNodeTransforms.size() != model.nodes().size())
            throw std::invalid_argument("Pose must provide one local transform per model node");
        if (globalTransforms.size() != model.nodes().size() || palette.size() != model.bindings().size())
            throw std::invalid_argument("Incorrect skeleton scratch/palette size");

        for (std::size_t i = 0; i < model.nodes().size(); ++i)
        {
            const auto& local = localNodeTransforms.empty()
                ? model.nodes()[i].localTransform : localNodeTransforms[i];
            if (!finite(local))
                throw std::invalid_argument("Pose contains a non-finite transform");
            const auto global = model.nodes()[i].parent == NoNode
                ? local : globalTransforms[model.nodes()[i].parent] * local;
            if (!finite(global))
                throw std::invalid_argument("Pose global transform is not finite");
            globalTransforms[i] = global;
        }

        for (std::size_t i = 0; i < model.bindings().size(); ++i)
        {
            const auto& bone = model.bindings()[i];
            if (bone.nodeIndex >= globalTransforms.size())
                throw std::logic_error("Bone is not linked to a model node: " + bone.name);
            const auto matrix = globalTransforms[bone.nodeIndex] * bone.offsetMatrix;
            if (!finite(matrix))
                throw std::invalid_argument("Skinning matrix is not finite: " + bone.name);
            palette[i] = matrix;
        }
    }

}
