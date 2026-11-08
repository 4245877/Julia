#include "julia/animation/Animation.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::animation
{
    void prepareAnimationClip(AnimationClip& clip, const Skeleton& skeleton)
    {
        if (!std::isfinite(clip.durationTicks) || clip.durationTicks < 0.0 ||
            !std::isfinite(clip.ticksPerSecond) || clip.ticksPerSecond <= 0.0 ||
            !std::isfinite(clip.durationSeconds()))
            throw std::invalid_argument("Invalid animation duration/tick rate: " + clip.name);
        std::vector<bool> mapped(skeleton.nodes().size(), false);
        for (auto& channel : clip.channels)
        {
            if (channel.nodeIndex >= skeleton.nodes().size() || mapped[channel.nodeIndex])
                throw std::invalid_argument("Invalid/duplicate animation node: " + clip.name);
            mapped[channel.nodeIndex] = true;
            const auto validateKeys = [&](const auto& keys) {
                double previous = -std::numeric_limits<double>::infinity();
                for (const auto& key : keys)
                {
                    if (!std::isfinite(key.ticks) || key.ticks < previous)
                        throw std::invalid_argument("Invalid animation key time: " + clip.name);
                    previous = key.ticks;
                    for (glm::length_t i = 0; i < key.value.length(); ++i)
                        if (!std::isfinite(key.value[i]))
                            throw std::invalid_argument("Non-finite animation key: " + clip.name);
                }
            };
            validateKeys(channel.translations);
            validateKeys(channel.rotations);
            validateKeys(channel.scales);
            for (auto& key : channel.rotations)
            {
                const float magnitude = glm::length(key.value);
                if (!std::isfinite(magnitude) || magnitude < 1e-12f)
                    throw std::invalid_argument("Invalid animation quaternion: " + clip.name);
                key.value = glm::normalize(key.value);
            }
        }
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

}
