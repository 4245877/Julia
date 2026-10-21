#include "julia/render_opengl/Animation.hpp"
#include "julia/render_opengl/GLModel.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::render_opengl
{
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

    void sampleAnimation(const GLModel& model, const AnimationClip& clip,
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

    AnimationPlayer::AnimationPlayer(const GLModel& model) : model_(model),
        locals_(model.nodes().size()), globals_(model.nodes().size()), palette_(model.boneCount())
    {
        evaluate();
    }

    const AnimationClip* AnimationPlayer::clip() const
    {
        return model_.animations().empty() ? nullptr : &model_.animations()[clipIndex_];
    }

    void AnimationPlayer::selectClip(std::size_t index)
    {
        if (index >= model_.animations().size()) throw std::out_of_range("Invalid animation clip");
        clipIndex_ = index;
        stop();
    }

    void AnimationPlayer::play()
    {
        if (!clip()) return;
        if (state_ == PlaybackState::Finished || timeSeconds_ >= clip()->durationSeconds())
            timeSeconds_ = 0.0;
        state_ = PlaybackState::Playing;
        evaluate();
    }

    void AnimationPlayer::pause()
    {
        if (state_ == PlaybackState::Playing) state_ = PlaybackState::Paused;
    }

    void AnimationPlayer::stop()
    {
        state_ = PlaybackState::Stopped;
        timeSeconds_ = 0.0;
        evaluate();
    }

    void AnimationPlayer::seek(double seconds)
    {
        if (!std::isfinite(seconds)) throw std::invalid_argument("Non-finite seek time");
        if (!clip()) return;
        timeSeconds_ = std::clamp(seconds, 0.0, clip()->durationSeconds());
        if (state_ != PlaybackState::Playing) state_ = PlaybackState::Paused;
        evaluate();
    }

    void AnimationPlayer::update(double deltaSeconds)
    {
        if (!std::isfinite(deltaSeconds) || deltaSeconds < 0.0)
            throw std::invalid_argument("Animation delta must be finite and nonnegative");
        if (state_ != PlaybackState::Playing || !clip() || deltaSeconds == 0.0) return;
        const double duration = clip()->durationSeconds();
        if (duration <= 0.0)
        {
            timeSeconds_ = 0.0;
            state_ = PlaybackState::Finished;
        }
        else if (looping_)
            timeSeconds_ = std::fmod(timeSeconds_ + std::fmod(deltaSeconds, duration), duration);
        else
        {
            // Avoid overflow even for an exceptionally large elapsed interval.
            timeSeconds_ += std::min(deltaSeconds, duration - timeSeconds_);
            if (timeSeconds_ >= duration) state_ = PlaybackState::Finished;
        }
        evaluate();
    }

    void AnimationPlayer::evaluate()
    {
        if (state_ != PlaybackState::Stopped && clip())
            sampleAnimation(model_, *clip(), timeSeconds_, locals_);
        else
            for (std::size_t i = 0; i < locals_.size(); ++i)
                locals_[i] = model_.nodes()[i].localTransform;
        model_.skinningMatrices(locals_, globals_, palette_);
    }
}
