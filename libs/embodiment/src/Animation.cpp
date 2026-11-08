#include "julia/embodiment/Animation.hpp"

#include "julia/embodiment/EyePoseLayer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::embodiment
{
    void sampleAnimation(const model::ModelAsset& model, const AnimationClip& clip,
        double seconds, std::span<glm::mat4> localTransforms)
    {
        animation::sampleAnimation(model.skeleton(), clip, seconds, localTransforms);
    }

    AnimationPlayer::AnimationPlayer(const model::ModelAsset& model) : model_(model),
        pose_(model.skeleton())
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
        const auto* base = state_ == PlaybackState::Stopped ? nullptr : clip();
        if (eyeLayer_ && state_ != PlaybackState::Paused && !EyePoseLayer::authoredEyes(base))
            eyeLayer_->update(deltaSeconds);
        if (state_ != PlaybackState::Playing || !clip() || deltaSeconds == 0.0)
        {
            if (eyeLayer_) evaluate();
            return;
        }
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
            animation::sampleAnimation(model_.skeleton(), *clip(), timeSeconds_, pose_.locals);
        else
            animation::setBindPose(model_.skeleton(), pose_.locals);
        if (eyeLayer_)
            eyeLayer_->apply(pose_.locals, state_ == PlaybackState::Stopped ? nullptr : clip());
        animation::computeGlobalTransforms(model_.skeleton(), pose_.locals, pose_.globals);
        animation::buildSkinningPalette(model_.skeleton(), pose_.globals, pose_.palette);
    }

    void AnimationPlayer::setEyeLayer(EyePoseLayer* layer)
    {
        if (layer && &layer->asset() != &model_)
            throw std::invalid_argument("Eye layer belongs to a different model asset");
        eyeLayer_ = layer;
        evaluate();
    }
}
