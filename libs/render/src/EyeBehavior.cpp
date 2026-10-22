#include "julia/render/EyeBehavior.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace julia::render
{
    namespace
    {
        constexpr double Step = 1.0 / 240.0;

        float ease(double t) noexcept
        {
            t = std::clamp(t, 0.0, 1.0);
            return static_cast<float>(t * t * t * (t * (t * 6.0 - 15.0) + 10.0));
        }
    }

    EyeBehavior::EyeBehavior(std::uint32_t seed) { reset(seed); }

    double EyeBehavior::random01() noexcept
    {
        // The mapping is explicitly specified; std::uniform_real_distribution is library-specific.
        randomState_ ^= randomState_ << 13;
        randomState_ ^= randomState_ >> 17;
        randomState_ ^= randomState_ << 5;
        return static_cast<double>(randomState_) / 4294967296.0;
    }

    double EyeBehavior::range(double low, double high) noexcept
    {
        return low + (high - low) * random01();
    }

    double EyeBehavior::blinkInterval() noexcept
    {
        return settings_.blinkMeanSeconds *
            (1.0 + settings_.blinkVariability * (random01() + random01() - 1.0));
    }

    glm::vec2 EyeBehavior::limitGaze(glm::vec2 gaze) const noexcept
    {
        gaze.x = std::clamp(gaze.x, -settings_.gazeYawDegrees, settings_.gazeYawDegrees);
        gaze.y = std::clamp(gaze.y, -settings_.gazePitchDegrees, settings_.gazePitchDegrees);
        return gaze;
    }

    void EyeBehavior::setSettings(const EyeBehaviorSettings& value)
    {
        if (!std::isfinite(value.blinkMeanSeconds) || value.blinkMeanSeconds < 1.0 || value.blinkMeanSeconds > 15.0 ||
            !std::isfinite(value.blinkVariability) || value.blinkVariability < 0.0 || value.blinkVariability > 0.8 ||
            !std::isfinite(value.doubleBlinkProbability) || value.doubleBlinkProbability < 0.0 || value.doubleBlinkProbability > 0.25 ||
            !std::isfinite(value.slowBlinkProbability) || value.slowBlinkProbability < 0.0 || value.slowBlinkProbability > 0.25 ||
            !std::isfinite(value.gazeYawDegrees) || value.gazeYawDegrees < 0.0f || value.gazeYawDegrees > 15.0f ||
            !std::isfinite(value.gazePitchDegrees) || value.gazePitchDegrees < 0.0f || value.gazePitchDegrees > 10.0f)
            throw std::invalid_argument("EyeBehavior settings outside safe ranges");
        settings_ = value;
        output_.gazeDegrees = limitGaze(output_.gazeDegrees);
        fixation_ = limitGaze(fixation_);
        gazeStart_ = limitGaze(gazeStart_);
        gazeTarget_ = limitGaze(gazeTarget_);
        if (!blinking_ && !nextIsDouble_) nextBlink_ = blinkInterval();
    }

    void EyeBehavior::reset(std::uint32_t seed) noexcept
    {
        randomState_ = seed == 0 ? 0x6d2b79f5u : seed;
        output_ = {};
        stats_ = {};
        accumulator_ = 0.0;
        manualBlink_ = manualGaze_ = blinking_ = recovering_ = false;
        pendingDouble_ = nextIsDouble_ = false;
        blinkElapsed_ = recoveryElapsed_ = gazeElapsed_ = gazeDuration_ = 0.0;
        blinkStart_ = recoveryStart_ = glm::vec2{0.0f};
        fixation_ = gazeStart_ = gazeTarget_ = glm::vec2{0.0f};
        nextBlink_ = blinkInterval();
        nextFixation_ = range(1.0, 2.5);
        nextCorrection_ = range(0.4, 1.2);
    }

    void EyeBehavior::moveGaze(glm::vec2 target, double seconds) noexcept
    {
        gazeStart_ = output_.gazeDegrees;
        gazeTarget_ = limitGaze(target);
        gazeElapsed_ = 0.0;
        gazeDuration_ = seconds;
    }

    void EyeBehavior::setGazeDegrees(glm::vec2 target)
    {
        if (!std::isfinite(target.x) || !std::isfinite(target.y))
            throw std::invalid_argument("EyeBehavior gaze must be finite");
        target = limitGaze(target);
        if (manualGaze_ && fixation_ == target) return;
        manualGaze_ = true;
        fixation_ = target;
        moveGaze(fixation_, 0.12);
    }

    void EyeBehavior::neutral() { setGazeDegrees(glm::vec2{0.0f}); }

    void EyeBehavior::releaseGaze() noexcept
    {
        manualGaze_ = false;
        fixation_ = output_.gazeDegrees;
        nextFixation_ = range(0.8, 1.8);
        nextCorrection_ = range(0.4, 1.0);
    }

    void EyeBehavior::releaseGazeFrom(glm::vec2 currentDegrees)
    {
        if (!std::isfinite(currentDegrees.x) || !std::isfinite(currentDegrees.y))
            throw std::invalid_argument("EyeBehavior gaze handoff must be finite");
        output_.gazeDegrees = limitGaze(currentDegrees);
        gazeStart_ = gazeTarget_ = output_.gazeDegrees;
        gazeElapsed_ = gazeDuration_ = 0.0;
        releaseGaze();
    }

    void EyeBehavior::setAutoGaze(bool enabled) noexcept
    {
        autoGaze_ = enabled;
        if (!enabled && !manualGaze_)
        {
            fixation_ = gazeStart_ = gazeTarget_ = output_.gazeDegrees;
            gazeDuration_ = 0.0;
        }
    }

    void EyeBehavior::setBlink(float left, float right)
    {
        if (!std::isfinite(left) || !std::isfinite(right))
            throw std::invalid_argument("EyeBehavior blink must be finite");
        manualBlink_ = true;
        blinking_ = recovering_ = pendingDouble_ = nextIsDouble_ = false;
        output_.blinkLeft = std::clamp(left, 0.0f, 1.0f);
        output_.blinkRight = std::clamp(right, 0.0f, 1.0f);
    }

    void EyeBehavior::releaseBlink() noexcept
    {
        if (!manualBlink_) return;
        manualBlink_ = false;
        recovering_ = true;
        recoveryElapsed_ = 0.0;
        recoveryStart_ = {output_.blinkLeft, output_.blinkRight};
        nextBlink_ = blinkInterval();
    }

    void EyeBehavior::beginBlink(bool left, bool right, bool automatic, bool second) noexcept
    {
        blinking_ = true;
        blinkLeft_ = left;
        blinkRight_ = right;
        blinkElapsed_ = 0.0;
        blinkStart_ = {output_.blinkLeft, output_.blinkRight};
        const bool slow = automatic && !second && random01() < settings_.slowBlinkProbability;
        closeSeconds_ = slow ? range(0.11, 0.15) : range(0.055, 0.080);
        holdSeconds_ = slow ? range(0.025, 0.04) : range(0.012, 0.025);
        openSeconds_ = slow ? range(0.17, 0.23) : range(0.10, 0.145);
        pendingDouble_ = automatic && !second && !slow && random01() < settings_.doubleBlinkProbability;
        ++stats_.blinkCount;
        if (slow) ++stats_.slowBlinkCount;
        if (second) ++stats_.doubleBlinkCount;
    }

    void EyeBehavior::triggerBlink(bool left, bool right)
    {
        if (!left && !right) return;
        if (blinking_) return; // Repeated inputs cannot restart a half-closed lid.
        manualBlink_ = recovering_ = nextIsDouble_ = false;
        beginBlink(left, right, false, false);
    }

    void EyeBehavior::tick() noexcept
    {
        if (!manualBlink_)
        {
            if (recovering_)
            {
                recoveryElapsed_ += Step;
                const glm::vec2 closure = recoveryStart_ * (1.0f - ease(recoveryElapsed_ / 0.12));
                output_.blinkLeft = closure.x;
                output_.blinkRight = closure.y;
                if (recoveryElapsed_ >= 0.12) recovering_ = false;
            }
            else if (blinking_)
            {
                blinkElapsed_ += Step;
                float closure = 1.0f;
                if (blinkElapsed_ < closeSeconds_) closure = ease(blinkElapsed_ / closeSeconds_);
                else if (blinkElapsed_ > closeSeconds_ + holdSeconds_)
                    closure = 1.0f - ease((blinkElapsed_ - closeSeconds_ - holdSeconds_) / openSeconds_);
                const float closing = ease(blinkElapsed_ / closeSeconds_);
                const float reopening = 1.0f - ease(blinkElapsed_ / openSeconds_);
                output_.blinkLeft = blinkLeft_ ?
                    (blinkElapsed_ < closeSeconds_ ? blinkStart_.x + (1.0f - blinkStart_.x) * closing : closure) :
                    blinkStart_.x * reopening;
                output_.blinkRight = blinkRight_ ?
                    (blinkElapsed_ < closeSeconds_ ? blinkStart_.y + (1.0f - blinkStart_.y) * closing : closure) :
                    blinkStart_.y * reopening;
                if (blinkElapsed_ >= closeSeconds_ + holdSeconds_ + openSeconds_)
                {
                    blinking_ = false;
                    nextIsDouble_ = pendingDouble_;
                    nextBlink_ = pendingDouble_ ? range(0.09, 0.16) : blinkInterval();
                    pendingDouble_ = false;
                }
            }
            else if (autoBlink_)
            {
                nextBlink_ -= Step;
                if (nextBlink_ <= 0.0)
                {
                    const bool second = nextIsDouble_;
                    nextIsDouble_ = false;
                    beginBlink(true, true, true, second);
                }
            }
        }

        if (!manualGaze_ && autoGaze_)
        {
            nextFixation_ -= Step;
            nextCorrection_ -= Step;
            if (nextFixation_ <= 0.0)
            {
                // Most destinations stay in the central third of the permitted range.
                const double choice = random01();
                const float amplitude = choice < 0.75 ? 0.33f : (choice < 0.95 ? 0.6f : 0.85f);
                fixation_ = {static_cast<float>(random01() + random01() - 1.0) * settings_.gazeYawDegrees * amplitude,
                    static_cast<float>(random01() + random01() - 1.0) * settings_.gazePitchDegrees * amplitude};
                moveGaze(fixation_, range(0.045, 0.085));
                nextFixation_ = range(1.0, 3.8);
                nextCorrection_ = range(0.4, 1.2);
            }
            else if (nextCorrection_ <= 0.0)
            {
                const glm::vec2 correction{static_cast<float>(range(-0.12, 0.12)), static_cast<float>(range(-0.08, 0.08))};
                moveGaze(fixation_ + correction, range(0.035, 0.055));
                nextCorrection_ = range(0.45, 1.4);
            }
        }
        if (gazeDuration_ > 0.0)
        {
            gazeElapsed_ += Step;
            const float t = ease(gazeElapsed_ / gazeDuration_);
            output_.gazeDegrees = gazeStart_ + (gazeTarget_ - gazeStart_) * t;
            if (gazeElapsed_ >= gazeDuration_) gazeDuration_ = 0.0;
        }
    }

    void EyeBehavior::update(double deltaSeconds)
    {
        if (!std::isfinite(deltaSeconds) || deltaSeconds < 0.0)
            throw std::invalid_argument("EyeBehavior delta must be finite and nonnegative");
        if (deltaSeconds > 0.5) return;
        accumulator_ += deltaSeconds;
        while (accumulator_ + 1e-10 >= Step)
        {
            accumulator_ -= Step;
            tick();
        }
        accumulator_ = std::max(0.0, accumulator_);
    }
}
