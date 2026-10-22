#include "julia/render/EyeBehavior.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <glm/geometric.hpp>

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
        const float yaw = std::min(12.0f, settings_.gazeYawDegrees);
        const float pitch = std::min(8.0f, settings_.gazePitchDegrees);
        gaze.x = std::clamp(gaze.x, -yaw, yaw);
        gaze.y = std::clamp(gaze.y, -pitch, pitch);
        // Use the same audited envelope as the pose layer, so motion is planned
        // from the direction actually applied, even with expanded settings.
        const float radius = glm::length(gaze / glm::vec2{12, 8});
        return radius > 1 ? gaze / radius : gaze;
    }

    void EyeBehavior::setSettings(const EyeBehaviorSettings& value)
    {
        if (!std::isfinite(value.blinkMeanSeconds) || value.blinkMeanSeconds < 1.0 || value.blinkMeanSeconds > 15.0 ||
            !std::isfinite(value.blinkVariability) || value.blinkVariability < 0.0 || value.blinkVariability > 0.8 ||
            !std::isfinite(value.doubleBlinkProbability) || value.doubleBlinkProbability < 0.0 || value.doubleBlinkProbability > 0.25 ||
            !std::isfinite(value.slowBlinkProbability) || value.slowBlinkProbability < 0.0 || value.slowBlinkProbability > 0.25 ||
            !std::isfinite(value.gazeYawDegrees) || value.gazeYawDegrees < 0.0f || value.gazeYawDegrees > 15.0f ||
            !std::isfinite(value.gazePitchDegrees) || value.gazePitchDegrees < 0.0f || value.gazePitchDegrees > 10.0f ||
            !std::isfinite(value.saccadeMinSeconds) || value.saccadeMinSeconds < 0.03 || value.saccadeMinSeconds > 0.3 ||
            !std::isfinite(value.saccadeMaxSeconds) || value.saccadeMaxSeconds < value.saccadeMinSeconds || value.saccadeMaxSeconds > 0.5 ||
            !std::isfinite(value.saccadeSecondsPerDegree) || value.saccadeSecondsPerDegree < 0.0 || value.saccadeSecondsPerDegree > 0.02 ||
            !std::isfinite(value.saccadeMaxSpeedDegrees) || value.saccadeMaxSpeedDegrees < 30.0 || value.saccadeMaxSpeedDegrees > 1000.0 ||
            !std::isfinite(value.pursuitTimeConstant) || value.pursuitTimeConstant < 0.01 || value.pursuitTimeConstant > 0.3 ||
            !std::isfinite(value.pursuitMaxSpeedDegrees) || value.pursuitMaxSpeedDegrees < 1.0 || value.pursuitMaxSpeedDegrees > 360.0 ||
            !std::isfinite(value.fixationMinSeconds) || value.fixationMinSeconds < 0.3 || value.fixationMinSeconds > 15.0 ||
            !std::isfinite(value.fixationMaxSeconds) || value.fixationMaxSeconds < value.fixationMinSeconds || value.fixationMaxSeconds > 30.0 ||
            !std::isfinite(value.correctionAmplitudeDegrees) || value.correctionAmplitudeDegrees < 0.0f || value.correctionAmplitudeDegrees > 0.2f)
            throw std::invalid_argument("EyeBehavior settings outside safe ranges");
        settings_ = value;
        output_.gazeDegrees = limitGaze(output_.gazeDegrees);
        fixation_ = limitGaze(fixation_);
        motion_.startDegrees = limitGaze(motion_.startDegrees);
        motion_.targetDegrees = limitGaze(motion_.targetDegrees);
        if (motion_.state == GazeState::Saccading) moveGaze(motion_.targetDegrees);
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
        blinkElapsed_ = recoveryElapsed_ = 0.0;
        blinkStart_ = recoveryStart_ = glm::vec2{0.0f};
        fixation_ = glm::vec2{0.0f};
        motion_ = {};
        nextBlink_ = blinkInterval();
        nextFixation_ = range(settings_.fixationMinSeconds, settings_.fixationMaxSeconds);
        nextCorrection_ = range(0.9, 2.0);
    }

    void EyeBehavior::moveGaze(glm::vec2 target) noexcept
    {
        motion_.startDegrees = output_.gazeDegrees;
        motion_.targetDegrees = limitGaze(target);
        motion_.elapsedSeconds = 0.0;
        motion_.velocityDegreesPerSecond = glm::vec2{0};
        // Small head-local angles have no wrap discontinuity. Their Euclidean
        // arc length also conservatively bounds the composed eye rotation.
        const double amplitude = glm::length(motion_.targetDegrees - motion_.startDegrees);
        motion_.durationSeconds = std::max(std::clamp(settings_.saccadeMinSeconds +
            settings_.saccadeSecondsPerDegree * amplitude, settings_.saccadeMinSeconds,
            settings_.saccadeMaxSeconds), 1.875 * amplitude / settings_.saccadeMaxSpeedDegrees);
        motion_.state = amplitude == 0.0 ? GazeState::Fixating : GazeState::Saccading;
    }

    void EyeBehavior::setGazeDegrees(glm::vec2 target)
    {
        if (!std::isfinite(target.x) || !std::isfinite(target.y))
            throw std::invalid_argument("EyeBehavior gaze must be finite");
        target = limitGaze(target);
        if (manualGaze_ && motion_.state != GazeState::Tracking && fixation_ == target) return;
        manualGaze_ = true;
        fixation_ = target;
        moveGaze(fixation_);
    }

    void EyeBehavior::trackGazeDegrees(glm::vec2 target)
    {
        if (!std::isfinite(target.x) || !std::isfinite(target.y))
            throw std::invalid_argument("EyeBehavior tracking gaze must be finite");
        if (motion_.state != GazeState::Tracking)
        {
            motion_.startDegrees = output_.gazeDegrees;
            motion_.elapsedSeconds = motion_.durationSeconds = 0.0;
            motion_.velocityDegreesPerSecond = glm::vec2{0};
        }
        manualGaze_ = true;
        fixation_ = motion_.targetDegrees = limitGaze(target);
        motion_.state = GazeState::Tracking;
    }

    void EyeBehavior::neutral() { setGazeDegrees(glm::vec2{0.0f}); }

    void EyeBehavior::releaseGaze() noexcept
    {
        manualGaze_ = false;
        fixation_ = output_.gazeDegrees;
        motion_ = {};
        motion_.startDegrees = motion_.targetDegrees = fixation_;
        nextFixation_ = range(settings_.fixationMinSeconds, settings_.fixationMaxSeconds);
        nextCorrection_ = range(0.9, 2.0);
    }

    void EyeBehavior::releaseGazeFrom(glm::vec2 currentDegrees)
    {
        if (!std::isfinite(currentDegrees.x) || !std::isfinite(currentDegrees.y))
            throw std::invalid_argument("EyeBehavior gaze handoff must be finite");
        output_.gazeDegrees = limitGaze(currentDegrees);
        releaseGaze();
    }

    void EyeBehavior::setAutoGaze(bool enabled) noexcept
    {
        autoGaze_ = enabled;
        if (!enabled && !manualGaze_)
        {
            fixation_ = output_.gazeDegrees;
            motion_ = {};
            motion_.startDegrees = motion_.targetDegrees = fixation_;
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

        if (!manualGaze_ && autoGaze_ && motion_.state == GazeState::Fixating)
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
                moveGaze(fixation_);
                nextFixation_ = range(settings_.fixationMinSeconds, settings_.fixationMaxSeconds);
                nextCorrection_ = range(0.9, 2.0);
            }
            else if (settings_.correctionAmplitudeDegrees > 0 && nextCorrection_ <= 0.0)
            {
                const float amount = settings_.correctionAmplitudeDegrees;
                const glm::vec2 correction{static_cast<float>(range(-amount, amount)), static_cast<float>(range(-amount, amount)) * .67f};
                moveGaze(fixation_ + correction);
                nextCorrection_ = range(0.9, 2.0);
            }
        }
        if (motion_.state == GazeState::Saccading)
        {
            motion_.elapsedSeconds += Step;
            const double u = std::clamp(motion_.elapsedSeconds / motion_.durationSeconds, 0.0, 1.0);
            const glm::vec2 arc = motion_.targetDegrees - motion_.startDegrees;
            output_.gazeDegrees = motion_.startDegrees + arc * ease(u);
            motion_.velocityDegreesPerSecond = arc * static_cast<float>(30 * u * u * (1 - u) * (1 - u) / motion_.durationSeconds);
            if (motion_.elapsedSeconds + 1e-10 >= motion_.durationSeconds)
            {
                output_.gazeDegrees = motion_.targetDegrees; // Exact endpoint, no drift.
                motion_.velocityDegreesPerSecond = glm::vec2{0};
                motion_.state = GazeState::Fixating;
            }
        }
        else if (motion_.state == GazeState::Tracking)
        {
            motion_.elapsedSeconds += Step;
            const glm::vec2 error = motion_.targetDegrees - output_.gazeDegrees;
            const double distance = glm::length(error);
            const double gain = -std::expm1(-Step / settings_.pursuitTimeConstant);
            const double travel = std::min(distance * gain, settings_.pursuitMaxSpeedDegrees * Step);
            const glm::vec2 movement = distance > 0 ? error * static_cast<float>(travel / distance) : glm::vec2{0};
            motion_.velocityDegreesPerSecond = movement / static_cast<float>(Step);
            output_.gazeDegrees += movement;
            if (distance <= 1e-5)
            {
                output_.gazeDegrees = motion_.targetDegrees;
                motion_.velocityDegreesPerSecond = glm::vec2{0};
            }
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
