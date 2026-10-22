#pragma once

#include <glm/vec2.hpp>
#include <cstdint>

namespace julia::render
{
    struct EyeBehaviorSettings
    {
        // Mean open-eye interval [1,15] s; variability [0,0.8] bounds a triangular distribution.
        double blinkMeanSeconds = 4.0;
        double blinkVariability = 0.45;
        // Rare-event probabilities may be adjusted in [0,0.25].
        double doubleBlinkProbability = 0.08;
        double slowBlinkProbability = 0.06;
        // Per-axis head-local limits: yaw [0,15], pitch [0,10] degrees.
        float gazeYawDegrees = 8.0f;
        float gazePitchDegrees = 5.0f;
        // Stylized, visible saccades. The speed bound may extend the nominal maximum.
        double saccadeMinSeconds = 0.100;
        double saccadeMaxSeconds = 0.220;
        double saccadeSecondsPerDegree = 0.004;
        double saccadeMaxSpeedDegrees = 240.0;
        double pursuitTimeConstant = 0.065;
        double pursuitMaxSpeedDegrees = 60.0;
        double fixationMinSeconds = 1.0;
        double fixationMaxSeconds = 3.8;
        // Zero gives stable fixations; optional rare corrections, in degrees.
        float correctionAmplitudeDegrees = 0.0f;
    };

    enum class GazeState { Fixating, Saccading, Tracking };

    struct EyeMotion
    {
        GazeState state = GazeState::Fixating;
        glm::vec2 startDegrees{0.0f};
        glm::vec2 targetDegrees{0.0f};
        glm::vec2 velocityDegreesPerSecond{0.0f};
        double elapsedSeconds = 0.0;
        double durationSeconds = 0.0;
    };

    struct EyeBehaviorOutput
    {
        // Head-local yaw and pitch, in degrees. Rig axis conversion belongs to the pose layer.
        glm::vec2 gazeDegrees{0.0f};
        float blinkLeft = 0.0f;
        float blinkRight = 0.0f;
    };

    struct EyeBehaviorStats
    {
        std::uint64_t blinkCount = 0;
        std::uint64_t doubleBlinkCount = 0; // Second blinks actually started.
        std::uint64_t slowBlinkCount = 0;
    };

    // Pure behavior, independent of a model, animation player or graphics context.
    // A fixed 240 Hz clock and a specified PRNG make results reproducible across frame rates.
    class EyeBehavior
    {
    public:
        explicit EyeBehavior(std::uint32_t seed = 0x52415048u);

        const EyeBehaviorSettings& settings() const noexcept { return settings_; }
        void setSettings(const EyeBehaviorSettings& settings);
        void setAutoBlink(bool enabled) noexcept { autoBlink_ = enabled; }
        bool autoBlink() const noexcept { return autoBlink_; }
        void setAutoGaze(bool enabled) noexcept;
        bool autoGaze() const noexcept { return autoGaze_; }

        void setGazeDegrees(glm::vec2 yawPitch);
        // An updated moving target; does not restart a finite saccade every frame.
        void trackGazeDegrees(glm::vec2 yawPitch);
        void releaseGaze() noexcept;
        // Handoff from an external tracking layer without exposing a hidden idle pose.
        void releaseGazeFrom(glm::vec2 currentDegrees);
        void neutral(); // Smoothly move to and hold a forward gaze.

        void setBlink(float left, float right); // Persistent manual override, clamped to [0,1].
        void releaseBlink() noexcept; // Smoothly reopen before resuming the automatic timer.
        void triggerBlink(bool left = true, bool right = true);

        // Invalid time throws. Gaps above 0.5 s freeze behavior (pause/suspend), rather than jump.
        void update(double deltaSeconds);
        // Reset RNG, clocks and overrides; preserve settings and automatic-mode toggles.
        void reset(std::uint32_t seed = 0x52415048u) noexcept;
        const EyeBehaviorOutput& output() const noexcept { return output_; }
        const EyeBehaviorStats& stats() const noexcept { return stats_; }
        const EyeMotion& motion() const noexcept { return motion_; }

    private:
        EyeBehaviorSettings settings_{};
        EyeBehaviorOutput output_{};
        EyeBehaviorStats stats_{};
        std::uint32_t randomState_ = 1;
        bool autoBlink_ = true;
        bool autoGaze_ = true;
        bool manualGaze_ = false;
        bool manualBlink_ = false;
        double accumulator_ = 0.0;

        bool blinking_ = false;
        bool blinkLeft_ = true;
        bool blinkRight_ = true;
        bool pendingDouble_ = false;
        bool nextIsDouble_ = false;
        glm::vec2 blinkStart_{0.0f};
        double blinkElapsed_ = 0.0;
        double closeSeconds_ = 0.07;
        double holdSeconds_ = 0.02;
        double openSeconds_ = 0.12;
        double nextBlink_ = 0.0;
        bool recovering_ = false;
        double recoveryElapsed_ = 0.0;
        glm::vec2 recoveryStart_{0.0f};

        glm::vec2 fixation_{0.0f};
        EyeMotion motion_{};
        double nextFixation_ = 0.0;
        double nextCorrection_ = 0.0;

        double random01() noexcept;
        double range(double low, double high) noexcept;
        double blinkInterval() noexcept;
        glm::vec2 limitGaze(glm::vec2 gaze) const noexcept;
        void moveGaze(glm::vec2 target) noexcept;
        void beginBlink(bool left, bool right, bool automatic, bool second) noexcept;
        void tick() noexcept;
    };
}
