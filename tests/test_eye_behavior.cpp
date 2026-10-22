#include "julia/render/EyeBehavior.hpp"
#include "julia_test.hpp"

#include <glm/geometric.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    template<class F> bool rejects(F&& action)
    {
        try { action(); }
        catch (const std::invalid_argument&) { return true; }
        return false;
    }

    bool same(const julia::render::EyeBehaviorOutput& a, const julia::render::EyeBehaviorOutput& b)
    {
        return a.gazeDegrees == b.gazeDegrees && a.blinkLeft == b.blinkLeft && a.blinkRight == b.blinkRight;
    }

    void advance(julia::render::EyeBehavior& behavior, double seconds)
    {
        const int ticks = static_cast<int>(std::round(seconds * 240.0));
        for (int i = 0; i < ticks; ++i) behavior.update(1.0 / 240.0);
    }

    void checkSaccades()
    {
        using namespace julia::render;
        struct Transition { glm::vec2 from, to; };
        const std::array<Transition, 8> transitions{{
            {{0, 0}, {-8, 0}}, {{-8, 0}, {8, 0}},
            {{0, 0}, {0, 5}}, {{0, 5}, {0, -5}},
            {{0, 0}, {7, 4}}, {{7, 4}, {0, 0}},
            {{1, .5f}, {1.4f, .7f}}, {{-8, -5}, {8, 5}}
        }};
        for (const auto& transition : transitions)
        {
            std::array<EyeBehavior, 4> eyes{EyeBehavior{91}, EyeBehavior{91}, EyeBehavior{91}, EyeBehavior{91}};
            for (auto& eye : eyes)
            {
                eye.setAutoGaze(false);
                eye.setAutoBlink(false);
                eye.setGazeDegrees(transition.from);
                advance(eye, .5);
                JULIA_CHECK(eye.output().gazeDegrees == transition.from);
                eye.setGazeDegrees(transition.to);
                // A command plans a transition; it cannot change the rendered orientation.
                JULIA_CHECK(eye.output().gazeDegrees == transition.from);
                JULIA_CHECK(eye.motion().state == GazeState::Saccading);
                JULIA_CHECK(eye.motion().startDegrees == transition.from);
                JULIA_CHECK(eye.motion().targetDegrees == transition.to);
            }
            const auto difference = transition.to - transition.from;
            const float distanceSquared = glm::dot(difference, difference);
            const double duration = eyes[0].motion().durationSeconds;
            int intermediateFrames = 0;
            auto previous = transition.from;
            for (int checkpoint = 0; checkpoint < 15; ++checkpoint)
            {
                eyes[0].update(1.0 / 30.0);
                for (int i = 0; i < 2; ++i) eyes[1].update(1.0 / 60.0);
                for (int i = 0; i < 4; ++i) eyes[2].update(1.0 / 120.0);
                // Uneven presentation intervals have the same total time and fixed-clock result.
                eyes[3].update(.005);
                eyes[3].update(.013);
                eyes[3].update(1.0 / 30.0 - .018);
                for (std::size_t i = 1; i < eyes.size(); ++i)
                    JULIA_CHECK(same(eyes[0].output(), eyes[i].output()));
                const auto actual = eyes[0].output().gazeDegrees;
                const auto travelled = actual - transition.from;
                const float progress = glm::dot(travelled, difference) / distanceSquared;
                JULIA_CHECK(progress >= -1e-6f && progress <= 1.000001f);
                // Diagonal paths retain one coordinated direction, without overshoot.
                JULIA_CHECK(glm::length(travelled - difference * progress) < 1e-5f);
                JULIA_CHECK(glm::dot(actual - previous, difference) >= -1e-6f);
                JULIA_CHECK(glm::length(actual - previous) <= eyes[0].settings().saccadeMaxSpeedDegrees / 30.0f + 1e-4f);
                JULIA_CHECK(glm::length(eyes[0].motion().velocityDegreesPerSecond) <= eyes[0].settings().saccadeMaxSpeedDegrees + 1e-3f);
                if (actual != transition.from && actual != transition.to) ++intermediateFrames;
                if (static_cast<double>(checkpoint + 1) / 30.0 >= duration + 1.0 / 240.0)
                {
                    JULIA_CHECK(actual == transition.to);
                    JULIA_CHECK(eyes[0].motion().state == GazeState::Fixating);
                }
                previous = actual;
            }
            // Even the shortest move exposes multiple real intermediate frames at 30 Hz.
            JULIA_CHECK(intermediateFrames >= 2);
            JULIA_CHECK(eyes[0].output().gazeDegrees == transition.to);
            JULIA_CHECK(eyes[0].motion().velocityDegreesPerSecond == glm::vec2{0});
        }

        EyeBehavior eye;
        eye.setAutoGaze(false);
        eye.setAutoBlink(false);
        eye.setGazeDegrees({1, 0});
        const double shortDuration = eye.motion().durationSeconds;
        eye.setGazeDegrees({8, 0});
        JULIA_CHECK(eye.motion().durationSeconds > shortDuration);
        advance(eye, .05);
        const auto visible = eye.output().gazeDegrees;
        eye.setGazeDegrees({-6, 4});
        JULIA_CHECK(eye.output().gazeDegrees == visible);
        JULIA_CHECK(eye.motion().startDegrees == visible);
        advance(eye, .025);
        const double elapsed = eye.motion().elapsedSeconds;
        eye.setGazeDegrees({-6, 4});
        JULIA_CHECK(eye.motion().elapsedSeconds == elapsed);
        advance(eye, .3);
        JULIA_CHECK(eye.output().gazeDegrees == glm::vec2(-6, 4));

        eye.setGazeDegrees({8, -5});
        advance(eye, .025);
        const auto released = eye.output().gazeDegrees;
        eye.releaseGaze();
        advance(eye, .5);
        JULIA_CHECK(eye.output().gazeDegrees == released);
        JULIA_CHECK(eye.motion().state == GazeState::Fixating);

        eye.setGazeDegrees({8, 0});
        advance(eye, .025);
        const auto beforePause = eye.output();
        const double beforePauseTime = eye.motion().elapsedSeconds;
        eye.update(10.0);
        eye.update(0.0);
        JULIA_CHECK(same(eye.output(), beforePause));
        JULIA_CHECK(eye.motion().elapsedSeconds == beforePauseTime);
        eye.update(.25); // A long ordinary frame completes the existing trajectory safely.
        JULIA_CHECK(eye.output().gazeDegrees == glm::vec2(8, 0));
        JULIA_CHECK(eye.motion().state == GazeState::Fixating);

        EyeBehavior curve;
        curve.setAutoGaze(false);
        curve.setAutoBlink(false);
        curve.setGazeDegrees({8, 0});
        advance(curve, .025);
        const auto accelerating = curve.motion().velocityDegreesPerSecond.x;
        advance(curve, .025);
        const auto mainMovement = curve.motion().velocityDegreesPerSecond.x;
        advance(curve, .075);
        const auto braking = curve.motion().velocityDegreesPerSecond.x;
        JULIA_CHECK(accelerating > 0 && mainMovement > accelerating);
        JULIA_CHECK(braking >= 0 && braking < mainMovement);
        advance(curve, .1);
        JULIA_CHECK(curve.motion().velocityDegreesPerSecond == glm::vec2{0});

        // Expanded settings must plan inside the same envelope applied to bones.
        auto limits = curve.settings();
        limits.gazeYawDegrees = 15;
        limits.gazePitchDegrees = 10;
        limits.saccadeMaxSpeedDegrees = 30;
        curve.setSettings(limits);
        curve.setGazeDegrees({15, 10});
        const auto safeTarget = curve.motion().targetDegrees;
        JULIA_CHECK(glm::length(safeTarget / glm::vec2{12, 8}) <= 1.000001f);
        JULIA_CHECK(curve.motion().durationSeconds > limits.saccadeMaxSeconds);
        for (int tick = 0; tick < 480; ++tick)
        {
            curve.update(1.0 / 240.0);
            JULIA_CHECK(glm::length(curve.motion().velocityDegreesPerSecond) <= 30.0001f);
            JULIA_CHECK(glm::length(curve.output().gazeDegrees / glm::vec2{12, 8}) <= 1.000001f);
        }
        JULIA_CHECK(curve.output().gazeDegrees == safeTarget);
    }

    void checkTracking()
    {
        using namespace julia::render;
        EyeBehavior eye;
        eye.setAutoGaze(false);
        eye.setAutoBlink(false);
        eye.trackGazeDegrees({8, 5});
        JULIA_CHECK(eye.motion().state == GazeState::Tracking);
        JULIA_CHECK(eye.output().gazeDegrees == glm::vec2{0});
        auto previous = eye.output().gazeDegrees;
        for (int i = 0; i < 480; ++i)
        {
            eye.trackGazeDegrees({8, 5}); // A refreshed target does not restart an interpolation.
            eye.update(1.0 / 240.0);
            const auto current = eye.output().gazeDegrees;
            JULIA_CHECK(glm::length(current - previous) <= eye.settings().pursuitMaxSpeedDegrees / 240.0f + 1e-5f);
            JULIA_CHECK(current.x >= previous.x && current.x <= 8.0f);
            JULIA_CHECK(current.y >= previous.y && current.y <= 5.0f);
            previous = current;
        }
        JULIA_CHECK(eye.output().gazeDegrees == glm::vec2(8, 5));
        JULIA_CHECK(eye.motion().state == GazeState::Tracking);

        std::array<EyeBehavior, 4> trackingRates{EyeBehavior{31}, EyeBehavior{31}, EyeBehavior{31}, EyeBehavior{31}};
        for (auto& tracker : trackingRates)
        {
            tracker.setAutoGaze(false);
            tracker.setAutoBlink(false);
        }
        for (int frame = 0; frame < 240; ++frame)
        {
            const double t = static_cast<double>(frame + 1) / 30.0;
            const glm::vec2 moving{static_cast<float>(4 * std::sin(1.6 * t)), static_cast<float>(2 * std::sin(.8 * t))};
            for (auto& tracker : trackingRates) tracker.trackGazeDegrees(moving);
            trackingRates[0].update(1.0 / 30.0);
            for (int i = 0; i < 2; ++i) trackingRates[1].update(1.0 / 60.0);
            for (int i = 0; i < 4; ++i) trackingRates[2].update(1.0 / 120.0);
            trackingRates[3].update(.005);
            trackingRates[3].update(.013);
            trackingRates[3].update(1.0 / 30.0 - .018);
            for (std::size_t i = 1; i < trackingRates.size(); ++i)
                JULIA_CHECK(same(trackingRates[0].output(), trackingRates[i].output()));
            JULIA_CHECK(glm::length(moving - trackingRates[0].output().gazeDegrees) < .7f);
        }

        // A moving target at ordinary pursuit speeds has bounded lag and stable stopping.
        eye.reset(71);
        glm::vec2 target{0};
        for (int i = 0; i < 8 * 240; ++i)
        {
            const double t = static_cast<double>(i + 1) / 240.0;
            target = {static_cast<float>(4 * std::sin(1.6 * t)), static_cast<float>(2 * std::sin(.8 * t))};
            previous = eye.output().gazeDegrees;
            eye.trackGazeDegrees(target);
            JULIA_CHECK(eye.output().gazeDegrees == previous);
            eye.update(1.0 / 240.0);
            JULIA_CHECK(glm::length(eye.output().gazeDegrees - previous) <= eye.settings().pursuitMaxSpeedDegrees / 240.0f + 1e-5f);
            JULIA_CHECK(glm::length(target - eye.output().gazeDegrees) < .7f);
        }
        advance(eye, 2.0);
        JULIA_CHECK(eye.output().gazeDegrees == target);
        JULIA_CHECK(eye.motion().velocityDegreesPerSecond == glm::vec2{0});
        const auto held = eye.output().gazeDegrees;
        eye.releaseGaze();
        advance(eye, .5);
        JULIA_CHECK(eye.output().gazeDegrees == held);

        // Switching from a moving pursuit to a saccade starts from its visible pose.
        eye.trackGazeDegrees({-8, -5});
        advance(eye, .025);
        const auto handoff = eye.output().gazeDegrees;
        eye.setGazeDegrees({6, 3});
        JULIA_CHECK(eye.output().gazeDegrees == handoff);
        JULIA_CHECK(eye.motion().startDegrees == handoff);
        advance(eye, .3);
        JULIA_CHECK(eye.output().gazeDegrees == glm::vec2(6, 3));
    }
}

int main()
{
    using namespace julia::render;

    checkSaccades();
    checkTracking();

    // Different presentation frame rates produce the same fixed-clock behavior at every second.
    std::array<EyeBehavior, 3> rates{EyeBehavior{12345}, EyeBehavior{12345}, EyeBehavior{12345}};
    constexpr std::array<int, 3> fps{24, 60, 144};
    for (int second = 0; second < 120; ++second)
    {
        for (std::size_t r = 0; r < rates.size(); ++r)
            for (int frame = 0; frame < fps[r]; ++frame) rates[r].update(1.0 / fps[r]);
        JULIA_CHECK(same(rates[0].output(), rates[1].output()));
        JULIA_CHECK(same(rates[0].output(), rates[2].output()));
        JULIA_CHECK(rates[0].stats().blinkCount == rates[2].stats().blinkCount);
    }

    EyeBehavior controlled{27};
    controlled.setAutoGaze(false);
    controlled.setAutoBlink(false);
    advance(controlled, 10.0);
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2{0.0f});
    JULIA_CHECK(controlled.stats().blinkCount == 0);
    controlled.setGazeDegrees({3.0f, -2.0f});
    advance(controlled, 0.05);
    JULIA_CHECK(controlled.output().gazeDegrees.x > 0.0f && controlled.output().gazeDegrees.x < 3.0f);
    for (int i = 0; i < 36; ++i)
    {
        controlled.setGazeDegrees({3.0f, -2.0f}); // Repeated input must not restart the movement.
        controlled.update(1.0 / 240.0);
    }
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2(3.0f, -2.0f));
    advance(controlled, 5.0);
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2(3.0f, -2.0f));
    controlled.setAutoGaze(true); // Held target has priority over idle behavior.
    advance(controlled, 10.0);
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2(3.0f, -2.0f));
    controlled.neutral();
    advance(controlled, 0.2);
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2{0.0f});
    controlled.setGazeDegrees({200.0f, -200.0f});
    advance(controlled, 0.2);
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2(8.0f, -5.0f));
    controlled.releaseGaze();
    advance(controlled, 10.0);
    JULIA_CHECK(controlled.output().gazeDegrees != glm::vec2(8.0f, -5.0f));

    controlled.setGazeDegrees({-3.0f, 2.0f});
    advance(controlled, 0.025); // A hidden idle/manual transition may still be in flight.
    controlled.releaseGazeFrom({6.0f, -4.0f});
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2(6.0f, -4.0f));
    advance(controlled, 0.2);
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2(6.0f, -4.0f));
    advance(controlled, 10.0);
    JULIA_CHECK(controlled.output().gazeDegrees != glm::vec2(6.0f, -4.0f));
    controlled.setAutoGaze(false);
    controlled.releaseGazeFrom({200.0f, -200.0f});
    advance(controlled, 10.0);
    JULIA_CHECK(controlled.output().gazeDegrees == glm::vec2(8.0f, -5.0f));

    controlled.setBlink(1.5f, -1.0f);
    controlled.setAutoBlink(true);
    advance(controlled, 30.0);
    JULIA_CHECK(controlled.output().blinkLeft == 1.0f && controlled.output().blinkRight == 0.0f);
    JULIA_CHECK(controlled.autoBlink());
    JULIA_CHECK(controlled.stats().blinkCount == 0);
    controlled.releaseBlink();
    advance(controlled, 0.05);
    JULIA_CHECK(controlled.output().blinkLeft > 0.0f && controlled.output().blinkLeft < 1.0f);
    advance(controlled, 0.15);
    JULIA_CHECK(controlled.output().blinkLeft == 0.0f);
    controlled.setAutoBlink(false);
    controlled.triggerBlink(true, false);
    float maxLeft = 0.0f;
    for (int i = 0; i < 120; ++i)
    {
        controlled.update(1.0 / 240.0);
        maxLeft = std::max(maxLeft, controlled.output().blinkLeft);
        JULIA_CHECK(controlled.output().blinkRight == 0.0f);
    }
    JULIA_CHECK(maxLeft == 1.0f && controlled.output().blinkLeft == 0.0f);
    JULIA_CHECK(controlled.stats().blinkCount == 1);
    controlled.triggerBlink(false, true);
    float maxRight = 0.0f;
    for (int i = 0; i < 120; ++i)
    {
        controlled.update(1.0 / 240.0);
        maxRight = std::max(maxRight, controlled.output().blinkRight);
        JULIA_CHECK(controlled.output().blinkLeft == 0.0f);
    }
    JULIA_CHECK(maxRight == 1.0f && controlled.output().blinkRight == 0.0f);
    controlled.triggerBlink(false, false);
    JULIA_CHECK(controlled.stats().blinkCount == 2);

    // Switching from a manually closed lid to a blink starts from its visible closure.
    controlled.setBlink(0.6f, 0.7f);
    controlled.triggerBlink(true, false);
    controlled.update(1.0 / 240.0);
    JULIA_CHECK(controlled.output().blinkLeft >= 0.6f && controlled.output().blinkLeft < 0.65f);
    JULIA_CHECK(controlled.output().blinkRight < 0.7f && controlled.output().blinkRight > 0.65f);
    advance(controlled, 0.5);
    JULIA_CHECK(controlled.output().blinkLeft == 0.0f && controlled.output().blinkRight == 0.0f);

    // A suspended desktop does not consume elapsed sleep time or jump its pose on resume.
    controlled.triggerBlink();
    advance(controlled, 0.025);
    const auto paused = controlled.output();
    controlled.update(600.0);
    JULIA_CHECK(same(paused, controlled.output()));
    advance(controlled, 0.5);
    JULIA_CHECK(controlled.output().blinkLeft == 0.0f && controlled.output().blinkRight == 0.0f);

    EyeBehavior soak{0};
    std::vector<double> primaryIntervals;
    std::uint64_t previousCount = 0;
    std::uint64_t previousDouble = 0;
    double lastBlink = 0.0;
    double closedSeconds = 0.0;
    double maxClosedSeconds = 0.0;
    double gazeSquaredSum = 0.0;
    glm::vec2 previousGaze = soak.output().gazeDegrees;
    auto previousMotion = soak.motion();
    double maxGazeStep = 0.0;
    double maxFixationDrift = 0.0;
    std::uint64_t completedSaccades = 0;
    constexpr int SoakTicks = 3600 * 240;
    for (int i = 0; i < SoakTicks; ++i)
    {
        soak.update(1.0 / 240.0);
        const auto& pose = soak.output();
        JULIA_CHECK(std::isfinite(pose.gazeDegrees.x) && std::isfinite(pose.gazeDegrees.y));
        JULIA_CHECK(std::abs(pose.gazeDegrees.x) <= 8.0f && std::abs(pose.gazeDegrees.y) <= 5.0f);
        JULIA_CHECK(pose.blinkLeft >= 0.0f && pose.blinkLeft <= 1.0f && pose.blinkLeft == pose.blinkRight);
        const auto& motion = soak.motion();
        const double gazeStep = glm::length(pose.gazeDegrees - previousGaze);
        maxGazeStep = std::max(maxGazeStep, gazeStep);
        JULIA_CHECK(gazeStep <= soak.settings().saccadeMaxSpeedDegrees / 240.0 + 1e-5);
        JULIA_CHECK(glm::length(motion.velocityDegreesPerSecond) <= soak.settings().saccadeMaxSpeedDegrees + 1e-3f);
        if (motion.state == GazeState::Fixating)
        {
            JULIA_CHECK(pose.gazeDegrees == motion.targetDegrees);
            JULIA_CHECK(motion.velocityDegreesPerSecond == glm::vec2{0});
            if (previousMotion.state == GazeState::Fixating)
                maxFixationDrift = std::max(maxFixationDrift, gazeStep);
            else if (previousMotion.state == GazeState::Saccading)
                ++completedSaccades;
        }
        else if (motion.state == GazeState::Saccading)
        {
            JULIA_CHECK(motion.durationSeconds > 0 && std::isfinite(motion.durationSeconds));
            JULIA_CHECK(motion.elapsedSeconds <= motion.durationSeconds + 1.0 / 240.0);
            if (previousMotion.state == GazeState::Saccading && previousMotion.targetDegrees == motion.targetDegrees)
            {
                JULIA_CHECK(motion.elapsedSeconds > previousMotion.elapsedSeconds);
                JULIA_CHECK(glm::length(motion.targetDegrees - pose.gazeDegrees) <=
                    glm::length(previousMotion.targetDegrees - previousGaze) + 1e-5f);
            }
        }
        previousGaze = pose.gazeDegrees;
        previousMotion = motion;
        closedSeconds = pose.blinkLeft > 0.01f ? closedSeconds + 1.0 / 240.0 : 0.0;
        maxClosedSeconds = std::max(maxClosedSeconds, closedSeconds);
        gazeSquaredSum += pose.gazeDegrees.x * pose.gazeDegrees.x + pose.gazeDegrees.y * pose.gazeDegrees.y;
        if (soak.stats().blinkCount != previousCount)
        {
            const double time = static_cast<double>(i + 1) / 240.0;
            if (previousCount > 0 && soak.stats().doubleBlinkCount == previousDouble)
                primaryIntervals.push_back(time - lastBlink);
            previousCount = soak.stats().blinkCount;
            previousDouble = soak.stats().doubleBlinkCount;
            lastBlink = time;
        }
    }
    JULIA_CHECK(soak.stats().blinkCount > 700 && soak.stats().blinkCount < 1100);
    JULIA_CHECK(soak.stats().doubleBlinkCount > 20 && soak.stats().doubleBlinkCount < 130);
    JULIA_CHECK(soak.stats().slowBlinkCount > 15 && soak.stats().slowBlinkCount < 100);
    JULIA_CHECK(maxClosedSeconds < 0.45);
    JULIA_CHECK(std::sqrt(gazeSquaredSum / SoakTicks) < 2.5); // Most attention remains near forward.
    JULIA_CHECK(completedSaccades > 500);
    JULIA_CHECK(maxFixationDrift == 0.0);
    const auto [smallest, largest] = std::minmax_element(primaryIntervals.begin(), primaryIntervals.end());
    JULIA_CHECK(*smallest > 2.2 && *largest < 6.3);
    JULIA_CHECK(*largest - *smallest > 2.0);
    double mean = 0.0;
    for (double interval : primaryIntervals) mean += interval;
    mean /= static_cast<double>(primaryIntervals.size());
    JULIA_CHECK(mean > 3.9 && mean < 4.6);
    std::printf("EyeBehavior 1h: %llu blinks, %llu doubles, %llu slow; main intervals %.3f..%.3f s (mean %.3f), max closure %.3f s\n",
        static_cast<unsigned long long>(soak.stats().blinkCount),
        static_cast<unsigned long long>(soak.stats().doubleBlinkCount),
        static_cast<unsigned long long>(soak.stats().slowBlinkCount), *smallest, *largest, mean, maxClosedSeconds);
    std::printf("EyeBehavior 1h gaze: %llu completed saccades, max 240Hz step %.6f deg, fixation drift %.6f deg\n",
        static_cast<unsigned long long>(completedSaccades), maxGazeStep, maxFixationDrift);

    // Reset recreates the seeded sequence and keeps explicit configuration.
    EyeBehavior resetA{42}, resetB{42};
    advance(resetA, 20.0);
    resetA.reset(42);
    advance(resetA, 12.0);
    advance(resetB, 12.0);
    JULIA_CHECK(same(resetA.output(), resetB.output()));
    JULIA_CHECK(resetA.stats().blinkCount == resetB.stats().blinkCount);

    EyeBehaviorSettings invalid;
    invalid.blinkMeanSeconds = 0.0;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.blinkVariability = 0.9;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.doubleBlinkProbability = -1.0;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.gazeYawDegrees = std::numeric_limits<float>::quiet_NaN();
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.saccadeMinSeconds = -1;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.saccadeMaxSeconds = invalid.saccadeMinSeconds / 2;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.saccadeSecondsPerDegree = -1;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.saccadeMaxSpeedDegrees = 0;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.pursuitTimeConstant = 0;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.pursuitMaxSpeedDegrees = 0;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.fixationMinSeconds = 0;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.fixationMaxSeconds = invalid.fixationMinSeconds / 2;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    invalid = {}; invalid.correctionAmplitudeDegrees = .21f;
    JULIA_CHECK(rejects([&] { soak.setSettings(invalid); }));
    JULIA_CHECK(rejects([&] { soak.setGazeDegrees({std::numeric_limits<float>::infinity(), 0}); }));
    JULIA_CHECK(rejects([&] { soak.trackGazeDegrees({0, std::numeric_limits<float>::infinity()}); }));
    JULIA_CHECK(rejects([&] { soak.releaseGazeFrom({0, std::numeric_limits<float>::quiet_NaN()}); }));
    JULIA_CHECK(rejects([&] { soak.setBlink(std::numeric_limits<float>::quiet_NaN(), 0); }));
    JULIA_CHECK(rejects([&] { soak.update(-0.01); }));
    JULIA_CHECK(rejects([&] { soak.update(std::numeric_limits<double>::infinity()); }));

    return JULIA_TEST_RESULT();
}
