#include "julia/render/EyeBehavior.hpp"
#include "julia_test.hpp"

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
}

int main()
{
    using namespace julia::render;

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
    constexpr int SoakTicks = 3600 * 240;
    for (int i = 0; i < SoakTicks; ++i)
    {
        soak.update(1.0 / 240.0);
        const auto& pose = soak.output();
        JULIA_CHECK(std::isfinite(pose.gazeDegrees.x) && std::isfinite(pose.gazeDegrees.y));
        JULIA_CHECK(std::abs(pose.gazeDegrees.x) <= 8.0f && std::abs(pose.gazeDegrees.y) <= 5.0f);
        JULIA_CHECK(pose.blinkLeft >= 0.0f && pose.blinkLeft <= 1.0f && pose.blinkLeft == pose.blinkRight);
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
    JULIA_CHECK(rejects([&] { soak.setGazeDegrees({std::numeric_limits<float>::infinity(), 0}); }));
    JULIA_CHECK(rejects([&] { soak.releaseGazeFrom({0, std::numeric_limits<float>::quiet_NaN()}); }));
    JULIA_CHECK(rejects([&] { soak.setBlink(std::numeric_limits<float>::quiet_NaN(), 0); }));
    JULIA_CHECK(rejects([&] { soak.update(-0.01); }));
    JULIA_CHECK(rejects([&] { soak.update(std::numeric_limits<double>::infinity()); }));

    return JULIA_TEST_RESULT();
}
