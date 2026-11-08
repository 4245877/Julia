#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace julia::runtime
{
    inline constexpr double BodyStepSeconds = 1.0 / 120.0;
    struct SchedulerDiagnostics
    {
        std::uint64_t overloadFrames = 0, suspendedFrames = 0;
        double discardedSeconds = 0, pausedSeconds = 0;
    };
    // Sequential CPU scheduler. It owns wall-time accumulation, never character state.
    class FixedStepScheduler
    {
    public:
        explicit FixedStepScheduler(std::uint32_t maxSteps = 8) : maxSteps_(maxSteps)
        { if (!maxSteps) throw std::invalid_argument("Zero simulation step budget"); }
        void setPaused(bool paused)
        {
            if (paused == paused_) return;
            diagnostics_.pausedSeconds += accumulator_;
            paused_ = paused; accumulator_ = 0;
        }
        bool paused() const { return paused_; }
        double alpha() const { return accumulator_ / BodyStepSeconds; }
        const SchedulerDiagnostics& diagnostics() const { return diagnostics_; }

        // A minimized window or >0.5 s OS gap freezes the body and discards the gap.
        // Ordinary overload drops whole excess ticks, retaining only the fraction.
        template<class Tick> std::uint32_t advance(double seconds, Tick&& tick, bool suspended = false)
        {
            if (!std::isfinite(seconds) || seconds < 0) throw std::invalid_argument("Invalid wall delta");
            if (paused_)
            { diagnostics_.pausedSeconds += seconds; accumulator_ = 0; return 0; }
            if (suspended || seconds > .5)
            {
                ++diagnostics_.suspendedFrames;
                diagnostics_.discardedSeconds += seconds + accumulator_;
                accumulator_ = 0;
                return 0;
            }
            accumulator_ += seconds;
            const auto available = static_cast<std::uint64_t>(std::floor((accumulator_ + 1e-12) / BodyStepSeconds));
            const auto count = static_cast<std::uint32_t>(std::min<std::uint64_t>(available, maxSteps_));
            if (available > maxSteps_)
            {
                ++diagnostics_.overloadFrames;
                const auto dropped = static_cast<double>(available - maxSteps_) * BodyStepSeconds;
                diagnostics_.discardedSeconds += dropped;
                accumulator_ -= dropped;
            }
            for (std::uint32_t i = 0; i < count; ++i) tick();
            accumulator_ = std::clamp(accumulator_ - count * BodyStepSeconds, 0.0, BodyStepSeconds);
            return count;
        }
        // Offline mode: no wall clock, budget or dropped steps; pause is explicit.
        template<class Tick> void runTicks(std::uint64_t count, Tick&& tick)
        {
            if (paused_) throw std::logic_error("Exact run requested while simulation is paused");
            for (std::uint64_t i = 0; i < count; ++i) tick();
        }
    private:
        std::uint32_t maxSteps_;
        double accumulator_ = 0;
        bool paused_ = false;
        SchedulerDiagnostics diagnostics_{};
    };
}
