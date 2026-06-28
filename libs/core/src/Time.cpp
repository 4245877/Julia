#include "julia/core/Time.hpp"

namespace julia::core
{
    namespace
    {
        Seconds toSeconds(Clock::clock_type::duration duration)
        {
            return std::chrono::duration<Seconds>(duration).count();
        }
    }

    Seconds Clock::tick()
    {
        const time_point current = clock_type::now();
        const Seconds delta = toSeconds(current - last_);
        last_ = current;
        return delta;
    }

    Seconds Clock::elapsed() const
    {
        return toSeconds(clock_type::now() - start_);
    }

    void Clock::reset()
    {
        start_ = clock_type::now();
        last_ = start_;
    }

    Seconds now()
    {
        return toSeconds(Clock::clock_type::now().time_since_epoch());
    }
}
