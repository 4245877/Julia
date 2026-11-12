#pragma once
#include "BoundedQp.hpp"
#include <chrono>

namespace julia::physics::qp
{
    // Diagnostic only. All exits are measured; time never enters solver decisions.
    class PhaseTimer
    {
        using Clock=std::chrono::steady_clock;
        Clock::time_point start_=Clock::now();
        double* section_;
    public:
        explicit PhaseTimer(Profile& p):section_(&p.inputUs) {}
        void next(double& section)
        {
            const auto now=Clock::now();
            *section_+=std::chrono::duration<double,std::micro>(now-start_).count();
            start_=now; section_=&section;
        }
        ~PhaseTimer() { *section_+=std::chrono::duration<double,std::micro>(Clock::now()-start_).count(); }
    };
}
