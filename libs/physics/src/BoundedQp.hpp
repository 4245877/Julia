#pragma once
#include <array>
#include <cstddef>

namespace julia::physics::qp
{
    constexpr int Variables=71,Equalities=18,Inequalities=128,Dimension=Variables+Equalities;
    enum class Code { Feasible, Infeasible, Budget, Numerical, Invalid };
    enum class Implementation { Reference7h, Optimized7i };
    struct Profile { double inputUs=0,linearUs=0,activeUs=0,verifyUs=0; };
    const char* name(Code code);
    // min .5*x'H*x-g'x; E*x=d; A*x>=b. Rows use fixed strides.
    struct Problem
    {
        int n=0,e=0,m=0;
        std::array<double,Variables*Variables> h{};
        std::array<double,Variables> g{};
        std::array<double,Equalities*Variables> equality{};
        std::array<double,Equalities> target{};
        std::array<double,Inequalities*Variables> inequality{};
        std::array<double,Inequalities> lower{};
    };
    struct Result
    {
        Code code=Code::Invalid;
        std::array<double,Variables> x{};
        std::array<double,Inequalities> multipliers{},certificate{};
        std::array<double,Equalities> equalityCertificate{};
        unsigned iterations=0,active=0;
        double equalityResidual=0,violation=0,projectedStationarity=0,certificateMargin=0;
    };
    // Deterministic cold start each solve. No retained primal/dual state to leak across reset.
    // Fixed workspace, no per-solve allocation, bounded dual active-set transitions.
    class Solver
    {
    public:
        Result solve(const Problem& p,unsigned budget=128,Implementation implementation=Implementation::Reference7h);
        const Profile& profile() const { return profile_; }
    private:
        Result solveReference(const Problem& p,unsigned budget);
        Result solveOptimized(const Problem& p,unsigned budget);
        Profile profile_;
        std::array<double,Dimension*Dimension> inverse_{},matrix_{};
        std::array<double,Variables*Inequalities> projected_{};
        std::array<double,Variables*Variables> gram_{};
        std::array<double,Variables> rhs_{},direction_{};
        std::array<int,Variables> active_{};
        std::array<double,Variables> dual_{};
    };
}
