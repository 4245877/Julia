#pragma once
#include <array>
#include <cstddef>

namespace julia::physics::qp
{
    constexpr int Variables=71,Equalities=18,Inequalities=128,Dimension=Variables+Equalities;
    enum class Code { Feasible, Infeasible, Budget, Numerical, Invalid };
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
        Result solve(const Problem& p,unsigned budget=128);
    private:
        std::array<double,Dimension*Dimension> inverse_{},matrix_{};
        std::array<double,Variables*Inequalities> projected_{};
        std::array<double,Variables*Variables> gram_{};
        std::array<double,Variables> rhs_{},direction_{};
        std::array<int,Variables> active_{};
        std::array<double,Variables> dual_{};
    };
}
