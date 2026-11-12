#include "BoundedQp.hpp"

namespace julia::physics::qp
{
    const char* name(Code c)
    {
        switch(c) { case Code::Feasible:return "feasible"; case Code::Infeasible:return "infeasible";
            case Code::Budget:return "iteration_budget"; case Code::Numerical:return "numerical_failure"; case Code::Invalid:return "invalid_problem"; }
        return "invalid_problem";
    }
    Result Solver::solve(const Problem& p,unsigned budget,Implementation implementation)
    {
        profile_={};
        if(implementation==Implementation::Reference7h) return solveReference(p,budget);
        if(implementation==Implementation::Optimized7i) return solveOptimized(p,budget);
        return {};
    }
}
