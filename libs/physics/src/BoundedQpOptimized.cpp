#include "BoundedQp.hpp"
#include "BoundedQpTiming.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace julia::physics::qp
{
    Result Solver::solveOptimized(const Problem& p,unsigned budget)
    {
        PhaseTimer timer{profile_};
        Result out;
        const int n=p.n,e=p.e,m=p.m,k=n+e;
        if(n<1||n>Variables||e<0||e>Equalities||m<0||m>Inequalities||budget>1024) return out;
        const auto finite=[](const auto& a) { return std::all_of(a.begin(),a.end(),[](double x){return std::isfinite(x);}); };
        if(!finite(p.h)||!finite(p.g)||!finite(p.equality)||!finite(p.target)||!finite(p.inequality)||!finite(p.lower)) return out;
        timer.next(profile_.linearUs);
        matrix_={}; inverse_={}; dual_={};
        for(int i=0;i<n;++i) for(int j=0;j<n;++j) matrix_[i*Dimension+j]=p.h[i*Variables+j];
        for(int r=0;r<e;++r) for(int j=0;j<n;++j)
            matrix_[j*Dimension+n+r]=matrix_[(n+r)*Dimension+j]=p.equality[r*Variables+j];
        for(int i=0;i<k;++i) inverse_[i*Dimension+i]=1;
        // Gauss-Jordan with deterministic largest-absolute partial pivots.
        for(int c=0;c<k;++c)
        {
            int pivot=c; for(int r=c+1;r<k;++r) if(std::abs(matrix_[r*Dimension+c])>std::abs(matrix_[pivot*Dimension+c])) pivot=r;
            const double v=matrix_[pivot*Dimension+c];
            if(std::abs(v)<1e-13||!std::isfinite(v)) { out.code=Code::Numerical; return out; }
            if(pivot!=c) for(int j=0;j<k;++j) { std::swap(matrix_[c*Dimension+j],matrix_[pivot*Dimension+j]); std::swap(inverse_[c*Dimension+j],inverse_[pivot*Dimension+j]); }
            for(int j=0;j<k;++j) { matrix_[c*Dimension+j]/=v; inverse_[c*Dimension+j]/=v; }
            for(int r=0;r<k;++r) if(r!=c)
            {
                const double f=matrix_[r*Dimension+c]; matrix_[r*Dimension+c]=0;
                for(int j=c+1;j<k;++j) matrix_[r*Dimension+j]-=f*matrix_[c*Dimension+j];
                for(int j=0;j<k;++j) inverse_[r*Dimension+j]-=f*inverse_[c*Dimension+j];
            }
        }
        for(int i=0;i<n;++i)
        {
            for(int j=0;j<n;++j) out.x[i]+=inverse_[i*Dimension+j]*p.g[j];
            for(int r=0;r<e;++r) out.x[i]+=inverse_[i*Dimension+n+r]*p.target[r];
        }
        // P is the equality-nullspace inverse Hessian; precompute P*A'.
        for(int r=0;r<m;++r) for(int i=0;i<n;++i)
        { double v=0; for(int j=0;j<n;++j) v+=inverse_[i*Dimension+j]*p.inequality[r*Variables+j]; projected_[r*Variables+i]=v; }
        timer.next(profile_.activeUs);
        int count=0;
        const auto dot=[&](int a,int b) { double v=0; for(int j=0;j<n;++j) v+=p.inequality[a*Variables+j]*projected_[b*Variables+j]; return v; };
        while(true)
        {
            int next=-1; double worst=1e-8;
            for(int r=0;r<m;++r)
            { double v=p.lower[r]; for(int j=0;j<n;++j) v-=p.inequality[r*Variables+j]*out.x[j]; if(v>worst) { next=r; worst=v; } }
            if(next<0) { out.code=Code::Feasible; break; }
            double added=0;
            while(true)
            {
                if(out.iterations>=budget) { out.code=Code::Budget; return out; } ++out.iterations;
                for(int i=0;i<count;++i) { rhs_[i]=dot(active_[i],next); for(int j=0;j<count;++j) gram_[i*Variables+j]=dot(active_[i],active_[j]); }
                // Active Gram is SPD for independent constraints.
                for(int c=0;c<count;++c)
                {
                    const double v=gram_[c*Variables+c];
                    if(v<1e-13||!std::isfinite(v)) { out.code=Code::Numerical; return out; }
                    for(int r=c+1;r<count;++r)
                    { const double f=gram_[r*Variables+c]/v; for(int j=c+1;j<count;++j) gram_[r*Variables+j]-=f*gram_[c*Variables+j]; rhs_[r]-=f*rhs_[c]; }
                }
                for(int i=count;i-->0;) { for(int j=i+1;j<count;++j) rhs_[i]-=gram_[i*Variables+j]*rhs_[j]; rhs_[i]/=gram_[i*Variables+i]; }
                for(int j=0;j<n;++j) { direction_[j]=projected_[next*Variables+j]; for(int i=0;i<count;++i) direction_[j]-=rhs_[i]*projected_[active_[i]*Variables+j]; }
                double denominator=0,violation=p.lower[next];
                for(int j=0;j<n;++j) { denominator+=p.inequality[next*Variables+j]*direction_[j]; violation-=p.inequality[next*Variables+j]*out.x[j]; }
                const double infinity=std::numeric_limits<double>::infinity();
                const double full=denominator>1e-11?std::max(0.,violation/denominator):infinity;
                double partial=infinity; int drop=-1;
                for(int i=0;i<count;++i) if(rhs_[i]>1e-10&&dual_[i]/rhs_[i]<partial) { partial=dual_[i]/rhs_[i]; drop=i; }
                if(!std::isfinite(full)&&!std::isfinite(partial))
                {
                    // Farkas witness: A' certificate + E' equalityCertificate=0,
                    // certificate>=0 and b'certificate+d'equalityCertificate>0.
                    out.certificate[next]=1;
                    for(int i=0;i<count;++i) out.certificate[active_[i]]=-rhs_[i];
                    for(int r=0;r<e;++r) for(int i=0;i<n;++i)
                    { double a=0; for(int t=0;t<m;++t) a+=p.inequality[t*Variables+i]*out.certificate[t]; out.equalityCertificate[r]-=inverse_[(n+r)*Dimension+i]*a; }
                    out.certificateMargin=0;
                    for(int r=0;r<m;++r) out.certificateMargin+=out.certificate[r]*p.lower[r];
                    for(int r=0;r<e;++r) out.certificateMargin+=out.equalityCertificate[r]*p.target[r];
                    double residual=0,minDual=0;
                    for(int r=0;r<m;++r) minDual=std::min(minDual,out.certificate[r]);
                    for(int j=0;j<n;++j)
                    { double a=0; for(int r=0;r<m;++r) a+=out.certificate[r]*p.inequality[r*Variables+j]; for(int r=0;r<e;++r) a+=out.equalityCertificate[r]*p.equality[r*Variables+j]; residual=std::max(residual,std::abs(a)); }
                    out.code=residual<1e-6&&minDual>=-1e-8&&out.certificateMargin>1e-8?Code::Infeasible:Code::Numerical; return out;
                }
                const double step=std::min(full,partial);
                for(int j=0;j<n;++j) out.x[j]+=step*direction_[j];
                for(int i=0;i<count;++i) dual_[i]-=step*rhs_[i]; added+=step;
                if(full<=partial)
                {
                    if(count>=n-e) { out.code=Code::Numerical; return out; }
                    active_[count]=next; dual_[count++]=added; break;
                }
                for(int i=drop;i<count-1;++i) { active_[i]=active_[i+1]; dual_[i]=dual_[i+1]; } --count;
            }
        }
        timer.next(profile_.verifyUs);
        out.active=static_cast<unsigned>(count);
        for(int i=0;i<count;++i) out.multipliers[active_[i]]=dual_[i];
        for(int r=0;r<e;++r) { double v=-p.target[r]; for(int j=0;j<n;++j) v+=p.equality[r*Variables+j]*out.x[j]; out.equalityResidual=std::max(out.equalityResidual,std::abs(v)); }
        for(int r=0;r<m;++r) { double v=p.lower[r]; for(int j=0;j<n;++j) v-=p.inequality[r*Variables+j]*out.x[j]; out.violation=std::max(out.violation,v); }
        // Gradient is independent of projected row i: form it exactly once.
        for(int j=0;j<n;++j)
        {
            double grad=-p.g[j];
            for(int t=0;t<n;++t) grad+=p.h[j*Variables+t]*out.x[t];
            for(int t=0;t<m;++t) grad-=p.inequality[t*Variables+j]*out.multipliers[t];
            rhs_[j]=grad;
        }
        for(int i=0;i<n;++i)
        {
            double v=0;
            for(int j=0;j<n;++j) v+=inverse_[i*Dimension+j]*rhs_[j];
            out.projectedStationarity=std::max(out.projectedStationarity,std::abs(v));
        }
        if(out.equalityResidual>1e-6||out.violation>1e-7||out.projectedStationarity>1e-5||!finite(out.x)) out.code=Code::Numerical;
        return out;
    }
}
