#include "StandControl.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace julia::physics
{
    bool StandControl::inspectQp(qp::Problem& problem,qp::Result& result) const
    {
        if(status.variant!=StandVariant::Bounded7h||status.sampleTime==0) return false;
        problem=problem_; result=result_; return true;
    }
    bool StandControl::bounded(const mjModel*,const mjData* d,const StateLayout& layout,const Definition& def,
        const std::array<double,47*47>& mass,const std::array<double,12*47>& j,
        const std::array<double,59*59>& cost,const std::array<double,59>& task,
        const std::array<double,12>& contact,const std::array<double,12>& jdot,
        const std::array<glm::dvec3,2>& points,const std::array<glm::dmat3,2>& rotations)
    {
        const auto start=std::chrono::steady_clock::now();
        constexpr int N=47,F=24,V=qp::Variables;
        auto& p=problem_; p={}; p.n=N+F; p.e=18;
        std::array<double,12*F> wrenchMap{};
        std::array<double,N*F> generalized{};
        std::array<glm::dmat3,2> forceFrames{};
        for(int side=0;side<2;++side)
        {
            // The stationary floor sets the reaction normal, even when a soft
            // foot tilts. Its heading only selects horizontal tangent axes.
            const auto x=glm::normalize(glm::dvec3{rotations[side][0].x,0,rotations[side][0].z});
            forceFrames[side]=glm::dmat3{x,glm::dvec3{0,1,0},glm::cross(x,glm::dvec3{0,1,0})};
        }
        // Variables are world acceleration and floor-frame corner force /100N.
        for(int side=0;side<2;++side) for(int c=0;c<4;++c)
        {
            const auto arm=rotations[side]*glm::dvec3{(c&1)?.045:-.045,0,(c&2)?.085:-.085};
            status.cornerPoints[side][c]=points[side]+arm;
            for(int axis=0;axis<3;++axis)
            {
                const int f=side*12+c*3+axis; const auto force=100.*forceFrames[side][axis],moment=glm::cross(arm,force);
                for(int r=0;r<3;++r) { wrenchMap[(side*6+r)*F+f]=force[r]; wrenchMap[(side*6+3+r)*F+f]=moment[r]; }
            }
        }
        for(int r=0;r<N;++r) for(int f=0;f<F;++f) for(int t=0;t<12;++t) generalized[r*F+f]+=j[t*N+r]*wrenchMap[t*F+f];
        for(int i=0;i<N;++i) { p.g[i]=task[i]; for(int k=0;k<N;++k) p.h[i*V+k]=cost[i*59+k]; }
        const std::array<double,6> weights{.1,.005,.1,.1,5,.1};
        // Small wrench objective: accelerations remain soft and are revised by all inequalities.
        for(int r=0;r<12;++r)
        {
            const auto side=r/6,axis=r%6; const double weight=.01*weights[axis]*weights[axis];
            const double target=axis<3?settings_.nominalReaction[side].force[axis]:settings_.nominalReaction[side].moment[axis-3];
            for(int i=0;i<F;++i)
            { p.g[N+i]+=weight*wrenchMap[r*F+i]*target; for(int k=0;k<F;++k) p.h[(N+i)*V+N+k]+=weight*wrenchMap[r*F+i]*wrenchMap[r*F+k]; }
        }
        for(int i=0;i<F;++i) p.h[(N+i)*V+N+i]+=1e-6;
        if(settings_.loadCommands&&settings_.loadTask&&status.load.baselineValid)
        {
            // Dimensionless differential normal force. Resultant/free-root dynamics
            // and every previous hard bound remain in the same coupled QP.
            std::array<double,F> row{};
            for(int c=0;c<4;++c) { row[c*3+1]=1-status.load.reference; row[12+c*3+1]=-status.load.reference; }
            for(int i=0;i<F;++i) for(int k=0;k<F;++k) p.h[(N+i)*V+N+k]+=row[i]*row[k];
        }
        for(int r=0;r<12;++r)
        { p.target[r]=contact[r]-jdot[r]; for(int i=0;i<N;++i) p.equality[r*V+i]=j[r*N+i]; }
        for(int r=0;r<6;++r)
        {
            const auto v=layout.rootQvel+r; p.target[12+r]=-(d->qfrc_bias[v]-d->qfrc_passive[v])/100.;
            for(int i=0;i<N;++i) p.equality[(12+r)*V+i]=mass[v*N+i]/100.;
            for(int i=0;i<F;++i) p.equality[(12+r)*V+N+i]=-generalized[v*F+i]/100.;
        }
        for(unsigned k=0;k<41;++k)
        {
            const auto v=layout.qvel[k]; const auto cap=def.drives[k].maxTorque;
            for(int sign:{-1,1})
            {
                const int r=p.m++; p.lower[r]=-1-sign*(d->qfrc_bias[v]-d->qfrc_passive[v])/cap;
                for(int i=0;i<N;++i) p.inequality[r*V+i]=sign*mass[v*N+i]/cap;
                for(int i=0;i<F;++i) p.inequality[r*V+N+i]=-sign*generalized[v*F+i]/cap;
            }
        }
        for(int c=0;c<8;++c)
        {
            const int i=N+3*c; int r=p.m++; p.inequality[r*V+i+1]=1;
            // Inner friction pyramid: diamond in the tangent plane, wholly inside mu=.6 cone.
            for(int sx:{-1,1}) for(int sz:{-1,1})
            { r=p.m++; p.inequality[r*V+i]=-sx; p.inequality[r*V+i+1]=.6; p.inequality[r*V+i+2]=-sz; }
        }
        for(int side=0;side<2;++side)
        { const int r=p.m++; p.lower[r]=.3; for(int c=0;c<4;++c) p.inequality[r*V+N+side*12+c*3+1]=1; }
        const auto prepared=std::chrono::steady_clock::now();
        const auto result=solver_.solve(p,128,settings_.boundedSolver==BoundedSolver::Optimized7i?
            qp::Implementation::Optimized7i:qp::Implementation::Reference7h);
        status.solverUs=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
        result_=result;
        const auto physicalStart=std::chrono::steady_clock::now();
        const auto sample=status.solverSampleCount;
        if(sample<4)
        {
            const auto& timing=solver_.profile();
            status.boundedProfiles[sample]={std::chrono::duration<double,std::micro>(prepared-start).count(),
                timing.inputUs,timing.linearUs,timing.activeUs,timing.verifyUs,0};
        }
        if(status.solverSampleCount<4)
        {
            const auto i=status.solverSampleCount++; status.solverTimesUs[i]=status.solverUs;
            status.solverIterations[i]=result.iterations; status.solverActive[i]=result.active;
        }
        status.solverCode=qp::name(result.code); status.qpIterations=result.iterations; status.activeInequalities=result.active;
        status.inequalityViolation=result.violation; status.stationarity=result.projectedStationarity;
        status.activeCaps=status.activeFriction=status.activeNormals=0;
        for(int r=0;r<p.m;++r) if(result.multipliers[r]>1e-9)
        { if(r<82) ++status.activeCaps; else if(r>=122||(r-82)%5==0) ++status.activeNormals; else ++status.activeFriction; }
        ++status.linearSolves;
        if(result.code!=qp::Code::Feasible)
        {
            if(sample<4) status.boundedProfiles[sample].physicalUs=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-physicalStart).count();
            return false;
        }
        for(int i=0;i<N;++i) status.acceleration[i]=result.x[i];
        status.contactAcceleration=contact; status.contactJdot=jdot;
        std::array<double,12> wrench{};
        for(int r=0;r<12;++r) for(int f=0;f<F;++f) wrench[r]+=wrenchMap[r*F+f]*result.x[N+f];
        status.baseResidual=0; status.contactResidual=0; status.requestedUtilization=0;
        for(int r=0;r<N;++r)
        {
            double tau=d->qfrc_bias[r]-d->qfrc_passive[r];
            for(int i=0;i<N;++i) tau+=mass[r*N+i]*result.x[i];
            for(int i=0;i<F;++i) tau-=generalized[r*F+i]*result.x[N+i];
            if(r>=layout.rootQvel&&r<layout.rootQvel+6) status.baseResidual=std::max(status.baseResidual,std::abs(tau));
            for(unsigned k=0;k<41;++k) if(layout.qvel[k]==r) { held_[k]=tau; status.requestedUtilization=std::max(status.requestedUtilization,std::abs(tau)/def.drives[k].maxTorque); }
        }
        for(int r=0;r<12;++r)
        { double residual=jdot[r]-contact[r]; for(int i=0;i<N;++i) residual+=j[r*N+i]*result.x[i]; status.contactResidual=std::max(status.contactResidual,std::abs(residual)); }
        for(int side=0;side<2;++side)
        {
            const int r=side*6; status.predictedReaction[side]={points[side],{wrench[r],wrench[r+1],wrench[r+2]},{wrench[r+3],wrench[r+4],wrench[r+5]}};
            for(int c=0;c<4;++c) { const int f=N+side*12+c*3; status.cornerForces[side][c]=forceFrames[side]*(100.*glm::dvec3{result.x[f],result.x[f+1],result.x[f+2]}); }
        }
        status.feasible=status.baseResidual<1e-6&&status.contactResidual<1e-6&&status.requestedUtilization<=1.000001;
        if(settings_.loadCommands)
        {
            const double sum=wrench[1]+wrench[7];
            status.load.predictedValid=sum>=60&&wrench[1]>=30&&wrench[7]>=30;
            status.load.predicted=status.load.predictedValid?wrench[1]/sum:std::numeric_limits<double>::quiet_NaN();
        }
        status.solverTorque=held_;
        if(!status.feasible) status.solverCode="physical_residual_failure";
        if(sample<4) status.boundedProfiles[sample].physicalUs=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-physicalStart).count();
        return status.feasible;
    }
}
