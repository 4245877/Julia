#include "StandControl.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace julia::physics
{
    namespace
    {
        template<std::size_t Count> double curveBound(std::array<glm::dvec3,Count> b,int depth)
        {
            if(depth==0) { double v=0; for(const auto x:b) v=std::max(v,glm::length(x)); return v; }
            std::array<glm::dvec3,Count> left{},right{}; left[0]=b[0]; right[Count-1]=b[Count-1];
            for(std::size_t level=1;level<Count;++level)
            { for(std::size_t i=0;i<Count-level;++i) b[i]=.5*(b[i]+b[i+1]); left[level]=b[0]; right[Count-1-level]=b[Count-1-level]; }
            return std::max(curveBound(left,depth-1),curveBound(right,depth-1));
        }
    }
    const char* comPhaseName(ComPhase p)
    {
        switch(p) { case ComPhase::Idle:return "idle"; case ComPhase::Moving:return "moving"; case ComPhase::Settling:return "settling";
            case ComPhase::Reached:return "reached"; case ComPhase::Cancelled:return "cancelled"; case ComPhase::Failed:return "failed"; }
        return "invalid";
    }
    bool StandControl::plan(glm::dvec3 goal,bool cancelCommand,std::string* reason)
    {
        const auto reject=[&](const char* text) { if(reason) *reason=text; return false; };
        if(!settings_.comCommands||status.phase!=StandPhase::Holding) return reject("COM command requires enabled commands and measured Holding");
        if(!std::isfinite(glm::length(goal))||glm::length(goal-initialCom_)>settings_.motion.maxDisplacement+1e-12) return reject("COM target outside frozen placement displacement bound");
        // Quintic Bezier. Convex-hull derivative bounds certify the complete curve,
        // including retargeting with continuous reference p,v,a (no sampled bound).
        double duration=2.; std::array<glm::dvec3,6> b{}; bool valid=false;
        for(unsigned attempt=0;attempt<40;++attempt)
        {
            b={reference_,reference_+status.referenceVelocity*duration/5.,
                reference_+2.*status.referenceVelocity*duration/5.+status.referenceAcceleration*duration*duration/20.,goal,goal,goal};
            valid=true;
            for(const auto v:b) valid=valid&&glm::length(v-initialCom_)<=settings_.motion.maxDisplacement+1e-12;
            std::array<glm::dvec3,5> v{}; std::array<glm::dvec3,4> a{};
            for(int i=0;i<5;++i) v[i]=5.*(b[i+1]-b[i])/duration;
            for(int i=0;i<4;++i) a[i]=20.*(b[i+2]-2.*b[i+1]+b[i])/(duration*duration);
            valid=valid&&curveBound(v,4)<=settings_.motion.maxSpeed+1e-12&&curveBound(a,4)<=settings_.motion.maxAcceleration+1e-12;
            if(valid) break; duration*=1.2;
        }
        if(!valid) return reject("No bounded reference curve within deterministic planning budget");
        curve_=b; curveTime_=0; curveDuration_=duration; cancelled_=cancelCommand;
        status.commandCom=goal; status.achievedSeconds=0; ++status.commandSerial; status.comPhase=ComPhase::Moving;
        if(reason) reason->clear(); return true;
    }
    bool StandControl::target(glm::dvec2 offset,std::string* reason)
    { return plan(initialCom_+settings_.rootGoal.orientation*glm::dvec3{offset.x,0,offset.y},false,reason); }
    bool StandControl::cancel(std::string* reason)
    { return plan(reference_+status.referenceVelocity,true,reason); }
    void StandControl::advanceReference(double dt)
    {
        if(curveDuration_==0) return;
        curveTime_=std::min(curveDuration_,curveTime_+dt); const double t=curveTime_/curveDuration_;
        auto b=curve_; std::array<glm::dvec3,5> v{}; std::array<glm::dvec3,4> a{};
        for(int i=0;i<5;++i) v[i]=5.*(b[i+1]-b[i])/curveDuration_;
        for(int i=0;i<4;++i) a[i]=4.*(v[i+1]-v[i])/curveDuration_;
        for(int count=6;count>1;--count) for(int i=0;i<count-1;++i) b[i]=(1-t)*b[i]+t*b[i+1];
        for(int count=5;count>1;--count) for(int i=0;i<count-1;++i) v[i]=(1-t)*v[i]+t*v[i+1];
        for(int count=4;count>1;--count) for(int i=0;i<count-1;++i) a[i]=(1-t)*a[i]+t*a[i+1];
        reference_=b[0]; status.comGoal=settings_.comGoal=reference_; status.referenceVelocity=v[0]; status.referenceAcceleration=a[0];
    }
    void comCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,time_s,variant,command_serial,action,achieved_s,command_error_m,solver,qp_iterations,active_inequalities,active_caps,active_friction,active_normals,inequality_violation,stationarity,solver_us";
        for(const auto* name:{"command","reference","reference_v","reference_a","actual","actual_v"}) for(const auto* axis:{"x","y","z"}) out<<','<<name<<'_'<<axis;
        for(const auto* side:{"left","right"})
        {
            for(const auto* component:{"fx","fy","fz","mx","my","mz"}) out<<','<<side<<"_measured_at_pred_"<<component;
            for(int corner=0;corner<4;++corner) for(const auto* component:{"px","py","pz","fx","fy","fz"}) out<<','<<side<<"_corner"<<corner<<'_'<<component;
        }
        out<<'\n';
    }
    void comCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& t=s.stand;
        out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.time<<','<<(t.variant==StandVariant::Bounded7h?"bounded7h":"equality7g")<<','<<t.commandSerial<<','<<comPhaseName(t.comPhase)<<','<<t.achievedSeconds<<','<<t.commandError<<','<<t.solverCode<<','<<t.qpIterations<<','<<t.activeInequalities<<','<<t.activeCaps<<','<<t.activeFriction<<','<<t.activeNormals<<','<<t.inequalityViolation<<','<<t.stationarity<<','<<t.solverUs;
        const auto vec=[&](glm::dvec3 v) { out<<','<<v.x<<','<<v.y<<','<<v.z; };
        vec(t.commandCom); vec(t.comGoal); vec(t.referenceVelocity); vec(t.referenceAcceleration); vec(s.totalComWorld); vec(s.linearMomentumWorld/s.totalMass);
        for(std::size_t side=0;side<2;++side)
        {
            const auto& w=s.support.feet[side].wrench;
            vec(w.force); vec(w.moment+glm::cross(w.origin-t.predictedReaction[side].origin,w.force));
            for(int c=0;c<4;++c) { vec(t.cornerPoints[side][c]); vec(t.cornerForces[side][c]); }
        }
        out<<'\n';
    }
    void comSolverCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,sample_time_s,solver";
        for(const auto* c:{"px","py","pz","qw","qx","qy","qz","vx","vy","vz","wx","wy","wz"}) out<<",root_"<<c;
        for(const auto* c:{"q","v","tau"}) for(int k=0;k<41;++k) out<<','<<c<<k;
        for(int k=0;k<47;++k) out<<",a"<<k;
        for(const auto* c:{"contact_a","jdot"}) for(int k=0;k<12;++k) out<<','<<c<<k;
        out<<'\n';
    }
    void comSolverCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& t=s.stand; if(t.variant!=StandVariant::Bounded7h||t.sampleTime==0) return;
        const auto& r=t.solverRoot;
        out<<std::setprecision(17)<<1<<','<<s.tick<<','<<t.sampleTime<<','<<t.solverCode;
        const auto vec=[&](glm::dvec3 v){out<<','<<v.x<<','<<v.y<<','<<v.z;};
        vec(r.position); out<<','<<r.orientation.w<<','<<r.orientation.x<<','<<r.orientation.y<<','<<r.orientation.z;
        vec(r.linearVelocityWorld); vec(r.angularVelocityLocal);
        for(const auto& a:{t.solverQ,t.solverV,t.solverTorque}) for(int k=0;k<41;++k) out<<','<<a[k];
        for(double a:t.acceleration) out<<','<<a;
        for(double a:t.contactAcceleration) out<<','<<a;
        for(double a:t.contactJdot) out<<','<<a;
        out<<'\n';
    }
}
