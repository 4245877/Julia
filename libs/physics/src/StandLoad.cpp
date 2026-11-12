#include "StandControl.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

namespace julia::physics
{
    namespace
    {
        constexpr double NaN=std::numeric_limits<double>::quiet_NaN();
        // Display-only filter: never enters the QP or reference. The predictive
        // task is sufficient for the measured criteria; no load integrator exists.
        constexpr double FilterSeconds=.05;
        template<std::size_t N> double bound(std::array<double,N> b,int depth)
        {
            if(depth==0) { double v=0;for(double x:b)v=std::max(v,std::abs(x));return v; }
            std::array<double,N> l{},r{};l[0]=b[0];r[N-1]=b[N-1];
            for(std::size_t k=1;k<N;++k)
            { for(std::size_t i=0;i<N-k;++i)b[i]=.5*(b[i]+b[i+1]);l[k]=b[0];r[N-1-k]=b[N-1-k]; }
            return std::max(bound(l,depth-1),bound(r,depth-1));
        }
    }
    bool StandControl::planLoad(double goal,bool cancelCommand,std::string* reason)
    {
        const auto reject=[&](const char* why){if(reason)*reason=why;return false;};
        const auto& t=status.load;
        if(!settings_.loadCommands||!t.baselineValid||status.phase!=StandPhase::Holding||!t.actualValid)
            return reject("Load command requires valid calibrated measured Holding");
        if(!std::isfinite(goal)||std::abs(goal-t.alpha0)>.05+1e-12||goal<.1||goal>.9)
            return reject("Load target outside calibrated +/-0.05 two-support range");
        // Near an endpoint a brake may need less than two seconds. In particular
        // the remaining suffix of an accepted rest-to-rest curve is a valid C2
        // stop to its endpoint; do not discard it by imposing a 2s lower bound.
        double duration=cancelCommand&&loadDuration_>loadTime_+1e-8?std::min(2.,loadDuration_-loadTime_):2.;
        std::array<double,6>b{};bool valid=false;
        for(unsigned attempt=0;attempt<40;++attempt)
        {
            b={t.reference,t.reference+t.velocity*duration/5.,t.reference+2*t.velocity*duration/5.+t.acceleration*duration*duration/20.,goal,goal,goal};
            auto displacement=b;for(double& x:displacement)x-=t.alpha0;
            valid=bound(displacement,4)<=.05+1e-12;
            std::array<double,5>v{};std::array<double,4>a{};
            for(int i=0;i<5;++i)v[i]=5*(b[i+1]-b[i])/duration;
            for(int i=0;i<4;++i)a[i]=20*(b[i+2]-2*b[i+1]+b[i])/(duration*duration);
            valid=valid&&bound(v,4)<=.02+1e-12&&bound(a,4)<=.04+1e-12;
            if(valid)break;duration*=1.2;
        }
        if(!valid)return reject("No C2 load curve within deterministic planning budget");
        loadCurve_=b;loadTime_=0;loadDuration_=duration;loadCancelled_=cancelCommand;
        status.load.command=goal;status.load.achievedSeconds=0;++status.load.serial;status.load.phase=ComPhase::Moving;
        if(reason)reason->clear();return true;
    }
    bool StandControl::loadTarget(double alpha,std::string* reason){return planLoad(alpha,false,reason);}
    bool StandControl::loadReturn(std::string* reason){return planLoad(status.load.alpha0,false,reason);}
    bool StandControl::loadCancel(std::string* reason)
    {
        // Cancel owns its generated stopping destination. Keep it within the
        // calibrated interval; explicit user targets are always rejected, never
        // projected. The entire resulting stop still passes the C2 certificate.
        const auto& t=status.load;
        return planLoad(std::clamp(t.reference+t.velocity,t.alpha0-.05,t.alpha0+.05),true,reason);
    }
    void StandControl::advanceLoad(double dt)
    {
        if(loadDuration_==0)return;
        loadTime_=std::min(loadDuration_,loadTime_+dt);const double t=loadTime_/loadDuration_;
        auto b=loadCurve_;std::array<double,5>v{};std::array<double,4>a{};
        for(int i=0;i<5;++i)v[i]=5*(b[i+1]-b[i])/loadDuration_;
        for(int i=0;i<4;++i)a[i]=4*(v[i+1]-v[i])/loadDuration_;
        for(int n=6;n>1;--n)for(int i=0;i<n-1;++i)b[i]=(1-t)*b[i]+t*b[i+1];
        for(int n=5;n>1;--n)for(int i=0;i<n-1;++i)v[i]=(1-t)*v[i]+t*v[i+1];
        for(int n=4;n>1;--n)for(int i=0;i<n-1;++i)a[i]=(1-t)*a[i]+t*a[i+1];
        status.load.reference=b[0];status.load.velocity=v[0];status.load.acceleration=a[0];
    }
    void StandControl::observeLoad(const Snapshot& s,double dt)
    {
        auto& t=status.load;const auto& o=s.support;
        t.actualValid=o.enabled&&o.floorAvailable&&o.dynamics.valid&&!o.otherEnvironmentActive&&o.combinedCop.point.has_value();
        double sum=0;for(const auto& f:o.feet)
        { t.actualValid=t.actualValid&&f.load.loaded&&f.active>0&&f.near&&f.cop.point.has_value()&&std::isfinite(f.normalForce)&&f.normalForce>=30&&f.slipSpeed<=.05;sum+=f.normalForce; }
        t.actualValid=t.actualValid&&std::isfinite(sum)&&sum>=60;
        t.actual=t.actualValid?o.feet[0].normalForce/sum:NaN;
        if(t.actualValid)
        { if(!loadFilterValid_)t.filtered=t.actual;else t.filtered+=(1-std::exp(-dt/FilterSeconds))*(t.actual-t.filtered);loadFilterValid_=true; }
        else {t.filtered=NaN;loadFilterValid_=false;t.integral=t.feedback=0;}
        const auto delta=s.totalComWorld-initialCom_,vel=s.linearMomentumWorld/s.totalMass;
        const bool com=glm::length(glm::dvec2{delta.x,delta.z})<=.0005&&glm::length(glm::dvec2{vel.x,vel.z})<=.001;
        const double elapsed=s.time-start_;
        if(elapsed>=2-1e-9&&elapsed<4-1e-9)
        {loadBaselineGood_=loadBaselineGood_&&t.actualValid&&com&&status.phase==StandPhase::Holding;if(t.actualValid)loadSum_+=t.actual*dt;t.baselineSeconds+=dt;}
        if(!t.baselineValid&&elapsed>=4-1e-9&&t.phase!=ComPhase::Failed)
        {
            if(loadBaselineGood_&&std::abs(t.baselineSeconds-2)<dt*.1)
            {t.baselineValid=true;t.alpha0=t.command=t.reference=loadSum_/t.baselineSeconds;t.phase=ComPhase::Idle;}
            else {t.alpha0=t.command=t.reference=NaN;t.phase=ComPhase::Failed;}
        }
        if(status.phase==StandPhase::Lost||status.phase==StandPhase::Disabled)
        {t.integral=t.feedback=t.achievedSeconds=0;t.predictedValid=false;t.predicted=NaN;return;}
        if(t.serial==0)return;
        const double displacement=t.command-t.alpha0;
        const bool change=loadCancelled_||std::abs(displacement)<.049999||((t.actual-t.alpha0)*std::copysign(1.,displacement)>=.04);
        const bool achieved=loadTime_>=loadDuration_&&t.actualValid&&std::abs(t.actual-t.command)<=.01&&com&&change;
        t.achievedSeconds=achieved?t.achievedSeconds+dt:0;
        if(loadTime_>=loadDuration_)t.phase=t.achievedSeconds>=2?(loadCancelled_?ComPhase::Cancelled:ComPhase::Reached):ComPhase::Settling;
    }
    void loadCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,time_s,phase,task_enabled,baseline_valid,actual_valid,predicted_valid,alpha0,command,reference,reference_v,reference_a,predicted,actual,filtered,integral,feedback,achieved_s,baseline_s,serial,left_fn,right_fn,initial_com_x,initial_com_y,initial_com_z,com_error_m,speed_m_s,stand_phase\n";
    }
    void loadCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& t=s.stand.load;if(!t.enabled)return;const auto d=s.totalComWorld-s.stand.initialCom,v=s.linearMomentumWorld/s.totalMass;
        out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.time<<','<<comPhaseName(t.phase)<<','<<t.taskEnabled<<','<<t.baselineValid<<','<<t.actualValid<<','<<t.predictedValid
            <<','<<(t.baselineValid?t.alpha0:NaN)<<','<<(t.baselineValid?t.command:NaN)<<','<<(t.baselineValid?t.reference:NaN)<<','<<t.velocity<<','<<t.acceleration
            <<','<<(t.predictedValid?t.predicted:NaN)<<','<<(t.actualValid?t.actual:NaN)<<','<<(t.actualValid?t.filtered:NaN)<<','<<t.integral<<','<<t.feedback<<','<<t.achievedSeconds<<','<<t.baselineSeconds<<','<<t.serial
            <<','<<s.support.feet[0].normalForce<<','<<s.support.feet[1].normalForce<<','<<s.stand.initialCom.x<<','<<s.stand.initialCom.y<<','<<s.stand.initialCom.z
            <<','<<glm::length(glm::dvec2{d.x,d.z})<<','<<glm::length(glm::dvec2{v.x,v.z})<<','<<standPhaseName(s.stand.phase)<<'\n';
    }
}
