#include "StandControl.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

// Stage7k: one atomic command {horizontal COM offset, absolute left-load fraction}.
// Both components are quintic Bezier C2 curves started from the current reference
// position/velocity/acceleration on ONE time base. A duration is accepted only when
// both complete curves are certified at that same duration; otherwise nothing changes.
namespace julia::physics
{
    namespace
    {
        constexpr double NaN=std::numeric_limits<double>::quiet_NaN();
        double magnitude(double x) { return std::abs(x); }
        double magnitude(const glm::dvec3& x) { return glm::length(x); }
        // Convex-hull bound of a Bezier polygon after `depth` binary subdivisions:
        // every point of the curve lies inside the hull of the refined control points.
        template<class T,std::size_t Count> double hull(std::array<T,Count> b,int depth)
        {
            if(depth==0) { double v=0; for(const auto& x:b) v=std::max(v,magnitude(x)); return v; }
            std::array<T,Count> left{},right{}; left[0]=b[0]; right[Count-1]=b[Count-1];
            for(std::size_t level=1;level<Count;++level)
            { for(std::size_t i=0;i<Count-level;++i) b[i]=.5*(b[i]+b[i+1]); left[level]=b[0]; right[Count-1-level]=b[Count-1-level]; }
            return std::max(hull(left,depth-1),hull(right,depth-1));
        }
        // b0=p, b1=p+vT/5, b2=p+2vT/5+aT^2/20 keep p,v,a continuous; b3=b4=b5=goal rest at the goal.
        template<class T> std::array<T,6> quintic(const T& p,const T& v,const T& a,const T& goal,double duration)
        { return {p,p+v*duration/5.,p+2.*v*duration/5.+a*duration*duration/20.,goal,goal,goal}; }
        template<class T> bool certify(const std::array<T,6>& b,const T& center,double radius,double speed,double acceleration,double duration)
        {
            auto displacement=b; for(auto& x:displacement) x-=center;
            std::array<T,5> v{}; std::array<T,4> a{};
            for(std::size_t i=0;i<5;++i) v[i]=5.*(b[i+1]-b[i])/duration;
            for(std::size_t i=0;i<4;++i) a[i]=20.*(b[i+2]-2.*b[i+1]+b[i])/(duration*duration);
            return hull(displacement,4)<=radius+1e-12&&hull(v,4)<=speed+1e-12&&hull(a,4)<=acceleration+1e-12;
        }
        template<class T,std::size_t Count> T casteljau(std::array<T,Count> b,double t)
        { for(std::size_t n=Count;n>1;--n) for(std::size_t i=0;i+1<n;++i) b[i]=(1-t)*b[i]+t*b[i+1]; return b[0]; }
    }
    bool StandControl::planCoordinated(glm::dvec3 comGoal,glm::dvec2 offset,double alphaGoal,bool cancelCommand,std::string* reason)
    {
        const auto reject=[&](const char* why) { if(reason) *reason=why; return false; };
        auto& c=status.coordinated; const auto& load=status.load;
        if(!settings_.coordinatedCommands||status.phase!=StandPhase::Holding||!load.baselineValid||!load.actualValid)
            return reject("Coordinated command requires coordinated mode, valid alpha0 calibration and measured Holding");
        if(!std::isfinite(comGoal.x)||!std::isfinite(comGoal.y)||!std::isfinite(comGoal.z)||!std::isfinite(offset.x)||!std::isfinite(offset.y)||
            glm::length(comGoal-initialCom_)>settings_.motion.maxDisplacement+1e-12)
            return reject("Coordinated COM target outside frozen placement displacement bound");
        if(!std::isfinite(alphaGoal)||std::abs(alphaGoal-load.alpha0)>.05+1e-12||alphaGoal<.1||alphaGoal>.9)
            return reject("Coordinated load target outside calibrated +/-0.05 two-support range");
        // Near the end of an accepted curve its remaining suffix is itself a C2 stop;
        // a cancel may start from that shorter candidate. Deterministic 40 candidates.
        double duration=cancelCommand&&coordDuration_>coordTime_+1e-8?std::min(2.,coordDuration_-coordTime_):2.;
        std::array<glm::dvec3,6> com{}; std::array<double,6> alpha{};
        bool valid=false; unsigned attempts=0;
        while(attempts<40&&!valid)
        {
            ++attempts;
            com=quintic(c.comReference,c.comVelocity,c.comAcceleration,comGoal,duration);
            alpha=quintic(c.alphaReference,c.alphaVelocity,c.alphaAcceleration,alphaGoal,duration);
            valid=certify(com,initialCom_,settings_.motion.maxDisplacement,settings_.motion.maxSpeed,settings_.motion.maxAcceleration,duration)&&
                certify(alpha,load.alpha0,.05,.02,.04,duration);
            if(!valid) duration*=1.2;
        }
        if(!valid) return reject("No common C2 COM/load curves within deterministic planning budget");
        coordCom_=com; coordAlpha_=alpha; coordTime_=0; coordDuration_=duration;
        c.goalCom=comGoal; c.goalOffset=offset; c.goalAlpha=alphaGoal; c.cancelled=cancelCommand; c.stopped=false;
        c.duration=duration; c.elapsed=0; c.achievedSeconds=0; c.planAttempts=attempts; ++c.serial; c.phase=ComPhase::Moving;
        status.load.command=alphaGoal;
        if(reason) reason->clear(); return true;
    }
    bool StandControl::coordinatedTarget(const CoordinatedTarget& target,std::string* reason)
    {
        if(!target.comOffset&&!target.alpha) { if(reason) *reason="Coordinated target requires a COM offset and/or alpha"; return false; }
        const auto& c=status.coordinated;
        // An omitted component keeps the accepted terminal goal exactly (world COM, absolute alpha).
        const auto offset=target.comOffset?*target.comOffset:c.goalOffset;
        const auto goal=target.comOffset?initialCom_+settings_.rootGoal.orientation*glm::dvec3{offset.x,0,offset.y}:c.goalCom;
        return planCoordinated(goal,offset,target.alpha?*target.alpha:c.goalAlpha,false,reason);
    }
    bool StandControl::coordinatedCancel(std::string* reason)
    {
        // Cancel owns its generated stop goals; only these are bounded. Explicit
        // user targets are rejected, never projected. Both curves are certified.
        const auto& c=status.coordinated; const double alpha0=status.load.alpha0;
        auto com=c.comReference+c.comVelocity*1.;
        const auto delta=com-initialCom_; const double radius=glm::length(delta);
        if(radius>settings_.motion.maxDisplacement) com=initialCom_+delta*(settings_.motion.maxDisplacement/radius);
        const double alpha=std::isfinite(alpha0)?std::clamp(c.alphaReference+c.alphaVelocity*1.,alpha0-.05,alpha0+.05):NaN;
        const auto local=glm::conjugate(settings_.rootGoal.orientation)*(com-initialCom_);
        return planCoordinated(com,{local.x,local.z},alpha,true,reason);
    }
    bool StandControl::coordinatedReturn(std::string* reason)
    { return planCoordinated(initialCom_,{0,0},status.load.alpha0,false,reason); }
    void StandControl::advanceCoordinated(double dt)
    {
        if(coordDuration_==0) return;
        auto& c=status.coordinated;
        coordTime_=std::min(coordDuration_,coordTime_+dt); const double t=coordTime_/coordDuration_;
        std::array<glm::dvec3,5> cv{}; std::array<glm::dvec3,4> ca{}; std::array<double,5> av{}; std::array<double,4> aa{};
        for(std::size_t i=0;i<5;++i) { cv[i]=5.*(coordCom_[i+1]-coordCom_[i])/coordDuration_; av[i]=5.*(coordAlpha_[i+1]-coordAlpha_[i])/coordDuration_; }
        for(std::size_t i=0;i<4;++i) { ca[i]=4.*(cv[i+1]-cv[i])/coordDuration_; aa[i]=4.*(av[i+1]-av[i])/coordDuration_; }
        c.comReference=casteljau(coordCom_,t); c.comVelocity=casteljau(cv,t); c.comAcceleration=casteljau(ca,t);
        c.alphaReference=casteljau(coordAlpha_,t); c.alphaVelocity=casteljau(av,t); c.alphaAcceleration=casteljau(aa,t);
        c.elapsed=coordTime_; c.stopped=coordTime_>=coordDuration_;
        // The QP load task reads status.load.reference; mirror the common-time alpha curve.
        status.load.reference=c.alphaReference; status.load.velocity=c.alphaVelocity; status.load.acceleration=c.alphaAcceleration;
        if(settings_.coordinatedComTask)
        { settings_.comGoal=status.comGoal=c.comReference; status.referenceVelocity=c.comVelocity; status.referenceAcceleration=c.comAcceleration; }
    }
    void StandControl::observeCoordinated(const Snapshot& s,double dt)
    {
        // Completed-substep endpoint measurements only; never predicted values.
        auto& c=status.coordinated; auto& t=status.load;
        const auto local=glm::conjugate(settings_.rootGoal.orientation)*(s.totalComWorld-initialCom_);
        const auto velocity=s.linearMomentumWorld/s.totalMass;
        const auto error=s.totalComWorld-c.goalCom,tracking=s.totalComWorld-c.comReference;
        c.actualOffset={local.x,local.z};
        c.comError=glm::length(glm::dvec2{error.x,error.z}); c.referenceError=glm::length(glm::dvec2{tracking.x,tracking.z});
        c.speed=glm::length(glm::dvec2{velocity.x,velocity.z});
        const double length=glm::length(c.goalOffset);
        c.directional=length>0?glm::dot(c.actualOffset,c.goalOffset/length):0;
        c.alphaError=t.actualValid?std::abs(t.actual-c.goalAlpha):NaN;
        c.stopped=coordTime_>=coordDuration_;
        if(status.phase==StandPhase::Lost||status.phase==StandPhase::Disabled)
        { c.achievedSeconds=0; c.comOk=c.alphaOk=c.supportsOk=false; t.predictedValid=false; t.predicted=NaN; return; }
        // Directional rules for the full-amplitude goals; cancel stop goals and return only need closeness.
        const bool fullCom=length>=.0029999,fullAlpha=std::abs(c.goalAlpha-t.alpha0)>=.049999;
        const double change=t.actualValid?(t.actual-t.alpha0)*std::copysign(1.,c.goalAlpha-t.alpha0):NaN;
        c.supportsOk=t.actualValid;
        c.comOk=c.comError<=.0005&&c.speed<=.001&&(c.cancelled||!fullCom||c.directional>=.002);
        c.alphaOk=t.actualValid&&c.alphaError<=.01&&(c.cancelled||!fullAlpha||change>=.04);
        if(c.serial==0) return;
        const bool achieved=c.stopped&&c.comOk&&c.alphaOk&&c.supportsOk;
        c.achievedSeconds=achieved?c.achievedSeconds+dt:0;
        if(c.stopped) c.phase=c.achievedSeconds>=2-1e-9?(c.cancelled?ComPhase::Cancelled:ComPhase::Reached):ComPhase::Settling;
    }
    void coordinatedCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,time_s,stand_phase,phase,serial,com_task,load_task,cancelled,stopped,duration_s,elapsed_s,achieved_s,plan_attempts,"
             "baseline_valid,alpha0,goal_offset_x,goal_offset_z,goal_com_x,goal_com_y,goal_com_z,"
             "com_ref_x,com_ref_y,com_ref_z,com_ref_vx,com_ref_vy,com_ref_vz,com_ref_ax,com_ref_ay,com_ref_az,"
             "com_x,com_y,com_z,com_vx,com_vy,com_vz,offset_x,offset_z,com_error_m,reference_error_m,speed_m_s,directional_m,"
             "alpha_goal,alpha_ref,alpha_ref_v,alpha_ref_a,alpha_predicted,predicted_valid,alpha_actual,actual_valid,alpha_filtered,alpha_error,"
             "left_fn,right_fn,com_ok,alpha_ok,supports_ok,controller_goal_x,controller_goal_y,controller_goal_z,"
             "initial_com_x,initial_com_y,initial_com_z,placement_qw,placement_qx,placement_qy,placement_qz,"
             "solver,qp_iterations,active_caps,active_friction,active_normals,integral,feedback\n";
    }
    void coordinatedCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& t=s.stand; const auto& c=t.coordinated; const auto& l=t.load; if(!c.enabled) return;
        const auto vec=[&](glm::dvec3 v) { out<<','<<v.x<<','<<v.y<<','<<v.z; };
        const auto velocity=s.linearMomentumWorld/s.totalMass;
        out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.time<<','<<standPhaseName(t.phase)<<','<<comPhaseName(c.phase)<<','<<c.serial
            <<','<<c.comTaskEnabled<<','<<c.loadTaskEnabled<<','<<c.cancelled<<','<<c.stopped<<','<<c.duration<<','<<c.elapsed<<','<<c.achievedSeconds<<','<<c.planAttempts
            <<','<<l.baselineValid<<','<<(l.baselineValid?l.alpha0:NaN)<<','<<c.goalOffset.x<<','<<c.goalOffset.y;
        vec(c.goalCom); vec(c.comReference); vec(c.comVelocity); vec(c.comAcceleration); vec(s.totalComWorld); vec(velocity);
        out<<','<<c.actualOffset.x<<','<<c.actualOffset.y<<','<<c.comError<<','<<c.referenceError<<','<<c.speed<<','<<c.directional
            <<','<<(l.baselineValid?c.goalAlpha:NaN)<<','<<(l.baselineValid?c.alphaReference:NaN)<<','<<c.alphaVelocity<<','<<c.alphaAcceleration
            <<','<<(l.predictedValid?l.predicted:NaN)<<','<<l.predictedValid<<','<<(l.actualValid?l.actual:NaN)<<','<<l.actualValid
            <<','<<(l.actualValid?l.filtered:NaN)<<','<<(l.actualValid?c.alphaError:NaN)
            <<','<<s.support.feet[0].normalForce<<','<<s.support.feet[1].normalForce<<','<<c.comOk<<','<<c.alphaOk<<','<<c.supportsOk;
        vec(t.comGoal); vec(t.initialCom);
        const auto& q=t.rootGoal.orientation; out<<','<<q.w<<','<<q.x<<','<<q.y<<','<<q.z;
        out<<','<<t.solverCode<<','<<t.qpIterations<<','<<t.activeCaps<<','<<t.activeFriction<<','<<t.activeNormals<<','<<l.integral<<','<<l.feedback<<'\n';
    }
}
