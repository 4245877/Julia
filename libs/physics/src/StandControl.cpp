#include "StandControl.hpp"
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>

namespace julia::physics
{
    namespace
    {
        constexpr int N=47,C=12;
        struct ControllerTimer
        {
            double& value;
            std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();
            ~ControllerTimer() { value+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count(); }
        };
        glm::dvec3 rotationError(glm::dquat goal,glm::dquat actual)
        {
            auto q=glm::normalize(goal*glm::conjugate(actual)); if(q.w<0) q=-q;
            const double length=glm::length(glm::dvec3{q.x,q.y,q.z});
            return length<1e-10?2.*glm::dvec3{q.x,q.y,q.z}:2*std::atan2(length,q.w)/length*glm::dvec3{q.x,q.y,q.z};
        }
        // Partial-pivot dense elimination. Fixed dimensions, bounded n^3 work.
        template<std::size_t Size> bool solve(std::array<double,Size*Size>& a,std::array<double,Size>& b)
        {
            for(std::size_t k=0;k<Size;++k)
            {
                auto pivot=k; for(auto r=k+1;r<Size;++r) if(std::abs(a[r*Size+k])>std::abs(a[pivot*Size+k])) pivot=r;
                if(!std::isfinite(a[pivot*Size+k])||std::abs(a[pivot*Size+k])<1e-12) return false;
                if(pivot!=k) { for(std::size_t j=k;j<Size;++j) std::swap(a[k*Size+j],a[pivot*Size+j]); std::swap(b[k],b[pivot]); }
                for(auto r=k+1;r<Size;++r)
                {
                    const double f=a[r*Size+k]/a[k*Size+k]; a[r*Size+k]=0;
                    for(auto j=k+1;j<Size;++j) a[r*Size+j]-=f*a[k*Size+j]; b[r]-=f*b[k];
                }
            }
            for(std::size_t r=Size;r-->0;)
            { for(auto j=r+1;j<Size;++j) b[r]-=a[r*Size+j]*b[j]; b[r]/=a[r*Size+r]; if(!std::isfinite(b[r])) return false; }
            return true;
        }
        glm::dmat4 footFrame(const Snapshot& s,const Definition& def,std::size_t side,int link)
        { return s.worldFromJoint[static_cast<std::size_t>(link)]*def.supportObservation->feet[side].jointFromSole; }
    }
    const char* standPhaseName(StandPhase p)
    {
        switch(p) { case StandPhase::Disabled:return "disabled"; case StandPhase::Establishing:return "establishing";
            case StandPhase::Stabilizing:return "stabilizing"; case StandPhase::Holding:return "holding"; case StandPhase::Lost:return "lost"; }
        return "invalid";
    }
    void standCsvHeader(std::ostream& out)
    {
        out<<"schema,tick,epoch,time_s,sample_time_s,phase,reason,feedback,feasible,hold_s,rest_s,com_error_m,speed_m_s,tilt_rad,thorax_tilt_rad,height_error_m,foot_displacement_m,foot_angle_rad,requested_utilization,base_residual,contact_residual,evaluations,linear_solves,controller_us,goal_com_x,goal_com_y,goal_com_z,desired_ax,desired_ay,desired_az,goal_root_x,goal_root_y,goal_root_z,goal_qw,goal_qx,goal_qy,goal_qz";
        for(const auto* side:{"left","right"}) out<<','<<side<<"_pred_origin_x,"<<side<<"_pred_origin_y,"<<side<<"_pred_origin_z,"<<side<<"_pred_fx,"<<side<<"_pred_fy,"<<side<<"_pred_fz,"<<side<<"_pred_mx,"<<side<<"_pred_my,"<<side<<"_pred_mz,"<<side<<"_measured_fy,"<<side<<"_loaded";
        out<<'\n';
    }
    void standCsvRow(std::ostream& out,const Snapshot& s)
    {
        const auto& t=s.stand; out<<std::setprecision(17)<<1<<','<<s.tick<<','<<s.epoch<<','<<s.time<<','<<t.sampleTime<<','<<standPhaseName(t.phase)<<','<<t.reason<<','<<t.balanceFeedback<<','<<t.feasible
            <<','<<t.holdSeconds<<','<<t.restSeconds<<','<<t.comError<<','<<t.speed<<','<<t.tilt<<','<<t.thoraxTilt<<','<<t.heightError<<','<<t.footDisplacement<<','<<t.footAngle<<','<<t.requestedUtilization<<','<<t.baseResidual<<','<<t.contactResidual
            <<','<<t.evaluations<<','<<t.linearSolves<<','<<t.controllerUs;
        const auto vec=[&](glm::dvec3 v) { out<<','<<v.x<<','<<v.y<<','<<v.z; };
        vec(t.comGoal); vec(t.desiredComAcceleration); vec(t.rootGoal.position);
        out<<','<<t.rootGoal.orientation.w<<','<<t.rootGoal.orientation.x<<','<<t.rootGoal.orientation.y<<','<<t.rootGoal.orientation.z;
        for(std::size_t side=0;side<2;++side) { const auto& w=t.predictedReaction[side]; vec(w.origin); vec(w.force); vec(w.moment); out<<','<<s.support.feet[side].normalForce<<','<<s.support.feet[side].load.loaded; }
        out<<'\n';
    }
    void StandControl::reset() { *this=StandControl{}; }
    void StandControl::start(const StandSettings& settings,const Snapshot& state,const Definition& definition)
    {
        reset(); settings_=settings; start_=state.time;
        status.phase=StandPhase::Establishing; status.reason="awaiting_measured_load"; status.balanceFeedback=settings.balanceFeedback;
        status.rootGoal=settings.rootGoal; status.comGoal=settings.comGoal;
        status.variant=settings.variant; status.initialCom=status.commandCom=initialCom_=reference_=settings.comGoal;
        for(std::size_t side=0;side<2;++side) initialFeet_[side]=footFrame(state,definition,side,state.support.feet[side].link);
    }
    void StandControl::disable() { status.phase=StandPhase::Disabled; status.reason="disabled"; status.feasible=false; status.comPhase=ComPhase::Idle; status.achievedSeconds=0; held_={}; }
    void StandControl::lost(const char* reason) { status.phase=StandPhase::Lost; status.reason=reason; status.feasible=false; status.comPhase=ComPhase::Failed; status.achievedSeconds=0; held_={}; }
    Coordinates StandControl::evaluate(const mjModel* m,mjData* d,const StateLayout& layout,
        const Definition& def,Snapshot& s,const std::vector<int>& bodies,const std::array<int,2>& footLinks,double dt,bool update)
    {
        const ControllerTimer timer{status.controllerUs};
        if(status.phase==StandPhase::Lost||status.phase==StandPhase::Disabled) return {};
        if(settings_.comCommands) advanceReference(dt);
        const auto& o=s.support;
        const auto velocity=s.linearMomentumWorld/s.totalMass;
        const auto error=settings_.comGoal-s.totalComWorld;
        status.comError=glm::length(glm::dvec2{error.x,error.z}); status.speed=glm::length(glm::dvec2{velocity.x,velocity.z});
        const auto orientError=rotationError(settings_.rootGoal.orientation,s.root.orientation);
        status.tilt=glm::length(orientError); status.heightError=std::abs(s.root.position.y-settings_.rootGoal.position.y);
        status.thoraxTilt=glm::length(rotationError(settings_.thoraxGoal,glm::quat_cast(glm::dmat3{s.worldFromJoint[2]})));
        status.footDisplacement=0; status.footAngle=0;
        bool contactGeometry=true;
        for(std::size_t side=0;side<2;++side)
        {
            const auto f=footFrame(s,def,side,footLinks[side]); const auto delta=glm::dvec3{f[3]-initialFeet_[side][3]};
            status.footDisplacement=std::max(status.footDisplacement,glm::length(glm::dvec2{delta.x,delta.z}));
            status.footAngle=std::max(status.footAngle,glm::length(rotationError(glm::quat_cast(glm::dmat3{initialFeet_[side]}),glm::quat_cast(glm::dmat3{f}))));
            if(settings_.variant==StandVariant::Bounded7h)
            {
                const auto link=static_cast<std::size_t>(footLinks[side]); const auto box=s.worldFromJoint[link]*def.links[link].jointFromBox;
                contactGeometry=contactGeometry&&o.feet[side].near&&box[1].y>=std::cos(.05);
                for(int corner=0;corner<4;++corner)
                {
                    const auto p=box*glm::dvec4{(corner&1)?.045:-.045,-.025,(corner&2)?.085:-.085,1};
                    const double height=p.y-o.planeOrigin.y;
                    contactGeometry=contactGeometry&&height>=-.004&&height<=.002;
                }
            }
        }
        if(!o.enabled||!o.floorAvailable||!o.dynamics.valid) lost("observation_unavailable");
        else if(o.otherEnvironmentActive) lost("nonfoot_contact");
        else if(status.comError>.035||status.speed>.12||status.tilt>.10||status.thoraxTilt>.10||status.heightError>.025) lost("outside_recovery_envelope");
        else if(status.footDisplacement>.010||status.footAngle>.05) lost("foot_displacement");
        else if(status.phase!=StandPhase::Establishing &&
            (!o.feet[0].load.loaded||!o.feet[1].load.loaded||o.feet[0].active==0||o.feet[1].active==0)) lost("support_lost");
        else if(status.phase!=StandPhase::Establishing &&
            (!o.feet[0].cop.point||!o.feet[1].cop.point||!o.combinedCop.point)) lost("cop_unavailable");
        else if(status.phase!=StandPhase::Establishing &&
            (o.feet[0].slipSpeed>.05||o.feet[1].slipSpeed>.05)) lost("sliding");
        else if(!contactGeometry) lost("contact_model_geometry");
        for(unsigned k=0;k<s.dofs;++k) if(s.limitViolation[k]>.005||s.drive.saturationRunSeconds[k]>.10) lost("joint_limit_or_saturation");
        if(status.phase==StandPhase::Lost) return {};
        if(settings_.comCommands)
        {
            const auto delta=status.commandCom-s.totalComWorld;
            status.commandError=glm::length(glm::dvec2{delta.x,delta.z});
            const bool achieved=curveTime_>=curveDuration_&&status.commandError<=.0005&&status.speed<=.001&&
                o.feet[0].load.loaded&&o.feet[1].load.loaded&&o.feet[0].active>0&&o.feet[1].active>0&&
                o.feet[0].normalForce>=30&&o.feet[1].normalForce>=30;
            status.achievedSeconds=achieved?status.achievedSeconds+dt:0;
            if(status.commandSerial>0&&curveTime_>=curveDuration_) status.comPhase=status.achievedSeconds>=2?(cancelled_?ComPhase::Cancelled:ComPhase::Reached):ComPhase::Settling;
        }
        if(status.phase==StandPhase::Establishing && o.feet[0].load.loaded&&o.feet[1].load.loaded)
        { status.phase=StandPhase::Stabilizing; status.reason="settling"; }
        if(status.phase==StandPhase::Establishing && s.time-start_>2) { lost("establishment_timeout"); return {}; }
        const bool resting=status.comError<=.006 && status.speed<=.015 && glm::length(s.root.angularVelocityLocal)<=.04;
        rest_=resting?rest_+dt:0; status.restSeconds=rest_;
        if(status.phase==StandPhase::Stabilizing&&s.time-start_>=.5&&rest_>=.25)
        { status.phase=StandPhase::Holding; status.reason="within_hold_envelope"; }
        if(status.phase==StandPhase::Stabilizing&&s.time-start_>4) { lost("stabilization_timeout"); return {}; }
        if(status.phase==StandPhase::Holding) status.holdSeconds+=dt;
        if(!update) return held_;
        ++status.evaluations; status.linearSolves=0; status.sampleTime=s.time;
        std::array<double,C*N> j{}; std::array<double,3*N> jc{};
        std::array<glm::dvec3,2> points{};
        std::array<glm::dmat3,2> rotations{};
        std::array<double,3*N> jp{},jr{};
        for(std::size_t side=0;side<2;++side)
        {
            const auto link=static_cast<std::size_t>(footLinks[side]); const auto box=s.worldFromJoint[link]*def.links[link].jointFromBox;
            rotations[side]=glm::dmat3{box}; points[side]=glm::dvec3{box*glm::dvec4{0,-.025,0,1}};
            const mjtNum p[3]{points[side].x,points[side].y,points[side].z};
            mj_jac(m,d,jp.data(),jr.data(),p,bodies[link]);
            for(int r=0;r<3;++r) for(int c=0;c<N;++c) { j[(side*6+static_cast<std::size_t>(r))*N+c]=jp[r*N+c]; j[(side*6+3+static_cast<std::size_t>(r))*N+c]=jr[r*N+c]; }
        }
        mj_jacSubtreeCom(m,d,jc.data(),bodies[0]);
        std::array<double,C> contactGoal{},jdot{}; std::array<double,3> comDot{};
        const double period=previous_?s.time-lastUpdate_:dt;
        for(int r=0;r<C;++r) for(int c=0;c<N;++c)
        {
            contactGoal[r]-=20*j[r*N+c]*d->qvel[c];
            if(previous_) jdot[r]+=(j[r*N+c]-previousJ_[r*N+c])/period*d->qvel[c];
        }
        for(int r=0;r<3;++r) for(int c=0;c<N;++c) if(previous_) comDot[r]+=(jc[r*N+c]-previousComJ_[r*N+c])/period*d->qvel[c];
        for(std::size_t side=0;side<2;++side)
        {
            const auto f=footFrame(s,def,side,footLinks[side]); const auto delta=glm::dvec3{initialFeet_[side][3]-f[3]};
            contactGoal[6*side]+=100*delta.x; contactGoal[6*side+2]+=100*delta.z;
            const auto angle=rotationError(glm::quat_cast(glm::dmat3{initialFeet_[side]}),glm::quat_cast(glm::dmat3{f}));
            for(int r=0;r<3;++r) contactGoal[6*side+3+static_cast<std::size_t>(r)]+=100*angle[r];
        }
        previousJ_=j; previousComJ_=jc; previous_=true; lastUpdate_=s.time;
        glm::dvec3 desired=settings_.balanceFeedback?40.*error-12.*velocity:glm::dvec3{0};
        if(settings_.comCommands&&settings_.balanceFeedback) desired+=12.*status.referenceVelocity+status.referenceAcceleration;
        desired.y=100*error.y-20*velocity.y; status.desiredComAcceleration=desired;
        // qvel root angular coordinates are local. Convert world orientation error.
        const auto localOrient=glm::conjugate(s.root.orientation)*orientError;
        const auto angular=settings_.balanceFeedback?100.*localOrient-20.*s.root.angularVelocityLocal:glm::dvec3{0};
        constexpr std::size_t K=N+C;
        std::array<double,K*K> ka{}; std::array<double,K> ba{};
        for(unsigned k=0;k<s.dofs;++k)
        { const auto v=static_cast<std::size_t>(layout.qvel[k]); ka[v*K+v]+=1; ba[v]=100*(settings_.posture[k]-s.q[k])-20*s.v[k]; }
        for(int r=0;r<3;++r)
        {
            const auto v=static_cast<std::size_t>(layout.rootQvel+3+r); ka[v*K+v]+=100; ba[v]+=100*angular[r];
            for(int i=0;i<N;++i)
            { ba[i]+=900*jc[r*N+i]*(desired[r]-comDot[r]); for(int c=0;c<N;++c) ka[i*K+c]+=900*jc[r*N+i]*jc[r*N+c]; }
        }
        for(int i=0;i<N;++i) ka[i*K+i]+=1e-8;
        for(int r=0;r<C;++r)
        { ba[N+r]=contactGoal[r]-jdot[r]; for(int c=0;c<N;++c) ka[(N+r)*K+c]=ka[c*K+N+r]=j[r*N+c]; }
        if(settings_.variant==StandVariant::Bounded7h)
        {
            status.solverRoot=s.root; status.solverQ=s.q; status.solverV=s.v;
            std::array<double,N*N> mass{}; mj_fullM(m,mass.data(),d->qM);
            if(!bounded(m,d,layout,def,mass,j,ka,ba,contactGoal,jdot,points,rotations)) lost(status.solverCode.c_str());
            return held_;
        }
        ++status.linearSolves;
        if(!solve<K>(ka,ba)) { lost("acceleration_solve_failed"); return {}; }
        std::array<double,N*N> mass{}; mj_fullM(m,mass.data(),d->qM);
        std::array<double,N> req{};
        for(int r=0;r<N;++r) { req[r]=d->qfrc_bias[r]-d->qfrc_passive[r]; for(int c=0;c<N;++c) req[r]+=mass[r*N+c]*ba[c]; }
        std::array<double,18*18> kw{}; std::array<double,18> bw{};
        const std::array<double,6> weights{.1,.005,.1,.1,5,.1};
        for(std::size_t side=0;side<2;++side) for(int r=0;r<6;++r)
        {
            const auto i=side*6+static_cast<std::size_t>(r); const double w=weights[r]*weights[r];
            kw[i*18+i]=w; bw[i]=w*(r<3?settings_.nominalReaction[side].force[r]:settings_.nominalReaction[side].moment[r-3]);
        }
        for(unsigned k=0;k<s.dofs;++k)
        {
            const auto v=layout.qvel[k]; const double cap=def.drives[k].maxTorque;
            for(int i=0;i<C;++i)
            { const double t=j[i*N+v]/cap; bw[i]+=.001*t*req[v]/cap; for(int c=0;c<C;++c) kw[i*18+c]+=.001*t*j[c*N+v]/cap; }
        }
        for(int r=0;r<6;++r)
        { const int v=layout.rootQvel+r; bw[12+r]=req[v]; for(int c=0;c<C;++c) kw[(12+r)*18+c]=kw[c*18+12+r]=j[c*N+v]; }
        ++status.linearSolves;
        if(!solve<18>(kw,bw)) { lost("reaction_solve_failed"); return {}; }
        status.baseResidual=0; status.contactResidual=0; status.requestedUtilization=0;
        for(int r=0;r<6;++r)
        { double residual=req[layout.rootQvel+r]; for(int c=0;c<C;++c) residual-=j[c*N+layout.rootQvel+r]*bw[c]; status.baseResidual=std::max(status.baseResidual,std::abs(residual)); }
        for(int r=0;r<C;++r)
        { double residual=jdot[r]-contactGoal[r]; for(int c=0;c<N;++c) residual+=j[r*N+c]*ba[c]; status.contactResidual=std::max(status.contactResidual,std::abs(residual)); }
        for(unsigned k=0;k<s.dofs;++k)
        {
            double tau=req[layout.qvel[k]]; for(int r=0;r<C;++r) tau-=j[r*N+layout.qvel[k]]*bw[r];
            held_[k]=tau; status.requestedUtilization=std::max(status.requestedUtilization,std::abs(tau)/def.drives[k].maxTorque);
        }
        status.feasible=status.baseResidual<1e-7 && status.contactResidual<1e-7 && status.requestedUtilization<=1.00001;
        for(std::size_t side=0;side<2;++side)
        {
            const auto k=side*6; auto& w=status.predictedReaction[side]; w={points[side],{bw[k],bw[k+1],bw[k+2]},{bw[k+3],bw[k+4],bw[k+5]}};
            const auto f=glm::transpose(rotations[side])*w.force,t=glm::transpose(rotations[side])*w.moment;
            status.feasible=status.feasible && f.y>=30 && glm::length(glm::dvec2{f.x,f.z})<=.6*f.y &&
                std::abs(t.x)<=.085*f.y && std::abs(t.z)<=.045*f.y && std::abs(t.y)<=.6*.04*f.y;
        }
        if(!status.feasible) lost("predicted_contact_or_effort_infeasible");
        return held_;
    }
}
