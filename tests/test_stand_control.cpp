#include "julia_test.hpp"
#include "../apps/julia_body_physics/BodyBenchIO.hpp"
#include <chrono>
#include <numeric>
#include <limits>
#include <sstream>

using namespace julia::physics;
namespace
{
    using Clock=std::chrono::steady_clock;
    struct Result { double squaredError=0,peakSpeed=0,hold=0; bool lost=false; std::string reason; std::vector<std::string> history; std::vector<double> error2; };
    std::string deterministic(const Snapshot& s)
    {
        auto copy=s; copy.epoch=0; copy.stand.controllerUs=0;
        std::ostringstream out; out<<std::setprecision(17);
        standCsvRow(out,copy);
        for(unsigned k=0;k<s.dofs;++k) out<<s.q[k]<<','<<s.v[k]<<','<<s.appliedTorque[k]<<','<<s.drive.deliveredWork[k]<<','<<s.drive.absorbedWork[k]<<',';
        const auto q=s.root.orientation; out<<s.root.position.x<<','<<s.root.position.y<<','<<s.root.position.z<<','<<q.w<<','<<q.x<<','<<q.y<<','<<q.z;
        for(const auto& c:s.contacts) out<<c.geom1<<','<<c.geom2<<','<<c.point.x<<','<<c.point.y<<','<<c.point.z<<','<<c.forceOn2World.x<<','<<c.forceOn2World.y<<','<<c.forceOn2World.z;
        return out.str();
    }
    void measurementHeader(std::ostream& out)
    {
        out<<"tick,time,com_x,com_y,com_z,v_x,v_y,v_z,root_x,root_y,root_z,qw,qx,qy,qz,thorax_qw,thorax_qx,thorax_qy,thorax_qz";
        for(const auto* side:{"left","right"}) out<<','<<side<<"_x,"<<side<<"_y,"<<side<<"_z,"<<side<<"_qw,"<<side<<"_qx,"<<side<<"_qy,"<<side<<"_qz,"<<side<<"_fn,"<<side<<"_slip,"<<side<<"_loaded";
        out<<",nonfoot,penetration,limit,saturated_coordinates,actual_utilization,fx,fy,fz\n";
    }
    void measurement(std::ostream& out,const Snapshot& s,const Definition& def)
    {
        out<<std::setprecision(17)<<s.tick<<','<<s.time;
        const auto vec=[&](glm::dvec3 v) { out<<','<<v.x<<','<<v.y<<','<<v.z; };
        const auto quat=[&](glm::dquat q) { out<<','<<q.w<<','<<q.x<<','<<q.y<<','<<q.z; };
        vec(s.totalComWorld); vec(s.linearMomentumWorld/s.totalMass); vec(s.root.position); quat(s.root.orientation);
        quat(glm::normalize(glm::quat_cast(glm::dmat3{s.worldFromJoint[2]})));
        for(std::size_t side=0;side<2;++side)
        {
            const auto& f=s.support.feet[side]; const auto frame=s.worldFromJoint[static_cast<std::size_t>(f.link)]*def.supportObservation->feet[side].jointFromSole;
            vec(glm::dvec3{frame[3]}); quat(glm::normalize(glm::quat_cast(glm::dmat3{frame})));
            out<<','<<f.normalForce<<','<<f.slipSpeed<<','<<f.load.loaded;
        }
        double limit=0,util=0; unsigned saturated=0;
        for(unsigned k=0;k<s.dofs;++k) { limit=std::max(limit,s.peakLimitViolationThisTick[k]); util=std::max(util,std::abs(s.appliedTorque[k])/def.drives[k].maxTorque); if(s.saturatedThisTick[k]) ++saturated; }
        out<<','<<s.support.otherEnvironmentActive<<','<<s.maxPenetrationThisTick<<','<<limit<<','<<saturated<<','<<util;
        vec(s.externalForce); out<<'\n';
    }
    Result run(const BodyRig& rig,const bodybench::Options& o,const std::filesystem::path& path,bool accepted,std::ostream& report,unsigned ticks=1920)
    {
        std::filesystem::create_directories(path); PhysicsWorld w{bodybench::definition(rig,o.scenario,o)};
        bodybench::reset(rig,w,o.scenario,o);
        std::ofstream telemetry{path/"stand-v1.csv"},measured{path/"measurements-v1.csv"}; standCsvHeader(telemetry); measurementHeader(measured);
        standCsvRow(telemetry,w.snapshot()); measurement(measured,w.snapshot(),w.definition());
        Result result;
        unsigned terminalTicks=0;
        for(unsigned n=0;n<ticks;++n)
        {
            bodybench::step(w,o.scenario,o); const auto& s=w.snapshot(); const auto& t=s.stand;
            standCsvRow(telemetry,s); measurement(measured,s,w.definition()); result.history.push_back(deterministic(s));
            if(t.phase==StandPhase::Lost&&!result.lost) { result.lost=true; result.reason=t.reason; }
            if(s.time>2)
            {
                const auto error=s.totalComWorld-t.comGoal; const double error2=error.x*error.x+error.z*error.z;
                result.squaredError+=error2; result.peakSpeed=std::max(result.peakSpeed,glm::length(glm::dvec2{s.support.dynamics.comVelocity.x,s.support.dynamics.comVelocity.z}));
                if(accepted)
                {
                    JULIA_CHECK(t.phase==StandPhase::Holding&&t.feasible);
                    JULIA_CHECK(s.support.feet[0].load.loaded&&s.support.feet[1].load.loaded);
                    JULIA_CHECK(s.support.feet[0].normalForce>=30&&s.support.feet[1].normalForce>=30&&s.support.otherEnvironmentActive==0);
                    JULIA_CHECK(t.comError<=.035&&t.speed<=.12&&t.tilt<=.10&&t.thoraxTilt<=.10&&t.heightError<=.025);
                    JULIA_CHECK(t.footDisplacement<=.010&&t.footAngle<=.05&&s.maxPenetrationThisTick<=.004);
                    JULIA_CHECK(s.support.feet[0].slipSpeed<=.05&&s.support.feet[1].slipSpeed<=.05);
                    for(unsigned k=0;k<s.dofs;++k) JULIA_CHECK(s.peakLimitViolationThisTick[k]<=.005&&s.drive.saturationRunSeconds[k]<=.10);
                }
            }
            const auto measuredError=s.totalComWorld-t.comGoal; result.error2.push_back(measuredError.x*measuredError.x+measuredError.z*measuredError.z);
            if(!accepted&&(t.phase==StandPhase::Lost||s.mode==DriveMode::Disabled)&&++terminalTicks>=24) break;
        }
        const auto& s=w.snapshot(); result.hold=s.stand.holdSeconds;
        if(accepted)
        {
            JULIA_CHECK(result.hold>=10&&!result.lost&&s.stand.restSeconds>=1);
            double saturated=0; for(unsigned k=0;k<s.dofs;++k) saturated+=s.drive.saturatedSeconds[k];
            JULIA_CHECK(saturated/(s.dofs*(s.time-2))<=.01);
        }
        report<<path.filename().string()<<" hold="<<result.hold<<" lost="<<result.lost<<" reason="<<result.reason<<" sum_error2="<<result.squaredError<<" peak_speed="<<result.peakSpeed<<'\n'; report.flush();
        return result;
    }
    void api(const BodyRig& rig,std::ostream& report)
    {
        PhysicsWorld a{rig.description()},b{rig.description()}; auto p=preparedStand(rig,a);
        a.reset(p.posture,p.rootGoal); b.reset(p.posture,p.rootGoal);
        auto invalid=p; invalid.comGoal.x=std::numeric_limits<double>::quiet_NaN();
        JULIA_CHECK(!a.startStand(invalid)&&a.snapshot().mode==DriveMode::Disabled);
        JULIA_CHECK(a.startStand(p)&&b.startStand(p));
        const auto q=a.snapshot().q;
        for(auto kind:{CommandKind::Target,CommandKind::SmoothTarget,CommandKind::Torque,CommandKind::Reach,CommandKind::Attach,CommandKind::MoveRoot}) JULIA_CHECK(!a.command(kind));
        JULIA_CHECK(!a.configureServo({})&&!a.startContact()&&!a.contactForceTarget(3)&&a.snapshot().q==q);
        std::vector<std::string> history;
        for(unsigned n=0;n<240;++n) { a.step(); b.step(); JULIA_CHECK(deterministic(a.snapshot())==deterministic(b.snapshot())); history.push_back(deterministic(a.snapshot())); }
        a.reset(p.posture,p.rootGoal); JULIA_CHECK(a.snapshot().stand.phase==StandPhase::Disabled&&a.snapshot().support.feet[0].load.confirmationSeconds==0);
        JULIA_CHECK(a.startStand(p));
        for(unsigned n=0;n<240;++n) { a.step(); JULIA_CHECK(deterministic(a.snapshot())==history[n]); }
        JULIA_CHECK(a.command(CommandKind::Disable)); a.step(); JULIA_CHECK(a.snapshot().stand.phase==StandPhase::Disabled);
        for(unsigned k=0;k<41;++k) JULIA_CHECK(a.snapshot().appliedTorque[k]==0);
        JULIA_CHECK(!a.startStand(p)); // explicit reset required after a run/disable
        a.reset(p.posture,p.rootGoal); b.reset(p.posture,p.rootGoal);
        for(unsigned n=0;n<120;++n) { a.step(); b.step(); JULIA_CHECK(deterministic(a.snapshot())==deterministic(b.snapshot())); }
        JULIA_CHECK(a.snapshot().root.position.y<.84); // physical free fall with inactive control
        report<<"API invalid commands, ownership, replay, independent instances, reset timers, disable, inactive free dynamics PASS\n";
    }
    void performance(const BodyRig& rig,const std::filesystem::path& dir,std::ostream& report)
    {
        PhysicsWorld w{rig.description()}; const auto plan=preparedStand(rig,w); w.reset(plan.posture,plan.rootGoal); JULIA_CHECK(w.startStand(plan));
        std::ofstream raw{dir/"performance.csv"}; raw<<"tick,tick_us,controller_us,evaluations,solves,solver_iterations\n";
        std::vector<double> tick,controller;
        for(unsigned n=0;n<2400;++n)
        {
            const auto start=Clock::now(); w.step(); const double elapsed=std::chrono::duration<double,std::micro>(Clock::now()-start).count();
            tick.push_back(elapsed); controller.push_back(w.snapshot().stand.controllerUs);
            raw<<n+1<<','<<std::setprecision(17)<<elapsed<<','<<controller.back()<<','<<w.snapshot().stand.evaluations<<','<<w.snapshot().stand.linearSolves<<','<<w.snapshot().solverIterations<<'\n';
        }
        const auto stats=[&](const char* name,std::vector<double> values)
        { std::sort(values.begin(),values.end()); const double mean=std::accumulate(values.begin(),values.end(),0.)/values.size(); const double p95=values[values.size()*95/100]; report<<name<<" mean="<<mean<<" p95="<<p95<<" max="<<values.back()<<" us\n"; return p95; };
        JULIA_CHECK(stats("tick",tick)<8333); stats("controller",controller);
        JULIA_CHECK(w.snapshot().stand.phase==StandPhase::Holding);
        report<<"stand_instance_bytes="<<w.standControllerBytes()<<" snapshot_sizeof="<<sizeof(Snapshot)<<" model="<<w.modelBytes()<<" data="<<w.dataBytes()<<" arena="<<w.arenaHighWaterBytes()<<" (not allocator or VRAM accounting)\n";
    }
}
int main(int argc,char** argv)
{
    try
    {
        const auto dir=std::filesystem::path{argc>1?argv[1]:"build/diagnostics/stage7g/cpu"}; std::filesystem::create_directories(dir);
        std::ofstream report{dir/"stand-control.txt"}; report<<std::setprecision(17);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx"); BodyRig rig{*asset};
        api(rig,report);
        bodybench::Options o; o.scenario=BodyCase::Stand; run(rig,o,dir/"nominal",true,report);
        o.scenario=BodyCase::StandPulse; const auto on=run(rig,o,dir/"pulse",true,report);
        JULIA_CHECK(on.peakSpeed>.001); o.feedbackOffTick=480; const auto off=run(rig,o,dir/"feedback-off",false,report); o.feedbackOffTick=0;
        const double sameWindowOn=std::accumulate(on.error2.begin()+480,on.error2.begin()+static_cast<std::ptrdiff_t>(off.error2.size()),0.);
        const double sameWindowOff=std::accumulate(off.error2.begin()+480,off.error2.end(),0.);
        report<<"causal identical first480 ticks="<<(on.history[479]==off.history[479])<<" same_window_error2 on="<<sameWindowOn<<" off="<<sameWindowOff<<'\n';
        JULIA_CHECK(on.history[479]==off.history[479]&&sameWindowOn<sameWindowOff*.5);
        for(const auto shift:{-.0005,.0005})
        { o.initialShiftX=shift; run(rig,o,dir/(shift<0?"offset-x-minus":"offset-x-plus"),true,report); }
        o.initialShiftX=0; o.initialShiftZ=.0005; run(rig,o,dir/"offset-z",true,report); o.initialShiftZ=0;
        o.initialTilt=.0005; run(rig,o,dir/"tilt",true,report); o.initialTilt=0;
        o.yaw=.73; run(rig,o,dir/"translated-yaw",true,report); o.yaw=0;
        o.substeps=32; run(rig,o,dir/"substeps32",true,report); o.substeps=16;
        for(const auto time:{.008,.012}) { o.contactTime=time; run(rig,o,dir/(time<.01?"contact008":"contact012"),true,report); } o.contactTime=.01;
        o.effortScale=.5; const auto weak=run(rig,o,dir/"weak",false,report,720); JULIA_CHECK(weak.lost&&weak.hold==0); o.effortScale=1;
        o.initialShiftX=.05; const auto outside=run(rig,o,dir/"outside-initial",false,report,720); JULIA_CHECK(outside.lost&&outside.hold==0); o.initialShiftX=0;
        o.standEvent="lift"; const auto lift=run(rig,o,dir/"support-loss",false,report,720); JULIA_CHECK(lift.lost);
        o.standEvent="outside"; const auto force=run(rig,o,dir/"outside-force",false,report,720); JULIA_CHECK(force.lost);
        o.standEvent="disable"; run(rig,o,dir/"disable",false,report,720);
        performance(rig,dir,report); report<<"PASS\n";
        return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"STAND FAILURE: "<<e.what()<<'\n'; return 1; }
}
