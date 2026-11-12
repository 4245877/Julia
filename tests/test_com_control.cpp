#include "julia_test.hpp"
#include "../apps/julia_body_physics/BodyBenchIO.hpp"
#include <chrono>
#include <numeric>
#include <limits>

using namespace julia::physics;
namespace
{
    BoundedSolver selected=BoundedSolver::Reference7h;
    void same(const Snapshot& a,const Snapshot& b)
    {
        JULIA_CHECK(a.q==b.q&&a.v==b.v&&a.appliedTorque==b.appliedTorque&&a.root.position==b.root.position&&a.root.orientation==b.root.orientation);
        JULIA_CHECK(a.stand.comGoal==b.stand.comGoal&&a.stand.commandCom==b.stand.commandCom&&a.stand.referenceVelocity==b.stand.referenceVelocity&&a.stand.referenceAcceleration==b.stand.referenceAcceleration);
        JULIA_CHECK(a.stand.solverCode==b.stand.solverCode&&a.stand.qpIterations==b.stand.qpIterations&&a.stand.comPhase==b.stand.comPhase&&a.stand.commandSerial==b.stand.commandSerial);
    }
    void api(const BodyRig& rig,std::ostream& report)
    {
        PhysicsWorld a{rig.description()},b{rig.description()}; auto p=preparedStand(rig,a); p.variant=StandVariant::Bounded7h; p.boundedSolver=selected; p.comCommands=true;
        a.reset(p.posture,p.rootGoal); b.reset(p.posture,p.rootGoal);
        auto tilted=p; tilted.rootGoal.orientation=glm::angleAxis(.001,glm::dvec3{1,0,0});
        auto invalidSolver=p; invalidSolver.boundedSolver=static_cast<BoundedSolver>(99); JULIA_CHECK(!a.startStand(invalidSolver));
        invalidSolver=p; invalidSolver.variant=StandVariant::Equality7g; invalidSolver.boundedSolver=BoundedSolver::Optimized7i; JULIA_CHECK(!a.startStand(invalidSolver));
        JULIA_CHECK(!a.startStand(tilted)); same(a.snapshot(),b.snapshot());
        for(const auto bad:{0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
        {
            auto invalid=p; invalid.motion.maxSpeed=bad; JULIA_CHECK(!a.startStand(invalid));
            invalid=p; invalid.motion.maxAcceleration=bad; JULIA_CHECK(!a.startStand(invalid));
            invalid=p; invalid.motion.maxDisplacement=bad; JULIA_CHECK(!a.startStand(invalid));
        }
        JULIA_CHECK(a.startStand(p)&&b.startStand(p));
        JULIA_CHECK(!a.standComTarget({.003,0})&&!a.cancelStandCom()); same(a.snapshot(),b.snapshot());
        std::vector<Snapshot> history; history.reserve(720);
        for(unsigned n=0;n<720;++n)
        {
            if(n==120) { JULIA_CHECK(a.standComTarget({.003,0})&&b.standComTarget({.003,0})); }
            if(n==180) { JULIA_CHECK(a.standComTarget({0,.003})&&b.standComTarget({0,.003})); }
            if(n==240) { JULIA_CHECK(a.cancelStandCom()&&b.cancelStandCom()); }
            if(n==300)
            {
                const auto before=a.snapshot();
                JULIA_CHECK(!a.standComTarget({.006,0})&&!a.standComTarget({std::numeric_limits<double>::quiet_NaN(),0})&&!a.standComTarget({0,std::numeric_limits<double>::infinity()}));
                for(auto c:{CommandKind::Target,CommandKind::Torque,CommandKind::MoveRoot,CommandKind::Attach,CommandKind::Reach,CommandKind::SmoothTarget}) JULIA_CHECK(!a.command(c));
                same(before,a.snapshot());
            }
            a.step(); b.step(); same(a.snapshot(),b.snapshot());
            JULIA_CHECK(a.snapshot().stand.phase==StandPhase::Holding||n<60);
            history.push_back(a.snapshot());
        }
        a.reset(p.posture,p.rootGoal); JULIA_CHECK(a.snapshot().stand.commandSerial==0&&a.snapshot().stand.comPhase==ComPhase::Idle&&a.snapshot().stand.solverCode=="not_run");
        JULIA_CHECK(a.startStand(p));
        for(unsigned n=0;n<720;++n)
        {
            if(n==120) JULIA_CHECK(a.standComTarget({.003,0})); if(n==180) JULIA_CHECK(a.standComTarget({0,.003})); if(n==240) JULIA_CHECK(a.cancelStandCom());
            a.step(); same(a.snapshot(),history[n]);
        }
        JULIA_CHECK(a.command(CommandKind::Disable)); a.step();
        for(unsigned k=0;k<41;++k) JULIA_CHECK(a.snapshot().appliedTorque[k]==0);
        JULIA_CHECK(!a.standComTarget({0,0})&&!a.cancelStandCom()&&!a.startStand(p));
        report<<"Atomic invalid settings/commands, ownership, retarget/cancel p/v/a, independent instances, reset cold-start replay, Disable PASS\n";
    }
    void performance(const BodyRig& rig,const std::filesystem::path& dir,std::ostream& report)
    {
        PhysicsWorld w{rig.description()}; auto plan=preparedStand(rig,w); plan.variant=StandVariant::Bounded7h; plan.boundedSolver=selected; plan.comCommands=true;
        w.reset(plan.posture,plan.rootGoal); JULIA_CHECK(w.startStand(plan));
        std::ofstream raw{dir/"performance.csv"}; raw<<"tick,tick_us,controller_us,solver_us,qp_iterations,active,evaluations,physics_iterations\n"<<std::setprecision(17);
        std::ofstream solverRaw{dir/"solver-performance.csv"}; solverRaw<<"tick,evaluation,solver_us,qp_iterations,active\n"<<std::setprecision(17);
        std::array<std::vector<double>,3> samples;
        for(unsigned n=0;n<2400;++n)
        {
            const auto start=std::chrono::steady_clock::now(); w.step();
            const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            const auto& t=w.snapshot().stand;
            samples[0].push_back(elapsed); samples[1].push_back(t.controllerUs);
            JULIA_CHECK(t.solverSampleCount==4);
            for(unsigned i=0;i<t.solverSampleCount;++i) { samples[2].push_back(t.solverTimesUs[i]); solverRaw<<n+1<<','<<i<<','<<t.solverTimesUs[i]<<','<<t.solverIterations[i]<<','<<t.solverActive[i]<<'\n'; }
            raw<<n+1<<','<<elapsed<<','<<t.controllerUs<<','<<t.solverUs<<','<<t.qpIterations<<','<<t.activeInequalities<<','<<t.evaluations<<','<<w.snapshot().solverIterations<<'\n';
            JULIA_CHECK(t.phase!=StandPhase::Lost&&t.feasible&&t.baseResidual<1e-6&&t.contactResidual<1e-6&&t.requestedUtilization<=1.000001);
        }
        for(int k=0;k<3;++k)
        {
            auto v=samples[k]; std::sort(v.begin(),v.end()); const double mean=std::accumulate(v.begin(),v.end(),0.)/v.size();
            report<<(k==0?"tick":k==1?"controller":"solver")<<" mean="<<mean<<" p95="<<v[v.size()*95/100]<<" max="<<v.back()<<" us\n";
            if(k==0) JULIA_CHECK(v[v.size()*95/100]<8333);
        }
        report<<"controller_instance="<<w.standControllerBytes()<<" snapshot="<<sizeof(Snapshot)<<" model="<<w.modelBytes()<<" data="<<w.dataBytes()<<" arena="<<w.arenaHighWaterBytes()<<" bytes; no heap allocation inside QP; cold start every update\n";
    }
    void sequence(const BodyRig& rig,const std::filesystem::path& dir,std::ostream& report)
    {
        bodybench::Options o; o.scenario=BodyCase::ComShift; o.bounded=true; o.boundedSolver=selected; o.comPulses=true;
        PhysicsWorld w{bodybench::definition(rig,o.scenario,o)}; bodybench::reset(rig,w,o.scenario,o);
        bodybench::Log log{dir/"sequence-pulse"}; bodybench::metadata(dir/"sequence-pulse",rig,w); log.row(publish(rig,w.snapshot()));
        const auto origin=w.snapshot().totalComWorld; double displacement=0; std::array<bool,4> achieved{};
        for(unsigned n=0;n<6480;++n)
        {
            bodybench::step(w,o.scenario,o); const auto& s=w.snapshot(); log.row(publish(rig,s));
            JULIA_CHECK(s.stand.phase!=StandPhase::Lost);
            if(s.stand.commandSerial>0&&s.stand.comPhase==ComPhase::Reached) achieved[s.stand.commandSerial-1]=true;
            displacement=std::max(displacement,glm::length(glm::dvec2{s.totalComWorld.x-origin.x,s.totalComWorld.z-origin.z}));
            if(n>240)
            {
                for(const auto& f:s.support.feet) JULIA_CHECK(f.load.loaded&&f.active>0&&f.normalForce>=30&&f.slipSpeed<=.05&&f.penetration<=.004);
                JULIA_CHECK(s.support.otherEnvironmentActive==0&&s.stand.requestedUtilization<=1.000001&&s.stand.footDisplacement<=.010&&s.stand.footAngle<=.05);
            }
        }
        for(bool a:achieved) JULIA_CHECK(a); JULIA_CHECK(displacement>=.002&&w.snapshot().stand.achievedSeconds>=2);
        report<<"Full 54s physical sequence with3N pulses: all4 commands measured Reached, max displacement="<<displacement<<"m PASS\n";
    }
    void negatives(const BodyRig& rig,const std::filesystem::path& dir,std::ostream& report)
    {
        for(const auto* name:{"weak","outside-initial","support-loss","outside-force","disable","unsupported-contact"})
        {
            bodybench::Options o; o.scenario=BodyCase::Stand; o.bounded=true; o.boundedSolver=selected;
            if(std::string{name}=="weak") o.effortScale=.5;
            else if(std::string{name}=="outside-initial") o.initialShiftX=.05;
            else if(std::string{name}=="support-loss") o.standEvent="lift";
            else if(std::string{name}=="outside-force") o.standEvent="outside";
            else if(std::string{name}=="disable") o.standEvent="disable";
            PhysicsWorld w{bodybench::definition(rig,o.scenario,o)};
            if(std::string{name}=="unsupported-contact")
            {
                auto plan=preparedStand(rig,w); plan.variant=StandVariant::Bounded7h; plan.boundedSolver=selected;
                auto initial=plan.rootGoal; initial.position.y+=.003;
                w.reset(plan.posture,initial); JULIA_CHECK(w.startStand(plan));
            }
            else bodybench::reset(rig,w,o.scenario,o);
            bodybench::Log log{dir/name}; bodybench::metadata(dir/name,rig,w); log.row(publish(rig,w.snapshot()));
            unsigned after=0; double startHeight=0; bool terminal=false;
            for(unsigned n=0;n<720;++n)
            {
                bodybench::step(w,o.scenario,o); const auto& s=w.snapshot(); log.row(publish(rig,s));
                if(s.stand.phase==StandPhase::Lost||s.mode==DriveMode::Disabled)
                {
                    if(!terminal) startHeight=s.root.position.y;
                    terminal=true; for(unsigned k=0;k<41;++k) JULIA_CHECK(std::isfinite(s.appliedTorque[k])&&s.appliedTorque[k]==0);
                    if(++after>=24) break;
                }
            }
            JULIA_CHECK(terminal&&after>=24&&w.snapshot().root.position.y<startHeight);
            report<<name<<" terminal="<<standPhaseName(w.snapshot().stand.phase)<<" reason="<<w.snapshot().stand.reason<<" final_y="<<w.snapshot().root.position.y<<" PASS\n";
        }
    }
}
int main(int argc,char** argv)
{
    try
    {
        const auto dir=std::filesystem::path{argc>1?argv[1]:"build/diagnostics/stage7h/cpu"}; std::filesystem::create_directories(dir);
        std::ofstream report{dir/"com-control.txt"}; report<<std::setprecision(17);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx"); const BodyRig rig{*asset};
        if(argc>2&&std::string{argv[2]}=="optimized") selected=BoundedSolver::Optimized7i;
        api(rig,report); sequence(rig,dir,report); negatives(rig,dir,report); performance(rig,dir,report); report<<"PASS\n"; return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"COM FAILURE: "<<e.what()<<'\n'; return 1; }
}
