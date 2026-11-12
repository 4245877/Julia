#include "julia_test.hpp"
#include "../apps/julia_body_physics/BodyBenchIO.hpp"
#include "../libs/physics/src/BoundedQp.hpp"
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

// Stage7k: atomic coordinated {COM offset, alpha} commands. api: contracts and
// exact replay; corpus: same-input reference/optimized QPs; perf: CPU samples.
using namespace julia::physics;
namespace
{
    constexpr double Inf=std::numeric_limits<double>::infinity();
    constexpr double NaN=std::numeric_limits<double>::quiet_NaN();
    bool equal(double a,double b) { return a==b||(std::isnan(a)&&std::isnan(b)); }
    bool equal(glm::dvec3 a,glm::dvec3 b) { return equal(a.x,b.x)&&equal(a.y,b.y)&&equal(a.z,b.z); }
    void samePhysics(const Snapshot& a,const Snapshot& b)
    {
        JULIA_CHECK(a.tick==b.tick&&a.q==b.q&&a.v==b.v&&a.appliedTorque==b.appliedTorque);
        JULIA_CHECK(a.root.position==b.root.position&&a.root.orientation==b.root.orientation);
        JULIA_CHECK(a.totalComWorld==b.totalComWorld&&a.linearMomentumWorld==b.linearMomentumWorld);
        JULIA_CHECK(a.stand.solverCode==b.stand.solverCode&&a.stand.qpIterations==b.stand.qpIterations&&a.stand.phase==b.stand.phase);
    }
    void sameCommand(const Snapshot& a,const Snapshot& b)
    {
        const auto& x=a.stand.coordinated; const auto& y=b.stand.coordinated;
        JULIA_CHECK(x.enabled==y.enabled&&x.serial==y.serial&&x.phase==y.phase&&x.cancelled==y.cancelled&&x.stopped==y.stopped);
        JULIA_CHECK(x.goalOffset==y.goalOffset&&equal(x.goalAlpha,y.goalAlpha)&&equal(x.goalCom,y.goalCom));
        JULIA_CHECK(equal(x.comReference,y.comReference)&&equal(x.comVelocity,y.comVelocity)&&equal(x.comAcceleration,y.comAcceleration));
        JULIA_CHECK(equal(x.alphaReference,y.alphaReference)&&equal(x.alphaVelocity,y.alphaVelocity)&&equal(x.alphaAcceleration,y.alphaAcceleration));
        JULIA_CHECK(equal(x.duration,y.duration)&&equal(x.elapsed,y.elapsed)&&equal(x.achievedSeconds,y.achievedSeconds));
        const auto& l=a.stand.load; const auto& m=b.stand.load;
        JULIA_CHECK(l.baselineValid==m.baselineValid&&l.actualValid==m.actualValid&&equal(l.alpha0,m.alpha0)&&equal(l.actual,m.actual));
        JULIA_CHECK(equal(l.reference,m.reference)&&equal(l.command,m.command)&&equal(l.predicted,m.predicted));
        JULIA_CHECK(a.stand.comGoal==b.stand.comGoal);
    }
    void same(const Snapshot& a,const Snapshot& b) { samePhysics(a,b); sameCommand(a,b); }
    StandSettings settings(const BodyRig& rig,PhysicsWorld& w,const glm::dmat4& placement=glm::dmat4{1})
    {
        auto p=preparedStand(rig,w,placement); p.variant=StandVariant::Bounded7h;
        p.boundedSolver=BoundedSolver::Optimized7i; p.coordinatedCommands=true; return p;
    }
    // Reference envelopes for every completed tick, including retarget/cancel instants.
    void limits(const Snapshot& s)
    {
        const auto& c=s.stand.coordinated; const auto& l=s.stand.load;
        JULIA_CHECK(glm::length(c.comVelocity)<=.001+1e-12&&glm::length(c.comAcceleration)<=.002+1e-12);
        JULIA_CHECK(glm::length(c.comReference-s.stand.initialCom)<=.005+1e-12);
        if(l.baselineValid)
            JULIA_CHECK(std::abs(c.alphaVelocity)<=.02+1e-12&&std::abs(c.alphaAcceleration)<=.04+1e-12&&std::abs(c.alphaReference-l.alpha0)<=.05+1e-12);
    }
    bool command(PhysicsWorld& w,std::optional<glm::dvec2> com,std::optional<double> alphaOffset)
    {
        CoordinatedTarget t; t.comOffset=com; if(alphaOffset) t.alpha=w.snapshot().stand.load.alpha0+*alphaOffset;
        return w.standCoordinatedTarget(t);
    }
    void api(const BodyRig& rig,const std::filesystem::path& dir)
    {
        std::ofstream report{dir/"api.txt"}; report<<std::setprecision(17);
        PhysicsWorld a{rig.description()},b{rig.description()}; const auto p=settings(rig,a);
        a.reset(p.posture,p.rootGoal); b.reset(p.posture,p.rootGoal);
        const auto pristine=a.snapshot();
        auto bad=p; bad.variant=StandVariant::Equality7g; bad.boundedSolver=BoundedSolver::Reference7h; JULIA_CHECK(!a.startStand(bad));
        bad=p; bad.boundedSolver=BoundedSolver::Reference7h; JULIA_CHECK(!a.startStand(bad));
        bad=p; bad.comCommands=true; JULIA_CHECK(!a.startStand(bad));
        bad=p; bad.loadCommands=true; JULIA_CHECK(!a.startStand(bad));
        bad=p; bad.coordinatedCommands=false; bad.coordinatedComTask=false; JULIA_CHECK(!a.startStand(bad));
        bad=p; bad.coordinatedCommands=false; bad.coordinatedLoadTask=false; JULIA_CHECK(!a.startStand(bad));
        samePhysics(pristine,a.snapshot());
        JULIA_CHECK(a.startStand(p)&&b.startStand(p));
        JULIA_CHECK(a.snapshot().stand.coordinated.enabled&&!a.snapshot().stand.load.enabled);
        const auto started=a.snapshot();
        JULIA_CHECK(!command(a,glm::dvec2{.003,0},.05)&&!a.cancelStandCoordinated()&&!a.returnStandCoordinated());
        same(started,a.snapshot());
        report<<"Settings: equality/reference/COM-mode/load-mode conflicts and task ablation without mode rejected atomically.\n";
        std::vector<Snapshot> history; history.reserve(1500);
        // Command script shared by both independent worlds and the replay.
        const auto script=[&](PhysicsWorld& w,unsigned n)
        {
            const auto before=w.snapshot(); const auto& x=before.stand.coordinated; bool issued=true,ok=false;
            if(n==480) ok=command(w,glm::dvec2{.003,0},.05);
            else if(n==600) ok=command(w,glm::dvec2{-.003,0},-.05);        // retarget both while moving
            else if(n==840) ok=command(w,glm::dvec2{0,.003},std::nullopt);   // COM only; alpha goal kept
            else if(n==1000) ok=command(w,std::nullopt,.05);                 // alpha only; COM goal kept
            else if(n==1100) ok=w.cancelStandCoordinated();
            else if(n==1300) ok=w.returnStandCoordinated();
            else issued=false;
            if(!issued) return;
            JULIA_CHECK(ok);
            const auto& y=w.snapshot().stand.coordinated;
            // C2 continuity at the command instant: the new curves start at current p/v/a.
            JULIA_CHECK(x.comReference==y.comReference&&x.comVelocity==y.comVelocity&&x.comAcceleration==y.comAcceleration);
            JULIA_CHECK(x.alphaReference==y.alphaReference&&x.alphaVelocity==y.alphaVelocity&&x.alphaAcceleration==y.alphaAcceleration);
            JULIA_CHECK(y.serial==x.serial+1&&y.phase==ComPhase::Moving&&!y.stopped&&y.elapsed==0&&y.duration>0);
            samePhysics(before,w.snapshot());
            if(n==840) JULIA_CHECK(y.goalAlpha==x.goalAlpha&&y.goalOffset==glm::dvec2(0,.003));
            if(n==1000) JULIA_CHECK(y.goalCom==x.goalCom&&y.goalOffset==x.goalOffset&&y.goalAlpha==before.stand.load.alpha0+.05);
            if(n==1100)
            {
                // Generated stop goals are published for both components.
                const double alpha=std::clamp(x.alphaReference+x.alphaVelocity,before.stand.load.alpha0-.05,before.stand.load.alpha0+.05);
                JULIA_CHECK(y.cancelled&&y.goalAlpha==alpha&&glm::length(y.goalCom-(x.comReference+x.comVelocity))<1e-15);
            }
            if(n==1300) JULIA_CHECK(!y.cancelled&&y.goalOffset==glm::dvec2{0}&&y.goalAlpha==before.stand.load.alpha0&&y.goalCom==before.stand.initialCom);
        };
        for(unsigned n=0;n<1500;++n)
        {
            if(n==300) { const auto before=a.snapshot(); JULIA_CHECK(!command(a,glm::dvec2{.003,0},.05)); same(before,a.snapshot()); }
            if(n==480)
            {
                const auto before=a.snapshot(); const double alpha0=before.stand.load.alpha0;
                JULIA_CHECK(before.stand.load.baselineValid&&before.stand.load.actualValid&&before.stand.phase==StandPhase::Holding);
                const std::vector<std::pair<std::optional<glm::dvec2>,std::optional<double>>> invalid{
                    {glm::dvec2{NaN,0},alpha0+.05},{glm::dvec2{.003,Inf},alpha0+.05},{glm::dvec2{.0051,0},alpha0+.05},{glm::dvec2{.004,.004},alpha0},
                    {glm::dvec2{.003,0},NaN},{glm::dvec2{.003,0},Inf},{glm::dvec2{.003,0},-Inf},{glm::dvec2{.003,0},alpha0+.06},{glm::dvec2{.003,0},alpha0-.0501},
                    {glm::dvec2{.003,0},.05},{std::nullopt,alpha0+.07},{glm::dvec2{-.006,0},std::nullopt},{std::nullopt,std::nullopt}};
                for(const auto& [com,alpha]:invalid)
                {
                    CoordinatedTarget t; t.comOffset=com; t.alpha=alpha; std::string reason;
                    JULIA_CHECK(!a.standCoordinatedTarget(t,&reason)&&!reason.empty()); same(before,a.snapshot());
                }
                JULIA_CHECK(!a.standComTarget({0,0})&&!a.cancelStandCom()&&!a.standLoadTarget(alpha0+.05)&&!a.cancelStandLoad()&&!a.returnStandLoad());
                same(before,a.snapshot());
                for(auto c:{CommandKind::Target,CommandKind::Torque,CommandKind::MoveRoot,CommandKind::Attach,CommandKind::Reach,CommandKind::SmoothTarget})
                { JULIA_CHECK(!a.command(c)); same(before,a.snapshot()); }
                report<<"Atomic rejection at4s: NaN/Inf/outside per component, empty target, separate COM/load APIs and drive commands; state unchanged.\n";
            }
            script(a,n); script(b,n); same(a.snapshot(),b.snapshot());
            a.step(); b.step(); same(a.snapshot(),b.snapshot());
            const auto& s=a.snapshot(); JULIA_CHECK(s.stand.phase!=StandPhase::Lost); limits(s);
            history.push_back(s);
        }
        report<<"Independent worlds:1500 exact completed-tick states: retarget both, COM-only, alpha-only, cancel with published stop goals, return.\n";
        // A rejected command during motion keeps both accepted curves (atomicity under motion).
        {
            PhysicsWorld w{rig.description()}; w.reset(p.posture,p.rootGoal); JULIA_CHECK(w.startStand(p));
            for(unsigned n=0;n<720;++n) { if(n==480) JULIA_CHECK(command(w,glm::dvec2{.003,0},.05)); w.step(); }
            const auto before=w.snapshot();
            JULIA_CHECK(!command(w,glm::dvec2{-.003,0},.08)&&!command(w,glm::dvec2{.0052,0},-.05));
            same(before,w.snapshot());
            w.step(); w.step(); const auto mid=w.snapshot();
            JULIA_CHECK(mid.stand.coordinated.serial==before.stand.coordinated.serial&&mid.stand.coordinated.goalAlpha==before.stand.coordinated.goalAlpha);
            report<<"Rejection during motion: one bad component leaves the moving accepted COM and alpha curves and serial unchanged.\n";
        }
        a.reset(p.posture,p.rootGoal);
        JULIA_CHECK(!a.snapshot().stand.load.baselineValid&&a.snapshot().stand.coordinated.serial==0);
        JULIA_CHECK(a.startStand(p));
        for(unsigned n=0;n<1500;++n) { script(a,n); a.step(); same(a.snapshot(),history[n]); }
        report<<"Reset/replay:1500 exact deterministic states; controller, calibration and curves reset.\n";
        JULIA_CHECK(a.command(CommandKind::Disable)); a.step();
        for(unsigned k=0;k<41;++k) JULIA_CHECK(a.snapshot().appliedTorque[k]==0);
        const auto evaluations=a.snapshot().stand.evaluations; a.step();
        JULIA_CHECK(a.snapshot().stand.evaluations==evaluations&&a.snapshot().stand.coordinated.phase==ComPhase::Idle);
        JULIA_CHECK(!command(a,glm::dvec2{0,0},0.)&&!a.cancelStandCoordinated()&&!a.returnStandCoordinated()&&!a.startStand(p));
        report<<"Disable: zero applied torque, no further evaluations, commands and restart without reset rejected.\n";
        // Both tasks off: identical commands/references are published but never enter the QP.
        {
            PhysicsWorld off{rig.description()},legacy{rig.description()};
            auto offPlan=settings(rig,off); offPlan.coordinatedComTask=false; offPlan.coordinatedLoadTask=false;
            auto oldPlan=offPlan; oldPlan.coordinatedCommands=false; oldPlan.coordinatedComTask=true; oldPlan.coordinatedLoadTask=true;
            off.reset(offPlan.posture,offPlan.rootGoal); legacy.reset(oldPlan.posture,oldPlan.rootGoal);
            JULIA_CHECK(off.startStand(offPlan)&&legacy.startStand(oldPlan));
            for(unsigned n=0;n<960;++n)
            {
                if(n==480) JULIA_CHECK(command(off,glm::dvec2{.003,0},.05));
                if(n==600) JULIA_CHECK(command(off,glm::dvec2{-.003,0},-.05));
                off.step(); legacy.step(); samePhysics(off.snapshot(),legacy.snapshot());
                JULIA_CHECK(off.snapshot().stand.comGoal==off.snapshot().stand.initialCom);
            }
            JULIA_CHECK(off.snapshot().stand.coordinated.serial==2&&glm::length(off.snapshot().stand.coordinated.comVelocity)>0);
            report<<"Both tasks off:960 exact physical states against unchanged optimized bounded controller with accepted moving commands.\n";
        }
        for(double direction:{1.,-1.})
        {
            PhysicsWorld edge{rig.description()}; const auto edgePlan=settings(rig,edge);
            edge.reset(edgePlan.posture,edgePlan.rootGoal); JULIA_CHECK(edge.startStand(edgePlan));
            std::ofstream raw{dir/(direction>0?"cancel-positive-coordinated-v1.csv":"cancel-negative-coordinated-v1.csv")}; coordinatedCsvHeader(raw);
            unsigned current=0,best=0;
            for(unsigned n=0;n<1860;++n)
            {
                if(n==480) JULIA_CHECK(command(edge,glm::dvec2{direction*.003,0},direction*.05));
                if(n==1020)
                {
                    const auto before=edge.snapshot(); const auto& x=before.stand.coordinated; const double alpha0=before.stand.load.alpha0;
                    // At8.5s the one-second alpha extrapolation leaves the calibrated range.
                    JULIA_CHECK(std::abs(x.alphaReference+x.alphaVelocity-alpha0)>.05&&x.elapsed<x.duration);
                    JULIA_CHECK(edge.cancelStandCoordinated());
                    const auto& y=edge.snapshot().stand.coordinated;
                    JULIA_CHECK(y.cancelled&&y.goalAlpha==alpha0+direction*.05&&y.serial==x.serial+1);
                    JULIA_CHECK(x.comReference==y.comReference&&x.comVelocity==y.comVelocity&&x.alphaReference==y.alphaReference&&x.alphaVelocity==y.alphaVelocity&&x.alphaAcceleration==y.alphaAcceleration);
                    JULIA_CHECK(y.duration<=x.duration-x.elapsed+1e-12);
                }
                edge.step(); const auto& s=edge.snapshot(); coordinatedCsvRow(raw,s);
                JULIA_CHECK(s.stand.phase!=StandPhase::Lost); limits(s);
                if(n>=1020)
                {
                    const auto& c=s.stand.coordinated; const auto& l=s.stand.load;
                    bool good=c.stopped&&l.actualValid&&std::abs(l.actual-c.goalAlpha)<=.01&&c.comError<=.0005&&c.speed<=.001;
                    for(const auto& foot:s.support.feet) good=good&&foot.load.loaded&&foot.active>0&&foot.normalForce>=30;
                    current=good?current+1:0; best=std::max(best,current);
                }
            }
            JULIA_CHECK(best>=241&&edge.snapshot().stand.coordinated.phase==ComPhase::Cancelled);
            report<<"Boundary cancel direction="<<direction<<" at8.5s: alpha stop goal clamped to alpha0"<<(direction>0?"+":"-")<<".05, common C2 stop, joint actual hold="<<(best>0?(best-1)*TickSeconds:0.)<<"s.\n";
        }
        report<<(JULIA_TEST_RESULT()==0?"PASS\n":"FAIL: see process assertion log\n");
    }
    template<class T> void binary(std::ostream& out,const T& value)
    { out.write(reinterpret_cast<const char*>(&value),sizeof(value)); }
    template<class T,std::size_t N> void binary(std::ostream& out,const std::array<T,N>& value)
    { out.write(reinterpret_cast<const char*>(value.data()),sizeof(T)*N); }
    double objective(const qp::Problem& p,const qp::Result& r)
    {
        double v=0;
        for(int i=0;i<p.n;++i)
        { v-=p.g[i]*r.x[i]; for(int j=0;j<p.n;++j) v+=.5*r.x[i]*p.h[i*qp::Variables+j]*r.x[j]; }
        return v;
    }
    void corpus(const BodyRig& rig,const std::filesystem::path& dir)
    {
        auto solver=std::make_unique<qp::Solver>(); auto p=std::make_unique<qp::Problem>();
        std::ofstream raw{dir/"corpus.bin",std::ios::binary}; raw.write("J7IQP001",8);
        std::ofstream summary{dir/"comparison.csv"};
        summary<<"scenario,tick,serial,alpha_goal,alpha_actual,offset_x,offset_z,active,iterations,relative_objective_error,max_primal_error,equality,violation,stationarity\n"<<std::setprecision(17);
        unsigned count=0;
        for(int scenario=0;scenario<2;++scenario)
        {
            bodybench::Options o; o.scenario=BodyCase::CoordShift; o.bounded=true; o.boundedSolver=BoundedSolver::Optimized7i; o.comPulses=scenario!=0;
            PhysicsWorld w{bodybench::definition(rig,o.scenario,o)}; bodybench::reset(rig,w,o.scenario,o);
            std::array<bool,qp::Inequalities> previous{};
            for(unsigned n=0;n<8400;++n)
            {
                bodybench::step(w,o.scenario,o); qp::Result r;
                if(w.snapshot().stand.phase==StandPhase::Lost||!w.inspectStandQp(*p,r))
                    throw std::runtime_error("Coordinated corpus lost control or has no QP at tick "+std::to_string(n+1));
                std::array<bool,qp::Inequalities> active{};
                for(int i=0;i<p->m;++i) active[i]=r.multipliers[i]>1e-9;
                const bool pulse=scenario&&((n>=1670&&n<=1730)||(n>=4310&&n<=4370));
                const bool select=n%30==0||active!=previous||pulse; previous=active; if(!select) continue;
                const auto ref=solver->solve(*p); const auto opt=solver->solve(*p,128,qp::Implementation::Optimized7i);
                JULIA_CHECK(ref.code==qp::Code::Feasible&&opt.code==qp::Code::Feasible);
                JULIA_CHECK(r.x==opt.x);
                const double x=objective(*p,ref),y=objective(*p,opt),relative=std::abs(x-y)/std::max(1.,std::abs(x));
                double error=0; for(int i=0;i<p->n;++i) error=std::max(error,std::abs(ref.x[i]-opt.x[i]));
                JULIA_CHECK(relative<=1e-8&&error<=1e-5&&opt.equalityResidual<=1e-6&&opt.violation<=1e-7&&opt.projectedStationarity<=1e-5);
                binary(raw,std::int32_t{scenario}); binary(raw,std::uint32_t{n+1});
                binary(raw,std::int32_t{p->n}); binary(raw,std::int32_t{p->e}); binary(raw,std::int32_t{p->m});
                binary(raw,p->h); binary(raw,p->g); binary(raw,p->equality); binary(raw,p->target); binary(raw,p->inequality); binary(raw,p->lower);
                for(const auto* result:{&ref,&opt})
                { binary(raw,result->x); binary(raw,result->multipliers); binary(raw,result->iterations); binary(raw,result->active); }
                const auto& c=w.snapshot().stand.coordinated;
                summary<<scenario<<','<<n+1<<','<<c.serial<<','<<c.goalAlpha<<','<<w.snapshot().stand.load.actual<<','<<c.actualOffset.x<<','<<c.actualOffset.y<<','<<opt.active<<','<<opt.iterations
                    <<','<<relative<<','<<error<<','<<opt.equalityResidual<<','<<opt.violation<<','<<opt.projectedStationarity<<'\n'; ++count;
            }
        }
        std::ofstream{dir/"corpus.txt"}<<(JULIA_TEST_RESULT()==0?"PASS":"FAIL")<<" identical-input coordinated QPs="<<count<<"; coord-shift quiet/pulses70s; every30th tick, last-update active transitions and pulse windows; J7IQP001\n";
    }
    void performance(const BodyRig& rig,const std::filesystem::path& dir)
    {
        bodybench::Options o; o.scenario=BodyCase::CoordShift; o.bounded=true; o.boundedSolver=BoundedSolver::Optimized7i;
        PhysicsWorld w{bodybench::definition(rig,o.scenario,o)}; bodybench::reset(rig,w,o.scenario,o);
        std::ofstream raw{dir/"performance.csv"}; raw<<"tick,tick_us,controller_us,evaluations,physical_steps,physics_iterations,phase,serial,alpha_goal,alpha_reference,alpha_actual,actual_valid,offset_x,offset_z,com_error_m,base_residual,contact_residual\n"<<std::setprecision(17);
        std::ofstream calls{dir/"calls.csv"}; calls<<"tick,call,assembly_and_solve_us,preparation_us,input_us,linear_us,active_us,verification_us,physical_us,iterations,active\n"<<std::setprecision(17);
        constexpr unsigned Warmup=480,Ticks=8400;
        for(unsigned n=0;n<Ticks;++n)
        {
            const auto start=std::chrono::steady_clock::now(); bodybench::step(w,o.scenario,o);
            const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            const auto& s=w.snapshot(); const auto& t=s.stand; const auto& c=t.coordinated;
            JULIA_CHECK(t.solverSampleCount==4&&t.feasible&&t.phase!=StandPhase::Lost&&s.worldSteps==16ull*(n+1));
            JULIA_CHECK(t.baseResidual<1e-6&&t.contactResidual<1e-6&&t.requestedUtilization<=1.000001);
            if(n<Warmup) continue;
            raw<<n+1<<','<<elapsed<<','<<t.controllerUs<<','<<t.solverSampleCount<<",16,"<<s.solverIterations<<','<<standPhaseName(t.phase)<<','<<c.serial<<','<<c.goalAlpha<<','<<c.alphaReference
                <<','<<t.load.actual<<','<<t.load.actualValid<<','<<c.actualOffset.x<<','<<c.actualOffset.y<<','<<c.comError<<','<<t.baseResidual<<','<<t.contactResidual<<'\n';
            for(unsigned i=0;i<t.solverSampleCount;++i)
            {
                const auto& q=t.boundedProfiles[i]; calls<<n+1<<','<<i<<','<<t.solverTimesUs[i]<<','<<q.preparationUs<<','<<q.inputUs<<','<<q.linearUs<<','<<q.activeUs<<','<<q.verificationUs<<','<<q.physicalUs<<','<<t.solverIterations[i]<<','<<t.solverActive[i]<<'\n';
            }
        }
        std::ofstream meta{dir/"metadata.json"};
        meta<<"{\"scenario\":\"coord-shift\",\"optimized\":true,\"coordinated\":true,\"com_task\":true,\"load_task\":true,\"pulses\":false,\"warmup_ticks\":"<<Warmup<<",\"measured_ticks\":"<<Ticks-Warmup
            <<",\"controller_bytes\":"<<w.standControllerBytes()<<",\"snapshot_bytes\":"<<sizeof(Snapshot)<<",\"model_bytes\":"<<w.modelBytes()<<",\"data_bytes\":"<<w.dataBytes()<<",\"arena_bytes\":"<<w.arenaHighWaterBytes();
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS_EX pm{}; pm.cb=sizeof(pm);
        if(GetProcessMemoryInfo(GetCurrentProcess(),reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&pm),sizeof(pm)))
            meta<<",\"working_set_bytes\":"<<pm.WorkingSetSize<<",\"peak_working_set_bytes\":"<<pm.PeakWorkingSetSize<<",\"private_bytes\":"<<pm.PrivateUsage;
#endif
        meta<<"}\n";
    }
}
int main(int argc,char** argv)
{
    try
    {
        if(argc!=3) throw std::invalid_argument("output api|perf|corpus");
        const std::filesystem::path dir{argv[1]}; std::filesystem::create_directories(dir);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx"); const BodyRig rig{*asset};
        const std::string mode{argv[2]};
        if(mode=="api") api(rig,dir); else if(mode=="perf") performance(rig,dir); else if(mode=="corpus") corpus(rig,dir);
        else throw std::invalid_argument("Expected api, perf or corpus");
        return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"STAGE7K FAILURE: "<<e.what()<<'\n'; return 1; }
}
