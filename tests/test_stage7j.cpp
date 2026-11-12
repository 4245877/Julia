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

using namespace julia::physics;
namespace
{
    bool equal(double a,double b) { return a==b||(std::isnan(a)&&std::isnan(b)); }
    void samePhysics(const Snapshot& a,const Snapshot& b)
    {
        JULIA_CHECK(a.q==b.q&&a.v==b.v&&a.appliedTorque==b.appliedTorque);
        JULIA_CHECK(a.root.position==b.root.position&&a.root.orientation==b.root.orientation);
        JULIA_CHECK(a.totalComWorld==b.totalComWorld&&a.linearMomentumWorld==b.linearMomentumWorld);
        JULIA_CHECK(a.stand.solverCode==b.stand.solverCode&&a.stand.qpIterations==b.stand.qpIterations);
    }
    void sameLoad(const Snapshot& a,const Snapshot& b)
    {
        const auto& x=a.stand.load; const auto& y=b.stand.load;
        JULIA_CHECK(x.enabled==y.enabled&&x.taskEnabled==y.taskEnabled&&x.baselineValid==y.baselineValid);
        JULIA_CHECK(x.actualValid==y.actualValid&&x.predictedValid==y.predictedValid&&x.serial==y.serial&&x.phase==y.phase);
        JULIA_CHECK(equal(x.alpha0,y.alpha0)&&equal(x.command,y.command)&&equal(x.reference,y.reference));
        JULIA_CHECK(equal(x.velocity,y.velocity)&&equal(x.acceleration,y.acceleration));
        JULIA_CHECK(equal(x.actual,y.actual)&&equal(x.predicted,y.predicted)&&equal(x.filtered,y.filtered));
        JULIA_CHECK(equal(x.integral,y.integral)&&equal(x.feedback,y.feedback));
        JULIA_CHECK(equal(x.achievedSeconds,y.achievedSeconds)&&equal(x.baselineSeconds,y.baselineSeconds));
    }
    void same(const Snapshot& a,const Snapshot& b) { samePhysics(a,b); sameLoad(a,b); }
    StandSettings settings(const BodyRig& rig,PhysicsWorld& w)
    {
        auto p=preparedStand(rig,w);p.variant=StandVariant::Bounded7h;
        p.boundedSolver=BoundedSolver::Optimized7i;p.loadCommands=true;return p;
    }
    void api(const BodyRig& rig,const std::filesystem::path& dir)
    {
        std::ofstream report{dir/"api.txt"};report<<std::setprecision(17);
        PhysicsWorld a{rig.description()},b{rig.description()};const auto p=settings(rig,a);
        a.reset(p.posture,p.rootGoal);b.reset(p.posture,p.rootGoal);
        auto bad=p;bad.variant=StandVariant::Equality7g;JULIA_CHECK(!a.startStand(bad));
        bad=p;bad.boundedSolver=BoundedSolver::Reference7h;JULIA_CHECK(!a.startStand(bad));
        bad=p;bad.comCommands=true;JULIA_CHECK(!a.startStand(bad));
        same(a.snapshot(),b.snapshot());
        JULIA_CHECK(a.startStand(p)&&b.startStand(p));
        const auto initial=a.snapshot();
        JULIA_CHECK(!a.standLoadTarget(.4)&&!a.cancelStandLoad()&&!a.returnStandLoad());
        same(initial,a.snapshot());
        std::vector<Snapshot> history;history.reserve(600);
        const auto commands=[&](PhysicsWorld& w,unsigned n)
        {
            if(n==480) JULIA_CHECK(w.standLoadTarget(w.snapshot().stand.load.alpha0+.05));
            if(n==510) JULIA_CHECK(w.standLoadTarget(w.snapshot().stand.load.alpha0-.05));
            if(n==540) JULIA_CHECK(w.cancelStandLoad());
            if(n==570) JULIA_CHECK(w.returnStandLoad());
        };
        for(unsigned n=0;n<600;++n)
        {
            if(n==480)
            {
                JULIA_CHECK(a.snapshot().stand.load.baselineValid&&a.snapshot().stand.load.actualValid);
                const auto before=a.snapshot();const double alpha0=before.stand.load.alpha0;
                for(double value:{alpha0+.06,alpha0-.06,-1.,2.,std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()})
                { JULIA_CHECK(!a.standLoadTarget(value));same(before,a.snapshot()); }
                JULIA_CHECK(!a.standComTarget({0,0})&&!a.cancelStandCom());same(before,a.snapshot());
                for(auto c:{CommandKind::Target,CommandKind::Torque,CommandKind::MoveRoot,CommandKind::Attach,CommandKind::Reach,CommandKind::SmoothTarget})
                { JULIA_CHECK(!a.command(c));same(before,a.snapshot()); }
            }
            const auto before=a.snapshot();commands(a,n);commands(b,n);
            if(n==480||n==510||n==540||n==570)
            {
                const auto& x=before.stand.load;const auto& y=a.snapshot().stand.load;
                JULIA_CHECK(x.reference==y.reference&&x.velocity==y.velocity&&x.acceleration==y.acceleration);
                JULIA_CHECK(y.serial==x.serial+1);same(a.snapshot(),b.snapshot());
            }
            a.step();b.step();same(a.snapshot(),b.snapshot());
            const auto& s=a.snapshot();JULIA_CHECK(s.stand.phase!=StandPhase::Lost);
            JULIA_CHECK(std::abs(s.stand.load.velocity)<=.02000000001&&std::abs(s.stand.load.acceleration)<=.04000000001);
            history.push_back(s);
        }
        report<<"Independent worlds:600 exact completed-tick states, including baseline and retarget/cancel/return.\n";
        a.reset(p.posture,p.rootGoal);
        JULIA_CHECK(!a.snapshot().stand.load.baselineValid&&a.snapshot().stand.load.serial==0);
        JULIA_CHECK(a.startStand(p));
        for(unsigned n=0;n<600;++n) { commands(a,n);a.step();same(a.snapshot(),history[n]); }
        report<<"Reset/replay:600 exact deterministic states; controller history reset.\n";
        JULIA_CHECK(a.command(CommandKind::Disable));a.step();
        for(unsigned k=0;k<41;++k) JULIA_CHECK(a.snapshot().appliedTorque[k]==0);
        JULIA_CHECK(!a.standLoadTarget(.4)&&!a.cancelStandLoad()&&!a.returnStandLoad()&&!a.startStand(p));
        report<<"Atomic invalid settings, NaN/Inf/range, COM conflict, exclusive ownership, C2 retarget/cancel/return and Disable checked.\n";

        PhysicsWorld off{rig.description()},legacy{rig.description()};auto offPlan=settings(rig,off);offPlan.loadTask=false;
        auto oldPlan=offPlan;oldPlan.loadCommands=false;
        off.reset(offPlan.posture,offPlan.rootGoal);legacy.reset(oldPlan.posture,oldPlan.rootGoal);
        JULIA_CHECK(off.startStand(offPlan)&&legacy.startStand(oldPlan));
        for(unsigned n=0;n<600;++n)
        {
            if(n==480) JULIA_CHECK(off.standLoadTarget(off.snapshot().stand.load.alpha0+.05));
            off.step();legacy.step();samePhysics(off.snapshot(),legacy.snapshot());
        }
        report<<"Task disabled:600 exact physical states against unchanged optimized bounded controller, including accepted load command.\n";
        for(double direction:{1.,-1.})
        {
            PhysicsWorld edge{rig.description()};const auto edgePlan=settings(rig,edge);
            edge.reset(edgePlan.posture,edgePlan.rootGoal);JULIA_CHECK(edge.startStand(edgePlan));
            std::ofstream raw{dir/(direction>0?"cancel-positive-load-v1.csv":"cancel-negative-load-v1.csv")};loadCsvHeader(raw);
            unsigned currentHold=0,bestHold=0;
            for(unsigned n=0;n<1860;++n)
            {
                if(n==480)JULIA_CHECK(edge.standLoadTarget(edge.snapshot().stand.load.alpha0+direction*.05));
                if(n==900)
                {
                    const auto before=edge.snapshot().stand.load;
                    // At7.5s the one-second extrapolation lies outside the command
                    // range on either side. Cancel must find a bounded C2 stop.
                    JULIA_CHECK(std::abs(before.reference+before.velocity-before.alpha0)>.05);
                    JULIA_CHECK(edge.cancelStandLoad());
                    const auto& after=edge.snapshot().stand.load;
                    JULIA_CHECK(before.reference==after.reference&&before.velocity==after.velocity&&before.acceleration==after.acceleration);
                    JULIA_CHECK(after.serial==before.serial+1);
                }
                edge.step();const auto& s=edge.snapshot();const auto& t=s.stand.load;loadCsvRow(raw,s);
                JULIA_CHECK(s.stand.phase!=StandPhase::Lost);
                JULIA_CHECK(std::abs(t.velocity)<=.02000000001&&std::abs(t.acceleration)<=.04000000001);
                if(t.baselineValid)JULIA_CHECK(std::abs(t.reference-t.alpha0)<=.05000000001);
                if(n>=900)
                {
                    const auto delta=s.totalComWorld-s.stand.initialCom,velocity=s.linearMomentumWorld/s.totalMass;
                    bool good=t.actualValid&&std::abs(t.actual-t.command)<=.01&&direction*(t.actual-t.alpha0)>=.04&&
                        glm::length(glm::dvec2{delta.x,delta.z})<=.0005&&glm::length(glm::dvec2{velocity.x,velocity.z})<=.001;
                    for(const auto& foot:s.support.feet)good=good&&foot.load.loaded&&foot.active>0&&foot.normalForce>=30;
                    currentHold=good?currentHold+1:0;bestHold=std::max(bestHold,currentHold);
                }
            }
            JULIA_CHECK(bestHold>=241&&edge.snapshot().stand.load.phase==ComPhase::Cancelled);
            report<<"Boundary cancel direction="<<direction<<" at7.5s; certified bounded C2 reference and actual hold="<<(bestHold>0?(bestHold-1)*TickSeconds:0.)<<"s.\n";
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
        {v-=p.g[i]*r.x[i];for(int j=0;j<p.n;++j)v+=.5*r.x[i]*p.h[i*qp::Variables+j]*r.x[j];}
        return v;
    }
    void corpus(const BodyRig& rig,const std::filesystem::path& dir)
    {
        auto solver=std::make_unique<qp::Solver>();auto p=std::make_unique<qp::Problem>();
        std::ofstream raw{dir/"corpus.bin",std::ios::binary};raw.write("J7IQP001",8);
        std::ofstream summary{dir/"comparison.csv"};
        summary<<"scenario,tick,serial,alpha_command,alpha_actual,active,iterations,relative_objective_error,max_primal_error,equality,violation,stationarity\n"<<std::setprecision(17);
        unsigned count=0;
        for(int scenario=0;scenario<2;++scenario)
        {
            bodybench::Options o;o.scenario=BodyCase::LoadShift;o.bounded=true;o.boundedSolver=BoundedSolver::Optimized7i;o.comPulses=scenario!=0;
            PhysicsWorld w{bodybench::definition(rig,o.scenario,o)};bodybench::reset(rig,w,o.scenario,o);
            std::array<bool,qp::Inequalities> previous{};
            for(unsigned n=0;n<7200;++n)
            {
                bodybench::step(w,o.scenario,o);qp::Result r;
                if(w.snapshot().stand.phase==StandPhase::Lost||!w.inspectStandQp(*p,r))
                    throw std::runtime_error("Load corpus lost control or has no QP at tick "+std::to_string(n+1));
                std::array<bool,qp::Inequalities> active{};
                for(int i=0;i<p->m;++i) active[i]=r.multipliers[i]>1e-9;
                const bool pulse=scenario&&((n>=1670&&n<=1730)||(n>=4070&&n<=4130));
                const bool select=n%30==0||active!=previous||pulse;previous=active;if(!select) continue;
                const auto ref=solver->solve(*p);const auto opt=solver->solve(*p,128,qp::Implementation::Optimized7i);
                JULIA_CHECK(ref.code==qp::Code::Feasible&&opt.code==qp::Code::Feasible);
                JULIA_CHECK(r.x==opt.x);
                const double a=objective(*p,ref),b=objective(*p,opt),relative=std::abs(a-b)/std::max(1.,std::abs(a));
                double error=0;for(int i=0;i<p->n;++i)error=std::max(error,std::abs(ref.x[i]-opt.x[i]));
                JULIA_CHECK(relative<=1e-8&&error<=1e-5&&opt.equalityResidual<=1e-6&&opt.violation<=1e-7&&opt.projectedStationarity<=1e-5);
                binary(raw,std::int32_t{scenario});binary(raw,std::uint32_t{n+1});
                binary(raw,std::int32_t{p->n});binary(raw,std::int32_t{p->e});binary(raw,std::int32_t{p->m});
                binary(raw,p->h);binary(raw,p->g);binary(raw,p->equality);binary(raw,p->target);binary(raw,p->inequality);binary(raw,p->lower);
                for(const auto* result:{&ref,&opt})
                {binary(raw,result->x);binary(raw,result->multipliers);binary(raw,result->iterations);binary(raw,result->active);}
                const auto& load=w.snapshot().stand.load;
                summary<<scenario<<','<<n+1<<','<<load.serial<<','<<load.command<<','<<load.actual<<','<<opt.active<<','<<opt.iterations<<','<<relative<<','<<error<<','<<opt.equalityResidual<<','<<opt.violation<<','<<opt.projectedStationarity<<'\n';++count;
            }
        }
        std::ofstream{dir/"corpus.txt"}<<(JULIA_TEST_RESULT()==0?"PASS":"FAIL")<<" identical-input load QPs="<<count<<"; quiet/pulses60s; every30th tick, last-update active transitions and pulse windows; J7IQP001\n";
    }
    void performance(const BodyRig& rig,const std::filesystem::path& dir)
    {
        bodybench::Options o;o.scenario=BodyCase::LoadShift;o.bounded=true;o.boundedSolver=BoundedSolver::Optimized7i;
        PhysicsWorld w{bodybench::definition(rig,o.scenario,o)};bodybench::reset(rig,w,o.scenario,o);
        std::ofstream raw{dir/"performance.csv"};raw<<"tick,tick_us,controller_us,evaluations,physical_steps,physics_iterations,phase,command,reference,actual,actual_valid,com_x,com_y,com_z,base_residual,contact_residual\n"<<std::setprecision(17);
        std::ofstream calls{dir/"calls.csv"};calls<<"tick,call,assembly_and_solve_us,preparation_us,input_us,linear_us,active_us,verification_us,physical_us,iterations,active\n"<<std::setprecision(17);
        for(unsigned n=0;n<2880;++n)
        {
            const auto start=std::chrono::steady_clock::now();bodybench::step(w,o.scenario,o);
            const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            const auto& s=w.snapshot();const auto& t=s.stand;
            JULIA_CHECK(t.solverSampleCount==4&&t.feasible&&t.phase!=StandPhase::Lost);
            JULIA_CHECK(t.baseResidual<1e-6&&t.contactResidual<1e-6&&t.requestedUtilization<=1.000001);
            if(n<480)continue;
            raw<<n+1<<','<<elapsed<<','<<t.controllerUs<<','<<t.solverSampleCount<<",16,"<<s.solverIterations<<','<<standPhaseName(t.phase)<<','<<t.load.command<<','<<t.load.reference<<','<<t.load.actual<<','<<t.load.actualValid<<','<<s.totalComWorld.x<<','<<s.totalComWorld.y<<','<<s.totalComWorld.z<<','<<t.baseResidual<<','<<t.contactResidual<<'\n';
            for(unsigned i=0;i<t.solverSampleCount;++i)
            {
                const auto& p=t.boundedProfiles[i];calls<<n+1<<','<<i<<','<<t.solverTimesUs[i]<<','<<p.preparationUs<<','<<p.inputUs<<','<<p.linearUs<<','<<p.activeUs<<','<<p.verificationUs<<','<<p.physicalUs<<','<<t.solverIterations[i]<<','<<t.solverActive[i]<<'\n';
            }
        }
        std::ofstream meta{dir/"metadata.json"};
        meta<<"{\"optimized\":true,\"load_task\":true,\"pulses\":false,\"warmup_ticks\":480,\"measured_ticks\":2400,\"controller_bytes\":"<<w.standControllerBytes()<<",\"snapshot_bytes\":"<<sizeof(Snapshot)<<",\"model_bytes\":"<<w.modelBytes()<<",\"data_bytes\":"<<w.dataBytes()<<",\"arena_bytes\":"<<w.arenaHighWaterBytes();
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS_EX pm{};pm.cb=sizeof(pm);
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
        if(argc!=3)throw std::invalid_argument("output api|perf|corpus");
        const std::filesystem::path dir{argv[1]};std::filesystem::create_directories(dir);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");const BodyRig rig{*asset};
        const std::string mode{argv[2]};
        if(mode=="api")api(rig,dir);else if(mode=="perf")performance(rig,dir);else if(mode=="corpus")corpus(rig,dir);
        else throw std::invalid_argument("Expected api, perf or corpus");
        return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e){std::cerr<<"STAGE7J FAILURE: "<<e.what()<<'\n';return 1;}
}
