#include "julia_test.hpp"
#include "../apps/julia_body_physics/BodyBenchIO.hpp"
#include "../libs/physics/src/BoundedQp.hpp"
#include <chrono>
#include <memory>
#include <cstdint>
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

using namespace julia::physics;
namespace
{
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
        std::ofstream summary{dir/"comparison.csv"}; summary<<"scenario,tick,active,iterations,objective_reference,objective_optimized,relative_objective_error,max_primal_error,equality,violation,stationarity\n"<<std::setprecision(17);
        unsigned count=0;
        for(int scenario=0;scenario<2;++scenario)
        {
            bodybench::Options o; o.scenario=scenario?BodyCase::ComShift:BodyCase::Stand; o.bounded=true; o.comPulses=scenario!=0;
            PhysicsWorld w{bodybench::definition(rig,o.scenario,o)}; bodybench::reset(rig,w,o.scenario,o);
            std::array<bool,qp::Inequalities> previous{};
            for(unsigned n=0;n<(scenario?6480u:2640u);++n)
            {
                bodybench::step(w,o.scenario,o); qp::Result r;
                JULIA_CHECK(w.snapshot().stand.phase!=StandPhase::Lost);
                JULIA_CHECK(w.inspectStandQp(*p,r));
                std::array<bool,qp::Inequalities> active{};
                for(int i=0;i<p->m;++i) active[i]=r.multipliers[i]>1e-9;
                const bool pulse=scenario&&((n>=1070&&n<=1130)||(n>=4670&&n<=4730));
                const bool select=n%30==0||active!=previous||pulse; previous=active;
                if(!select) continue;
                const auto ref=solver->solve(*p);
                const auto opt=solver->solve(*p,128,qp::Implementation::Optimized7i);
                JULIA_CHECK(ref.code==qp::Code::Feasible&&opt.code==qp::Code::Feasible);
                JULIA_CHECK(r.x==ref.x);
                const double a=objective(*p,ref),b=objective(*p,opt),relative=std::abs(a-b)/std::max(1.,std::abs(a));
                double error=0; for(int i=0;i<p->n;++i) error=std::max(error,std::abs(ref.x[i]-opt.x[i]));
                JULIA_CHECK(relative<=1e-8&&error<=1e-5&&opt.equalityResidual<=1e-6&&opt.violation<=1e-7&&opt.projectedStationarity<=1e-5);
                binary(raw,std::int32_t{scenario}); binary(raw,std::uint32_t{n+1});
                binary(raw,std::int32_t{p->n}); binary(raw,std::int32_t{p->e}); binary(raw,std::int32_t{p->m});
                binary(raw,p->h);binary(raw,p->g);binary(raw,p->equality);binary(raw,p->target);binary(raw,p->inequality);binary(raw,p->lower);
                for(const auto* result:{&ref,&opt})
                { binary(raw,result->x);binary(raw,result->multipliers);binary(raw,result->iterations);binary(raw,result->active); }
                summary<<scenario<<','<<n+1<<','<<opt.active<<','<<opt.iterations<<','<<a<<','<<b<<','<<relative<<','<<error<<','<<opt.equalityResidual<<','<<opt.violation<<','<<opt.projectedStationarity<<'\n';
                ++count;
            }
        }
        std::ofstream{dir/"corpus.txt"}<<"PASS identical-input QPs="<<count<<"; selection every30th tick, last-update active membership transitions, pulse windows; reference trajectories\n";
    }
    void performance(const BodyRig& rig,const std::filesystem::path& dir,bool optimized,bool motion)
    {
        bodybench::Options o; o.scenario=motion?BodyCase::ComShift:BodyCase::Stand; o.bounded=true; o.comPulses=motion;
        o.boundedSolver=optimized?BoundedSolver::Optimized7i:BoundedSolver::Reference7h;
        PhysicsWorld w{bodybench::definition(rig,o.scenario,o)}; bodybench::reset(rig,w,o.scenario,o);
        std::ofstream raw{dir/"performance.csv"}; raw<<"tick,tick_us,controller_us,evaluations,physical_steps,physics_iterations,phase,com_x,com_y,com_z,base_residual,contact_residual\n"<<std::setprecision(17);
        std::ofstream calls{dir/"calls.csv"}; calls<<"tick,call,assembly_and_solve_us,preparation_us,input_us,linear_us,active_us,verification_us,physical_us,iterations,active\n"<<std::setprecision(17);
        for(unsigned n=0;n<(motion?6480u:2640u);++n)
        {
            const auto start=std::chrono::steady_clock::now(); bodybench::step(w,o.scenario,o);
            const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            const auto& s=w.snapshot();const auto& t=s.stand;
            JULIA_CHECK(t.solverSampleCount==4&&t.feasible&&t.phase!=StandPhase::Lost);
            JULIA_CHECK(t.baseResidual<1e-6&&t.contactResidual<1e-6&&t.requestedUtilization<=1.000001);
            if(n<240) continue;
            raw<<n+1<<','<<elapsed<<','<<t.controllerUs<<','<<t.solverSampleCount<<",16,"<<s.solverIterations<<','<<standPhaseName(t.phase)<<','<<s.totalComWorld.x<<','<<s.totalComWorld.y<<','<<s.totalComWorld.z<<','<<t.baseResidual<<','<<t.contactResidual<<'\n';
            for(unsigned i=0;i<4;++i)
            {
                const auto& p=t.boundedProfiles[i];
                calls<<n+1<<','<<i<<','<<t.solverTimesUs[i]<<','<<p.preparationUs<<','<<p.inputUs<<','<<p.linearUs<<','<<p.activeUs<<','<<p.verificationUs<<','<<p.physicalUs<<','<<t.solverIterations[i]<<','<<t.solverActive[i]<<'\n';
            }
        }
        std::ofstream meta{dir/"metadata.json"};
        meta<<"{\"optimized\":"<<(optimized?"true":"false")<<",\"motion\":"<<(motion?"true":"false")<<",\"warmup_ticks\":240,\"measured_ticks\":"<<(motion?6240:2400)<<",\"controller_bytes\":"<<w.standControllerBytes()<<",\"snapshot_bytes\":"<<sizeof(Snapshot)<<",\"model_bytes\":"<<w.modelBytes()<<",\"data_bytes\":"<<w.dataBytes()<<",\"arena_bytes\":"<<w.arenaHighWaterBytes();
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
        if(argc<3) throw std::invalid_argument("output mode [reference|optimized] [hold|motion]");
        const auto dir=std::filesystem::path{argv[1]};std::filesystem::create_directories(dir);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");const BodyRig rig{*asset};
        if(std::string{argv[2]}=="corpus") corpus(rig,dir);
        else
        {
            if(argc!=5) throw std::invalid_argument("performance requires solver and scenario");
            if(std::string{argv[2]}!="perf"||(std::string{argv[3]}!="reference"&&std::string{argv[3]}!="optimized")||
                (std::string{argv[4]}!="hold"&&std::string{argv[4]}!="motion")) throw std::invalid_argument("Expected perf reference|optimized hold|motion");
            performance(rig,dir,std::string{argv[3]}=="optimized",std::string{argv[4]}=="motion");
        }
        return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
