#include "julia/physics/ArmRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include "julia_test.hpp"
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

using namespace julia::physics;
namespace
{
    double difference(Coordinates a,Coordinates b) { return std::max(std::abs(a[0]-b[0]),std::abs(a[1]-b[1])); }
    double matrixError(const glm::dmat4& a,const glm::dmat4& b)
    {
        double e=0; for(int c=0;c<4;++c) for(int r=0;r<4;++r) e=std::max(e,std::abs(a[c][r]-b[c][r])); return e;
    }
    std::size_t privateBytes()
    {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS_EX mem{};
        if(GetProcessMemoryInfo(GetCurrentProcess(),reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&mem),sizeof(mem))) return mem.PrivateUsage;
#endif
        return 0;
    }
    Definition pendulum()
    {
        Definition def; Link b; b.name="Independent one metre uniform rod";
        b.mass=1; b.radius=.01; b.comInSegment={0,-.5,0};
        b.inertia={1./12.,.001,1./12.}; b.capsuleEnd={0,-1,0};
        b.hinges={{0,{0,0,1},-1,1,0}}; def.links.push_back(b); return def;
    }
    void analyticTest(std::ostream& out)
    {
        PhysicsWorld world{pendulum()}; world.reset({.01,0});
        const double moment=1./12.+.25, omega=std::sqrt(9.81*.5/moment), period=2*glm::pi<double>()/omega;
        const double velocityExpected=-9.81*.5/moment*std::sin(.01)*TickSeconds;
        world.step();
        const double initialError=std::abs(world.snapshot().v[0]-velocityExpected);
        JULIA_CHECK(initialError<=2e-4);
        double maxQError=0; std::vector<double> crossings;
        double previousQ=.01, previousTime=0;
        world.reset({.01,0});
        for(unsigned i=0;i<600;++i)
        {
            world.step(); const auto& s=world.snapshot();
            if(s.time<=.5) maxQError=std::max(maxQError,std::abs(s.q[0]-.01*std::cos(omega*s.time)));
            if(previousQ>0&&s.q[0]<=0) crossings.push_back(previousTime+(s.time-previousTime)*previousQ/(previousQ-s.q[0]));
            previousQ=s.q[0]; previousTime=s.time;
        }
        JULIA_CHECK(crossings.size()>=2);
        const double measured=crossings.size()>=2?crossings[1]-crossings[0]:0;
        JULIA_CHECK(std::abs(measured-period)<.004); JULIA_CHECK(maxQError<3e-5);
        out<<"pendulum expected_period_s="<<period<<" measured="<<measured<<" initial_v_error="<<initialError<<" max_q_error="<<maxQError<<'\n';
    }
    void frameTests(const ArmRig& rig,std::ostream& out)
    {
        PhysicsWorld world{rig.description()}; const auto parity=verifyFramesBeforeDynamics(rig,world);
        out<<"FK max_position_m="<<parity.position<<" max_rotation_element="<<parity.rotationElement<<'\n';
        const auto zero=world.forwardKinematics({0,0});
        const auto upper=glm::length(glm::dvec3{zero[1][3]-zero[0][3]}), fore=glm::length(glm::dvec3{zero[2][3]-zero[1][3]});
        JULIA_CHECK(std::abs(upper-.264514)<30e-6); JULIA_CHECK(std::abs(fore-.245795)<30e-6);
        JULIA_CHECK(std::abs(glm::length(rig.binding().profile().bodyFromResource[0])-.4)<1e-6);
        out<<"lengths_m="<<upper<<','<<fore<<'\n';
        for(const Coordinates q:std::array<Coordinates,4>{{{0,0},{.3,.7},{1.1,1.7},{.2,1.78}}})
        {
            const auto pose=rig.visualPose(q); const auto physical=world.forwardKinematics(q);
            const auto visual=rig.visualFrames(pose);
            for(std::size_t i=0;i<3;++i) JULIA_CHECK(matrixError(visual[i],physical[i])<3e-5);
            if(q[1]<=1.7)
            {
                julia::embodiment::DirectBodyControl control{rig.binding()}; control.setEnabled(true);
                const auto a=control.setCoordinate(julia::embodiment::JointId::LeftShoulder,0,static_cast<float>(q[0]));
                const auto b=control.setCoordinate(julia::embodiment::JointId::LeftElbow,0,static_cast<float>(q[1]));
                JULIA_CHECK(a.status==julia::embodiment::CommandStatus::Applied&&b.status==julia::embodiment::CommandStatus::Applied);
                auto locals=pose.locals; control.apply(locals); JULIA_CHECK(locals==pose.locals);
            }
        }
        const auto before=world.snapshot(); world.forwardKinematics({1,1});
        JULIA_CHECK(before.q==world.snapshot().q&&world.snapshot().worldSteps==0);
        for(std::size_t i=0;i<3;++i)
        {
            const auto& b=world.definition().links[i];
            const auto expected=glm::dvec3{world.snapshot().worldFromJoint[i]*b.jointFromSegment*glm::dvec4{b.comInSegment,1}};
            JULIA_CHECK(glm::length(expected-world.snapshot().comWorld[i])<1e-7);
        }
        auto bad=rig.description(); bad.links[0].inertia.x=-1;
        bool rejected=false; try { PhysicsWorld invalid{bad}; } catch(const std::invalid_argument&) { rejected=true; } JULIA_CHECK(rejected);
        bad=rig.description(); bad.links[0].parentFromJoint[0][0]=.4;
        rejected=false; try { PhysicsWorld invalid{bad}; } catch(const std::invalid_argument&) { rejected=true; } JULIA_CHECK(rejected);
        bad=rig.description(); bad.links[0].inertia={1,.001,.001};
        rejected=false; try { PhysicsWorld invalid{bad}; } catch(const std::invalid_argument&) { rejected=true; } JULIA_CHECK(rejected);
    }
    void commands(const ArmRig& rig,std::ostream& out)
    {
        auto def=rig.description(); def.gravity={0,0,0}; PhysicsWorld w{def}; w.reset({.2,.3});
        const auto before=w.snapshot(); std::string reason;
        for(const auto command:{CommandKind::Reach,CommandKind::Attach,CommandKind::MoveRoot,static_cast<CommandKind>(99)})
        { JULIA_CHECK(!w.command(command,{},&reason)); JULIA_CHECK(!reason.empty()); }
        JULIA_CHECK(!w.command(CommandKind::Target,{2,0},&reason));
        JULIA_CHECK(!w.command(CommandKind::Torque,{std::numeric_limits<double>::quiet_NaN(),0},&reason));
        JULIA_CHECK(w.snapshot().q==before.q&&w.snapshot().target==before.target);
        JULIA_CHECK(PhysicsWorld::capabilities().fixedBase&&!PhysicsWorld::capabilities().freeRoot&&!PhysicsWorld::capabilities().ik);
        JULIA_CHECK(w.command(CommandKind::Torque,{100,-100}));
        JULIA_CHECK(w.snapshot().q==before.q);
        w.step(); const auto s=w.snapshot();
        JULIA_CHECK(s.saturatedThisTick[0]&&s.saturatedThisTick[1]);
        JULIA_CHECK(std::abs(s.appliedTorque[0]-12)<1e-9&&std::abs(s.appliedTorque[1]+8)<1e-9);
        JULIA_CHECK(s.calculatedTorque[0]==100&&s.calculatedTorque[1]==-100);
        w.reset({.2,.3}); JULIA_CHECK(w.snapshot().mode==DriveMode::Disabled&&w.snapshot().v==Coordinates{});
        for(unsigned i=0;i<120;++i) w.step();
        JULIA_CHECK(difference(w.snapshot().q,{.2,.3})<1e-12);
        out<<"effort limits=12,8 Nm, saturation observed; zero-g drift="<<difference(w.snapshot().q,{.2,.3})<<'\n';
        // An intentionally excessive acceleration must fail observably and latch.
        auto explosive=pendulum(); auto& b=explosive.links[0];
        b.mass=.001; b.comInSegment={0,0,0}; b.inertia={1e-8,1e-8,1e-8};
        explosive.drives[0].maxTorque=100; explosive.gravity={0,0,0};
        PhysicsWorld unstable{explosive}; JULIA_CHECK(unstable.command(CommandKind::Torque,{100,0}));
        bool failed=false; try { unstable.step(); } catch(const std::runtime_error& e) { failed=true; out<<"expected diagnostic failure: "<<e.what()<<'\n'; }
        JULIA_CHECK(failed); JULIA_CHECK(!unstable.command(CommandKind::Target,{},&reason));
        unstable.reset(); unstable.step(); JULIA_CHECK(unstable.snapshot().warnings==0);
    }
    std::vector<Snapshot> history(PhysicsWorld& world,BenchCase scenario)
    {
        resetBench(world,scenario); std::vector<Snapshot> samples; samples.reserve(720);
        for(unsigned i=0;i<720;++i) { stepBench(world,scenario); samples.push_back(world.snapshot()); }
        return samples;
    }
    void replayEqual(const Snapshot& a,const Snapshot& b)
    {
        JULIA_CHECK(a.tick==b.tick&&a.worldSteps==b.worldSteps&&a.mode==b.mode);
        JULIA_CHECK(difference(a.q,b.q)<=1e-12&&difference(a.v,b.v)<=1e-12);
        JULIA_CHECK(a.calculatedTorque==b.calculatedTorque&&a.appliedTorque==b.appliedTorque);
        JULIA_CHECK(a.saturatedThisTick==b.saturatedThisTick&&a.contacts.size()==b.contacts.size());
        JULIA_CHECK(a.peakNormalForceThisTick==b.peakNormalForceThisTick&&a.limitViolation==b.limitViolation);
        for(std::size_t i=0;i<a.worldFromJoint.size();++i) JULIA_CHECK(matrixError(a.worldFromJoint[i],b.worldFromJoint[i])<1e-6);
    }
    void dynamicCases(const ArmRig& rig,const std::filesystem::path& dir,std::ostream& out)
    {
        std::array<std::vector<Snapshot>,7> results;
        for(auto scenario:{BenchCase::Passive,BenchCase::Hold,BenchCase::Heavy,BenchCase::Inertia,BenchCase::Weak,BenchCase::Impulse,BenchCase::Contact})
        {
            PhysicsWorld world{benchDefinition(rig,scenario)}; verifyFramesBeforeDynamics(rig,world);
            auto& run=results[static_cast<std::size_t>(scenario)]; run=history(world,scenario);
            const auto repeat=history(world,scenario);
            for(std::size_t i=0;i<run.size();++i) replayEqual(run[i],repeat[i]);
            const auto& last=run.back();
            double maxForce=0,maxPenetration=0,maxViolation=0; unsigned saturation=0;
            for(const auto& s:run)
            {
                maxForce=std::max(maxForce,s.peakNormalForceThisTick); maxPenetration=std::max(maxPenetration,s.maxPenetrationThisTick);
                maxViolation=std::max({maxViolation,s.limitViolation[0],s.limitViolation[1]});
                if(s.saturatedThisTick[0]||s.saturatedThisTick[1]) ++saturation;
                JULIA_CHECK(s.worldSteps==s.tick*Substeps&&std::abs(s.time-static_cast<double>(s.tick)*TickSeconds)<1e-9);
            }
            out<<benchCaseName(scenario)<<" final_q="<<last.q[0]<<','<<last.q[1]<<" target_error="<<difference(last.q,last.target)
                <<" saturated_ticks="<<saturation<<" force_N="<<maxForce<<" penetration_m="<<maxPenetration<<" limit_violation_rad="<<maxViolation<<'\n';
            std::ofstream csv{dir/(benchCaseName(scenario)+".csv")}; csvHeader(csv);
            for(const auto& s:run) csvRow(csv,publish(rig,s));
            if(scenario==BenchCase::Contact)
            {
                JULIA_CHECK(maxForce>.1&&maxPenetration<.005); JULIA_CHECK(difference(last.q,last.target)>.1);
                bool measured=false; for(const auto& s:run) for(const auto& c:s.contacts) if(c.force&&c.force->x>.1) measured=true;
                JULIA_CHECK(measured);
            }
            if(scenario==BenchCase::Weak) { JULIA_CHECK(difference(last.q,last.target)>.25); JULIA_CHECK(saturation>600); }
            if(scenario==BenchCase::Hold) JULIA_CHECK(difference(last.q,last.target)<.12);
            if(scenario==BenchCase::Passive) { JULIA_CHECK(difference(run[119].q,{.2,.3})>.1); JULIA_CHECK(maxViolation>0); }
            // Reset after load/contact/impulse must erase all dynamic/controller history.
            world.reset({.2,.3}); const auto& reset=world.snapshot();
            JULIA_CHECK(reset.tick==0&&reset.worldSteps==0&&reset.v==Coordinates{}&&reset.mode==DriveMode::Disabled);
            JULIA_CHECK(reset.contacts.empty()&&reset.peakNormalForceThisTick==0&&reset.externalForce==glm::dvec3{0});
            JULIA_CHECK(!reset.saturatedThisTick[0]&&!reset.saturatedThisTick[1]);
        }
        const auto& hold=results[1]; const auto& heavy=results[2]; const auto& inertia=results[3]; const auto& pulse=results[5];
        double inertiaDifference=0,pulseDifference=0;
        for(std::size_t i=0;i<120;++i) inertiaDifference=std::max(inertiaDifference,difference(hold[i].q,inertia[i].q));
        for(std::size_t i=240;i<360;++i) pulseDifference=std::max(pulseDifference,difference(hold[i].q,pulse[i].q));
        JULIA_CHECK(difference(hold.back().q,heavy.back().q)>.03); JULIA_CHECK(inertiaDifference>.02);
        JULIA_CHECK(pulseDifference>.02); JULIA_CHECK(difference(hold.back().q,pulse.back().q)<.015);
        out<<"heavy_delta_rad="<<difference(hold.back().q,heavy.back().q)<<" inertia_transient_delta="<<inertiaDifference
            <<" pulse_peak_delta="<<pulseDifference<<" recovery_delta="<<difference(hold.back().q,pulse.back().q)<<'\n';
        for(auto scenario:{BenchCase::Impulse,BenchCase::Contact})
        {
            const auto& reference=results[static_cast<std::size_t>(scenario)];
            for(const unsigned fps:{30u,60u,144u})
            {
                PhysicsWorld world{benchDefinition(rig,scenario)}; resetBench(world,scenario);
                julia::runtime::FixedStepScheduler scheduler;
                for(unsigned frame=0;frame<fps*6;++frame)
                {
                    scheduler.advance(1./fps,[&] { stepBench(world,scenario); replayEqual(reference.at(static_cast<std::size_t>(world.snapshot().tick-1)),world.snapshot()); });
                    const auto before=world.snapshot();
                    const auto view=publish(rig,before), again=publish(rig,before);
                    JULIA_CHECK(view.actualPose.palette==again.actualPose.palette&&world.snapshot().q==before.q&&world.snapshot().tick==before.tick);
                }
                JULIA_CHECK(world.snapshot().tick==720&&world.snapshot().worldSteps==720*Substeps);
                JULIA_CHECK(scheduler.diagnostics().overloadFrames==0);
                out<<"replay "<<benchCaseName(scenario)<<" fps="<<fps<<" ticks="<<world.snapshot().tick<<" PASS\n";
            }
        }
    }
    void performance(const ArmRig& rig,std::ostream& out)
    {
        PhysicsWorld world{benchDefinition(rig,BenchCase::Contact)}; resetBench(world,BenchCase::Contact);
        const auto memoryBefore=privateBytes();
        std::array<std::vector<double>,4> times;
        for(auto& sample:times) sample.reserve(10000);
        for(unsigned i=0;i<10000;++i)
        {
            world.step(); const auto view=publish(rig,world.snapshot());
            times[0].push_back(view.physics.timing.controlUs); times[1].push_back(view.physics.timing.stepUs);
            times[2].push_back(view.physics.timing.readUs); times[3].push_back(view.mappingUs);
        }
        for(std::size_t k=0;k<4;++k)
        {
            auto& t=times[k]; double sum=0; for(const auto x:t) sum+=x;
            std::sort(t.begin(),t.end());
            out<<"perf "<<std::array<const char*,4>{"control_8x","substeps_8x","read_forward_8x","publish_visual"}[k]
                <<" mean_us="<<sum/static_cast<double>(t.size())<<" p95_us="<<t[t.size()*95/100]<<" max_us="<<t.back()<<'\n';
        }
        out<<"memory model="<<world.modelBytes()<<" data_each="<<world.dataBytes()<<" live_plus_scratch="<<2*world.dataBytes()
            <<" arena_high_water="<<world.arenaHighWaterBytes()<<" snapshot_struct="<<sizeof(Snapshot)
            <<" process_private_before="<<memoryBefore<<" after="<<privateBytes()<<'\n';
    }
}
int main(int argc,char** argv)
{
    try
    {
        const std::filesystem::path dir=argc>1?argv[1]:"build/diagnostics/stage7a/cpu";
        std::filesystem::create_directories(dir); std::ofstream out{dir/"physics.txt"}; out<<std::setprecision(17);
        analyticTest(out);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const ArmRig rig{*asset}; frameTests(rig,out); commands(rig,out); dynamicCases(rig,dir,out); performance(rig,out);
        out<<"failures="<<JULIA_TEST_RESULT()<<'\n';
        std::cout<<"physics failures="<<JULIA_TEST_RESULT()<<" report="<<(dir/"physics.txt").string()<<'\n'; return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"PHYSICS TEST FAILURE: "<<e.what()<<'\n'; return 1; }
}
