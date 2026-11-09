#include "julia/physics/ArmRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include "julia_test.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>

using namespace julia::physics;
namespace
{
    double difference(Coordinates a,Coordinates b)
    {
        double e=0; for(unsigned k=0;k<MaxCoordinates;++k) e=std::max(e,std::abs(a[k]-b[k])); return e;
    }
    double matrixError(const glm::dmat3& a,const glm::dmat3& b)
    {
        double e=0; for(int c=0;c<3;++c) for(int r=0;r<3;++r) e=std::max(e,std::abs(a[c][r]-b[c][r])); return e;
    }
    std::vector<glm::dmat4> cpuFrames(const Definition& def,Coordinates q)
    {
        std::vector<glm::dmat4> result; glm::dmat4 parent{1};
        for(const auto& b:def.links)
        {
            auto f=parent*b.parentFromJoint;
            for(const auto& h:b.hinges) f=glm::rotate(f,q[static_cast<std::size_t>(h.coordinate)],h.axis);
            result.push_back(f); parent=f;
        }
        return result;
    }
    void inertiaTests(const ArmRig& rig,std::ostream& out)
    {
        const Coordinates q{.6,.3,-.2,.9,.2,-.1};
        for(const double payload:{0.,2.})
        {
            const auto def=rig.description(1,1,payload); PhysicsWorld w{def}; const auto a=w.massAudit(q);
            JULIA_CHECK(std::abs(a.totalMass-(2.8+payload))<1e-10);
            const auto f=cpuFrames(def,q); glm::dvec3 com{0}; glm::dmat3 totalInertia{0};
            for(std::size_t i=0;i<def.links.size();++i)
            {
                const auto& b=def.links[i]; const glm::dmat3 s{b.jointFromSegment};
                glm::dmat3 diag{0}; for(int k=0;k<3;++k) diag[k][k]=b.inertia[k];
                JULIA_CHECK(std::abs(a.masses[i]-b.mass)<1e-10);
                JULIA_CHECK(glm::length(a.comInJoint[i]-glm::dvec3{b.jointFromSegment*glm::dvec4{b.comInSegment,1}})<1e-8);
                JULIA_CHECK(matrixError(a.inertiaInJoint[i],s*diag*glm::transpose(s))<1e-8);
                com+=b.mass*glm::dvec3{f[i]*b.jointFromSegment*glm::dvec4{b.comInSegment,1}};
            }
            com/=a.totalMass; JULIA_CHECK(glm::length(a.comWorld-com)<1e-8);
            std::array<std::vector<glm::dvec3>,MaxCoordinates> linear,angular;
            const double eps=1e-5;
            for(unsigned k=0;k<MaxCoordinates;++k)
            {
                auto plus=q,minus=q; plus[k]+=eps; minus[k]-=eps;
                const auto fp=cpuFrames(def,plus),fm=cpuFrames(def,minus);
                for(std::size_t i=0;i<def.links.size();++i)
                {
                    const auto comLocal=def.links[i].jointFromSegment*glm::dvec4{def.links[i].comInSegment,1};
                    linear[k].push_back(glm::dvec3{fp[i]*comLocal-fm[i]*comLocal}/(2*eps));
                    const auto spin=(glm::dmat3{fp[i]}-glm::dmat3{fm[i]})/(2*eps)*glm::transpose(glm::dmat3{f[i]});
                    angular[k].push_back({spin[1][2],spin[2][0],spin[0][1]});
                }
            }
            std::array<double,MaxCoordinates*MaxCoordinates> matrix{};
            for(std::size_t i=0;i<def.links.size();++i)
            {
                const auto& b=def.links[i]; const glm::dmat3 r{f[i]*b.jointFromSegment};
                glm::dmat3 diag{0}; for(int k=0;k<3;++k) diag[k][k]=b.inertia[k];
                const auto inertia=r*diag*glm::transpose(r);
                const auto delta=glm::dvec3{f[i]*b.jointFromSegment*glm::dvec4{b.comInSegment,1}}-com;
                totalInertia+=inertia+b.mass*(glm::dot(delta,delta)*glm::dmat3{1}-glm::outerProduct(delta,delta));
                for(unsigned j=0;j<MaxCoordinates;++j) for(unsigned k=0;k<MaxCoordinates;++k)
                    matrix[j*MaxCoordinates+k]+=b.mass*glm::dot(linear[j][i],linear[k][i])+glm::dot(angular[j][i],inertia*angular[k][i]);
            }
            JULIA_CHECK(matrixError(totalInertia,a.inertiaAtComWorld)<1e-8);
            double maxError=0;
            for(std::size_t i=0;i<matrix.size();++i) maxError=std::max(maxError,std::abs(matrix[i]-a.massMatrix[i]));
            JULIA_CHECK(maxError<1e-7);
            // Cholesky checks all directions of generalized kinetic energy.
            std::array<double,MaxCoordinates*MaxCoordinates> l{};
            for(unsigned i=0;i<MaxCoordinates;++i) for(unsigned j=0;j<=i;++j)
            {
                double x=a.massMatrix[i*MaxCoordinates+j];
                for(unsigned k=0;k<j;++k) x-=l[i*MaxCoordinates+k]*l[j*MaxCoordinates+k];
                if(i==j) { JULIA_CHECK(x>1e-7); l[i*MaxCoordinates+j]=std::sqrt(x); }
                else l[i*MaxCoordinates+j]=x/l[j*MaxCoordinates+j];
            }
            out<<"inertia payload="<<payload<<" mass="<<a.totalMass<<" com="<<com.x<<','<<com.y<<','<<com.z
                <<" mass_matrix_max_error="<<maxError<<" tensor_diag="<<totalInertia[0][0]<<','<<totalInertia[1][1]<<','<<totalInertia[2][2]<<'\n';
            JULIA_CHECK(w.snapshot().tick==0&&w.snapshot().q==Coordinates{});
        }
        PhysicsWorld two{ArmRig{rig.binding().asset(),rig.side(),ArmChain::Two}.description()};
        JULIA_CHECK(std::abs(two.massAudit().totalMass-2.8)<1e-10);
    }
    void commandTests(const ArmRig& rig,std::ostream& out)
    {
        auto def=rig.description(); def.gravity={0,0,0}; PhysicsWorld w{def};
        const Coordinates initial{.2,.1,.1,.3,.1,.05}; w.reset(initial);
        const auto before=w.snapshot();
        for(unsigned k=0;k<MaxCoordinates;++k)
        {
            auto q=initial; q[k]=std::numeric_limits<double>::infinity();
            JULIA_CHECK(!w.command(CommandKind::Target,q));
            q[k]=std::numeric_limits<double>::quiet_NaN(); JULIA_CHECK(!w.command(CommandKind::Torque,q));
            bool rejected=false; try { w.reset(q); } catch(const std::invalid_argument&) { rejected=true; } JULIA_CHECK(rejected);
            rejected=false; try { w.forwardKinematics(q); } catch(const std::invalid_argument&) { rejected=true; } JULIA_CHECK(rejected);
        }
        JULIA_CHECK(w.snapshot().q==before.q&&w.snapshot().epoch==before.epoch);
        JULIA_CHECK(w.command(CommandKind::Torque,{100,-100,100,-100,100,-100}));
        JULIA_CHECK(w.snapshot().q==initial); w.step();
        for(unsigned k=0;k<MaxCoordinates;++k)
        {
            JULIA_CHECK(w.snapshot().saturatedThisTick[k]);
            JULIA_CHECK(std::abs(std::abs(w.snapshot().appliedTorque[k])-def.drives[k].maxTorque)<1e-9);
        }
        w.reset(initial); for(unsigned i=0;i<120;++i) w.step();
        JULIA_CHECK(difference(w.snapshot().q,initial)<=1e-12);
        // Soft-limit overshoot is passed verbatim to the visual adapter.
        auto q=initial; q[4]=def.links[2].hinges[0].upper+.01;
        const auto raw=rig.visualFrames(rig.visualPose(q)); auto clipped=q; clipped[4]=def.links[2].hinges[0].upper;
        JULIA_CHECK(matrixError(glm::dmat3{raw[2]},glm::dmat3{rig.visualFrames(rig.visualPose(clipped))[2]})>.005);
        // Failure from an excessive six-coordinate acceleration must latch.
        auto explosive=def; for(auto& b:explosive.links) { b.mass=.001; b.inertia={1e-8,1e-8,1e-8}; b.comInSegment={0,0,0}; }
        PhysicsWorld bad{explosive}; JULIA_CHECK(bad.command(CommandKind::Torque,{100,100,100,100,100,100}));
        bool failed=false; try { bad.step(); } catch(const std::runtime_error& e) { failed=true; out<<"expected failure "<<e.what()<<'\n'; }
        JULIA_CHECK(failed); JULIA_CHECK(!bad.command(CommandKind::Target)); bad.reset(); bad.step();
        JULIA_CHECK(bad.snapshot().warnings==0);
    }
    void individualAndHold(const ArmRig& rig,std::ostream& out)
    {
        auto def=rig.description(); def.gravity={0,0,0}; PhysicsWorld w{def};
        const Coordinates initial{.3,.2,.1,.6,.1,.05};
        for(unsigned k=0;k<MaxCoordinates;++k)
        {
            w.reset(initial); auto target=initial; target[k]+=.12;
            JULIA_CHECK(w.command(CommandKind::Target,target)); JULIA_CHECK(w.snapshot().q==initial);
            for(unsigned n=0;n<480;++n) w.step();
            const auto e=difference(w.snapshot().q,target); JULIA_CHECK(e<.003);
            JULIA_CHECK(std::abs(w.snapshot().q[k]-initial[k])>.08);
            out<<"individual coordinate="<<k<<" max_error="<<e<<'\n';
        }
        PhysicsWorld gravity{rig.description()};
        for(const Coordinates target:std::array<Coordinates,3>{{{.7,.25,.2,.8,.25,.15},{.25,.1,-.25,.5,-.2,-.15},{1.05,.65,.4,1.5,.4,.22}}})
        {
            gravity.reset(initial); JULIA_CHECK(gravity.command(CommandKind::Target,target));
            for(unsigned n=0;n<720;++n) gravity.step();
            const auto e=difference(gravity.snapshot().q,target); JULIA_CHECK(e<.16);
            out<<"hold max_error="<<e<<" q="; for(auto x:gravity.snapshot().q) out<<x<<','; out<<'\n';
        }
    }
    using History=std::vector<Snapshot>;
    History history(PhysicsWorld& w,BenchCase c)
    {
        resetBench(w,c); History h; h.reserve(720);
        for(unsigned i=0;i<720;++i) { stepBench(w,c); h.push_back(w.snapshot()); } return h;
    }
    void equal(const Snapshot& a,const Snapshot& b)
    {
        JULIA_CHECK(difference(a.q,b.q)<=1e-12&&difference(a.v,b.v)<=1e-12);
        JULIA_CHECK(a.tick==b.tick&&a.worldSteps==b.worldSteps&&a.calculatedTorque==b.calculatedTorque&&a.appliedTorque==b.appliedTorque);
        JULIA_CHECK(a.saturatedThisTick==b.saturatedThisTick&&a.limitViolation==b.limitViolation&&a.peakLimitViolationThisTick==b.peakLimitViolationThisTick);
        JULIA_CHECK(a.contacts.size()==b.contacts.size()&&a.peakNormalForceThisTick==b.peakNormalForceThisTick&&a.maxPenetrationThisTick==b.maxPenetrationThisTick);
    }
    void cases(const ArmRig& rig,const std::filesystem::path& dir,std::ostream& out)
    {
        std::array<History,11> runs;
        for(const auto c:{BenchCase::Passive,BenchCase::Hold,BenchCase::Heavy,BenchCase::Inertia,BenchCase::Weak,BenchCase::Impulse,
            BenchCase::Contact,BenchCase::ZeroGravity,BenchCase::Spatial,BenchCase::ImpulseX,BenchCase::ImpulseY})
        {
            PhysicsWorld w{benchDefinition(rig,c)}; auto& run=runs[static_cast<std::size_t>(c)]; run=history(w,c);
            const auto replay=history(w,c); for(std::size_t i=0;i<run.size();++i) equal(run[i],replay[i]);
            double force=0,penetration=0,violation=0; unsigned saturated=0;
            for(const auto& s:run)
            {
                bool sat=false;
                for(unsigned k=0;k<MaxCoordinates;++k)
                {
                    JULIA_CHECK(std::isfinite(s.q[k])&&std::isfinite(s.v[k])&&std::abs(s.appliedTorque[k])<=w.definition().drives[k].maxTorque+1e-9);
                    violation=std::max(violation,s.peakLimitViolationThisTick[k]); sat=sat||s.saturatedThisTick[k];
                }
                if(sat) ++saturated;
                JULIA_CHECK(s.warnings==0&&s.worldSteps==s.tick*Substeps&&std::abs(s.time-s.tick*TickSeconds)<1e-9);
                force=std::max(force,s.peakNormalForceThisTick); penetration=std::max(penetration,s.maxPenetrationThisTick);
            }
            const auto& last=run.back(); const double error=difference(last.q,last.target);
            out<<benchCaseName(c)<<" target_error="<<error<<" saturated="<<saturated<<" force="<<force<<" penetration="<<penetration<<" violation="<<violation<<" final_q=";
            for(auto x:last.q) out<<x<<','; out<<'\n';
            std::ofstream csv{dir/(benchCaseName(c)+".csv")}; csvHeader(csv); for(const auto& s:run) csvRow(csv,publish(rig,s));
            if(c==BenchCase::Contact) { JULIA_CHECK(force>.1&&penetration<.006&&error>.1); }
            if(c==BenchCase::Hold) JULIA_CHECK(error<.16);
            if(c==BenchCase::Weak) { JULIA_CHECK(error>.25&&saturated>600); }
            if(c==BenchCase::Passive) JULIA_CHECK(difference(run[119].q,run.front().target)>.1);
            if(c==BenchCase::ZeroGravity) JULIA_CHECK(difference(last.q,last.target)<=1e-12);
            // Independent instance remains untouched by reset, commands, force and stepping of w.
            PhysicsWorld other{w.definition()}; const auto untouched=other.snapshot();
            w.applyWrench(2,{20,10,-10}); w.command(CommandKind::Torque,{1,1,1,1,1,1});
            const auto epochBefore=w.snapshot().epoch; w.reset();
            const auto& reset=w.snapshot();
            JULIA_CHECK(reset.epoch==epochBefore+1&&reset.tick==0&&reset.worldSteps==0&&reset.time==0&&reset.q==Coordinates{}&&reset.v==Coordinates{}&&reset.target==Coordinates{});
            JULIA_CHECK(reset.mode==DriveMode::Disabled&&reset.contacts.empty()&&reset.externalForce==glm::dvec3{0}&&reset.externalTorque==glm::dvec3{0});
            JULIA_CHECK(reset.calculatedTorque==Coordinates{}&&reset.appliedTorque==Coordinates{});
            JULIA_CHECK(std::none_of(reset.saturatedThisTick.begin(),reset.saturatedThisTick.end(),[](bool x) { return x; }));
            JULIA_CHECK(std::none_of(reset.saturated.begin(),reset.saturated.end(),[](bool x) { return x; }));
            JULIA_CHECK(reset.warnings==0&&reset.solverIterations==0&&reset.peakContactsThisTick==0&&reset.peakNormalForceThisTick==0&&reset.peakLimitViolationThisTick==Coordinates{});
            equal(other.snapshot(),untouched); w.step(); JULIA_CHECK(w.snapshot().externalForce==glm::dvec3{0});
        }
        const auto& hold=runs[1]; JULIA_CHECK(difference(hold.back().q,runs[2].back().q)>.03);
        double inertia=0; for(unsigned i=0;i<120;++i) inertia=std::max(inertia,difference(hold[i].q,runs[3][i].q)); JULIA_CHECK(inertia>.01);
        out<<"heavy_delta="<<difference(hold.back().q,runs[2].back().q)<<" inertia_transient="<<inertia<<'\n';
        PhysicsWorld payload{rig.description(1,1,2)}; const auto load=history(payload,BenchCase::Hold); double loadDelta=0;
        for(unsigned i=0;i<120;++i) loadDelta=std::max(loadDelta,difference(hold[i].q,load[i].q));
        JULIA_CHECK(loadDelta>.01); out<<"payload_only_transient="<<loadDelta<<'\n';
        // Scale segment masses and their physical inertia together, without payload.
        PhysicsWorld massive{rig.description(2,1,0)}; const auto massRun=history(massive,BenchCase::Hold);
        const auto massDelta=difference(hold.back().q,massRun.back().q);
        JULIA_CHECK(massDelta>.03&&std::abs(massive.massAudit().totalMass-5.6)<1e-10);
        PhysicsWorld heavy{benchDefinition(rig,BenchCase::Heavy)}; JULIA_CHECK(std::abs(heavy.massAudit().totalMass-7.6)<1e-10);
        out<<"mass_only_final_delta="<<massDelta<<'\n';
        for(const auto c:{BenchCase::Impulse,BenchCase::ImpulseX,BenchCase::ImpulseY})
        {
            const auto& run=runs[static_cast<std::size_t>(c)]; double peak=0;
            for(unsigned i=240;i<360;++i) peak=std::max(peak,difference(hold[i].q,run[i].q));
            const auto recovery=difference(hold.back().q,run.back().q); JULIA_CHECK(peak>.005&&recovery<.02);
            out<<benchCaseName(c)<<" delta="<<peak<<" recovery="<<recovery<<'\n';
        }
        for(const auto c:{BenchCase::Impulse,BenchCase::Contact,BenchCase::Spatial}) for(unsigned fps:{30u,60u,144u})
        {
            PhysicsWorld w{benchDefinition(rig,c)}; resetBench(w,c); julia::runtime::FixedStepScheduler scheduler;
            for(unsigned frame=0;frame<fps*6;++frame)
            {
                scheduler.advance(1./fps,[&] { stepBench(w,c); equal(w.snapshot(),runs[static_cast<std::size_t>(c)].at(static_cast<std::size_t>(w.snapshot().tick-1))); });
                const auto before=w.snapshot(); const auto a=publish(rig,before),b=publish(rig,before);
                JULIA_CHECK(a.actualPose.palette==b.actualPose.palette&&w.snapshot().q==before.q&&w.snapshot().tick==before.tick);
            }
            JULIA_CHECK(w.snapshot().tick==720&&w.snapshot().worldSteps==5760&&scheduler.diagnostics().overloadFrames==0);
            out<<"replay "<<benchCaseName(c)<<" fps="<<fps<<" PASS\n";
        }
    }
    void performance(const ArmRig& rig,const std::filesystem::path& dir,std::ostream& out)
    {
        PhysicsWorld w{benchDefinition(rig,BenchCase::Contact)}; resetBench(w,BenchCase::Contact);
        std::array<std::vector<double>,5> samples;
        std::ofstream csv{dir/"performance.csv"}; csv<<std::setprecision(17)<<"tick,whole_tick_us,visual_publish_us,control_us,step_us,read_us\n";
        for(unsigned i=0;i<10000;++i)
        {
            const auto start=std::chrono::steady_clock::now(); w.step();
            const double tick=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            const auto view=publish(rig,w.snapshot());
            const std::array t{tick,view.mappingUs,view.physics.timing.controlUs,view.physics.timing.stepUs,view.physics.timing.readUs};
            csv<<i+1; for(const auto x:t) csv<<','<<x; csv<<'\n';
            for(unsigned k=0;k<t.size();++k) samples[k].push_back(t[k]);
        }
        for(unsigned k=0;k<samples.size();++k)
        {
            auto& t=samples[k]; const auto mean=std::accumulate(t.begin(),t.end(),0.)/t.size(); std::sort(t.begin(),t.end());
            out<<"perf "<<std::array{"whole_tick","visual_publish","control8","step8","read8"}[k]<<" mean_us="<<mean<<" p95_us="<<t[t.size()*95/100]<<" max_us="<<t.back()<<'\n';
        }
        const auto& s=w.snapshot();
        out<<"memory model="<<w.modelBytes()<<" data_each="<<w.dataBytes()<<" live_scratch="<<w.dataBytes()*2<<" arena="<<w.arenaHighWaterBytes()
            <<" snapshot_struct="<<sizeof(Snapshot)<<" snapshot_vectors="<<(s.worldFromJoint.size()*sizeof(glm::dmat4)+s.comWorld.size()*sizeof(glm::dvec3)+s.contacts.size()*sizeof(Contact))
            <<" pose_pair_arrays="<<2*(rig.binding().asset().nodes().size()*2+rig.binding().asset().bones().size())*sizeof(glm::mat4)<<'\n';
    }
}
int main(int argc,char** argv)
{
    try
    {
        const std::filesystem::path dir=argc>1?argv[1]:"build/diagnostics/stage7b/cpu"; std::filesystem::create_directories(dir);
        std::ofstream out{dir/"physics-full.txt"}; out<<std::setprecision(17);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        for(const auto side:{ArmSide::Left,ArmSide::Right})
        {
            const std::string name=side==ArmSide::Left?"left":"right"; out<<"SIDE "<<name<<'\n';
            const auto subdir=dir/name; std::filesystem::create_directories(subdir); const ArmRig rig{*asset,side,ArmChain::Full};
            PhysicsWorld w{rig.description()}; const auto parity=verifyFramesBeforeDynamics(rig,w);
            out<<"FK configurations="<<parity.configurations<<" position="<<parity.position<<" rotation_element="<<parity.rotationElement<<" min_shoulder_axis_det="<<parity.minShoulderAxisDeterminant<<'\n';
            std::ofstream calibration{subdir/"calibration.txt"}; writeCalibration(calibration,rig,w);
            inertiaTests(rig,out); commandTests(rig,out); individualAndHold(rig,out); cases(rig,subdir,out); performance(rig,subdir,out); out.flush();
        }
        out<<"failures="<<JULIA_TEST_RESULT()<<'\n'; std::cout<<"full physics failures="<<JULIA_TEST_RESULT()<<'\n'; return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"FULL PHYSICS FAILURE: "<<e.what()<<'\n'; return 1; }
}
