#include "julia/physics/BodyRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "julia_test.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>

using namespace julia::physics;
namespace
{
    double error(const Coordinates& a,const Coordinates& b,unsigned n)
    { double e=0; for(unsigned k=0;k<n;++k) e=std::max(e,std::abs(a[k]-b[k])); return e; }
    std::string deterministic(Snapshot s,unsigned substeps)
    { s.epoch=0; std::ostringstream o; driveCsvRow(o,s,substeps); return o.str(); }
    void servo(PhysicsWorld& w,Coordinates goal,bool gravity)
    {
        JULIA_CHECK(w.configureServo({.6,2,gravity}));
        JULIA_CHECK(w.command(CommandKind::SmoothTarget,goal));
    }
    Definition fixture()
    {
        Definition d; Link b; b.name="Independent 2kg horizontal rod";
        b.mass=2; b.comInSegment={.5,0,0}; b.inertia={.002,1./6.,1./6.};
        b.capsuleEnd={1,0,0}; b.hinges={{0,{0,0,1},-2,2,0}};
        d.links={b}; d.drives[0]={80,8,12}; return d;
    }
    void referenceTests(std::ostream& out)
    {
        JointReference r; r.reset(.2); const ServoSettings settings{.6,2,false}; r.target(.9,settings);
        double maxV=0,maxA=0,continuity=0;
        const double h=1./1920;
        for(unsigned n=0;n<8000;++n)
        {
            if(n==311||n==725||n==1131)
            {
                const double q=r.q(),v=r.v(); r.target(n==725?1.1:-.1,settings);
                continuity=std::max(continuity,std::max(std::abs(q-r.q()),std::abs(v-r.v())));
            }
            const double v=r.v(); r.advance(h);
            maxV=std::max(maxV,std::abs(r.v())); maxA=std::max(maxA,std::abs(r.v()-v)/h);
        }
        JULIA_CHECK(maxV<=.6+1e-10&&maxA<=2+1e-10&&continuity==0&&r.reached()&&std::abs(r.q()+.1)<1e-10&&r.v()==0);
        // Repeated goals and a zero-distance target do not restart or stall.
        r.reset(0); r.target(0,settings); JULIA_CHECK(r.reached()); r.target(.1,settings);
        for(unsigned n=0;n<2000;++n) { r.target(.1,settings); r.advance(h); }
        JULIA_CHECK(r.reached()&&r.q()==.1);
        const double saved=r.q(); bool rejected=false;
        try { r.target(.2,{.6,1e-300,false}); } catch(const std::invalid_argument&) { rejected=true; }
        JULIA_CHECK(rejected&&r.q()==saved&&r.reached());
        rejected=false;
        try { r.target(std::numeric_limits<double>::max(),{1e-4,2,false}); } catch(const std::invalid_argument&) { rejected=true; }
        JULIA_CHECK(rejected&&r.q()==saved&&r.reached());
        const auto split=integratePower(2,-2,1); JULIA_CHECK(split.delivered==.5&&split.absorbed==.5);
        out<<"reference max_speed="<<maxV<<" max_acceleration="<<maxA<<" retarget_discontinuity="<<continuity<<'\n';
    }
    void fixtureTests(const std::filesystem::path& dir,std::ostream& out)
    {
        PhysicsWorld w{fixture()}; const auto saved=w.snapshot();
        for(double q:{-.8,0.,.6,1.1})
        {
            const auto g=w.gravityCompensation({q}); const double analytic=9.81*std::cos(q);
            JULIA_CHECK(std::abs(g[0]-analytic)<1e-10);
            JULIA_CHECK(w.snapshot().q==saved.q&&w.snapshot().v==saved.v&&w.snapshot().worldSteps==0);
            out<<"gravity_fixture q="<<q<<" analytic_Nm="<<analytic<<" RNE_Nm="<<g[0]<<'\n';
        }
        w.reset({.6}); servo(w,{.6},true);
        for(unsigned n=0;n<240;++n) w.step();
        JULIA_CHECK(std::abs(w.snapshot().q[0]-.6)<1e-10);
        auto weak=fixture(); weak.drives[0].maxTorque=.35; PhysicsWorld limited{weak}; servo(limited,{},true); limited.step();
        JULIA_CHECK(limited.snapshot().drive.feedforward[0]>9&&limited.snapshot().calculatedTorque[0]>.35);
        JULIA_CHECK(limited.snapshot().appliedTorque[0]==.35&&limited.snapshot().saturatedThisTick[0]);
        JULIA_CHECK(std::abs(limited.snapshot().drive.saturatedSeconds[0]-TickSeconds)<1e-15);
        double previous=1;
        std::ofstream csv{dir/"fixture-work.csv"}; driveCsvHeader(csv);
        for(unsigned sub:{8u,16u,32u})
        {
            auto d=fixture(); d.gravity={0,0,0}; d.substeps=sub;
            PhysicsWorld energy{d}; JULIA_CHECK(energy.command(CommandKind::Torque,{.4}));
            for(unsigned n=0;n<120;++n) energy.step();
            const auto s=energy.snapshot(); const double work=s.drive.deliveredWork[0]-s.drive.absorbedWork[0];
            const double residual=std::abs(work-.4*s.q[0]);
            const double analyticEnergy=.5*(2./3.)*std::pow(.4/(2./3.),2);
            JULIA_CHECK(std::abs(work-analyticEnergy)<1e-9&&std::abs(work-s.kineticEnergy)<1e-9);
            JULIA_CHECK(residual/(.4*s.q[0])<.002&&residual<previous*.6); previous=residual;
            JULIA_CHECK(std::abs(s.drive.power[0]-s.appliedTorque[0]*s.v[0])<1e-12);
            driveCsvRow(csv,s,sub);
            JULIA_CHECK(energy.command(CommandKind::Torque,{-.4}));
            for(unsigned n=0;n<60;++n) energy.step();
            const auto braking=energy.snapshot(); JULIA_CHECK(braking.drive.power[0]<0&&braking.drive.absorbedWork[0]>0);
            JULIA_CHECK(braking.drive.deliveredWork[0]>=0&&braking.drive.absorbedWork[0]>=0);
            out<<"work_fixture substeps="<<sub<<" work_J="<<work<<" kinetic_J="<<s.kineticEnergy<<" analytic_J="<<analyticEnergy
                <<" displacement_residual_J="<<residual<<" absorbed_J="<<braking.drive.absorbedWork[0]<<'\n';
        }
        auto externalDef=fixture(); externalDef.gravity={0,0,0}; PhysicsWorld external{externalDef};
        for(unsigned n=0;n<120;++n) { external.applyWrench(0,{0,0,0},{0,0,.4}); external.step(); }
        JULIA_CHECK(std::abs(external.snapshot().drive.externalWork-.12)<1e-9&&external.snapshot().drive.deliveredWork[0]==0);
        JULIA_CHECK(std::abs(external.snapshot().drive.mechanicalEnergyChange-.12)<1e-9);
        auto dampedDef=externalDef; dampedDef.links[0].hinges[0].damping=.2; PhysicsWorld damped{dampedDef};
        damped.command(CommandKind::Torque,{.4}); for(unsigned n=0;n<120;++n) damped.step();
        const auto energized=damped.snapshot(); damped.command(CommandKind::Disable); for(unsigned n=0;n<120;++n) damped.step();
        const auto passive=damped.snapshot();
        JULIA_CHECK(passive.drive.passiveDampingPower<0&&passive.drive.deliveredWork==energized.drive.deliveredWork);
        JULIA_CHECK(std::abs(passive.kineticEnergy-energized.kineticEnergy-(passive.drive.passiveDampingWork-energized.drive.passiveDampingWork))<.001);
        out<<"external_work_J="<<external.snapshot().drive.externalWork<<" passive_damping_delta_J="<<passive.drive.passiveDampingWork-energized.drive.passiveDampingWork<<'\n';
    }
    void commandTests(std::ostream& out)
    {
        PhysicsWorld w{fixture()}; servo(w,{.8},true); for(unsigned n=0;n<30;++n) w.step();
        const auto before=w.snapshot(); std::string reason;
        JULIA_CHECK(!w.configureServo({.2,1,false},&reason)&&!reason.empty());
        for(auto bad:std::array<Coordinates,3>{{{3},{std::numeric_limits<double>::quiet_NaN()},{0,1}}})
            JULIA_CHECK(!w.command(CommandKind::SmoothTarget,bad,&reason)&&!reason.empty());
        JULIA_CHECK(!w.command(CommandKind::MoveRoot,{},&reason));
        JULIA_CHECK(deterministic(w.snapshot(),8)==deterministic(before,8));
        JULIA_CHECK(w.command(CommandKind::SmoothTarget,{-.4}));
        JULIA_CHECK(w.snapshot().drive.qRef==before.drive.qRef&&w.snapshot().drive.vRef==before.drive.vRef);
        w.command(CommandKind::Disable); w.step();
        JULIA_CHECK(w.snapshot().appliedTorque[0]==0&&w.snapshot().drive.feedforward[0]==0&&!w.snapshot().drive.referenceReached[0]);
        JULIA_CHECK(!w.configureServo({0,2,false},&reason));
        JULIA_CHECK(!w.configureServo({1e-300,2,false},&reason));
        JULIA_CHECK(!w.configureServo({.6,1e-300,false},&reason));
        JULIA_CHECK(!w.configureServo({.6,std::numeric_limits<double>::infinity(),false},&reason));
        w.command(CommandKind::Torque,{50}); w.step();
        JULIA_CHECK(w.snapshot().appliedTorque[0]==12&&w.snapshot().drive.feedforward[0]==0);
        const auto q=w.snapshot().q; w.command(CommandKind::SmoothTarget,{});
        JULIA_CHECK(w.snapshot().drive.qRef==q&&w.snapshot().drive.vRef==Coordinates{});
        const auto run=[&]()
        {
            w.reset(); servo(w,{.7},true);
            for(unsigned n=0;n<240;++n)
            {
                if(n==61) w.command(CommandKind::SmoothTarget,{.2});
                if(n==100) w.applyWrench(0,{0,-2,0});
                w.step();
            }
            return w.snapshot();
        };
        const auto a=run(),b=run(); JULIA_CHECK(a.q==b.q&&a.v==b.v&&deterministic(a,8)==deterministic(b,8));
        w.reset(); JULIA_CHECK(w.snapshot().mode==DriveMode::Disabled&&!w.snapshot().servo.gravityCompensation);
        JULIA_CHECK(w.snapshot().drive.deliveredWork==Coordinates{}&&w.snapshot().drive.saturatedSeconds==Coordinates{});
        out<<"commands invalid_atomicity/retarget/disable/torque/reenable/reset_replay PASS\n";
    }
    struct MotionResult { std::vector<Snapshot> samples; double rms=0,overshoot=0,settle=0,maxActualSpeed=0,maxRefSpeed=0; };
    MotionResult motion(const ArmRig& rig,bool gravity,bool pulse,const std::filesystem::path& file)
    {
        PhysicsWorld w{rig.description(1,1,.5)}; resetBench(w,BenchCase::Retarget); servo(w,w.snapshot().target,gravity);
        MotionResult result; std::ofstream csv{file}; driveCsvHeader(csv);
        double sum=0,finish=-1,lastBad=-1;
        for(unsigned n=0;n<720;++n)
        {
            if(pulse&&n>=240&&n<246) w.applyWrench(2,{0,0,-15});
            stepBench(w,BenchCase::Retarget); const auto s=w.snapshot(); result.samples.push_back(s); driveCsvRow(csv,s,8);
            for(unsigned k=0;k<6;++k)
            {
                sum+=s.drive.positionError[k]*s.drive.positionError[k];
                result.maxActualSpeed=std::max(result.maxActualSpeed,std::abs(s.v[k]));
                result.maxRefSpeed=std::max(result.maxRefSpeed,std::abs(s.drive.vRef[k]));
            }
            const bool reached=std::all_of(s.drive.referenceReached.begin(),s.drive.referenceReached.begin()+6,[](bool b) { return b; });
            if(n>=181&&reached)
            {
                if(finish<0) finish=s.time;
                const double e=error(s.q,s.target,6); result.overshoot=std::max(result.overshoot,e);
                if(e>.01||error(s.v,{},6)>.03) lastBad=s.time;
            }
        }
        result.rms=std::sqrt(sum/(720*6)); result.settle=std::max(0.,lastBad-finish+TickSeconds);
        if(gravity) JULIA_CHECK(result.rms<=.10&&result.overshoot<=.08&&result.settle<=1.5&&finish>0);
        return result;
    }
    void massTests(const ArmRig& rig,std::ostream& out)
    {
        PhysicsWorld base{rig.description()},scaled{rig.description(2)},loaded{rig.description(1,1,.5)};
        const auto b=base.massAudit(),s=scaled.massAudit(),l=loaded.massAudit();
        JULIA_CHECK(std::abs(s.totalMass-2*b.totalMass)<1e-10&&std::abs(l.totalMass-b.totalMass-.5)<1e-10);
        for(unsigned i=0;i<3;++i)
        {
            JULIA_CHECK(glm::length(s.comInJoint[i]-b.comInJoint[i])<1e-10);
            for(int c=0;c<3;++c) for(int r=0;r<3;++r) JULIA_CHECK(std::abs(s.inertiaInJoint[i][c][r]-2*b.inertiaInJoint[i][c][r])<1e-8);
        }
        const auto& p=loaded.definition().links.back();
        JULIA_CHECK(p.mass==.5&&glm::length(p.inertia-glm::dvec3{.4*.5*.04*.04})<1e-12);
        const auto frames=loaded.forwardKinematics({}); glm::dvec3 weighted{0};
        for(std::size_t i=0;i<loaded.definition().links.size();++i)
        {
            const auto& link=loaded.definition().links[i];
            const auto com=glm::dvec3{link.jointFromSegment*glm::dvec4{link.comInSegment,1}};
            JULIA_CHECK(glm::length(l.comInJoint[i]-com)<1e-8);
            weighted+=link.mass*glm::dvec3{frames[i]*glm::dvec4{com,1}};
        }
        JULIA_CHECK(glm::length(l.comWorld-weighted/l.totalMass)<1e-8);
        const Coordinates q{.45,.18,-.1,.65,.12,.08}; const auto gravity=loaded.gravityCompensation(q);
        double gradientError=0;
        for(unsigned k=0;k<6;++k)
        {
            auto plus=q,minus=q; plus[k]+=1e-6; minus[k]-=1e-6;
            const auto a=loaded.massAudit(plus),z=loaded.massAudit(minus);
            const double gradient=-glm::dot(loaded.definition().gravity,a.totalMass*a.comWorld-z.totalMass*z.comWorld)/(2e-6);
            gradientError=std::max(gradientError,std::abs(gravity[k]-gradient));
        }
        JULIA_CHECK(gradientError<1e-7&&loaded.snapshot().worldSteps==0&&loaded.snapshot().q==Coordinates{});
        out<<"mass base/scaled/loaded_kg="<<b.totalMass<<'/'<<s.totalMass<<'/'<<l.totalMass<<" COM_and_inertia PASS\n";
        out<<"gravity_vs_potential_gradient_max_Nm="<<gradientError<<'\n';
    }
    void armTests(const julia::model::ModelAsset& asset,const std::filesystem::path& dir,std::ostream& out)
    {
        const std::array<Coordinates,3> poses{{{.45,.18,-.1,.65,.12,.08},{.75,.3,.15,.9,.2,.12},{.25,.1,.05,.4,-.1,-.08}}};
        for(auto side:{ArmSide::Left,ArmSide::Right})
        {
            const auto name=side==ArmSide::Left?"left":"right"; const ArmRig rig{asset,side,ArmChain::Full}; massTests(rig,out);
            for(double payload:{0.,.5}) for(std::size_t p=0;p<poses.size();++p)
            {
                double errors[2]{};
                for(unsigned comp=0;comp<2;++comp)
                {
                    PhysicsWorld w{rig.description(1,1,payload)}; w.reset(poses[p]); servo(w,poses[p],comp!=0);
                    for(unsigned n=0;n<720;++n) w.step();
                    std::ofstream sample{dir/(std::string{name}+"-static-"+std::to_string(p)+"-"+std::to_string(payload)+"-"+std::to_string(comp)+".csv")};
                    driveCsvHeader(sample); driveCsvRow(sample,w.snapshot(),8);
                    errors[comp]=error(w.snapshot().q,poses[p],6);
                    if(comp) for(unsigned k=0;k<6;++k) JULIA_CHECK(!w.snapshot().saturated[k]);
                }
                JULIA_CHECK(errors[1]<=.005&&errors[1]<=.2*errors[0]);
                out<<"static "<<name<<" pose="<<p<<" payload_kg="<<payload<<" no_comp_error_rad="<<errors[0]<<" comp_error_rad="<<errors[1]<<'\n';
            }
            // Physical segment mass scaling with matching inertia and gravity.
            PhysicsWorld scaled{rig.description(1.5)}; scaled.reset(poses[0]); servo(scaled,poses[0],true);
            for(unsigned n=0;n<720;++n) scaled.step(); JULIA_CHECK(error(scaled.snapshot().q,poses[0],6)<.005);
            const auto plain=motion(rig,false,false,dir/(std::string{name}+"-move-no-comp.csv"));
            const auto comp=motion(rig,true,false,dir/(std::string{name}+"-move-comp.csv"));
            const auto pulse=motion(rig,true,true,dir/(std::string{name}+"-pulse.csv"));
            double disturbance=0;
            for(unsigned n=240;n<300;++n) disturbance=std::max(disturbance,error(pulse.samples[n].q,comp.samples[n].q,6));
            JULIA_CHECK(disturbance>.005&&error(pulse.samples.back().q,pulse.samples.back().drive.qRef,6)<.01);
            JULIA_CHECK(comp.rms<plain.rms);
            PhysicsWorld weak{benchDefinition(rig,BenchCase::Weak)}; resetBench(weak,BenchCase::Weak); servo(weak,weak.snapshot().target,true);
            unsigned saturated=0;
            for(unsigned n=0;n<720;++n) { weak.step(); if(std::any_of(weak.snapshot().saturatedThisTick.begin(),weak.snapshot().saturatedThisTick.begin()+6,[](bool b) { return b; })) ++saturated; }
            const double weakError=error(weak.snapshot().q,weak.snapshot().target,6);
            JULIA_CHECK(saturated>360&&weakError>.1);
            out<<"motion "<<name<<" RMS_no_comp="<<plain.rms<<" RMS_comp="<<comp.rms<<" overshoot="<<comp.overshoot<<" settle_s="<<comp.settle
                <<" pulse_delta="<<disturbance<<" pulse_final_error="<<error(pulse.samples.back().q,pulse.samples.back().drive.qRef,6)
                <<" pulse_max_actual_speed="<<pulse.maxActualSpeed<<" max_ref_speed="<<pulse.maxRefSpeed
                <<" weak_error="<<weakError<<" weak_saturated_ticks="<<saturated<<'\n'; out.flush();
        }
    }
    void bodyTests(const julia::model::ModelAsset& asset,std::ostream& out)
    {
        BodyRig rig{asset}; PhysicsWorld w{bodyBenchDefinition(rig,BodyCase::Internal)}; resetBodyBench(rig,w,BodyCase::Internal);
        const auto saved=w.snapshot(); std::string reason;
        JULIA_CHECK(!w.configureServo({.6,2,true},&reason)&&reason.find("fixed base")!=std::string::npos);
        JULIA_CHECK(deterministic(w.snapshot(),16)==deterministic(saved,16)&&!w.capabilities().fixedBaseGravity);
        JULIA_CHECK(w.layout().nu==41&&w.layout().nv==47&&w.layout().nq==48);
        servo(w,{},false); const auto initial=w.snapshot(); double com=0,p=0,L=0; Coordinates moved{};
        for(unsigned n=0;n<480;++n)
        {
            stepBodyBench(w,BodyCase::Internal); const auto& s=w.snapshot();
            com=std::max(com,glm::length(s.totalComWorld-initial.totalComWorld-s.time*initial.linearMomentumWorld/initial.totalMass));
            p=std::max(p,glm::length(s.linearMomentumWorld-initial.linearMomentumWorld));
            L=std::max(L,glm::length(s.angularMomentumAboutComWorld-initial.angularMomentumAboutComWorld));
            for(unsigned k=0;k<41;++k) moved[k]=std::max(moved[k],std::abs(s.q[k]));
        }
        JULIA_CHECK(com<=.003&&p<=.1&&L<=.1); for(unsigned k=0;k<41;++k) JULIA_CHECK(moved[k]>.005);
        out<<"body_zero_g COM_m="<<com<<" momentum_kg_m_s="<<p<<" angular_kg_m2_s="<<L<<" all_41_move PASS\n";
        w.command(CommandKind::Disable); stepBodyBench(w,BodyCase::Internal,true);
        JULIA_CHECK(w.snapshot().mode==DriveMode::Disabled&&w.snapshot().appliedTorque==Coordinates{});
        PhysicsWorld fall{bodyBenchDefinition(rig,BodyCase::Tilt)}; resetBodyBench(rig,fall,BodyCase::Tilt); servo(fall,{},false);
        const auto root=fall.snapshot().root.position; bool nonfoot=false;
        for(unsigned n=0;n<720;++n)
        {
            stepBodyBench(fall,BodyCase::Tilt);
            for(const auto& c:fall.snapshot().contacts)
                if((c.link1==-1&&c.link2!=16&&c.link2!=19)||(c.link2==-1&&c.link1!=16&&c.link1!=19)) nonfoot=true;
        }
        JULIA_CHECK(nonfoot&&glm::length(fall.snapshot().root.position-root)>.1);
        out<<"body_fall nonfoot="<<nonfoot<<" root_displacement_m="<<glm::length(fall.snapshot().root.position-root)<<'\n';
    }
    void stats(std::ostream& out,const char* label,std::vector<double> samples)
    {
        const auto mean=std::accumulate(samples.begin(),samples.end(),0.)/static_cast<double>(samples.size());
        std::sort(samples.begin(),samples.end()); out<<label<<" mean_us="<<mean<<" p95_us="<<samples[samples.size()*95/100]<<" max_us="<<samples.back()<<'\n';
        if(std::string{label}=="tick") JULIA_CHECK(samples[samples.size()*95/100]<8333);
    }
    void performance(const julia::model::ModelAsset& asset,const std::filesystem::path& dir,std::ostream& out)
    {
        const ArmRig arm{asset,ArmSide::Left,ArmChain::Full}; const BodyRig body{asset};
        for(unsigned test=0;test<5;++test)
        {
            const bool full=test>=3, smooth=test==1||test==2||test==4;
            const auto name=full?(smooth?"body-servo":"body-legacy"):(test==0?"arm-legacy":test==1?"arm-servo":"arm-servo-gravity");
            PhysicsWorld w{full?bodyBenchDefinition(body,BodyCase::Internal):arm.description(1,1,.5)};
            if(full) resetBodyBench(body,w,BodyCase::Internal); else resetBench(w,BenchCase::Retarget);
            if(smooth) servo(w,w.snapshot().target,test==2);
            std::ofstream csv{dir/(std::string{name}+"-performance.csv")}; csv<<"tick,tick_us,control_us,telemetry_us,step_us,read_us,publication_us\n";
            std::vector<double> tick,control,telemetry,publication;
            for(unsigned n=0;n<10000;++n)
            {
                const auto start=std::chrono::steady_clock::now();
                if(full) stepBodyBench(w,BodyCase::Internal); else w.step();
                const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
                const auto& t=w.snapshot().timing; tick.push_back(elapsed); control.push_back(t.controlUs); telemetry.push_back(t.telemetryUs);
                const double mapping=full?publish(body,w.snapshot()).mappingUs:publish(arm,w.snapshot()).mappingUs;
                publication.push_back(mapping);
                csv<<n+1<<','<<elapsed<<','<<t.controlUs<<','<<t.telemetryUs<<','<<t.stepUs<<','<<t.readUs<<','<<mapping<<'\n';
            }
            out<<"performance "<<name<<" 10000 ticks, no import/pose/CSV IO/GPU in timer\n";
            stats(out,"tick",tick); stats(out,"control",control); stats(out,"telemetry",telemetry); stats(out,"publication",publication); out.flush();
        }
    }
}
int main(int argc,char** argv)
{
    try
    {
        const std::filesystem::path dir=argc>1?argv[1]:"build/diagnostics/stage7d/cpu"; std::filesystem::create_directories(dir);
        std::ofstream out{dir/"joint-drive.txt"}; out<<std::setprecision(17);
        referenceTests(out); fixtureTests(dir,out); commandTests(out); out.flush();
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        armTests(*asset,dir,out); bodyTests(*asset,out); out.flush(); performance(*asset,dir,out);
        out<<"failures="<<JULIA_TEST_RESULT()<<'\n'; std::cout<<"Joint drive failures="<<JULIA_TEST_RESULT()<<'\n'; return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"JOINT DRIVE TEST FAILURE: "<<e.what()<<'\n'; return 1; }
}

