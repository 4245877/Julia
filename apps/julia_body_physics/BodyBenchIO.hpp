#pragma once
#include "julia/physics/BodyRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>

namespace bodybench
{
    using namespace julia::physics;
    inline bool isCom(BodyCase c) { return c==BodyCase::ComShift||c==BodyCase::ComRetarget; }
    inline bool isStand(BodyCase c) { return c==BodyCase::Stand||c==BodyCase::StandPulse||isCom(c); }
    struct Options
    {
        BodyCase scenario=BodyCase::Drop;
        std::uint64_t ticks=720;
        unsigned fps=60;
        bool verify=false, explicitTicks=false;
        bool servo=false,gravityCompensation=false;
        bool observer=true;
        unsigned substeps=16;
        bool standFeedback=true, bounded=false, explicitController=false, comPulses=false;
        unsigned feedbackOffTick=0;
        double yaw=0,initialShiftX=0,initialShiftZ=0,initialTilt=0,effortScale=1,contactTime=.01;
        std::string standEvent="none";
        std::filesystem::path output="build/diagnostics/stage7d/interactive-body";
    };
    inline Options parse(int argc,char** argv)
    {
        Options o;
        for(int i=1;i<argc;++i)
        {
            const std::string flag=argv[i];
            const auto value=[&]() { if(++i>=argc) throw std::invalid_argument("Missing value: "+flag); return std::string{argv[i]}; };
            if(flag=="--stand-controller") { const auto v=value(); if(v!="bounded"&&v!="equality") throw std::invalid_argument("Controller: bounded|equality"); o.bounded=v=="bounded"; o.explicitController=true; }
            else if(flag=="--com-pulses") { const auto v=value(); if(v!="on"&&v!="off") throw std::invalid_argument("COM pulses: on|off"); o.comPulses=v=="on"; }
            else if(flag=="--case") o.scenario=parseBodyCase(value());
            else if(flag=="--stand-feedback-off-tick") { const auto v=value(); std::size_t used=0; const auto n=std::stoul(v,&used); if(used!=v.size()||n<1||n>1200000) throw std::invalid_argument("Invalid ablation tick"); o.feedbackOffTick=static_cast<unsigned>(n); }
            else if(flag=="--stand-feedback") { const auto v=value(); if(v!="on"&&v!="off") throw std::invalid_argument("Stand feedback: on|off"); o.standFeedback=v=="on"; }
            else if(flag=="--stand-event") { o.standEvent=value(); if(o.standEvent!="none"&&o.standEvent!="disable"&&o.standEvent!="lift"&&o.standEvent!="outside") throw std::invalid_argument("Invalid stand event"); }
            else if(flag=="--yaw"||flag=="--initial-shift-x"||flag=="--initial-shift-z"||flag=="--initial-tilt"||flag=="--effort-scale"||flag=="--contact-time")
            {
                const auto text=value(); std::size_t used=0; const double v=std::stod(text,&used);
                if(used!=text.size()||!std::isfinite(v)) throw std::invalid_argument("Finite stand parameter required");
                if(flag=="--yaw") o.yaw=v;
                else if(flag=="--initial-shift-x") o.initialShiftX=v;
                else if(flag=="--initial-shift-z") o.initialShiftZ=v;
                else if(flag=="--initial-tilt") o.initialTilt=v;
                else if(flag=="--effort-scale") o.effortScale=v;
                else o.contactTime=v;
            }
            else if(flag=="--observe") { const auto v=value(); if(v!="on"&&v!="off") throw std::invalid_argument("Observe: on|off"); o.observer=v=="on"; }
            else if(flag=="--substeps") { const auto v=value(); if(v!="16"&&v!="32"&&v!="64") throw std::invalid_argument("Substeps:16|32|64"); o.substeps=static_cast<unsigned>(std::stoul(v)); }
            else if(flag=="--drive")
            {
                const auto s=value();
                if(s!="legacy"&&s!="servo"&&s!="servo-gravity") throw std::invalid_argument("Drive: legacy|servo|servo-gravity");
                o.servo=s!="legacy"; o.gravityCompensation=s=="servo-gravity";
            }
            else if(flag=="--output") o.output=value();
            else if(flag=="--verify") { o.verify=true; o.output=value(); }
            else if(flag=="--ticks"||flag=="--fps")
            {
                const auto s=value(); std::size_t used=0;
                if(s.empty()||s[0]=='-') throw std::invalid_argument("Invalid count");
                const auto n=std::stoull(s,&used);
                if(used!=s.size()||n==0||n>1200000) throw std::invalid_argument("Invalid count");
                if(flag=="--ticks") { o.ticks=n; o.explicitTicks=true; }
                else { if(n!=30&&n!=60&&n!=144) throw std::invalid_argument("Frame rate must be 30/60/144"); o.fps=static_cast<unsigned>(n); }
            }
            else throw std::invalid_argument("Unknown option: "+flag);
        }
        const bool stand=isStand(o.scenario);
        if(isCom(o.scenario)&&!o.explicitController) o.bounded=true;
        if(stand&&!o.explicitTicks) o.ticks=isCom(o.scenario)?(o.scenario==BodyCase::ComShift?6480:2160):1920;
        if(std::abs(o.yaw)>3.15||std::abs(o.initialShiftX)>.1||std::abs(o.initialShiftZ)>.1||std::abs(o.initialTilt)>.2||o.effortScale<.05||o.effortScale>1||o.contactTime<.008||o.contactTime>.012)
            throw std::invalid_argument("Stand parameter outside experiment bounds");
        if(o.comPulses&&!isCom(o.scenario)) throw std::invalid_argument("COM pulses require COM case");
        if(!stand&&(o.explicitController||!o.standFeedback||o.feedbackOffTick!=0||o.yaw!=0||o.initialShiftX!=0||o.initialShiftZ!=0||o.initialTilt!=0||o.effortScale!=1||o.contactTime!=.01||o.standEvent!="none")) throw std::invalid_argument("Stand parameters require a stand case");
        if(stand&&(!o.observer||o.servo)) throw std::invalid_argument("Stand requires observer and exclusive stand drive");
        return o;
    }
    inline Definition definition(const BodyRig& rig,BodyCase scenario,const Options& o)
    {
        auto d=bodyBenchDefinition(rig,scenario); d.substeps=o.substeps;
        if(!o.observer) d.supportObservation.reset();
        if(isStand(scenario))
        { for(auto& drive:d.drives) drive.maxTorque*=o.effortScale; d.contactTimeConstant=o.contactTime; }
        return d;
    }
    inline void reset(const BodyRig& rig,PhysicsWorld& world,BodyCase scenario,const Options& o)
    {
        if(isStand(scenario))
        {
            // Translation and yaw belong to the entire preparation, including goals.
            auto placement=glm::rotate(glm::dmat4{1},o.yaw,glm::dvec3{0,1,0});
            if(o.yaw!=0) placement[3]=glm::dvec4{1.2,0,-.7,1};
            auto plan=preparedStand(rig,world,placement); plan.balanceFeedback=o.standFeedback;
            plan.variant=(o.bounded||(isCom(scenario)&&!o.explicitController))?StandVariant::Bounded7h:StandVariant::Equality7g; plan.comCommands=isCom(scenario);
            auto initial=plan.rootGoal; initial.position+=plan.rootGoal.orientation*glm::dvec3{o.initialShiftX,0,o.initialShiftZ};
            initial.orientation=initial.orientation*glm::angleAxis(o.initialTilt,glm::dvec3{1,0,0});
            world.reset(plan.posture,initial); std::string reason;
            if(!world.startStand(plan,&reason)) throw std::invalid_argument(reason);
        }
        else resetBodyBench(rig,world,scenario);
        if(o.servo)
        {
            if(scenario==BodyCase::Passive) throw std::invalid_argument("Passive cannot enable servo");
            std::string reason;
            if(!world.configureServo({.6,2,o.gravityCompensation},&reason)||
                !world.command(CommandKind::SmoothTarget,world.snapshot().target,&reason)) throw std::invalid_argument(reason);
        }
    }
    inline void step(PhysicsWorld& world,BodyCase scenario,const Options& o)
    {
        const auto tick=world.snapshot().tick;
        if(o.feedbackOffTick!=0&&tick==o.feedbackOffTick) { std::string reason; if(!world.standBalanceFeedback(false,&reason)) throw std::runtime_error(reason); }
        if(tick==360&&o.standEvent=="disable") world.command(CommandKind::Disable);
        if(tick>=360&&tick<384&&o.standEvent=="lift") world.applyWrench(16,{0,450,0});
        if(tick>=360&&tick<384&&o.standEvent=="outside") world.applyWrench(3,{150,0,0});
        if(isCom(scenario))
        {
            const auto target=[&](glm::dvec2 goal) { std::string reason; if(!world.standComTarget(goal,&reason)) throw std::runtime_error(reason); };
            if(tick==240) target({.003,0});
            if(scenario==BodyCase::ComShift)
            {
                if(tick==1440) target({-.003,0});
                if(tick==3360) target({0,.003});
                if(tick==5040) target({0,0});
                const auto rotation=world.snapshot().stand.rootGoal.orientation;
                if(o.comPulses&&tick>=1080&&tick<1092) world.applyWrench(3,rotation*glm::dvec3{3,0,0});
                if(o.comPulses&&tick>=4680&&tick<4692) world.applyWrench(3,rotation*glm::dvec3{0,0,3});
            }
            else
            {
                if(tick==360) target({0,.003});
                if(tick==480) { std::string reason; if(!world.cancelStandCom(&reason)) throw std::runtime_error(reason); }
                if(tick==1200) target({0,0});
            }
        }
        stepBodyBench(world,scenario,o.servo);
    }
    struct Log
    {
        std::ofstream state,joints,contacts,drive,support,dynamics,regions,points,substeps,stand,com,comSolver;
        unsigned substepsPerTick=16;
        explicit Log(const std::filesystem::path& dir,unsigned steps=16):substepsPerTick(steps)
        {
            std::filesystem::create_directories(dir);
            state.open(dir/"state.csv"); joints.open(dir/"joints.csv"); contacts.open(dir/"contacts.csv");
            com.open(dir/"com-v1.csv"); comCsvHeader(com);
            comSolver.open(dir/"com-solver-v1.csv"); comSolverCsvHeader(comSolver);
            stand.open(dir/"stand-v1.csv"); standCsvHeader(stand);
            if(!state||!joints||!contacts) throw std::runtime_error("Cannot create CSV");
            state<<"tick,epoch,time_s,world_steps,root_x_m,root_y_m,root_z_m,qw,qx,qy,qz,vx_world_m_s,vy_world_m_s,vz_world_m_s,wx_local_rad_s,wy_local_rad_s,wz_local_rad_s,com_x_m,com_y_m,com_z_m,px_kg_m_s,py_kg_m_s,pz_kg_m_s,Lx_kg_m2_s,Ly_kg_m2_s,Lz_kg_m2_s,kinetic_J,potential_J,contacts,peak_contacts,peak_normal_N,peak_penetration_m,solver_iterations,warnings,external_fx_N,external_fy_N,external_fz_N,mapping_us\n";
            joints<<"tick,epoch,coordinate,q_rad,v_rad_s,target_rad,calculated_Nm,applied_Nm,saturated_tick,limit_rad,peak_limit_rad\n";
            contacts<<"tick,epoch,geom1,geom2,link1,link2,distance_m,x_m,y_m,z_m,nx,ny,nz,normal_N,tangent1_N,tangent2_N\n";
            drive.open(dir/"drive-v1.csv"); driveCsvHeader(drive);
            support.open(dir/"support-v1.csv"); supportCsvHeader(support);
            dynamics.open(dir/"dynamics-v1.csv"); dynamicsCsvHeader(dynamics);
            regions.open(dir/"support-regions-v1.csv"); supportRegionsCsvHeader(regions);
            points.open(dir/"support-points-v1.csv"); contactPointsCsvHeader(points);
            substeps.open(dir/"dynamics-substeps-v1.csv"); dynamicsSubstepsCsvHeader(substeps);
            state<<std::setprecision(17); joints<<std::setprecision(17); contacts<<std::setprecision(17);
        }
        void row(const BodyRenderSnapshot& view)
        {
            const auto& s=view.physics; const auto& r=s.root;
            driveCsvRow(drive,s,substepsPerTick);
            supportCsvRow(support,s); dynamicsCsvRow(dynamics,s); supportRegionsCsvRow(regions,s); contactPointsCsvRow(points,s);
            dynamicsSubstepsCsvRow(substeps,s);
            standCsvRow(stand,s); comCsvRow(com,s); comSolverCsvRow(comSolver,s);
            const auto vec=[&](glm::dvec3 v) { state<<','<<v.x<<','<<v.y<<','<<v.z; };
            state<<s.tick<<','<<s.epoch<<','<<s.time<<','<<s.worldSteps;
            vec(r.position); state<<','<<r.orientation.w<<','<<r.orientation.x<<','<<r.orientation.y<<','<<r.orientation.z;
            vec(r.linearVelocityWorld); vec(r.angularVelocityLocal); vec(s.totalComWorld); vec(s.linearMomentumWorld); vec(s.angularMomentumAboutComWorld);
            state<<','<<s.kineticEnergy<<','<<s.potentialEnergy<<','<<s.contacts.size()<<','<<s.peakContactsThisTick<<','<<s.peakNormalForceThisTick
                <<','<<s.maxPenetrationThisTick<<','<<s.solverIterations<<','<<s.warnings;
            vec(s.externalForce); state<<','<<view.mappingUs<<'\n';
            for(unsigned k=0;k<s.dofs;++k) joints<<s.tick<<','<<s.epoch<<','<<k<<','<<s.q[k]<<','<<s.v[k]<<','<<s.target[k]<<','<<s.calculatedTorque[k]
                <<','<<s.appliedTorque[k]<<','<<s.saturatedThisTick[k]<<','<<s.limitViolation[k]<<','<<s.peakLimitViolationThisTick[k]<<'\n';
            for(const auto& c:s.contacts)
            {
                contacts<<s.tick<<','<<s.epoch<<','<<c.geom1<<','<<c.geom2<<','<<c.link1<<','<<c.link2<<','<<c.distance<<','<<c.point.x<<','<<c.point.y<<','<<c.point.z
                    <<','<<c.normal.x<<','<<c.normal.y<<','<<c.normal.z;
                if(c.force) contacts<<','<<c.force->x<<','<<c.force->y<<','<<c.force->z; else contacts<<",NA,NA,NA";
                contacts<<'\n';
            }
            if(!state||!joints||!contacts||!drive||!support||!dynamics||!regions||!points||!substeps||!com||!stand) throw std::runtime_error("CSV write failed");
        }
    };
    inline void metadata(const std::filesystem::path& dir,const BodyRig& rig,const PhysicsWorld& world)
    {
        std::ofstream info{dir/"calibration.txt"}; writeBodyCalibration(info,rig,world);
        std::ofstream xml{dir/"model.xml"}; xml<<world.modelXml();
    }
}
