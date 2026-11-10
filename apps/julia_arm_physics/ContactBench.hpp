#pragma once
#include "BenchIO.hpp"
#include <algorithm>

namespace contact_bench
{
    using namespace julia::physics;
    struct Options
    {
        ArmSide side=ArmSide::Left;
        std::string scenario="cycle";
        unsigned pose=0,fps=60,substeps=8,delay=0;
        std::uint64_t ticks=1920;
        bool verify=false,explicitTicks=false,feedback=true;
        double payload=0,offset=0,friction=.6,softness=.006;
#if defined(JULIA_PHYSICS_ASSETS)
        std::filesystem::path output=std::filesystem::path{JULIA_PHYSICS_ASSETS}.parent_path()/"build/diagnostics/stage7e/interactive";
#else
        std::filesystem::path output="build/diagnostics/stage7e/interactive";
#endif
    };
    inline Options parse(int argc,char** argv)
    {
        Options o;
        for(int i=1;i<argc;++i)
        {
            const std::string flag=argv[i];
            const auto value=[&]() { if(++i>=argc) throw std::invalid_argument("Missing value for "+flag); return std::string{argv[i]}; };
            if(flag=="--side") { const auto s=value(); if(s!="left"&&s!="right") throw std::invalid_argument("Side: left|right"); o.side=s=="left"?ArmSide::Left:ArmSide::Right; }
            else if(flag=="--case") o.scenario=value();
            else if(flag=="--output") o.output=value();
            else if(flag=="--verify") { o.verify=true; o.output=value(); }
            else if(flag=="--feedback") { const auto s=value(); if(s!="on"&&s!="off") throw std::invalid_argument("Feedback: on|off"); o.feedback=s=="on"; }
            else if(flag=="--payload"||flag=="--offset"||flag=="--friction"||flag=="--softness")
            {
                const auto s=value(); std::size_t used=0; const auto x=std::stod(s,&used);
                if(used!=s.size()||!std::isfinite(x)) throw std::invalid_argument("Invalid number "+flag);
                if(flag=="--payload") o.payload=x; else if(flag=="--offset") o.offset=x;
                else if(flag=="--friction") o.friction=x; else o.softness=x;
            }
            else if(flag=="--ticks"||flag=="--fps"||flag=="--pose"||flag=="--substeps"||flag=="--delay")
            {
                const auto s=value(); std::size_t used=0;
                if(s.empty()||s[0]=='-') throw std::invalid_argument("Invalid count");
                const auto x=std::stoull(s,&used); if(used!=s.size()||x>120000) throw std::invalid_argument("Invalid count");
                if(flag=="--ticks") { o.ticks=x; o.explicitTicks=true; }
                else if(flag=="--fps") o.fps=static_cast<unsigned>(x);
                else if(flag=="--pose") o.pose=static_cast<unsigned>(x);
                else if(flag=="--substeps") o.substeps=static_cast<unsigned>(x);
                else o.delay=static_cast<unsigned>(x);
            }
            else throw std::invalid_argument("Unknown contact option "+flag);
        }
        const std::array<std::string,10> cases{"cycle","normal-disturbance","small-tangent","slip","lost","absent","foreign","weak","cancel","disable"};
        if(std::find(cases.begin(),cases.end(),o.scenario)==cases.end()) throw std::invalid_argument("Unknown contact case");
        if(!o.ticks||(o.scenario=="disable"&&o.ticks<25)||(o.fps!=30&&o.fps!=60&&o.fps!=144)||o.pose>1||o.delay>32||
            (o.substeps!=8&&o.substeps!=16&&o.substeps!=32)||o.payload<0||o.payload>1||
            o.offset<-.01||o.offset>.02||o.friction<.01||o.friction>1||o.softness<.004||o.softness>.02)
            throw std::invalid_argument("Contact option outside experiment domain");
        return o;
    }
    inline Coordinates initial(unsigned pose)
    { return pose==0?Coordinates{.45,.18,-.1,.65,.12,.08}:Coordinates{.35,.22,.08,.55,.05,-.04}; }
    inline Definition description(const ArmRig& rig,const Options& o)
    {
        if(rig.chain()!=ArmChain::Full) throw std::invalid_argument("Managed contact requires Full arm");
        auto def=rig.description(1,1,o.payload);
        auto& hand=def.links[2];
        const auto centre=hand.capsuleEnd;
        hand.capsuleStart=hand.capsuleEnd=centre; hand.radius=.018;
        ContactTool tool; tool.pointInJoint=centre; tool.radius=hand.radius;
        tool.friction=o.scenario=="slip"?.05:o.friction; def.contactTool=tool;
        def.substeps=o.substeps; def.contactTimeConstant=o.softness;
        const auto frame=rig.visualFrames(rig.visualPose(initial(o.pose)))[2];
        const auto p=glm::dvec3{frame*glm::dvec4{centre,1}};
        const double gap=o.scenario=="absent"?.25:.028+o.offset;
        def.wall=Wall{p+glm::dvec3{0,0,tool.radius+gap+.02},{.12,.12,.02}};
        if(o.scenario=="foreign")
        {
            const auto frames=rig.visualFrames(rig.visualPose(initial(o.pose)));
            const auto& upper=def.links[0];
            const auto a=glm::dvec3{frames[0]*glm::dvec4{upper.capsuleStart,1}};
            const auto b=glm::dvec3{frames[0]*glm::dvec4{upper.capsuleEnd,1}};
            const auto tip=a.z<b.z?a:b;
            def.wall=Wall{tip+glm::dvec3{0,0,-upper.radius-.0198},{.005,.005,.02}};
        }
        if(o.scenario=="weak") for(auto& d:def.drives) d.maxTorque=.35;
        return def;
    }
    inline ContactSettings settings(const Options& o)
    { ContactSettings s; s.delaySamples=o.delay; s.holdSeconds=o.scenario=="disable"?10:3.5; if(!o.feedback) s.forceP=s.forceI=0; return s; }
    inline void reset(PhysicsWorld& world,const Options& o)
    {
        world.reset(initial(o.pose)); std::string reason;
        if(!world.startContact(settings(o),&reason)) throw std::invalid_argument(reason);
    }
    // Scenario event decisions use completed states and fixed simulation times.
    // Per-run event state is derived from the goal/failure, never a global latch.
    inline void step(PhysicsWorld& world,const Options& o)
    {
        const auto& s=world.snapshot(); const auto& c=s.contact;
        if(o.scenario=="disable"&&s.tick+1==o.ticks-24) world.command(CommandKind::Disable);
        if(c.phase==ContactPhase::Hold&&c.goalForce==3&&c.phaseSeconds>=2.2)
        {
            if(o.scenario=="cancel") world.cancelContact();
            else world.contactForceTarget(6);
        }
        if(c.phase==ContactPhase::Hold&&c.phaseSeconds>=.8&&c.phaseSeconds<=1.3)
        {
            if(o.scenario=="small-tangent") world.applyWrench(2,{.5,0,0});
            if(o.scenario=="slip") world.applyWrench(2,{4,0,0});
            if(o.scenario=="lost") world.applyWrench(2,{0,0,-12});
        }
        if(o.scenario=="normal-disturbance"&&c.phase==ContactPhase::Hold&&c.phaseSeconds>=.8&&c.phaseSeconds<=2)
            world.applyWrench(2,{0,0,1}); // Unknown normal load, not supplied to the controller.
        world.step();
    }
    inline void metadata(const std::filesystem::path& dir,const ArmRig& rig,const PhysicsWorld& world,const Options& o)
    {
        bench::metadata(dir,rig,world);
        const auto& def=world.definition(); const auto& t=*def.contactTool; const auto s=settings(o);
        const auto compiled=*world.contactMaterial(); const auto& hand=def.links[t.link];
        std::ofstream out{dir/"contact-setup.txt"}; out<<std::setprecision(17);
        out<<"stage7e fixed Full arm, SI, MuJoCo "<<world.backendVersion()<<"\n";
        out<<"tool=g"<<t.link<<" sphere radius_m="<<t.radius<<" joint_from_tool_translation_m="<<t.pointInJoint.x<<','<<t.pointInJoint.y<<','<<t.pointInJoint.z<<" rotation=identity\n";
        out<<"proxy replaces hand collider; explicit hand mass_kg="<<hand.mass<<" principal_inertia_kg_m2="<<hand.inertia.x<<','<<hand.inertia.y<<','<<hand.inertia.z<<" (no collider mass added)\n";
        out<<"pair_compiled_geom1="<<compiled.geom1<<" geom2="<<compiled.geom2<<" dimension="<<compiled.dimension<<" friction=";
        for(auto x:compiled.friction) out<<x<<' '; out<<" solref="; for(auto x:compiled.solref) out<<x<<' ';
        out<<" solimp="; for(auto x:compiled.solimp) out<<x<<' '; out<<'\n';
        out<<"external_Hz=120 control_Hz="<<120*def.substeps<<" substep_s="<<TickSeconds/def.substeps<<" feedback_extra_samples="<<s.delaySamples<<" filter_s="<<s.filterSeconds<<'\n';
        out<<"force_goals_N=3,6 force_rate_N_s="<<s.forceRate<<" P="<<s.forceP<<" I_per_s="<<s.forceI<<" integral_bound_N_s="<<s.integralLimit<<'\n';
        out<<"free_m="<<s.freeTravel<<" free_s="<<s.freeSeconds<<" search_speed_m_s="<<s.searchSpeed<<" search_limit_m="<<s.searchTravel<<" search_limit_s="<<s.searchSeconds<<'\n';
        out<<"confirm_N="<<s.confirmForce<<" loss_N="<<s.lossForce<<" confirm_s="<<s.confirmSeconds<<" loss_s="<<s.lossSeconds<<'\n';
        out<<"normal_damping_N_s_m="<<s.normalDamping<<" tangent_K_N_m="<<s.tangentK<<" tangent_D_N_s_m="<<s.tangentD<<" posture_K_Nm_rad="<<s.postureK<<" posture_D_Nm_s_rad="<<s.postureD<<'\n';
        out<<"hold_tolerance_N="<<s.forceTolerance<<" good_dwell_s="<<s.holdDwell<<" hold_s="<<s.holdSeconds<<" settling_s="<<s.settleSeconds<<" penetration_limit_m="<<s.maxPenetration<<" normal_speed_limit_m_s="<<s.maxNormalSpeed<<'\n';
        out<<"saturation_timeout_s="<<s.saturationSeconds<<" retreat_m="<<s.retreatDistance<<" retreat_s="<<s.retreatSeconds<<'\n';
    }
    struct Log
    {
        bench::Log legacy;
        std::ofstream contact,points,timing;
        explicit Log(const std::filesystem::path& dir):legacy(dir),contact(dir/"contact-v1.csv"),points(dir/"contact-points-v1.csv"),timing(dir/"timing.csv")
        {
            contactCsvHeader(contact); contactPointsCsvHeader(points);
            timing<<"tick,tick_us,control_us,step_us,read_us,drive_telemetry_us,contact_us,contact_control_us\n";
        }
        void row(const RenderSnapshot& view,const Definition& def)
        {
            legacy.row(view,def); contactCsvRow(contact,view.physics); contactPointsCsvRow(points,view.physics);
            const auto& t=view.physics.timing;
            timing<<std::setprecision(17)<<view.physics.tick<<','<<t.controlUs+t.stepUs+t.readUs+t.telemetryUs<<','
                <<t.controlUs<<','<<t.stepUs<<','<<t.readUs<<','<<t.telemetryUs<<','<<t.contactUs<<','<<t.contactControlUs<<'\n';
        }
    };
}
