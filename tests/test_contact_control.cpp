#include "julia_test.hpp"
#include "../apps/julia_arm_physics/ContactBench.hpp"
#include "julia/physics/BodyRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include <chrono>
#include <limits>
#include <numeric>

using namespace julia::physics;
namespace
{
    void thresholdFixture(std::ostream& report)
    {
        ContactController controller; ContactSettings settings; settings.freeSeconds=.5;
        ContactInput in; const double h=TickSeconds/8;
        controller.start(settings,{0,0,-1},in);
        const auto sample=[&](double force,unsigned count) {
            for(unsigned k=0;k<count;++k) {
                controller.evaluate(in,h); controller.applied(false,h);
                in.time+=h; in.pair.normalForce=force; in.pair.active=force>0?1u:0u; controller.observe(in,h);
            }
        };
        sample(0,500); JULIA_CHECK(controller.status().phase==ContactPhase::Search);
        sample(.7,10); sample(.4,100);
        JULIA_CHECK(!controller.status().confirmed&&controller.status().confirmationSeconds==0&&controller.status().phase==ContactPhase::Confirm);
        sample(.7,38); JULIA_CHECK(!controller.status().confirmed);
        sample(.7,1); JULIA_CHECK(controller.status().confirmed&&controller.status().phase==ContactPhase::Ramp);
        sample(0,76); JULIA_CHECK(controller.status().failure==ContactFailure::None);
        sample(0,1); JULIA_CHECK(controller.status().failure==ContactFailure::ContactLost&&controller.status().integral==0);
        report<<"threshold fixture: .4N between hysteresis levels does not confirm; .7N requires39 samples; zero force loss requires77 samples at960Hz\n";
    }
    Definition fixture(bool sphere=false,unsigned substeps=8)
    {
        Definition d; d.freeRoot=true; d.floorHeight=0; d.substeps=substeps;
        Link b; b.name="independent free contact fixture"; b.parent=-1;
        b.parentFromJoint[3].y=.051;
        b.mass=2; b.radius=.05; b.inertia={.002,.002,.002};
        if(!sphere) { b.boxHalfSize=glm::dvec3{.1,.05,.08}; b.inertia={.0059333333333333,.0109333333333333,.0083333333333333}; }
        d.links.push_back(b);
        ContactTool t; t.link=0; t.surface="floor"; t.normalWorld={0,1,0}; t.radius=.05; t.friction=.3;
        d.contactTool=t;
        return d;
    }
    void fixtures(const std::filesystem::path& dir,std::ostream& report)
    {
        PhysicsWorld w{fixture()}; RootState r; r.position={0,.051,0};
        w.reset({},r);
        for(unsigned i=0;i<360;++i) w.step();
        JULIA_CHECK(std::abs(w.snapshot().contact.pair.normalPower)+std::abs(w.snapshot().contact.pair.tangentPower)<1e-7);
        report<<"fixture zero-lateral static normal_W="<<w.snapshot().contact.pair.normalPower
            <<" tangent_W="<<w.snapshot().contact.pair.tangentPower<<'\n';
        for(unsigned i=0;i<360;++i) { w.applyWrench(0,{1,0,2}); w.step(); }
        const auto& s=w.snapshot(); const auto& p=s.contact.pair;
        JULIA_CHECK(p.active==4);
        JULIA_CHECK(glm::length(p.forceOnToolWorld-glm::dvec3{-1,19.62,-2})<1e-7);
        JULIA_CHECK(std::abs(p.normalForce-19.62)<1e-7);
        JULIA_CHECK(std::abs(p.friction-.3)<1e-12);
        auto reversed=aggregateContacts(s.contacts,p.surfaceGeom,p.toolGeom,{0,-1,0});
        JULIA_CHECK(glm::length(reversed.forceOnToolWorld+p.forceOnToolWorld)<1e-10);
        JULIA_CHECK(std::abs(reversed.normalForce-p.normalForce)<1e-10);
        JULIA_CHECK(std::abs(reversed.tangentPower-p.tangentPower)<1e-10);
        auto none=aggregateContacts(s.contacts,p.toolGeom,999,{0,1,0});
        JULIA_CHECK(none.normalForce==0&&none.active==0&&none.foreignActive==4);
        double orthogonalError=0;
        for(const auto& c:s.contacts)
        {
            const auto gram=glm::transpose(c.worldFromContact)*c.worldFromContact;
            for(int col=0;col<3;++col) for(int row=0;row<3;++row)
                orthogonalError=std::max(orthogonalError,std::abs(gram[col][row]-(col==row?1.:0.)));
            JULIA_CHECK(glm::length(c.forceOn2World-c.worldFromContact**c.force)<1e-12);
        }
        JULIA_CHECK(orthogonalError<1e-12);
        report<<"fixture loaded equilibrium (soft-friction creep) points="<<p.active<<" force_world="<<p.forceOnToolWorld.x<<','<<p.forceOnToolWorld.y<<','<<p.forceOnToolWorld.z
            <<" normal_W="<<p.normalPower<<" tangent_W="<<p.tangentPower<<" basis_error="<<orthogonalError<<'\n';
        std::ofstream points{dir/"fixture-static-points.csv"}; contactPointsCsvHeader(points); contactPointsCsvRow(points,s);
        std::array<double,3> work{}; unsigned variant=0;
        for(unsigned sub:{8u,16u,32u})
        {
            PhysicsWorld slide{fixture(true,sub)};
            RootState root; root.position={0,.0499,0}; root.linearVelocityWorld={.4,0,.2}; root.angularVelocityLocal={0,2,1};
            slide.reset({},root);
            double virtualPowerError=0,velocityError=0;
            std::ofstream csv{dir/("fixture-slide-"+std::to_string(sub)+".csv")}; contactCsvHeader(csv);
            for(unsigned tick=0;tick<60;++tick)
            {
                slide.step(); const auto& a=slide.snapshot(); const auto& pair=a.contact.pair;
                virtualPowerError=std::max(virtualPowerError,std::abs(pair.normalPower+pair.tangentPower-a.constraintPower));
                const auto omega=glm::dmat3{rootTransform(a.root)}*a.root.angularVelocityLocal;
                for(const auto& c:a.contacts)
                {
                    const auto expected=a.root.linearVelocityWorld+glm::cross(omega,c.point-a.root.position);
                    const auto actual=c.link2==0?c.velocity2World:c.velocity1World;
                    velocityError=std::max(velocityError,glm::length(expected-actual));
                }
                JULIA_CHECK(pair.tangentPower<=1e-9);
                contactCsvRow(csv,a);
            }
            const auto& end=slide.snapshot();
            JULIA_CHECK(end.contact.tangentWork<-.001&&end.contact.slipDistance>.001);
            JULIA_CHECK(virtualPowerError<1e-8&&velocityError<1e-10);
            work[variant++]=end.contact.tangentWork;
            report<<"fixture slide sub="<<sub<<" tangent_J="<<end.contact.tangentWork<<" normal_J="<<end.contact.normalWork
                <<" virtual_power_error_W="<<virtualPowerError<<" point_velocity_error="<<velocityError<<'\n';
        }
        JULIA_CHECK(std::abs(work[2]-work[1])<std::abs(work[1]-work[0]));
        // Body COM differs from the free-joint origin: this independently catches
        // confusion between BODY/inertial-COM and XBODY/origin velocities.
        auto offsetDef=fixture(true); offsetDef.links[0].comInSegment={.01,.005,-.008};
        PhysicsWorld offset{offsetDef}; RootState offsetRoot;
        offsetRoot.position={0,.0499,0}; offsetRoot.linearVelocityWorld={.4,0,.2}; offsetRoot.angularVelocityLocal={.3,2,1};
        offset.reset({},offsetRoot); double offsetError=0; unsigned checked=0;
        for(unsigned tick=0;tick<12;++tick)
        {
            offset.step(); const auto& a=offset.snapshot();
            const auto omega=glm::dmat3{rootTransform(a.root)}*a.root.angularVelocityLocal;
            for(const auto& c:a.contacts)
            {
                const auto expected=a.root.linearVelocityWorld+glm::cross(omega,c.point-a.root.position);
                offsetError=std::max(offsetError,glm::length(expected-(c.link2==0?c.velocity2World:c.velocity1World))); ++checked;
            }
            JULIA_CHECK(std::abs(a.contact.pair.normalPower+a.contact.pair.tangentPower-a.constraintPower)<1e-8);
        }
        JULIA_CHECK(checked>0&&offsetError<1e-10);
        report<<"fixture offset-COM point velocities checked="<<checked<<" max_error_m_s="<<offsetError<<'\n';
    }
    std::string record(const Snapshot& s)
    {
        auto copy=s; copy.epoch=0;
        std::ostringstream o; driveCsvRow(o,copy,8); contactCsvRow(o,copy); contactPointsCsvRow(o,copy); return o.str();
    }
    void commands(const julia::model::ModelAsset& asset,std::ostream& report)
    {
        ArmRig rig{asset,ArmSide::Left,ArmChain::Full}; contact_bench::Options o;
        PhysicsWorld w{contact_bench::description(rig,o)}; contact_bench::reset(w,o);
        for(unsigned i=0;i<60;++i) w.step();
        const auto before=record(w.snapshot()); std::string why;
        auto bad=ContactSettings{}; bad.force=std::numeric_limits<double>::quiet_NaN();
        JULIA_CHECK(!w.startContact(bad,&why)&&!why.empty());
        for(unsigned i=0;i<7;++i)
        {
            auto invalid=ContactSettings{};
            if(i==0) invalid.forceRate=std::numeric_limits<double>::infinity();
            if(i==1) invalid.searchTravel=1e-300;
            if(i==2) invalid.filterSeconds=0;
            if(i==3) invalid.delaySamples=33;
            if(i==4) invalid.lossForce=invalid.confirmForce;
            if(i==5) invalid.holdSeconds=.01;
            if(i==6) invalid.retreatSeconds=0;
            JULIA_CHECK(!w.startContact(invalid,&why)&&!why.empty());
        }
        JULIA_CHECK(!w.contactForceTarget(9,&why)&&!why.empty());
        JULIA_CHECK(!w.contactForceTarget(std::numeric_limits<double>::infinity(),&why));
        JULIA_CHECK(!w.command(CommandKind::SmoothTarget,{.5,.2,0,.6,0,0},&why));
        JULIA_CHECK(!w.command(CommandKind::Torque,{1},&why));
        JULIA_CHECK(!w.configureServo({},&why));
        JULIA_CHECK(before==record(w.snapshot()));
        JULIA_CHECK(w.command(CommandKind::Disable)); w.step();
        JULIA_CHECK(w.snapshot().mode==DriveMode::Disabled&&w.snapshot().contact.phase==ContactPhase::Disabled);
        for(unsigned k=0;k<6;++k) JULIA_CHECK(w.snapshot().appliedTorque[k]==0);
        JULIA_CHECK(w.command(CommandKind::SmoothTarget,contact_bench::initial(0)));
        JULIA_CHECK(!w.startContact({},&why));
        JULIA_CHECK(w.command(CommandKind::Disable));
        contact_bench::reset(w,o);
        std::vector<std::string> first;
        for(unsigned i=0;i<700;++i) { contact_bench::step(w,o); first.push_back(record(w.snapshot())); }
        const auto end=w.snapshot();
        contact_bench::reset(w,o);
        JULIA_CHECK(w.snapshot().contact.normalWork==0&&w.snapshot().contact.tangentWork==0&&w.snapshot().contact.filteredForce==0);
        for(unsigned i=0;i<700;++i) { contact_bench::step(w,o); JULIA_CHECK(first[i]==record(w.snapshot())); }
        JULIA_CHECK(w.snapshot().q==end.q);
        w.cancelContact(); for(unsigned i=0;i<700;++i) w.step();
        JULIA_CHECK(w.snapshot().contact.failure==ContactFailure::Cancelled&&w.snapshot().contact.phase==ContactPhase::Complete);
        w.command(CommandKind::Disable); w.step(); const auto accumulated=w.snapshot().contact;
        JULIA_CHECK(w.startContact({}));
        JULIA_CHECK(w.snapshot().contact.normalWork==accumulated.normalWork&&w.snapshot().contact.tangentWork==accumulated.tangentWork);
        JULIA_CHECK(w.snapshot().contact.filteredForce==0&&w.snapshot().contact.integral==0);
        BodyRig body{asset}; PhysicsWorld free{body.description(false)};
        JULIA_CHECK(!free.startContact({},&why)&&!why.empty());
        JULIA_CHECK(!free.capabilities().normalForceControl&&w.capabilities().normalForceControl);
        PhysicsWorld two{ArmRig{asset}.description()}; JULIA_CHECK(!two.startContact({},&why));
        report<<"atomic commands, exclusive ownership, Disable, reset/replay, cancel, unsupported body/two tested\n";
    }
    void run(const julia::model::ModelAsset& asset,contact_bench::Options o,const std::filesystem::path& dir,std::ostream& report)
    {
        const std::string name=(o.side==ArmSide::Left?"left-":"right-")+o.scenario+"-p"+std::to_string(o.pose)+
            "-load"+std::to_string(o.payload)+"-off"+std::to_string(o.offset)+"-sub"+std::to_string(o.substeps)+
            "-tc"+std::to_string(o.softness)+"-delay"+std::to_string(o.delay)+"-feedback"+std::to_string(o.feedback);
        const auto dest=dir/name; std::filesystem::create_directories(dest);
        ArmRig rig{asset,o.side,ArmChain::Full}; PhysicsWorld w{contact_bench::description(rig,o),contact_bench::initial(o.pose)};
        const auto fk=verifyFramesBeforeDynamics(rig,w); JULIA_CHECK(fk.configurations==185);
        contact_bench::reset(w,o); contact_bench::metadata(dest,rig,w,o);
        const auto toolPoint=glm::dvec3{rig.visualFrames(rig.visualPose(contact_bench::initial(o.pose)))[2]*glm::dvec4{w.definition().contactTool->pointInJoint,1}};
        JULIA_CHECK(glm::length(toolPoint-w.snapshot().contact.toolPointWorld)<30e-6);
        const auto material=w.contactMaterial(); JULIA_CHECK(material&&material->dimension==3);
        JULIA_CHECK(std::abs(material->friction[0]-w.definition().contactTool->friction)<1e-12);
        std::ofstream csv{dest/"contact-v1.csv"},drive{dest/"drive-v1.csv"}; contactCsvHeader(csv); driveCsvHeader(drive);
        double error2=0,maxError=0,peakSlip=0,maxDisplacement=0,initialX=w.snapshot().contact.toolPointWorld.x;
        unsigned samples=0,goodGoals=0; double previousRef=w.snapshot().contact.referenceForce;
        double disturbanceError2=0; unsigned disturbanceSamples=0;
        std::array<bool,12> phases{};
        for(std::uint64_t i=0;i<o.ticks;++i)
        {
            contact_bench::step(w,o); const auto& s=w.snapshot(); const auto& c=s.contact;
            phases[static_cast<unsigned>(c.phase)]=true;
            if(c.phase==ContactPhase::Hold&&c.holdSucceeded) goodGoals|=c.goalForce==3?1u:2u;
            if(c.phase==ContactPhase::Hold&&c.phaseSeconds>=1.5)
            { const auto error=std::abs(c.pair.normalForce-c.goalForce); error2+=error*error; maxError=std::max(maxError,error); ++samples; }
            if(o.scenario=="normal-disturbance"&&c.phase==ContactPhase::Hold&&c.phaseSeconds>=1.7&&c.phaseSeconds<=1.95)
            { const auto e=c.pair.normalForce-c.goalForce; disturbanceError2+=e*e; ++disturbanceSamples; }
            if(c.phase!=ContactPhase::Fault&&c.phase!=ContactPhase::Disabled)
                JULIA_CHECK(std::abs(c.referenceForce-previousRef)<=3*TickSeconds+1e-10);
            previousRef=c.referenceForce;
            JULIA_CHECK(c.sampleFeedbackTime<=s.time-TickSeconds/o.substeps+1e-10);
            JULIA_CHECK(std::abs(c.pair.friction-w.definition().contactTool->friction)<1e-12);
            for(unsigned k=0;k<6;++k)
            {
                JULIA_CHECK(std::abs(s.appliedTorque[k])<=w.definition().drives[k].maxTorque+1e-12);
                JULIA_CHECK(!s.drive.referenceReached[k]);
            }
            peakSlip=std::max(peakSlip,c.pair.slipSpeed);
            maxDisplacement=std::max(maxDisplacement,std::abs(c.toolPointWorld.x-initialX));
            contactCsvRow(csv,s); driveCsvRow(drive,s,o.substeps);
        }
        const auto& c=w.snapshot().contact;
        const double rms=samples?std::sqrt(error2/samples):0;
        report<<name<<" phase="<<contactPhaseName(c.phase)<<" failure="<<contactFailureName(c.failure)<<" goals="<<goodGoals
            <<" samples="<<samples<<" RMS_N="<<rms<<" max_error_N="<<maxError<<" peak_N="<<c.peakForce<<" pen_m="<<c.peakPenetration
            <<" peak_slip_m_s="<<peakSlip<<" dx_m="<<maxDisplacement<<" tangent_J="<<c.tangentWork<<" search_m="<<c.searchDisplacement<<'\n';
        if(o.scenario=="normal-disturbance")
        {
            const auto disturbedRms=disturbanceSamples?std::sqrt(disturbanceError2/disturbanceSamples):0;
            report<<"feedback_comparison enabled="<<o.feedback<<" samples="<<disturbanceSamples<<" normal_disturbance_RMS_N="<<disturbedRms<<'\n';
            JULIA_CHECK(disturbanceSamples>=40);
            if(o.feedback) JULIA_CHECK(disturbedRms<=.25&&c.failure==ContactFailure::None);
            else JULIA_CHECK(disturbedRms>=.8);
        }
        report.flush();
        if(o.scenario=="cycle")
        {
            JULIA_CHECK(c.phase==ContactPhase::Complete&&c.failure==ContactFailure::None&&goodGoals==3);
            JULIA_CHECK(samples>=120&&rms<=.5&&maxError<=1&&c.peakForce<=10&&c.peakPenetration<=.002);
            for(auto phase:{ContactPhase::FreeMotion,ContactPhase::Search,ContactPhase::Confirm,ContactPhase::Ramp,ContactPhase::Hold,ContactPhase::Unload,ContactPhase::Retreat})
                JULIA_CHECK(phases[static_cast<unsigned>(phase)]);
        }
        if(o.scenario=="absent") JULIA_CHECK(c.phase==ContactPhase::Fault&&c.failure==ContactFailure::SearchLimit&&!c.confirmed&&!c.holdSucceeded&&c.searchDisplacement<=.0605);
        if(o.scenario=="lost") JULIA_CHECK(c.phase==ContactPhase::Fault&&c.failure==ContactFailure::ContactLost&&!c.holdSucceeded);
        if(o.scenario=="foreign") JULIA_CHECK(c.phase==ContactPhase::Fault&&c.failure==ContactFailure::ForeignContact&&!c.holdSucceeded);
        if(o.scenario=="weak") JULIA_CHECK(c.phase==ContactPhase::Fault&&c.failure==ContactFailure::Saturation&&!c.holdSucceeded);
        if(o.scenario=="small-tangent") JULIA_CHECK(c.failure==ContactFailure::None&&peakSlip<=.005&&maxDisplacement<=.005);
        if(o.scenario=="slip") JULIA_CHECK(peakSlip>.01&&maxDisplacement>.005&&c.tangentWork<0);
        if(o.scenario=="cancel") JULIA_CHECK(c.phase==ContactPhase::Complete&&c.failure==ContactFailure::Cancelled);
        if(o.scenario=="disable") JULIA_CHECK(c.phase==ContactPhase::Disabled&&w.snapshot().mode==DriveMode::Disabled);
    }
    void performance(const julia::model::ModelAsset& asset,const std::filesystem::path& dir,std::ostream& report)
    {
        ArmRig rig{asset,ArmSide::Left,ArmChain::Full}; contact_bench::Options o;
        for(unsigned variant=0;variant<3;++variant)
        {
            auto def=contact_bench::description(rig,o);
            if(variant==0) def.contactTool.reset();
            PhysicsWorld w{def};
            if(variant==2) contact_bench::reset(w,o);
            else { w.reset(contact_bench::initial(0)); w.configureServo({.6,2,true}); w.command(CommandKind::SmoothTarget,contact_bench::initial(0)); }
            std::array<std::vector<double>,4> samples;
            std::ofstream csv{dir/("performance-"+std::to_string(variant)+".csv")}; csv<<"tick,tick_us,control_us,contact_us,contact_control_us\n";
            for(unsigned i=0;i<10000;++i)
            {
                const auto& c=w.snapshot().contact;
                if(variant==2&&c.phase==ContactPhase::Hold&&c.phaseSeconds>=2) w.contactForceTarget(c.goalForce==3?6:3);
                const auto begin=std::chrono::steady_clock::now(); w.step();
                const auto total=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
                const auto& t=w.snapshot().timing;
                samples[0].push_back(total); samples[1].push_back(t.controlUs); samples[2].push_back(t.contactUs); samples[3].push_back(t.contactControlUs);
                csv<<std::setprecision(17)<<i+1<<','<<total<<','<<t.controlUs<<','<<t.contactUs<<','<<t.contactControlUs<<'\n';
            }
            report<<"performance variant="<<variant<<" (servo/no selected pair, servo/pair observations, contact/pair observations)";
            for(auto& v:samples)
            { const auto mean=std::accumulate(v.begin(),v.end(),0.)/v.size(); std::sort(v.begin(),v.end()); report<<" mean/p95/max_us="<<mean<<'/'<<v[9500]<<'/'<<v.back(); }
            report<<'\n'; JULIA_CHECK(samples[0][9500]<8333);
            if(variant==2) JULIA_CHECK(w.snapshot().contact.failure==ContactFailure::None&&w.snapshot().contact.confirmed);
        }
    }
}
int main(int argc,char** argv)
{
    try
    {
        const auto dir=std::filesystem::path{argc>1?argv[1]:"build/diagnostics/stage7e/cpu"}; std::filesystem::create_directories(dir);
        const std::string mode=argc>2?argv[2]:"all";
        if(argc>3||(mode!="all"&&mode!="--performance-only"&&mode!="--fixtures-only")) throw std::invalid_argument("Expected output directory and optional --performance-only|--fixtures-only");
        std::ofstream report{dir/"contact-control.txt"}; report<<std::setprecision(17);
        if(mode!="--performance-only") { thresholdFixture(report); fixtures(dir,report); }
        if(mode=="--fixtures-only") { report<<"failures="<<JULIA_TEST_RESULT()<<'\n'; return JULIA_TEST_RESULT(); }
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        if(mode!="--performance-only") {
        commands(*asset,report);
        for(const auto side:{ArmSide::Left,ArmSide::Right})
        {
            for(unsigned pose=0;pose<2;++pose) for(double offset:{0.,.01,-.005})
            { contact_bench::Options o; o.side=side; o.pose=pose; o.offset=offset; run(*asset,o,dir,report); }
            for(const auto& scenario:{"small-tangent","slip","lost","absent","foreign","weak","cancel","disable"})
            { contact_bench::Options o; o.side=side; o.scenario=scenario; run(*asset,o,dir,report); }
            for(bool feedback:{true,false})
            { contact_bench::Options o; o.side=side; o.scenario="normal-disturbance"; o.feedback=feedback; run(*asset,o,dir,report); }
            contact_bench::Options load; load.side=side; load.payload=.5; run(*asset,load,dir,report);
            for(unsigned variant=0;variant<3;++variant)
            { contact_bench::Options o; o.side=side; if(variant==0) o.substeps=16; if(variant==1) o.softness=.012; if(variant==2) o.delay=4; run(*asset,o,dir,report); }
        }
        }
        performance(*asset,dir,report);
        report<<"failures="<<JULIA_TEST_RESULT()<<'\n';
        std::cout<<"contact_control failures="<<JULIA_TEST_RESULT()<<'\n'; return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"CONTACT TEST EXCEPTION: "<<e.what()<<'\n'; return 1; }
}
