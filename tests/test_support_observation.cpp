#include "julia_test.hpp"
#include "../apps/julia_body_physics/BodyBenchIO.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <chrono>
#include <numeric>
#include <sstream>

using namespace julia::physics;
namespace
{
    void analytic(std::ostream& report)
    {
        std::vector<Contact> contacts;
        for(int i=0;i<3;++i)
        {
            Contact c; c.geom1=91; c.geom2=17; c.link2=2; c.point={.1*i,-.001*i,.03*i};
            c.force=glm::dvec3{100+50*i,2-i,3+i}; c.active=true;
            c.forceOn2World={2.-i,100.+50*i,3.+i}; c.torqueOn2World={.01*i,.02*i,-.03*i}; contacts.push_back(c);
        }
        const glm::dvec3 origin{.7,.1,-.3}; const auto w=pairWrench(contacts,17,91,origin);
        glm::dvec3 sum{0},moment{0};
        for(const auto& c:contacts) { sum+=c.forceOn2World; moment+=glm::cross(c.point-origin,c.forceOn2World)+c.torqueOn2World; }
        JULIA_CHECK(glm::length(w.force-sum)<1e-10&&glm::length(w.moment-moment)<1e-10);
        const auto plane=w.about({0,0,0}); const auto cop=pressureCenter(w,{0,0,0});
        JULIA_CHECK(cop.point&&glm::length(*cop.point-glm::dvec3{plane.moment.z/sum.y,0,-plane.moment.x/sum.y})<1e-10);
        const auto shifted=pairWrench(contacts,17,91,{-1,2,.5});
        JULIA_CHECK(glm::length(shifted.moment-w.about({-1,2,.5}).moment)<1e-10);
        const auto opposite=pairWrench(contacts,91,17,origin);
        JULIA_CHECK(glm::length(opposite.force+w.force)+glm::length(opposite.moment+w.moment)<1e-10);
        for(auto& c:contacts) { std::swap(c.geom1,c.geom2); std::swap(c.link1,c.link2); c.forceOn2World=-c.forceOn2World; c.torqueOn2World=-c.torqueOn2World; }
        const auto reversed=pairWrench(contacts,17,91,origin);
        JULIA_CHECK(glm::length(reversed.force-w.force)+glm::length(reversed.moment-w.moment)<1e-10);
        JULIA_CHECK(!pressureCenter(Wrench{},{0,0,0}).point);
        Wrench small; small.force.y=19.99; JULIA_CHECK(!pressureCenter(small,{0,0,0}).point);
        small.force.y=20; JULIA_CHECK(pressureCenter(small,{0,0,0}).point.has_value());
        JULIA_CHECK(horizontalHull({}).kind==RegionKind::Empty);
        JULIA_CHECK(horizontalHull({{1,0,2},{1,0,2}}).kind==RegionKind::Point);
        JULIA_CHECK(horizontalHull({{0,0,0},{1,0,1},{2,0,2}}).kind==RegionKind::Segment);
        JULIA_CHECK(horizontalHull({{0,0,0},{1,0,0},{.5,0,1e-10}}).kind==RegionKind::Segment);
        const auto square=horizontalHull({{1,0,1},{0,0,1},{1,0,0},{0,0,0},{.5,0,.5}});
        JULIA_CHECK(square.kind==RegionKind::Polygon&&std::abs(square.area-1)<1e-10);
        LoadState load; SupportSettings settings; const double h=TickSeconds/16;
        for(unsigned i=0;i<38;++i) load.observe(30,h,settings);
        JULIA_CHECK(!load.loaded); load.observe(20,h,settings); JULIA_CHECK(load.confirmationSeconds==0);
        for(unsigned i=0;i<39;++i) load.observe(30,h,settings);
        JULIA_CHECK(load.loaded);
        for(unsigned i=0;i<76;++i) load.observe(10,h,settings);
        JULIA_CHECK(load.loaded); load.observe(10,h,settings); JULIA_CHECK(!load.loaded);
        report<<"analytic force/moment/shift/CoP/pair reversal <=1e-10; empty/point/line/almost-line/polygon; 38/39 confirm and76/77 loss samples PASS\n";
    }
    Definition fixture(unsigned sub=16)
    {
        Definition d; d.freeRoot=true; d.floorHeight=0; d.substeps=sub; d.supportObservation=SupportSettings{};
        Link root; root.name="known 55kg fixture"; root.segment="FixtureRoot"; root.parent=-1; root.mass=53.4;
        root.comInSegment={0,.12,0}; root.capsuleStart=root.capsuleEnd=root.comInSegment; root.radius=.04; root.inertia={.6,.6,.6};
        root.parentFromJoint[3].y=.051; d.links.push_back(root);
        for(int side=0;side<2;++side)
        {
            Link foot; foot.name=foot.segment=side==0?"LeftFoot":"RightFoot"; foot.parent=0; foot.mass=.8;
            foot.parentFromJoint[3].x=side==0?-.2:.2; foot.boxHalfSize=glm::dvec3{.1,.05,.15}; foot.inertia={.006666666666666667,.008666666666666666,.003333333333333334};
            d.links.push_back(foot);
            d.supportObservation->feet[static_cast<std::size_t>(side)]={foot.segment,"g"+std::to_string(side+1),glm::translate(glm::dmat4{1},glm::dvec3{0,-.05,0})};
        }
        return d;
    }
    void fixtures(const std::filesystem::path& dir,std::ostream& report)
    {
        PhysicsWorld world{fixture()}; for(unsigned n=0;n<360;++n) world.step();
        const auto s=world.snapshot(); const auto& o=s.support;
        JULIA_CHECK(std::abs(s.totalMass-55)<1e-10);
        for(const auto& f:o.feet)
        {
            JULIA_CHECK(f.load.loaded&&f.active==4&&std::abs(f.normalForce-55*9.81/2)<.05);
            JULIA_CHECK(f.cop.point&&f.loadedRegion.kind==RegionKind::Polygon);
        }
        JULIA_CHECK(std::abs(o.feet[0].normalForce+o.feet[1].normalForce-55*9.81)<.01);
        JULIA_CHECK(o.combinedCop.point&&glm::length(*o.combinedCop.point-o.comProjection)<1e-4);
        report<<"static55kg left_N="<<o.feet[0].normalForce<<" right_N="<<o.feet[1].normalForce<<" CoP_COM_m="<<glm::length(*o.combinedCop.point-o.comProjection)<<'\n';
        auto asymmetric=fixture(); asymmetric.links[0].comInSegment.x=.04;
        asymmetric.links[0].capsuleStart=asymmetric.links[0].capsuleEnd=asymmetric.links[0].comInSegment;
        asymmetric.links[1].boxHalfSize->x=asymmetric.links[2].boxHalfSize->x=1e-5;
        PhysicsWorld narrow{asymmetric}; for(unsigned n=0;n<360;++n) narrow.step();
        const auto& ns=narrow.snapshot(); const double expectedRight=55*9.81*(.2+ns.totalComWorld.x)/.4;
        JULIA_CHECK(std::abs(ns.support.feet[1].normalForce-expectedRight)<.05);
        JULIA_CHECK(std::abs(ns.support.feet[0].normalForce-(55*9.81-expectedRight))<.05);
        report<<"asymmetric narrow bars: measured_right_N="<<ns.support.feet[1].normalForce<<" ideal_zero_width_right_N="<<expectedRight<<" finite_width_force_bound_N="<<55*9.81*1e-5/.4<<'\n';
        std::ofstream points{dir/"fixture-static-points-v1.csv"}; contactPointsCsvHeader(points); contactPointsCsvRow(points,s);
        std::ofstream supports{dir/"fixture-static-support-v1.csv"}; supportCsvHeader(supports); supportCsvRow(supports,s);
        for(double yaw:{0.,.73})
        {
            RootState root; root.position={1.2,.051,-.7}; root.orientation=glm::angleAxis(yaw,glm::dvec3{0,1,0}); world.reset({},root);
            for(unsigned n=0;n<360;++n) world.step();
            const auto& a=world.snapshot(); JULIA_CHECK(a.support.combinedCop.point&&glm::length(*a.support.combinedCop.point-a.support.comProjection)<1e-4);
            JULIA_CHECK(std::abs(a.support.feet[0].normalForce-269.775)<.05&&std::abs(a.support.feet[1].normalForce-269.775)<.05);
        }
        auto order=fixture(); order.links[0].mass-=1;
        Link dummy; dummy.name="DFS order perturbation"; dummy.segment="Dummy"; dummy.parent=1; dummy.mass=1;
        dummy.parentFromJoint[3]={.2,.5,0,1}; dummy.capsuleStart=dummy.capsuleEnd={0,0,0}; dummy.radius=.02; dummy.inertia={.001,.001,.001};
        order.links.push_back(dummy); PhysicsWorld reordered{order}; for(unsigned n=0;n<360;++n) reordered.step();
        const auto& remapped=reordered.snapshot().support;
        JULIA_CHECK(remapped.feet[1].link==2&&remapped.feet[1].geom!=2);
        JULIA_CHECK(remapped.feet[0].active==4&&remapped.feet[1].active==4);
        auto wrong=fixture(); wrong.supportObservation->feet[0].geom="g2";
        bool rejected=false; try { PhysicsWorld invalid{wrong}; } catch(const std::invalid_argument&) { rejected=true; }
        JULIA_CHECK(rejected);
        report<<"semantic/compiled name lookup with changed DFS geom order and mismatched binding rejection PASS\n";
        // Classification is verified with independent foreign/self records; body
        // dynamics below additionally exercises real nonfoot and self contacts.
        auto synthetic=s;
        Contact other=s.contacts.front(); other.geom1=80; other.link1=-1; other.geom2=90; other.link2=0;
        Contact self=other; self.geom1=o.feet[0].geom; self.link1=o.feet[0].link; self.geom2=o.feet[1].geom; self.link2=o.feet[1].link;
        synthetic.contacts.push_back(other); synthetic.contacts.push_back(self);
        observeSupports(synthetic.support,synthetic,*world.definition().supportObservation,{1,2},{o.feet[0].geom,o.feet[1].geom},o.feet[0].surfaceGeom,0,
            {glm::dmat4{1},glm::dmat4{1}},{{.1,.05,.15},{.1,.05,.15}},0);
        JULIA_CHECK(synthetic.support.otherEnvironmentActive==1&&synthetic.support.selfActive==1);
        JULIA_CHECK(synthetic.support.feet[0].active==4&&synthetic.support.feet[1].active==4);
        auto one=fixture(); one.links[2].parentFromJoint[3].y=.25;
        PhysicsWorld oneFoot{one}; for(unsigned n=0;n<12;++n) oneFoot.step();
        JULIA_CHECK(oneFoot.snapshot().support.feet[0].active>0&&oneFoot.snapshot().support.feet[1].active==0);
        JULIA_CHECK(!oneFoot.snapshot().support.feet[1].cop.point);
        RootState high; high.position={0,3,0}; high.linearVelocityWorld={.2,.1,-.3};
        world.reset({},high); const auto initial=world.snapshot();
        for(unsigned n=0;n<60;++n)
        {
            // Proportional loads pass through total COM; a single load at root
            // COM would introduce a real moment and rotational integration error.
            for(unsigned link=0;link<3;++link) world.applyWrench(link,glm::dvec3{100,0,20}*(world.definition().links[link].mass/55));
            world.step();
        }
        const auto& free=world.snapshot(); JULIA_CHECK(glm::length(free.support.dynamics.linearResidual)<1e-8&&glm::length(free.support.dynamics.angularResidual)<1e-8);
        JULIA_CHECK(!free.support.feet[0].cop.point&&!free.support.feet[0].near&&free.support.feet[0].normalForce==0);
        const auto independent=(free.totalComWorld-initial.totalComWorld)/.5;
        // Constant acceleration: average velocity agrees with the average
        // endpoint velocity up to first-order semi-implicit position error.
        JULIA_CHECK(glm::length(free.support.dynamics.comVelocity-free.linearMomentumWorld/free.totalMass)<1e-10);
        report<<"free applied+gravity residual_Ns="<<glm::length(free.support.dynamics.linearResidual)<<" residual_Nms="<<glm::length(free.support.dynamics.angularResidual)<<" average_v="<<glm::length(independent)<<'\n';
        world.reset({},high); world.applyWrench(1,{0,0,10}); world.applyWrench(2,{0,0,-10}); world.step();
        const auto& couple=world.snapshot().support.dynamics;
        JULIA_CHECK(glm::length(couple.applied.force)<1e-10&&std::abs(couple.applied.moment.y-4)<.001);
        report<<"explicit equal/opposite loads counted once: net_force="<<glm::length(couple.applied.force)<<" couple_y_Nm="<<couple.applied.moment.y<<'\n';
        // Offset COM and rotating root, exact analytic point velocity and an
        // independent central difference of mass-weighted scratch FK COM.
        auto rotating=fixture(); rotating.gravity={0,0,0}; rotating.floorHeight.reset();
        PhysicsWorld rotation{rotating}; high.angularVelocityLocal={.3,2,1}; rotation.reset({},high); rotation.step();
        const auto a=rotation.snapshot(); const double eps=1e-6;
        auto minus=a.root,plus=a.root; minus.position-=eps*a.root.linearVelocityWorld; plus.position+=eps*a.root.linearVelocityWorld;
        const auto axis=glm::normalize(a.root.angularVelocityLocal); const double angle=eps*glm::length(a.root.angularVelocityLocal);
        minus.orientation=minus.orientation*glm::angleAxis(-angle,axis); plus.orientation=plus.orientation*glm::angleAxis(angle,axis);
        const auto fd=(rotation.massAudit({},plus).comWorld-rotation.massAudit({},minus).comWorld)/(2*eps);
        JULIA_CHECK(glm::length(fd-a.support.dynamics.comVelocity)<1e-5);
        JULIA_CHECK(glm::length(a.support.dynamics.comVelocityJacobian-a.support.dynamics.comVelocity)<1e-10);
        std::array<double,3> residual{}; unsigned variant=0;
        for(unsigned sub:{16u,32u,64u})
        {
            PhysicsWorld slide{fixture(sub)}; RootState r; r.position={0,.0499,0}; r.linearVelocityWorld={.8,0,.1}; r.angularVelocityLocal={.1,.2,.1}; slide.reset({},r);
            double maxSlip=0,maxVelocityError=0;
            std::ofstream csv{dir/("fixture-slide-"+std::to_string(sub)+".csv")}; dynamicsCsvHeader(csv);
            for(unsigned n=0;n<60;++n)
            {
                slide.step(); const auto& state=slide.snapshot(); dynamicsCsvRow(csv,state);
                const auto omega=glm::dmat3{rootTransform(state.root)}*state.root.angularVelocityLocal;
                for(const auto& c:state.contacts) if(forceActive(c))
                {
                    const auto expected=state.root.linearVelocityWorld+glm::cross(omega,c.point-state.root.position);
                    maxVelocityError=std::max(maxVelocityError,glm::length(expected-(c.link2>=0?c.velocity2World:c.velocity1World)));
                }
                for(const auto& f:state.support.feet) maxSlip=std::max(maxSlip,f.slipSpeed);
            }
            const auto& d=slide.snapshot().support.dynamics; residual[variant++]=glm::length(d.linearResidual)+glm::length(d.angularResidual);
            JULIA_CHECK(maxSlip>.01&&maxVelocityError<1e-10);
            report<<"slide sub="<<sub<<" slip_m_s="<<maxSlip<<" point_velocity_error="<<maxVelocityError<<" integral_residual="<<residual[variant-1]<<'\n';
        }
        JULIA_CHECK(residual[2]<residual[0]);
        report<<"translation/yaw, one/two feet, foreign/self classification, independent COM finite difference PASS\n";
    }
    std::string physical(Snapshot s)
    {
        s.epoch=0; s.timing={}; std::ostringstream out; out<<std::setprecision(17);
        for(const auto x:{s.root.position,s.root.linearVelocityWorld,s.root.angularVelocityLocal,s.totalComWorld,s.linearMomentumWorld,s.angularMomentumAboutComWorld}) out<<x.x<<','<<x.y<<','<<x.z<<',';
        out<<s.root.orientation.w<<','<<s.root.orientation.x<<','<<s.root.orientation.y<<','<<s.root.orientation.z<<','<<s.kineticEnergy<<','<<s.potentialEnergy;
        for(unsigned k=0;k<s.dofs;++k) out<<','<<s.q[k]<<','<<s.v[k]<<','<<s.appliedTorque[k];
        driveCsvRow(out,s,16); contactPointsCsvRow(out,s); return out.str();
    }
    std::string observation(Snapshot s)
    { s.epoch=0; s.timing={}; std::ostringstream out; supportCsvRow(out,s); dynamicsCsvRow(out,s); supportRegionsCsvRow(out,s); return out.str(); }
    void body(const BodyRig& rig,const std::filesystem::path& dir,std::ostream& report)
    {
        bool firstLeft=false,firstRight=false,single=false,both=false,lost=false,other=false,self=false;
        for(auto c:{BodyCase::Drop,BodyCase::Tilt,BodyCase::Pulse,BodyCase::Passive,BodyCase::Internal})
        {
            auto def=bodyBenchDefinition(rig,c); PhysicsWorld on{def}; def.supportObservation.reset(); PhysicsWorld off{def};
            resetBodyBench(rig,on,c); resetBodyBench(rig,off,c);
            std::string why; JULIA_CHECK(!on.startContact({},&why)&&!why.empty());
            bodybench::Log log{dir/bodyCaseName(c)}; bodybench::metadata(dir/bodyCaseName(c),rig,on); log.row(publish(rig,on.snapshot()));
            double maxP=0,maxL=0,maxV=0; std::vector<std::string> replay;
            for(unsigned tick=0;tick<720;++tick)
            {
                stepBodyBench(on,c); stepBodyBench(off,c); const auto& s=on.snapshot();
                JULIA_CHECK(physical(s)==physical(off.snapshot())); replay.push_back(observation(s));
                log.row(publish(rig,s)); const auto& o=s.support; const auto& d=o.dynamics;
                maxP=std::max(maxP,glm::length(d.linearResidual)); maxL=std::max(maxL,glm::length(d.angularResidual)); maxV=std::max(maxV,glm::length(d.comVelocity-d.comVelocityJacobian));
                firstLeft=firstLeft||o.feet[0].active>0; firstRight=firstRight||o.feet[1].active>0;
                const unsigned loaded=static_cast<unsigned>(o.feet[0].load.loaded)+static_cast<unsigned>(o.feet[1].load.loaded);
                single=single||loaded==1; both=both||loaded==2; if(both&&loaded==0) lost=true;
                other=other||o.otherEnvironmentActive>0; self=self||o.selfActive>0;
            }
            JULIA_CHECK(maxP<=8&&maxL<=4&&maxV<1e-10);
            JULIA_CHECK(on.snapshot().support.dynamics.samples==720*16);
            report<<bodyCaseName(c)<<" max_residual_Ns="<<maxP<<" max_residual_Nms="<<maxL<<" COM_velocity_error="<<maxV<<" observer_ON_OFF_exact PASS\n";
            resetBodyBench(rig,on,c);
            for(unsigned tick=0;tick<720;++tick) { stepBodyBench(on,c); JULIA_CHECK(observation(on.snapshot())==replay[tick]); }
        }
        JULIA_CHECK(firstLeft&&firstRight&&single&&both&&lost&&other&&self);
        report<<"first feet="<<firstLeft<<','<<firstRight<<" single="<<single<<" double="<<both<<" lost="<<lost<<" other="<<other<<" self="<<self<<" replay+independent instances PASS\n";
        for(auto c:{BodyCase::Drop,BodyCase::Pulse})
        {
            std::array<double,3> p{},l{}; unsigned variant=0;
            for(unsigned sub:{16u,32u,64u})
            {
                auto def=bodyBenchDefinition(rig,c); def.substeps=sub; PhysicsWorld w{def}; resetBodyBench(rig,w,c);
                std::ofstream csv{dir/(bodyCaseName(c)+"-convergence-"+std::to_string(sub)+".csv")}; dynamicsCsvHeader(csv);
                for(unsigned tick=0;tick<720;++tick)
                {
                    stepBodyBench(w,c); dynamicsCsvRow(csv,w.snapshot()); const auto& d=w.snapshot().support.dynamics;
                    p[variant]=std::max(p[variant],glm::length(d.linearResidual)); l[variant]=std::max(l[variant],glm::length(d.angularResidual));
                }
                report<<bodyCaseName(c)<<" sub="<<sub<<" max_residual_Ns="<<p[variant]<<" max_residual_Nms="<<l[variant]<<'\n'; ++variant;
            }
            JULIA_CHECK(p[2]<p[0]&&l[2]<l[0]);
        }
    }
    void performance(const BodyRig& rig,const std::filesystem::path& dir,std::ostream& report)
    {
        for(bool observer:{false,true})
        {
            auto def=bodyBenchDefinition(rig,BodyCase::Drop); if(!observer) def.supportObservation.reset();
            PhysicsWorld w{def}; resetBodyBench(rig,w,BodyCase::Drop); std::vector<double> ticks,times;
            std::ofstream csv{dir/(observer?"performance-on.csv":"performance-off.csv")}; csv<<"tick,tick_us,observer_us\n"<<std::setprecision(17);
            for(unsigned n=0;n<10000;++n)
            {
                const auto start=std::chrono::steady_clock::now(); w.step(); const double time=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
                ticks.push_back(time); times.push_back(w.snapshot().timing.supportUs); csv<<n+1<<','<<time<<','<<times.back()<<'\n';
            }
            const auto stats=[&](const char* name,std::vector<double> v) {
                const double mean=std::accumulate(v.begin(),v.end(),0.)/static_cast<double>(v.size()); std::sort(v.begin(),v.end());
                report<<"observer="<<observer<<' '<<name<<" mean="<<mean<<" p95="<<v[9500]<<" max="<<v.back()<<'\n'; return v[9500];
            };
            JULIA_CHECK(stats("tick_us",ticks)<8333); stats("observer_us",times);
            std::size_t dynamic=0; const auto& o=w.snapshot().support;
            dynamic+=o.dynamics.substeps.size()*sizeof(DynamicsObservation::Sample);
            for(const auto& f:o.feet) dynamic+=(f.soleProjection.vertices.size()+f.loadedRegion.vertices.size())*sizeof(glm::dvec3);
            dynamic+=(o.soleProjection.vertices.size()+o.loadedRegion.vertices.size())*sizeof(glm::dvec3);
            report<<"snapshot_bytes="<<sizeof(Snapshot)<<" support_struct_bytes="<<sizeof(SupportObservation)<<" support_live_vector_bytes="<<dynamic<<" model_bytes="<<w.modelBytes()<<" data_bytes="<<w.dataBytes()<<'\n';
        }
    }
}
int main(int argc,char** argv)
{
    try
    {
        const std::filesystem::path dir=argc>1?argv[1]:"build/diagnostics/stage7f/cpu"; std::filesystem::create_directories(dir);
        std::ofstream report{dir/"support-observation.txt"}; report<<std::setprecision(17);
        analytic(report); fixtures(dir,report); report.flush();
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        BodyRig rig{*asset}; body(rig,dir,report); report.flush(); performance(rig,dir,report);
        report<<"failures="<<JULIA_TEST_RESULT()<<'\n'; return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
