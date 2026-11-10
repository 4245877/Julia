#include "julia/physics/BodyRig.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include "../apps/julia_body_physics/BodyBenchIO.hpp"
#include "julia_test.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <numeric>

using namespace julia::physics;
namespace
{
    using Clock=std::chrono::steady_clock;
    double us(Clock::time_point t) { return std::chrono::duration<double,std::micro>(Clock::now()-t).count(); }
    double matrixError(const glm::dmat3& a,const glm::dmat3& b)
    { double e=0; for(int c=0;c<3;++c) for(int r=0;r<3;++r) e=std::max(e,std::abs(a[c][r]-b[c][r])); return e; }
    void criteria(std::ostream& o)
    {
        o<<"Acceptance fixed before dynamic experiments; SI units, double MuJoCo 3.3.7\n"
            "FK <=30e-6 m / 3e-5 rotation element, 130 whole body configurations including tilted translated root\n"
            "Compiled mass <=1e-10 kg; COM/tensor <=1e-8; independent mass matrix <=2e-6 absolute; Cholesky pivots >1e-8\n"
            "Free sphere: no-force translation <=1e-10 m; gravity precontact <=.003 m / 1e-9 m/s at .5 s\n"
            "Free angular velocity: body-local quaternion integration <=1e-9 rotation element\n"
            "Internal drives 4 s zero-g: COM straight line <=.003 m; momentum drift <=.1 kg m/s; angular momentum <=.1 kg m2/s\n"
            "Body drop/tilt/pulse/passive: 720 ticks, exactly ticks*definition.substeps steps, no warnings/reset/nonfinite\n"
            "Initial default proxies: no penetrating contacts. Both feet: positive floor force >1 N, unit normal within 1e-9, penetration <.02 m\n"
            "Tilt: nonfoot floor contact, root moves >.1 m; pulse: COM displacement from reference >.01 m\n"
            "Runtime: joint speed 80 rad/s, limit .12 rad, penetration .02 m, force 30000 N, KE 5000 J, contacts 128\n"
            "Root runtime: linear 15 m/s, angular 40 rad/s, unit quaternion within 1e-8\n"
            "Reset/replay and scheduler 30/60/144 Hz: q/v/root/contact telemetry agree within 1e-12; timings excluded\n";
    }
    Definition sphere()
    {
        Definition d; d.freeRoot=true; d.gravity={0,0,0}; d.floorHeight=0; d.contactTimeConstant=.004;
        Link b; b.name="independent 2 kg sphere"; b.parent=-1; b.mass=2; b.radius=.1;
        b.parentFromJoint=glm::translate(glm::dmat4{1},glm::dvec3{0,3,0});
        b.capsuleStart=b.capsuleEnd={0,0,0}; b.inertia=glm::dvec3{.4*2*.1*.1};
        d.links.push_back(b); return d;
    }
    void simple(std::ostream& out)
    {
        auto d=sphere(); PhysicsWorld w{d}; RootState r; r.position={1,3,-2}; r.linearVelocityWorld={.4,-.2,.3};
        r.orientation=glm::angleAxis(.8,glm::normalize(glm::dvec3{1,2,-1}));
        w.reset({},r); JULIA_CHECK(w.layout().nq==7&&w.layout().nv==6&&w.layout().nu==0);
        const auto a=w.massAudit({},r);
        for(unsigned j=0;j<6;++j) for(unsigned k=0;k<6;++k)
            JULIA_CHECK(std::abs(a.massMatrix[j*6+k]-(j==k?(j<3?2:.008):0))<1e-12);
        for(unsigned i=0;i<60;++i) w.step();
        const double translation=glm::length(w.snapshot().root.position-(r.position+.5*r.linearVelocityWorld));
        JULIA_CHECK(translation<1e-10);
        r.angularVelocityLocal={.3,-.4,.2}; w.reset({},r);
        for(unsigned i=0;i<60;++i) w.step();
        const auto expected=r.orientation*glm::angleAxis(.5*glm::length(r.angularVelocityLocal),glm::normalize(r.angularVelocityLocal));
        const auto rotation=matrixError(glm::mat3_cast(expected),glm::mat3_cast(w.snapshot().root.orientation));
        JULIA_CHECK(rotation<1e-9);
        r.angularVelocityLocal={0,0,0}; w.reset({},r); w.applyWrench(0,{200,0,0}); w.step();
        JULIA_CHECK(glm::length(w.snapshot().root.linearVelocityWorld-(r.linearVelocityWorld+glm::dvec3{200*TickSeconds/2,0,0}))<1e-10);
        JULIA_CHECK(w.snapshot().externalForce==glm::dvec3(200,0,0));
        w.step(); JULIA_CHECK(w.snapshot().externalForce==glm::dvec3{0});
        d.gravity={0,-9.81,0}; PhysicsWorld falling{d}; r.angularVelocityLocal={0,0,0}; falling.reset({},r);
        for(unsigned i=0;i<60;++i) falling.step();
        const auto s=falling.snapshot();
        const double position=glm::length(s.root.position-(r.position+.5*r.linearVelocityWorld+.5*.5*.5*d.gravity));
        const double velocity=glm::length(s.root.linearVelocityWorld-(r.linearVelocityWorld+.5*d.gravity));
        JULIA_CHECK(s.contacts.empty()&&position<.003&&velocity<1e-9);
        bool contact=false; for(unsigned i=0;i<120;++i) { falling.step(); contact=contact||!falling.snapshot().contacts.empty(); }
        JULIA_CHECK(contact);
        out<<"sphere translation_m="<<translation<<" rotation_element="<<rotation<<" gravity_position_m="<<position<<" gravity_velocity_m_s="<<velocity<<" contact="<<contact<<'\n';
        // Explicit sibling pair proves that branch self collision is active.
        auto self=sphere(); self.floorHeight.reset(); self.selfCollision=true;
        for(int sign:{-1,1})
        {
            Link b=self.links[0]; b.parent=0; b.name="sibling"; b.radius=.06;
            b.parentFromJoint=glm::translate(glm::dmat4{1},glm::dvec3{sign*.055,.3,0});
            b.capsuleStart=b.capsuleEnd=b.comInSegment={0,.2,0};
            b.hinges={{sign<0?0:1,{0,0,1},-1,1}}; self.links.push_back(b);
        }
        PhysicsWorld on{self}; JULIA_CHECK(!on.snapshot().contacts.empty()); on.step();
        JULIA_CHECK(on.snapshot().peakNormalForceThisTick>1);
        self.selfCollision=false; PhysicsWorld off{self}; JULIA_CHECK(off.snapshot().contacts.empty());
        // Input order deliberately differs from MuJoCo's depth-first order.
        auto branched=sphere(); branched.floorHeight.reset();
        for(unsigned k=0;k<3;++k)
        {
            Link b=branched.links[0]; b.name="address test"; b.parent=k==2?1:0;
            b.parentFromJoint=glm::translate(glm::dmat4{1},glm::dvec3{.2,.3,0});
            b.hinges={{static_cast<int>(k),{0,0,1},-1,1}}; branched.links.push_back(b);
        }
        PhysicsWorld tree{branched}; const auto& layout=tree.layout();
        JULIA_CHECK(layout.qpos[1]>layout.qpos[2]&&layout.qvel[1]>layout.qvel[2]);
        const Coordinates angles{.1,.2,.3}; tree.reset(angles,r);
        JULIA_CHECK(tree.snapshot().q==angles&&tree.snapshot().v==Coordinates{});
        tree.command(CommandKind::Torque,{1,2,3}); tree.step();
        JULIA_CHECK(std::abs(tree.snapshot().appliedTorque[0]-1)<1e-12&&std::abs(tree.snapshot().appliedTorque[1]-2)<1e-12&&std::abs(tree.snapshot().appliedTorque[2]-3)<1e-12);
        out<<"non-depth-first qpos="<<layout.qpos[0]<<','<<layout.qpos[1]<<','<<layout.qpos[2]<<" qvel="<<layout.qvel[0]<<','<<layout.qvel[1]<<','<<layout.qvel[2]<<" actuator mapping PASS\n";
    }
    std::vector<glm::dmat4> cpuFrames(const Definition& d,Coordinates q,const RootState& root)
    {
        std::vector<glm::dmat4> f{rootTransform(root)};
        for(std::size_t i=1;i<d.links.size();++i)
        {
            const auto& b=d.links[i]; auto m=f[static_cast<std::size_t>(b.parent)]*b.parentFromJoint;
            for(const auto& h:b.hinges) m=glm::rotate(m,q[static_cast<std::size_t>(h.coordinate)],h.axis);
            f.push_back(m);
        }
        return f;
    }
    void mass(const BodyRig& rig,PhysicsWorld& w,std::ostream& out)
    {
        const auto& d=w.definition(); const auto& l=w.layout(); const unsigned nv=l.nv;
        for(unsigned configuration=0;configuration<4;++configuration)
        {
            Coordinates q{};
            for(const auto& b:d.links) for(const auto& h:b.hinges)
                if(configuration) q[static_cast<std::size_t>(h.coordinate)]=(h.upper+h.lower)*.5+.25*(h.upper-h.lower)*std::sin(static_cast<double>(h.coordinate+configuration));
            auto transform=glm::rotate(glm::dmat4{1},configuration*.4,glm::normalize(glm::dvec3{1,2,3})); transform[3]={.8,.3,-1.2,1};
            const auto root=rig.initialRoot(transform); const auto a=w.massAudit(q,root); const auto f=cpuFrames(d,q,root);
            JULIA_CHECK(std::abs(a.totalMass-55)<1e-10);
            glm::dvec3 com{0}; glm::dmat3 total{0};
            for(std::size_t i=0;i<d.links.size();++i)
            {
                const auto& b=d.links[i]; glm::dmat3 diag{0}; for(int k=0;k<3;++k) diag[k][k]=b.inertia[k];
                const glm::dmat3 r{b.jointFromSegment};
                const auto local=glm::dvec3{b.jointFromSegment*glm::dvec4{b.comInSegment,1}};
                JULIA_CHECK(std::abs(a.masses[i]-b.mass)<1e-10&&glm::length(a.comInJoint[i]-local)<1e-8);
                JULIA_CHECK(matrixError(a.inertiaInJoint[i],r*diag*glm::transpose(r))<1e-8);
                com+=b.mass*glm::dvec3{f[i]*glm::dvec4{local,1}};
            }
            com/=55.; JULIA_CHECK(glm::length(a.comWorld-com)<1e-8);
            const double eps=1e-5;
            std::vector<std::vector<glm::dvec3>> jv(nv),jw(nv);
            for(unsigned k=0;k<nv;++k)
            {
                auto qp=q,qm=q; auto rp=root,rm=root;
                if(k<3) { rp.position[static_cast<int>(k)]+=eps; rm.position[static_cast<int>(k)]-=eps; }
                else if(k<6)
                {
                    glm::dvec3 axis{0}; axis[static_cast<int>(k-3)]=1;
                    rp.orientation=root.orientation*glm::angleAxis(eps,axis); rm.orientation=root.orientation*glm::angleAxis(-eps,axis);
                }
                else
                {
                    const auto it=std::find(l.qvel.begin(),l.qvel.end(),static_cast<int>(k));
                    const auto index=static_cast<std::size_t>(it-l.qvel.begin()); qp[index]+=eps; qm[index]-=eps;
                }
                const auto fp=cpuFrames(d,qp,rp),fm=cpuFrames(d,qm,rm);
                for(std::size_t i=0;i<f.size();++i)
                {
                    const auto local=d.links[i].jointFromSegment*glm::dvec4{d.links[i].comInSegment,1};
                    jv[k].push_back(glm::dvec3{fp[i]*local-fm[i]*local}/(2*eps));
                    const auto spin=(glm::dmat3{fp[i]}-glm::dmat3{fm[i]})/(2*eps)*glm::transpose(glm::dmat3{f[i]});
                    jw[k].push_back({spin[1][2],spin[2][0],spin[0][1]});
                }
            }
            std::vector<double> matrix(nv*nv,0),chol(nv*nv,0);
            for(std::size_t i=0;i<f.size();++i)
            {
                const auto& b=d.links[i]; const glm::dmat3 r{f[i]*b.jointFromSegment};
                glm::dmat3 diag{0}; for(int k=0;k<3;++k) diag[k][k]=b.inertia[k];
                const auto inertia=r*diag*glm::transpose(r);
                const auto delta=glm::dvec3{f[i]*b.jointFromSegment*glm::dvec4{b.comInSegment,1}}-com;
                total+=inertia+b.mass*(glm::dot(delta,delta)*glm::dmat3{1}-glm::outerProduct(delta,delta));
                for(unsigned j=0;j<nv;++j) for(unsigned k=0;k<nv;++k)
                    matrix[j*nv+k]+=b.mass*glm::dot(jv[j][i],jv[k][i])+glm::dot(jw[j][i],inertia*jw[k][i]);
            }
            JULIA_CHECK(matrixError(total,a.inertiaAtComWorld)<1e-8);
            double maxError=0,minPivot=1e9;
            for(std::size_t i=0;i<matrix.size();++i) maxError=std::max(maxError,std::abs(matrix[i]-a.massMatrix[i]));
            JULIA_CHECK(maxError<2e-6);
            for(unsigned i=0;i<nv;++i) for(unsigned j=0;j<=i;++j)
            {
                double x=a.massMatrix[i*nv+j]; for(unsigned k=0;k<j;++k) x-=chol[i*nv+k]*chol[j*nv+k];
                if(i==j) { minPivot=std::min(minPivot,x); JULIA_CHECK(x>1e-8); chol[i*nv+j]=std::sqrt(x); }
                else chol[i*nv+j]=x/chol[j*nv+j];
            }
            out<<"mass configuration="<<configuration<<" total_kg="<<a.totalMass<<" COM_m="<<com.x<<','<<com.y<<','<<com.z<<" matrix_error="<<maxError<<" min_Cholesky_pivot="<<minPivot<<'\n';
        }
    }
    void equal(const Snapshot& a,const Snapshot& b)
    {
        JULIA_CHECK(a.q==b.q&&a.v==b.v&&a.target==b.target&&a.tick==b.tick&&a.worldSteps==b.worldSteps&&a.time==b.time);
        JULIA_CHECK(a.root.position==b.root.position&&a.root.orientation==b.root.orientation&&a.root.linearVelocityWorld==b.root.linearVelocityWorld&&a.root.angularVelocityLocal==b.root.angularVelocityLocal);
        JULIA_CHECK(a.calculatedTorque==b.calculatedTorque&&a.appliedTorque==b.appliedTorque&&a.saturatedThisTick==b.saturatedThisTick&&a.peakLimitViolationThisTick==b.peakLimitViolationThisTick);
        JULIA_CHECK(a.contacts.size()==b.contacts.size()&&a.totalComWorld==b.totalComWorld&&a.linearMomentumWorld==b.linearMomentumWorld&&a.peakNormalForceThisTick==b.peakNormalForceThisTick&&a.maxPenetrationThisTick==b.maxPenetrationThisTick&&a.solverIterations==b.solverIterations&&a.warnings==b.warnings);
        for(std::size_t i=0;i<a.contacts.size()&&i<b.contacts.size();++i)
        {
            const auto& x=a.contacts[i]; const auto& y=b.contacts[i];
            JULIA_CHECK(x.geom1==y.geom1&&x.geom2==y.geom2&&x.point==y.point&&x.normal==y.normal&&x.distance==y.distance&&x.force==y.force);
        }
    }
    std::vector<Snapshot> run(const BodyRig& rig,BodyCase c,const std::filesystem::path& dir,std::ostream& out)
    {
        PhysicsWorld w{bodyBenchDefinition(rig,c)}; resetBodyBench(rig,w,c);
        bodybench::Log log{dir}; bodybench::metadata(dir,rig,w); log.row(publish(rig,w.snapshot()));
        double initialPen=0; for(const auto& contact:w.snapshot().contacts) initialPen=std::max(initialPen,-contact.distance);
        JULIA_CHECK(initialPen<1e-9);
        std::vector<Snapshot> history; unsigned left=0,right=0,other=0,self=0; double penetration=0,force=0,violation=0; unsigned contacts=0,iterations=0;
        const auto start=w.snapshot(); double comError=0,pError=0,lError=0;
        for(unsigned i=0;i<720;++i)
        {
            stepBodyBench(w,c); const auto& s=w.snapshot(); history.push_back(s); log.row(publish(rig,s));
            JULIA_CHECK(s.worldSteps==s.tick*w.definition().substeps&&s.warnings==0);
            for(unsigned k=0;k<s.dofs;++k)
            { JULIA_CHECK(std::abs(s.appliedTorque[k])<=w.definition().drives[k].maxTorque+1e-9); violation=std::max(violation,s.peakLimitViolationThisTick[k]); }
            penetration=std::max(penetration,s.maxPenetrationThisTick); force=std::max(force,s.peakNormalForceThisTick);
            contacts=std::max(contacts,s.peakContactsThisTick); iterations=std::max(iterations,s.solverIterations);
            for(const auto& contact:s.contacts)
            {
                JULIA_CHECK(std::abs(glm::length(contact.normal)-1)<1e-9);
                if(contact.link1>=0&&contact.link2>=0) ++self;
                if(contact.link1<0||contact.link2<0)
                {
                    const int link=std::max(contact.link1,contact.link2);
                    if(contact.force&&contact.force->x>1) { if(link==16) ++left; else if(link==19) ++right; else ++other; }
                }
            }
            if(c==BodyCase::Internal&&i<480)
            {
                comError=std::max(comError,glm::length(s.totalComWorld-(start.totalComWorld+s.time*start.linearMomentumWorld/start.totalMass)));
                pError=std::max(pError,glm::length(s.linearMomentumWorld-start.linearMomentumWorld));
                lError=std::max(lError,glm::length(s.angularMomentumAboutComWorld-start.angularMomentumAboutComWorld));
            }
        }
        if(c==BodyCase::Drop) JULIA_CHECK(left>0&&right>0);
        if(c==BodyCase::Tilt) { JULIA_CHECK(other>0); JULIA_CHECK(glm::length(w.snapshot().root.position-start.root.position)>.1); }
        if(c==BodyCase::Internal) { JULIA_CHECK(comError<.003&&pError<.1&&lError<.1); }
        out<<bodyCaseName(c)<<" feet_contact_samples="<<left<<','<<right<<" other="<<other<<" self="<<self<<" peak_pen_m="<<penetration<<" force_N="<<force<<" limit_rad="<<violation
            <<" contacts="<<contacts<<" solver_iterations="<<iterations<<" COM_error_m="<<comError<<" momentum_error="<<pError<<" angular_momentum_error="<<lError<<'\n'; out.flush();
        // Dirty the pending force, then verify that reset removes it and all history.
        w.applyWrench(3,{100,20,-10}); resetBodyBench(rig,w,c);
        JULIA_CHECK(w.snapshot().epoch>start.epoch&&w.snapshot().tick==0&&w.snapshot().worldSteps==0&&w.snapshot().time==0&&w.snapshot().externalForce==glm::dvec3{0});
        for(unsigned i=0;i<720;++i) { stepBodyBench(w,c); equal(history[i],w.snapshot()); }
        for(const unsigned fps:{30u,60u,144u})
        {
            resetBodyBench(rig,w,c); julia::runtime::FixedStepScheduler scheduler; unsigned frame=0;
            while(w.snapshot().tick<720)
            {
                scheduler.advance(1./fps,[&] { if(w.snapshot().tick<720) { const auto n=static_cast<std::size_t>(w.snapshot().tick); stepBodyBench(w,c); equal(history[n],w.snapshot()); } });
                ++frame;
            }
            JULIA_CHECK(frame==(720*fps+119)/120); out<<"scheduler "<<bodyCaseName(c)<<" hz="<<fps<<" frames="<<frame<<" exact replay PASS\n";
        }
        return history;
    }
    void performance(const BodyRig& rig,const std::filesystem::path& dir,std::ostream& out)
    {
        PhysicsWorld w{bodyBenchDefinition(rig,BodyCase::Drop)}; resetBodyBench(rig,w,BodyCase::Drop);
        std::vector<double> steps,mapping,counts; std::ofstream raw{dir/"performance.csv"}; raw<<"tick,step_us,mapping_us,contacts\n";
        for(unsigned i=0;i<10000;++i)
        {
            const auto t=Clock::now(); w.step(); const auto elapsed=us(t); const auto v=publish(rig,w.snapshot());
            steps.push_back(elapsed); mapping.push_back(v.mappingUs); counts.push_back(static_cast<double>(w.snapshot().contacts.size()));
            raw<<i+1<<','<<elapsed<<','<<v.mappingUs<<','<<w.snapshot().contacts.size()<<'\n';
        }
        const auto stats=[&](const char* name,std::vector<double> values)
        { const double mean=std::accumulate(values.begin(),values.end(),0.)/static_cast<double>(values.size()); std::sort(values.begin(),values.end()); out<<name<<" mean="<<mean<<" p95="<<values[9499]<<" max="<<values.back()<<'\n'; };
        stats("tick_us",steps); stats("visual_pose_us",mapping); stats("contacts",counts);
        const auto view=publish(rig,w.snapshot());
        const auto poseBytes=(view.actualPose.locals.size()+view.actualPose.globals.size()+view.actualPose.palette.size())*sizeof(glm::mat4);
        out<<"memory model="<<w.modelBytes()<<" one_data="<<w.dataBytes()<<" live_plus_scratch="<<2*w.dataBytes()<<" arena_high_water="<<w.arenaHighWaterBytes()
            <<" snapshot_struct="<<sizeof(Snapshot)<<" snapshot_elements="<<w.snapshot().worldFromJoint.size()*sizeof(glm::dmat4)+w.snapshot().comWorld.size()*sizeof(glm::dvec3)+w.snapshot().contacts.size()*sizeof(Contact)<<" pose_elements="<<poseBytes<<'\n';
    }
    void convergence(const BodyRig& rig,std::ostream& out)
    {
        double previous=1e9;
        for(unsigned substeps:{8u,16u,32u})
        {
            auto def=bodyBenchDefinition(rig,BodyCase::Internal); def.substeps=substeps;
            PhysicsWorld w{def}; resetBodyBench(rig,w,BodyCase::Internal); const auto initial=w.snapshot();
            double p=0,com=0,angular=0;
            for(unsigned tick=0;tick<480;++tick)
            {
                stepBodyBench(w,BodyCase::Internal); const auto& s=w.snapshot();
                p=std::max(p,glm::length(s.linearMomentumWorld-initial.linearMomentumWorld));
                com=std::max(com,glm::length(s.totalComWorld-(initial.totalComWorld+s.time*initial.linearMomentumWorld/initial.totalMass)));
                angular=std::max(angular,glm::length(s.angularMomentumAboutComWorld-initial.angularMomentumAboutComWorld));
            }
            JULIA_CHECK(p<previous*.6); if(substeps>=16) JULIA_CHECK(p<.1&&com<.003&&angular<.1);
            previous=p; out<<"convergence substeps="<<substeps<<" COM_m="<<com<<" momentum_kg_m_s="<<p<<" angular_kg_m2_s="<<angular<<'\n'; out.flush();
        }
    }
}
int main(int argc,char** argv)
{
    try
    {
        const std::filesystem::path dir=argc>1?argv[1]:"body-physics-results"; std::filesystem::create_directories(dir);
        std::ofstream acceptance{dir/"criteria.txt"}; criteria(acceptance); acceptance.close();
        std::ofstream out{dir/"body-physics.txt"}; out<<std::setprecision(17);
        const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const BodyRig rig{*asset}; PhysicsWorld w{rig.description()};
        const auto before=w.snapshot(); const auto parity=verifyFramesBeforeDynamics(rig,w);
        equal(before,w.snapshot()); JULIA_CHECK(w.capabilities().freeRoot&&!w.capabilities().fixedBase);
        JULIA_CHECK(w.snapshot().dofs==41&&w.layout().nq==48&&w.layout().nv==47&&w.layout().nu==41);
        out<<"FK configurations="<<parity.configurations<<" position_m="<<parity.position<<" rotation_element="<<parity.rotationElement<<'\n';
        mass(rig,w,out); out.flush(); simple(out); out.flush();
        const auto drop=run(rig,BodyCase::Drop,dir/"drop",out);
        run(rig,BodyCase::Tilt,dir/"tilt",out);
        const auto pulse=run(rig,BodyCase::Pulse,dir/"pulse",out);
        JULIA_CHECK(glm::length(pulse[130].totalComWorld-drop[130].totalComWorld)>.01);
        JULIA_CHECK(glm::length(pulse[130].root.position-drop[130].root.position)>.01);
        run(rig,BodyCase::Internal,dir/"internal",out); run(rig,BodyCase::Passive,dir/"passive",out);
        // Bad input is transactional; a finite but excessive wrench-induced speed latches failure.
        const auto saved=w.snapshot(); RootState invalid=saved.root; invalid.orientation={2,0,0,0}; bool rejected=false;
        try { w.reset({},invalid); } catch(const std::invalid_argument&) { rejected=true; } JULIA_CHECK(rejected); equal(saved,w.snapshot());
        JULIA_CHECK(!w.command(CommandKind::MoveRoot));
        auto explosive=sphere(); explosive.floorHeight.reset(); explosive.links[0].mass=.001; PhysicsWorld bad{explosive};
        bad.applyWrench(0,{500,0,0}); bool failed=false;
        try { bad.step(); } catch(const std::runtime_error& e) { failed=true; out<<"expected failure: "<<e.what()<<'\n'; }
        JULIA_CHECK(failed&&!bad.command(CommandKind::Target)); bad.reset(); bad.step(); JULIA_CHECK(bad.snapshot().warnings==0);
        convergence(rig,out); performance(rig,dir,out); out<<"failures="<<JULIA_TEST_RESULT()<<'\n';
        std::cout<<"Body physics failures="<<JULIA_TEST_RESULT()<<'\n'; return JULIA_TEST_RESULT();
    }
    catch(const std::exception& e) { std::cerr<<"BODY TEST FAILURE: "<<e.what()<<'\n'; return 1; }
}
