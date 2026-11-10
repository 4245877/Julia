#include "julia/physics/BodyRig.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <stdexcept>

namespace julia::physics
{
    namespace
    {
        glm::dmat4 proper(const glm::mat4& source)
        {
            auto m=glm::mat4_cast(glm::normalize(glm::quat_cast(glm::dmat3{source})));
            m[3]=glm::dvec4{glm::dvec3{source[3]},1}; return m;
        }
    }
    BodyRig::BodyRig(const model::ModelAsset& asset):binding_(embodiment::raphtaliaBodyProfile(),asset)
    {
        const auto& joints=binding_.profile().joints;
        for(std::size_t i=0;i<joints.size();++i)
        {
            int parent=-1;
            auto node=asset.nodes()[binding_.jointNode(joints[i].id)].parent;
            while(node!=animation::NoNode && parent<0)
            {
                for(std::size_t j=0;j<i;++j) if(binding_.jointNode(joints[j].id)==node) parent=static_cast<int>(j);
                node=asset.nodes()[node].parent;
            }
            if(i>0&&parent<0) throw std::runtime_error("Disconnected physical profile");
            parents_.push_back(parent);
        }
    }
    Definition BodyRig::description(bool floor) const
    {
        Definition def; def.freeRoot=true; def.selfCollision=true;
        def.substeps=16;
        if(floor) def.floorHeight=0;
        def.contactTimeConstant=.01;
        // Envelope for the documented 55 kg drop, tilt and pulse experiments.
        def.maxJointSpeed=80; def.maxLimitViolation=.12; def.maxPenetration=.02;
        def.maxContactForce=30000; def.maxKineticEnergy=5000; def.maxContacts=128;
        def.maxRootLinearSpeed=15; def.maxRootAngularSpeed=40;
        const std::array<double,20> masses{8,4,7,8,1,4,.3,1.6,.9,.3,.3,1.6,.9,.3,5,2.6,.8,5,2.6,.8};
        const std::array<double,20> radii{.07,.06,.065,.065,.025,.08,.022,.032,.027,.032,.022,.032,.027,.032,.045,.037,.04,.045,.037,.04};
        const auto& p=binding_.profile(); int coordinate=0;
        SupportSettings observation;
        const std::array<Drive,6> armDrives{{{80,8,12},{60,6,12},{12,.4,3},{40,4,8},{8,.25,2},{6,.2,1.5}}};
        for(std::size_t i=0;i<p.joints.size();++i)
        {
            const auto& j=p.joints[i]; const auto frame=proper(j.frameInBodyBind);
            const auto node=std::find_if(binding_.asset().nodes().begin(),binding_.asset().nodes().end(),[&](const auto& n) { return n.name==j.segmentEnd.name; });
            if(node==binding_.asset().nodes().end()) throw std::runtime_error("Missing body segment endpoint");
            const auto end=glm::dvec3{p.bodyFromResource*node->globalBindTransform*glm::vec4{0,0,0,1}};
            const auto delta=glm::dvec3{glm::inverse(frame)*glm::dvec4{end,1}};
            const double length=glm::length(delta), radius=radii[i], mass=masses[i];
            const auto y=delta/length, ref=std::abs(y.z)<.95?glm::dvec3{0,0,1}:glm::dvec3{1,0,0};
            const auto x=glm::normalize(glm::cross(y,ref)), z=glm::cross(x,y);
            Link b; b.name=j.name; b.parent=parents_[i];
            b.segment=j.name;
            b.parentFromJoint=b.parent<0?frame:glm::inverse(proper(p.joints[static_cast<std::size_t>(b.parent)].frameInBodyBind))*frame;
            b.jointFromSegment[0]=glm::dvec4{x,0}; b.jointFromSegment[1]=glm::dvec4{y,0}; b.jointFromSegment[2]=glm::dvec4{z,0};
            b.mass=mass; b.radius=radius; b.comInSegment={0,length*.5,0};
            const double transverse=mass*(3*radius*radius+length*length)/12;
            b.inertia={transverse,mass*radius*radius/2,transverse};
            b.capsuleStart=delta*.08; b.capsuleEnd=delta*.92;
            if(i==5) // Engineering head sphere, independent of the eye endpoint.
            {
                b.jointFromSegment=glm::dmat4{1}; b.comInSegment={0,.09,0};
                b.capsuleStart=b.comInSegment; b.capsuleEnd=b.comInSegment;
                b.inertia=glm::dvec3{.4*mass*radius*radius};
            }
            if(j.segment==embodiment::SegmentId::LeftFoot||j.segment==embodiment::SegmentId::RightFoot)
            {
                const bool left=j.segment==embodiment::SegmentId::LeftFoot;
                const auto id=left?embodiment::EffectorId::LeftSole:embodiment::EffectorId::RightSole;
                const auto e=std::find_if(p.effectors.begin(),p.effectors.end(),[&](const auto& f) { return f.id==id; });
                b.segment=left?"LeftFoot":"RightFoot";
                observation.feet[left?0:1]={b.segment,"g"+std::to_string(i),glm::inverse(frame)*proper(e->frameInBodyBind)};
                b.jointFromBox=glm::translate(glm::inverse(frame)*proper(e->frameInBodyBind),glm::dvec3{0,.025,-.035});
                b.boxHalfSize=glm::dvec3{.045,.025,.085};
                b.jointFromSegment=b.jointFromBox; b.comInSegment={0,0,0};
                const auto h=*b.boxHalfSize;
                b.inertia=mass/3.*glm::dvec3{h.y*h.y+h.z*h.z,h.x*h.x+h.z*h.z,h.x*h.x+h.y*h.y};
            }
            if(i!=0) for(std::size_t k=0;k<j.coordinates.size();++k)
            {
                const auto& c=j.coordinates[k];
                const double damping=i<=5?.5:(i>=14?.1:.15);
                b.hinges.push_back({coordinate,glm::dvec3{c.axis},c.lower,c.upper,damping,.0025});
                Drive drive{50,2,20};
                if(i==4||i==5) drive={12,.5,5};
                if(i==6||i==10) drive={20,.5,5};
                if(i==7||i==11) drive=armDrives[k];
                if(i==8||i==12) drive=armDrives[3];
                if(i==9||i==13) drive=armDrives[4+k];
                if(i==14||i==17) drive=k==2?Drive{25,1,12}:Drive{100,5,40};
                if(i==15||i==18) drive={80,4,30};
                if(i==16||i==19) drive={30,1,12};
                def.drives[static_cast<std::size_t>(coordinate++)]=drive;
            }
            def.links.push_back(std::move(b));
        }
        def.supportObservation=observation;
        return def;
    }
    animation::Pose BodyRig::visualPose(Coordinates measured) const
    {
        animation::Pose pose{binding_.asset().skeleton()};
        std::vector<glm::vec3> q(binding_.profile().joints.size(),glm::vec3{0});
        std::size_t coordinate=0;
        for(std::size_t i=1;i<q.size();++i)
            for(std::size_t k=0;k<binding_.profile().joints[i].coordinates.size();++k)
                q[i][static_cast<int>(k)]=static_cast<float>(measured[coordinate++]);
        for(;coordinate<MaxCoordinates;++coordinate)
            if(!std::isfinite(measured[coordinate])||measured[coordinate]!=0) throw std::invalid_argument("Inactive body coordinate");
        binding_.applyMeasuredCoordinates(pose.locals,q);
        animation::computeGlobalTransforms(binding_.asset().skeleton(),pose.locals,pose.globals);
        animation::buildSkinningPalette(binding_.asset().skeleton(),pose.globals,pose.palette);
        return pose;
    }
    glm::dmat4 BodyRig::worldFromBody(const RootState& root) const
    { return rootTransform(root)*glm::inverse(glm::dmat4{binding_.profile().joints[0].frameInBodyBind}); }
    RootState BodyRig::initialRoot(const glm::dmat4& worldFromBody) const
    {
        const auto f=worldFromBody*glm::dmat4{binding_.profile().joints[0].frameInBodyBind};
        RootState root; root.position=glm::dvec3{f[3]}; root.orientation=glm::normalize(glm::quat_cast(glm::dmat3{f})); return root;
    }
    std::vector<glm::dmat4> BodyRig::visualFrames(const animation::Pose& pose,const RootState& root) const
    {
        std::vector<glm::dmat4> frames; const auto w=worldFromBody(root);
        for(const auto& j:binding_.profile().joints) frames.push_back(w*glm::dmat4{binding_.jointFrame(j.id,pose.globals)});
        return frames;
    }
    BodyRenderSnapshot publish(const BodyRig& rig,const Snapshot& state)
    {
        const auto start=std::chrono::steady_clock::now();
        BodyRenderSnapshot view{state,rig.visualPose(state.q),rig.worldFromBody(state.root)*glm::dmat4{rig.binding().profile().bodyFromResource}};
        if(state.mode==DriveMode::Servo) view.referenceFrames=rig.visualFrames(rig.visualPose(state.drive.qRef),state.root);
        view.mappingUs=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count(); return view;
    }
    FkParity verifyFramesBeforeDynamics(const BodyRig& rig,const PhysicsWorld& world)
    {
        FkParity error; const auto& def=world.definition(); const auto& p=rig.binding().profile();
        for(unsigned n=0;n<130;++n)
        {
            Coordinates q{};
            for(const auto& b:def.links) for(const auto& h:b.hinges)
            {
                const auto k=static_cast<std::size_t>(h.coordinate);
                if(n>0 && n<83) { if(k==(n-1)/2) q[k]=(n%2)?h.lower:h.upper; }
                else if(n>=83) q[k]=h.lower+(h.upper-h.lower)*static_cast<double>((n*(13+2*k)+7*k)%97)/96.;
            }
            auto w=glm::rotate(glm::dmat4{1},.73,glm::normalize(glm::dvec3{.2,.8,-.4})); w[3]=glm::dvec4{1.2,.4,-.7,1};
            const auto root=rig.initialRoot(n%2?w:glm::dmat4{1});
            const auto physical=world.forwardKinematics(q,root); const auto pose=rig.visualPose(q);
            const auto visual=rig.visualFrames(pose,root);
            const auto compare=[&](const glm::dmat4& a,const glm::dmat4& b)
            {
                error.position=std::max(error.position,glm::length(glm::dvec3{a[3]-b[3]}));
                for(int c=0;c<3;++c) for(int r=0;r<3;++r) error.rotationElement=std::max(error.rotationElement,std::abs(a[c][r]-b[c][r]));
            };
            for(std::size_t i=0;i<physical.size();++i)
            { compare(physical[i],visual[i]); compare(physical[i]*def.links[i].jointFromSegment,visual[i]*def.links[i].jointFromSegment); }
            for(const auto& e:p.effectors)
            {
                const auto j=std::find_if(p.joints.begin(),p.joints.end(),[&](const auto& x) { return x.node.name==e.node.name; });
                const auto i=static_cast<std::size_t>(j-p.joints.begin());
                compare(physical[i]*glm::inverse(glm::dmat4{j->frameInBodyBind})*glm::dmat4{e.frameInBodyBind},
                    rig.worldFromBody(root)*glm::dmat4{rig.binding().effectorFrame(e.id,pose.globals)});
            }
            ++error.configurations;
        }
        if(error.position>3e-5||error.rotationElement>3e-5) throw std::runtime_error("Whole-body FK parity failed: "+std::to_string(error.position)+" / "+std::to_string(error.rotationElement));
        return error;
    }
    BodyCase parseBodyCase(const std::string& name)
    {
        for(const auto c:{BodyCase::Drop,BodyCase::Tilt,BodyCase::Pulse,BodyCase::Internal,BodyCase::Passive,BodyCase::Stand,BodyCase::StandPulse}) if(bodyCaseName(c)==name) return c;
        throw std::invalid_argument("Unknown body scenario: "+name);
    }
    std::string bodyCaseName(BodyCase c)
    {
        switch(c) { case BodyCase::Drop:return "drop"; case BodyCase::Tilt:return "tilt"; case BodyCase::Pulse:return "pulse";
            case BodyCase::Internal:return "internal"; case BodyCase::Passive:return "passive"; case BodyCase::Stand:return "stand"; case BodyCase::StandPulse:return "stand-pulse"; }
        throw std::invalid_argument("Invalid body scenario");
    }
    Definition bodyBenchDefinition(const BodyRig& rig,BodyCase c)
    {
        auto def=rig.description(c!=BodyCase::Internal);
        if(c==BodyCase::Internal) def.gravity={0,0,0};
        return def;
    }
    void resetBodyBench(const BodyRig& rig,PhysicsWorld& world,BodyCase c)
    {
        if(c==BodyCase::Stand||c==BodyCase::StandPulse)
        { const auto plan=preparedStand(rig,world); world.reset(plan.posture,plan.rootGoal); if(!world.startStand(plan)) throw std::runtime_error("Stand start rejected"); return; }
        auto transform=glm::translate(glm::dmat4{1},glm::dvec3{0,.25,0});
        if(c==BodyCase::Tilt) transform=glm::translate(glm::dmat4{1},glm::dvec3{0,.5,0})*glm::rotate(glm::dmat4{1},.65,glm::dvec3{0,0,1});
        auto root=rig.initialRoot(transform);
        if(c==BodyCase::Internal) root.linearVelocityWorld={.3,-.1,.2};
        world.reset({},root);
        if(c!=BodyCase::Passive) world.command(CommandKind::Target);
    }
    void stepBodyBench(PhysicsWorld& world,BodyCase c,bool smoothScript)
    {
        const auto tick=world.snapshot().tick;
        if(c==BodyCase::StandPulse&&world.snapshot().mode==DriveMode::Stand)
        {
            // Application is an explicit experiment load at the Chest COM.
            const auto rotation=world.snapshot().stand.rootGoal.orientation;
            if(tick>=600&&tick<612) world.applyWrench(3,rotation*glm::dvec3{3,0,0});
            if(tick>=1080&&tick<1092) world.applyWrench(3,rotation*glm::dvec3{0,0,3});
        }
        if(c==BodyCase::Pulse&&tick>=90&&tick<102) world.applyWrench(3,{200,0,0});
        if(c==BodyCase::Internal && (!smoothScript||world.snapshot().mode==DriveMode::Servo) &&
            (world.snapshot().mode!=DriveMode::Servo||tick%120==0))
        {
            Coordinates target{}; const double time=world.snapshot().time;
            for(const auto& b:world.definition().links) for(const auto& h:b.hinges)
            {
                const auto k=static_cast<std::size_t>(h.coordinate);
                target[k]=(h.lower+h.upper)*.5+(h.upper-h.lower)*.2*std::sin(2*time+.2*static_cast<double>(k));
            }
            if(!world.command(world.snapshot().mode==DriveMode::Servo?CommandKind::SmoothTarget:CommandKind::Target,target)) throw std::runtime_error("Internal command rejected");
        }
        world.step();
    }
    void writeBodyCalibration(std::ostream& out,const BodyRig& rig,const PhysicsWorld& world)
    {
        out<<std::setprecision(17)<<"MuJoCo="<<PhysicsWorld::backendVersion()<<" scale_m_per_resource_unit=0.4 fingerprint="<<rig.binding().profile().rigFingerprint<<'\n';
        const auto& l=world.layout(); out<<"substeps="<<world.definition().substeps<<" nq="<<l.nq<<" nv="<<l.nv<<" nu="<<l.nu<<" root_qpos="<<l.rootQpos<<" size="<<l.rootQposSize<<" root_qvel="<<l.rootQvel<<" size="<<l.rootQvelSize<<'\n';
        const auto a=world.massAudit();
        for(std::size_t i=0;i<world.definition().links.size();++i)
        {
            const auto& b=world.definition().links[i]; const auto com=a.comInJoint[i];
            out<<i<<' '<<b.name<<" parent="<<b.parent<<" mass_kg="<<a.masses[i]<<" com_joint_m="<<com.x<<','<<com.y<<','<<com.z<<" inertia_joint=";
            for(int r=0;r<3;++r) for(int c=0;c<3;++c) out<<a.inertiaInJoint[i][c][r]<<',';
            out<<'\n';
            for(const auto& h:b.hinges)
            {
                const auto k=static_cast<std::size_t>(h.coordinate); const auto& d=world.definition().drives[k];
                out<<" q="<<k<<" qpos="<<l.qpos[k]<<" qvel="<<l.qvel[k]<<" actuator="<<l.actuator[k]<<" axis="<<h.axis.x<<','<<h.axis.y<<','<<h.axis.z
                    <<" limits_rad="<<h.lower<<','<<h.upper<<" PD="<<d.kp<<','<<d.kd<<','<<d.maxTorque<<'\n';
            }
        }
        out<<"total_mass_kg="<<a.totalMass<<" bind_com_m="<<a.comWorld.x<<','<<a.comWorld.y<<','<<a.comWorld.z<<'\n';
    }
}

