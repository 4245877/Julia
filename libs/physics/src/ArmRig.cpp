#include "julia/physics/ArmRig.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace julia::physics
{
    using embodiment::JointId;
    ArmRig::ArmRig(const model::ModelAsset& asset,ArmSide side,ArmChain chain)
        :binding_(embodiment::raphtaliaBodyProfile(),asset),side_(side),chain_(chain)
    {
        if((side!=ArmSide::Left&&side!=ArmSide::Right)||(chain!=ArmChain::Two&&chain!=ArmChain::Full))
            throw std::invalid_argument("Invalid arm side/chain");
    }
    std::array<JointId,3> ArmRig::jointIds() const
    {
        return side_==ArmSide::Left?std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
            std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist};
    }
    std::array<embodiment::EffectorId,3> ArmRig::effectorIds() const
    {
        using E=embodiment::EffectorId;
        return side_==ArmSide::Left?std::array{E::LeftWrist,E::LeftPalm,E::LeftGrip}:std::array{E::RightWrist,E::RightPalm,E::RightGrip};
    }
    Definition ArmRig::description(double massScale,double inertiaScale,double payloadKg,bool wall) const
    {
        if(!std::isfinite(massScale)||massScale<=0||!std::isfinite(inertiaScale)||inertiaScale<=0||
            !std::isfinite(payloadKg)||payloadKg<0||payloadKg>20) throw std::invalid_argument("Invalid arm load case");
        const auto& profile=binding_.profile();
        const auto ids=jointIds();
        std::array<glm::dmat4,3> frames;
        for(std::size_t i=0;i<3;++i) frames[i]=glm::dmat4{profile.joints[binding_.jointIndex(ids[i])].frameInBodyBind};
        Definition def;
        if(chain_==ArmChain::Full)
        {
            def.drives={{{80,8,12},{60,6,12},{12,.4,3},{40,4,8},{8,.25,2},{6,.2,1.5}}};
            def.contactTimeConstant=.006;
        }
        int coordinate=0;
        const std::array<double,3> masses{1.6,.9,.3}, radii{.032,.027,.032};
        for(std::size_t i=0;i<3;++i)
        {
            const auto& joint=profile.joints[binding_.jointIndex(ids[i])];
            const auto& nodes=binding_.asset().nodes();
            const auto endNode=std::find_if(nodes.begin(),nodes.end(),[&](const auto& n) { return n.name==joint.segmentEnd.name; });
            if(endNode==nodes.end()) throw std::runtime_error("Missing physical segment end");
            const auto end=glm::dvec3{profile.bodyFromResource*endNode->globalBindTransform*glm::vec4{0,0,0,1}};
            const auto delta=glm::dvec3{glm::inverse(frames[i])*glm::dvec4{end,1}};
            const double length=glm::length(delta), mass=masses[i]*massScale, radius=radii[i];
            const auto y=delta/length;
            const auto ref=std::abs(y.z)<.95?glm::dvec3{0,0,1}:glm::dvec3{1,0,0};
            const auto x=glm::normalize(glm::cross(y,ref)), z=glm::cross(x,y);
            Link b; b.name=joint.name;
            b.parentFromJoint=i==0?frames[i]:glm::inverse(frames[i-1])*frames[i];
            b.jointFromSegment[0]=glm::dvec4{x,0}; b.jointFromSegment[1]=glm::dvec4{y,0}; b.jointFromSegment[2]=glm::dvec4{z,0};
            b.mass=mass; b.radius=radius; b.comInSegment={0,length*.5,0};
            const double transverse=mass*(3*radius*radius+length*length)/12;
            b.inertia=glm::dvec3{transverse,mass*radius*radius/2,transverse}*inertiaScale;
            b.capsuleStart=delta*.08; b.capsuleEnd=delta*.92;
            const auto count=chain_==ArmChain::Full?joint.coordinates.size():(i<2?std::size_t{1}:std::size_t{0});
            for(std::size_t k=0;k<count;++k)
            {
                const auto& c=joint.coordinates[k];
                b.hinges.push_back({coordinate++,glm::dvec3{c.axis},c.lower,c.upper,.02,chain_==ArmChain::Full?.005:.01});
            }
            def.links.push_back(b);
        }
        if(payloadKg>0)
        {
            const auto grip=std::find_if(profile.effectors.begin(),profile.effectors.end(),
                [&](const auto& e) { return e.id==effectorIds()[2]; });
            Link b; b.name="Welded engineering payload (not a grasp)";
            b.parentFromJoint=glm::inverse(frames[2])*glm::dmat4{grip->frameInBodyBind};
            b.mass=payloadKg; b.radius=.04; b.inertia=glm::dvec3{.4*payloadKg*b.radius*b.radius};
            b.capsuleStart={0,0,0}; b.capsuleEnd={0,0,0};
            def.links.push_back(b);
        }
        if(wall)
        {
            def.wall=Wall{};
            if(side_==ArmSide::Right) def.wall->center.x=-def.wall->center.x;
            if(chain_==ArmChain::Full)
            {
                const auto target=visualFrames(visualPose({.8,.3,.2,.9,.3,.18}))[2];
                const auto p=glm::dvec3{target*glm::dvec4{(def.links[2].capsuleStart+def.links[2].capsuleEnd)*.5,1}};
                def.wall->center={p.x,1.05,p.z+.01};
            }
        }
        return def;
    }
    animation::Pose ArmRig::visualPose(Coordinates measured) const
    {
        animation::Pose result{binding_.asset().skeleton()};
        std::vector<glm::vec3> q(binding_.profile().joints.size(),glm::vec3{0});
        const auto ids=jointIds(); std::size_t coordinate=0;
        for(std::size_t i=0;i<3;++i)
        {
            const auto index=binding_.jointIndex(ids[i]);
            const auto count=chain_==ArmChain::Full?binding_.profile().joints[index].coordinates.size():(i<2?std::size_t{1}:std::size_t{0});
            for(std::size_t k=0;k<count;++k) q[index][static_cast<int>(k)]=static_cast<float>(measured[coordinate++]);
        }
        for(;coordinate<MaxCoordinates;++coordinate)
            if(!std::isfinite(measured[coordinate])||measured[coordinate]!=0) throw std::invalid_argument("Inactive visual coordinate");
        binding_.applyMeasuredCoordinates(result.locals,q);
        animation::computeGlobalTransforms(binding_.asset().skeleton(),result.locals,result.globals);
        animation::buildSkinningPalette(binding_.asset().skeleton(),result.globals,result.palette);
        return result;
    }
    std::array<glm::dmat4,3> ArmRig::visualFrames(const animation::Pose& pose) const
    {
        const auto ids=jointIds();
        return {glm::dmat4{binding_.jointFrame(ids[0],pose.globals)},glm::dmat4{binding_.jointFrame(ids[1],pose.globals)},
            glm::dmat4{binding_.jointFrame(ids[2],pose.globals)}};
    }
    RenderSnapshot publish(const ArmRig& rig,const Snapshot& state)
    {
        const auto start=std::chrono::steady_clock::now();
        RenderSnapshot view{state,rig.visualPose(state.q),rig.visualPose(state.target)};
        view.actualWrist=glm::dvec3{rig.visualFrames(view.actualPose)[2][3]};
        view.targetWrist=glm::dvec3{rig.visualFrames(view.targetPose)[2][3]};
        view.mappingUs=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
        return view;
    }
    FkParity verifyFramesBeforeDynamics(const ArmRig& rig,const PhysicsWorld& world)
    {
        FkParity error;
        std::vector<Coordinates> configurations;
        const auto& def=world.definition();
        if(rig.chain()==ArmChain::Two)
            configurations={{0,0},{.2,.3},{.8,.9},{-.2,.1},{1.1,1.6},{.4,-.08},{.3,1.78}};
        else
        {
            configurations.push_back({});
            // Independent signed rotations, all 64 corners and a deterministic
            // interior grid. Scratch FK also checks small soft-limit overshoots.
            Coordinates lo{},hi{};
            for(const auto& b:def.links) for(const auto& h:b.hinges)
            {
                const auto k=static_cast<std::size_t>(h.coordinate); lo[k]=h.lower; hi[k]=h.upper;
                for(double angle:{h.lower,h.upper,h.lower-.02,h.upper+.02})
                { Coordinates q{}; q[k]=angle; configurations.push_back(q); }
            }
            for(unsigned mask=0;mask<64;++mask)
            {
                Coordinates q{}; for(unsigned k=0;k<ArmCoordinates;++k) q[k]=(mask&(1u<<k))?hi[k]-.001:lo[k]+.001;
                configurations.push_back(q);
            }
            for(unsigned n=0;n<96;++n)
            {
                Coordinates q{}; for(unsigned k=0;k<ArmCoordinates;++k)
                    q[k]=lo[k]+(hi[k]-lo[k])*static_cast<double>((n*(13+2*k)+7*k)%97)/96.;
                configurations.push_back(q);
            }
        }
        const auto& profile=rig.binding().profile();
        const auto wristBind=glm::dmat4{profile.joints[rig.binding().jointIndex(rig.jointIds()[2])].frameInBodyBind};
        for(const auto& q:configurations)
        {
            const auto physical=world.forwardKinematics(q);
            const auto pose=rig.visualPose(q); const auto visual=rig.visualFrames(pose);
            const auto compare=[&](const glm::dmat4& a,const glm::dmat4& b)
            {
                error.position=std::max(error.position,glm::length(glm::dvec3{a[3]-b[3]}));
                for(int c=0;c<3;++c) for(int r=0;r<3;++r)
                    error.rotationElement=std::max(error.rotationElement,std::abs(a[c][r]-b[c][r]));
            };
            for(std::size_t i=0;i<3;++i)
            {
                compare(physical[i],visual[i]);
                compare(physical[i]*def.links[i].jointFromSegment,visual[i]*def.links[i].jointFromSegment);
            }
            for(const auto id:rig.effectorIds())
            {
                const auto e=std::find_if(profile.effectors.begin(),profile.effectors.end(),[&](const auto& f) { return f.id==id; });
                compare(physical[2]*glm::inverse(wristBind)*glm::dmat4{e->frameInBodyBind},
                    glm::dmat4{rig.binding().effectorFrame(id,pose.globals)});
            }
            if(rig.chain()==ArmChain::Full)
            {
                const auto& h=def.links[0].hinges;
                const auto r0=glm::rotate(glm::dmat4{1},q[0],h[0].axis);
                const auto r1=glm::rotate(r0,q[1],h[1].axis);
                const glm::dmat3 axes{h[0].axis,glm::dvec3{r0*glm::dvec4{h[1].axis,0}},glm::dvec3{r1*glm::dvec4{h[2].axis,0}}};
                error.minShoulderAxisDeterminant=std::min(error.minShoulderAxisDeterminant,std::abs(glm::determinant(axes)));
            }
            ++error.configurations;
        }
        if(error.position>30e-6||error.rotationElement>3e-5)
            throw std::runtime_error("Physical/visual FK parity failed before dynamics; position="+std::to_string(error.position));
        return error;
    }
    BenchCase parseBenchCase(const std::string& name)
    {
        for(auto c:{BenchCase::Passive,BenchCase::Hold,BenchCase::Heavy,BenchCase::Inertia,BenchCase::Weak,BenchCase::Impulse,BenchCase::Contact,BenchCase::ZeroGravity,BenchCase::Spatial,BenchCase::ImpulseX,BenchCase::ImpulseY})
            if(benchCaseName(c)==name) return c;
        throw std::invalid_argument("Unknown case; use passive|zero-g|hold|heavy|inertia|weak|impulse|impulse-x|impulse-y|contact|spatial");
    }
    std::string benchCaseName(BenchCase scenario)
    {
        switch(scenario) {
        case BenchCase::Passive:return "passive"; case BenchCase::Hold:return "hold"; case BenchCase::Heavy:return "heavy";
        case BenchCase::Inertia:return "inertia"; case BenchCase::Weak:return "weak"; case BenchCase::Impulse:return "impulse";
        case BenchCase::Contact:return "contact"; case BenchCase::ZeroGravity:return "zero-g";
        case BenchCase::Spatial:return "spatial"; case BenchCase::ImpulseX:return "impulse-x"; case BenchCase::ImpulseY:return "impulse-y"; }
        throw std::invalid_argument("Invalid bench case");
    }
    Definition benchDefinition(const ArmRig& rig,BenchCase scenario)
    {
        auto def=rig.description(scenario==BenchCase::Heavy?2.:1.,scenario==BenchCase::Inertia?4.:1.,
            scenario==BenchCase::Heavy||scenario==BenchCase::Weak?2.:0.,scenario==BenchCase::Contact);
        if(scenario==BenchCase::Weak) for(auto& d:def.drives) d.maxTorque=.35;
        if(scenario==BenchCase::ZeroGravity) def.gravity={0,0,0};
        return def;
    }
    void resetBench(PhysicsWorld& world,BenchCase scenario)
    {
        const bool full=world.snapshot().dofs==ArmCoordinates;
        world.reset(full?Coordinates{.2,.1,.1,.3,.1,.05}:Coordinates{.2,.3});
        if(scenario!=BenchCase::Passive&&scenario!=BenchCase::ZeroGravity&&
            !world.command(CommandKind::Target,full?Coordinates{.8,.3,.2,.9,.3,.18}:Coordinates{.8,.9}))
            throw std::runtime_error("Bench target rejected");
    }
    void stepBench(PhysicsWorld& world,BenchCase scenario)
    {
        const auto next=world.snapshot().tick+1;
        if(next>=241&&next<=246)
        {
            if(scenario==BenchCase::Impulse) world.applyWrench(2,{0,0,-30});
            if(scenario==BenchCase::ImpulseX) world.applyWrench(2,{30,0,0});
            if(scenario==BenchCase::ImpulseY) world.applyWrench(2,{0,30,0});
        }
        if(scenario==BenchCase::Spatial)
        {
            if(world.snapshot().dofs!=ArmCoordinates) throw std::invalid_argument("Spatial requires --chain full");
            Coordinates target{.55,.25,0,.7,0,0};
            const Coordinates amplitude{.25,.18,.25,.3,.3,.2};
            for(unsigned k=0;k<ArmCoordinates;++k) target[k]+=amplitude[k]*std::sin(next*TickSeconds*(1.+.17*k));
            if(!world.command(CommandKind::Target,target)) throw std::runtime_error("Spatial target rejected");
        }
        world.step();
    }
    void csvHeader(std::ostream& out)
    {
        out<<"tick,epoch,time_s,world_steps,mode";
        for(const auto* label:{"target","q","v","calculated","applied","saturated","limit","saturated_last","peak_limit"})
            for(unsigned k=0;k<ArmCoordinates;++k) out<<','<<label<<k;
        out<<",wrist_x_m,wrist_y_m,wrist_z_m,target_x_m,target_y_m,target_z_m,contacts,peak_contacts,peak_normal_N,"
            "max_penetration_m,solver_iterations,warnings,external_x_N,external_y_N,external_z_N,kinetic_J,potential_J,"
            "control_us,substeps_us,read_us,mapping_us\n";
    }
    void csvRow(std::ostream& out,const RenderSnapshot& view)
    {
        const auto& s=view.physics; out<<std::setprecision(17)<<s.tick<<','<<s.epoch<<','<<s.time<<','<<s.worldSteps<<','<<static_cast<int>(s.mode);
        for(const auto& pair:{s.target,s.q,s.v,s.calculatedTorque,s.appliedTorque}) for(unsigned k=0;k<ArmCoordinates;++k) out<<','<<pair[k];
        for(unsigned k=0;k<ArmCoordinates;++k) out<<','<<s.saturatedThisTick[k];
        for(unsigned k=0;k<ArmCoordinates;++k) out<<','<<s.limitViolation[k];
        for(unsigned k=0;k<ArmCoordinates;++k) out<<','<<s.saturated[k];
        for(unsigned k=0;k<ArmCoordinates;++k) out<<','<<s.peakLimitViolationThisTick[k];
        for(int i=0;i<3;++i) out<<','<<view.actualWrist[i];
        for(int i=0;i<3;++i) out<<','<<view.targetWrist[i];
        out<<','<<s.contacts.size()<<','<<s.peakContactsThisTick<<','<<s.peakNormalForceThisTick<<','<<s.maxPenetrationThisTick
            <<','<<s.solverIterations<<','<<s.warnings;
        for(int i=0;i<3;++i) out<<','<<s.externalForce[i];
        out<<','<<s.kineticEnergy<<','<<s.potentialEnergy<<','<<s.timing.controlUs<<','<<s.timing.stepUs<<','<<s.timing.readUs<<','<<view.mappingUs<<'\n';
    }
    void writeCalibration(std::ostream& out,const ArmRig& rig,const PhysicsWorld& world)
    {
        out<<std::setprecision(17)<<"MuJoCo "<<world.backendVersion()<<"; fixed base; 120 Hz, "<<Substeps<<" substeps; PD "<<120*Substeps<<" Hz\n"
            <<"metres, kg, seconds, rad, Nm; engineering parameters, not validated biomechanics\n"
            <<"side="<<(rig.side()==ArmSide::Left?"left":"right")<<" chain="<<(rig.chain()==ArmChain::Full?"full":"two")<<'\n'
            <<"profile="<<rig.binding().profile().name<<"; fingerprint="<<rig.binding().profile().rigFingerprint<<'\n';
        for(const auto& b:world.definition().links)
        {
            out<<b.name<<" mass_kg="<<b.mass<<" radius_m="<<b.radius<<" COM_segment_m="<<b.comInSegment.x<<','<<b.comInSegment.y<<','<<b.comInSegment.z
                <<" I_COM_segment_kg_m2="<<b.inertia.x<<','<<b.inertia.y<<','<<b.inertia.z<<'\n';
            for(const auto& h:b.hinges) out<<" coordinate="<<h.coordinate<<" axis="<<h.axis.x<<','<<h.axis.y<<','<<h.axis.z<<" limits="<<h.lower<<','<<h.upper<<'\n';
        }
        for(std::size_t k=0;k<world.snapshot().dofs;++k) { const auto& d=world.definition().drives[k]; out<<"drive"<<k<<" kp="<<d.kp<<" kd="<<d.kd<<" limit_Nm="<<d.maxTorque<<'\n'; }
        const auto audit=world.massAudit();
        out<<"compiled_total_mass_kg="<<audit.totalMass<<" COM_world="<<audit.comWorld.x<<','<<audit.comWorld.y<<','<<audit.comWorld.z<<'\n';
        out<<"model_bytes="<<world.modelBytes()<<" data_bytes_each="<<world.dataBytes()<<" (live + scratch)\n"
            <<"contact_solref_seconds="<<world.definition().contactTimeConstant<<'\n'
            <<"q/v/frames/contact force: endpoint of tick; calculated/applied torque: last substep held control; saturation/peaks: over eight substeps\n"
            <<"Contact force: endpoint solver estimate available; fixed-base reaction and physical standing unavailable\n";
    }
}
