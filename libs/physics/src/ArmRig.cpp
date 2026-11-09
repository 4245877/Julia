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
    ArmRig::ArmRig(const model::ModelAsset& asset):binding_(embodiment::raphtaliaBodyProfile(),asset) {}
    Definition ArmRig::description(double massScale,double inertiaScale,double payloadKg,bool wall) const
    {
        if(!std::isfinite(massScale)||massScale<=0||!std::isfinite(inertiaScale)||inertiaScale<=0||
            !std::isfinite(payloadKg)||payloadKg<0||payloadKg>20) throw std::invalid_argument("Invalid arm load case");
        const auto& profile=binding_.profile();
        const std::array<JointId,3> ids{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist};
        std::array<glm::dmat4,3> frames;
        for(std::size_t i=0;i<3;++i) frames[i]=glm::dmat4{profile.joints[binding_.jointIndex(ids[i])].frameInBodyBind};
        Definition def;
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
            if(i<2)
            {
                const auto& c=joint.coordinates[0];
                b.hinge={static_cast<int>(i),glm::dvec3{c.axis},c.lower,c.upper,.02};
            }
            def.links.push_back(b);
        }
        if(payloadKg>0)
        {
            const auto grip=std::find_if(profile.effectors.begin(),profile.effectors.end(),
                [](const auto& e) { return e.id==embodiment::EffectorId::LeftGrip; });
            Link b; b.name="Welded engineering payload (not a grasp)";
            b.parentFromJoint=glm::inverse(frames[2])*glm::dmat4{grip->frameInBodyBind};
            b.mass=payloadKg; b.radius=.04; b.inertia=glm::dvec3{.4*payloadKg*b.radius*b.radius};
            b.capsuleStart={0,0,0}; b.capsuleEnd={0,0,0};
            def.links.push_back(b);
        }
        if(wall) def.wall=Wall{};
        return def;
    }
    animation::Pose ArmRig::visualPose(Coordinates measured) const
    {
        animation::Pose result{binding_.asset().skeleton()};
        std::vector<glm::vec3> q(binding_.profile().joints.size(),glm::vec3{0});
        q[binding_.jointIndex(JointId::LeftShoulder)].x=static_cast<float>(measured[0]);
        q[binding_.jointIndex(JointId::LeftElbow)].x=static_cast<float>(measured[1]);
        binding_.applyMeasuredCoordinates(result.locals,q);
        animation::computeGlobalTransforms(binding_.asset().skeleton(),result.locals,result.globals);
        animation::buildSkinningPalette(binding_.asset().skeleton(),result.globals,result.palette);
        return result;
    }
    std::array<glm::dmat4,3> ArmRig::visualFrames(const animation::Pose& pose) const
    {
        return {glm::dmat4{binding_.jointFrame(JointId::LeftShoulder,pose.globals)},
            glm::dmat4{binding_.jointFrame(JointId::LeftElbow,pose.globals)},
            glm::dmat4{binding_.jointFrame(JointId::LeftWrist,pose.globals)}};
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
        for(const Coordinates q:std::array<Coordinates,7>{{{0,0},{.2,.3},{.8,.9},{-.2,.1},{1.1,1.6},{.4,-.08},{.3,1.78}}})
        {
            const auto physical=world.forwardKinematics(q);
            const auto visual=rig.visualFrames(rig.visualPose(q));
            for(std::size_t i=0;i<3;++i)
            {
                error.position=std::max(error.position,glm::length(glm::dvec3{physical[i][3]-visual[i][3]}));
                for(int c=0;c<3;++c) for(int r=0;r<3;++r)
                    error.rotationElement=std::max(error.rotationElement,std::abs(physical[i][c][r]-visual[i][c][r]));
            }
        }
        if(error.position>30e-6||error.rotationElement>3e-5)
            throw std::runtime_error("Physical/visual FK parity failed before dynamics; position="+std::to_string(error.position));
        return error;
    }
    BenchCase parseBenchCase(const std::string& name)
    {
        for(auto c:{BenchCase::Passive,BenchCase::Hold,BenchCase::Heavy,BenchCase::Inertia,BenchCase::Weak,BenchCase::Impulse,BenchCase::Contact})
            if(benchCaseName(c)==name) return c;
        throw std::invalid_argument("Unknown case; use passive|hold|heavy|inertia|weak|impulse|contact");
    }
    std::string benchCaseName(BenchCase scenario)
    {
        switch(scenario) {
        case BenchCase::Passive:return "passive"; case BenchCase::Hold:return "hold"; case BenchCase::Heavy:return "heavy";
        case BenchCase::Inertia:return "inertia"; case BenchCase::Weak:return "weak"; case BenchCase::Impulse:return "impulse";
        case BenchCase::Contact:return "contact"; }
        throw std::invalid_argument("Invalid bench case");
    }
    Definition benchDefinition(const ArmRig& rig,BenchCase scenario)
    {
        auto def=rig.description(scenario==BenchCase::Heavy?2.:1.,scenario==BenchCase::Inertia?4.:1.,
            scenario==BenchCase::Heavy||scenario==BenchCase::Weak?2.:0.,scenario==BenchCase::Contact);
        if(scenario==BenchCase::Weak) { def.drives[0].maxTorque=.35; def.drives[1].maxTorque=.35; }
        return def;
    }
    void resetBench(PhysicsWorld& world,BenchCase scenario)
    {
        world.reset({.2,.3});
        if(scenario!=BenchCase::Passive&&!world.command(CommandKind::Target,{.8,.9}))
            throw std::runtime_error("Bench target rejected");
    }
    void stepBench(PhysicsWorld& world,BenchCase scenario)
    {
        const auto next=world.snapshot().tick+1;
        if(scenario==BenchCase::Impulse&&next>=241&&next<=246) world.applyWrench(2,{0,0,-30});
        world.step();
    }
    void csvHeader(std::ostream& out)
    {
        out<<"tick,epoch,time_s,world_steps,mode,target0_rad,target1_rad,q0_rad,q1_rad,v0_rad_s,v1_rad_s,"
            "calculated0_Nm,calculated1_Nm,applied0_Nm,applied1_Nm,saturated0,saturated1,limit0_rad,limit1_rad,"
            "wrist_x_m,wrist_y_m,wrist_z_m,target_x_m,target_y_m,target_z_m,contacts,peak_contacts,peak_normal_N,"
            "max_penetration_m,solver_iterations,warnings,external_x_N,external_y_N,external_z_N,kinetic_J,potential_J,"
            "control_us,substeps_us,read_us,mapping_us\n";
    }
    void csvRow(std::ostream& out,const RenderSnapshot& view)
    {
        const auto& s=view.physics; out<<std::setprecision(17)<<s.tick<<','<<s.epoch<<','<<s.time<<','<<s.worldSteps<<','<<static_cast<int>(s.mode);
        for(const auto& pair:{s.target,s.q,s.v,s.calculatedTorque,s.appliedTorque}) for(const auto x:pair) out<<','<<x;
        for(auto b:s.saturatedThisTick) out<<','<<b;
        for(auto x:s.limitViolation) out<<','<<x;
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
            <<"profile="<<rig.binding().profile().name<<"; fingerprint="<<rig.binding().profile().rigFingerprint<<'\n';
        for(const auto& b:world.definition().links)
        {
            out<<b.name<<" mass_kg="<<b.mass<<" radius_m="<<b.radius<<" COM_segment_m="<<b.comInSegment.x<<','<<b.comInSegment.y<<','<<b.comInSegment.z
                <<" I_COM_segment_kg_m2="<<b.inertia.x<<','<<b.inertia.y<<','<<b.inertia.z<<" hinge="<<b.hinge.coordinate<<'\n';
        }
        for(std::size_t k=0;k<2;++k) { const auto& d=world.definition().drives[k]; out<<"drive"<<k<<" kp="<<d.kp<<" kd="<<d.kd<<" limit_Nm="<<d.maxTorque<<'\n'; }
        out<<"model_bytes="<<world.modelBytes()<<" data_bytes_each="<<world.dataBytes()<<" (live + scratch)\n"
            <<"Contact force: endpoint solver estimate available; fixed-base reaction and physical standing unavailable\n";
    }
}
