#pragma once
#include "julia/embodiment/BodyInstance.hpp"
#include "SpatialDemo.hpp"
#include <ostream>
#include <iomanip>

namespace julia::app
{
    inline embodiment::RigidPose interactionDemoPose(const embodiment::BodyRigBinding& binding,
        embodiment::ArmSide arm,unsigned sample=0,const embodiment::RootPose& root={})
    {
        using namespace embodiment;
        DirectBodyControl source(binding); source.setEnabled(true);
        const bool left=arm==ArmSide::Left;
        const auto shoulder=left ? JointId::LeftShoulder:JointId::RightShoulder;
        source.setCoordinate(shoulder,0,.65f+static_cast<float>(sample)*.08f);
        source.setCoordinate(shoulder,1,.45f); source.setCoordinate(shoulder,2,0);
        source.setCoordinate(left ? JointId::LeftElbow:JointId::RightElbow,0,.9f);
        source.setCoordinate(left ? JointId::LeftWrist:JointId::RightWrist,0,.12f);
        animation::Pose p(binding.asset().skeleton()); source.apply(p.locals);
        animation::computeGlobalTransforms(binding.asset().skeleton(),p.locals,p.globals);
        return rigidPose(rigidMatrix({root.position,root.rotation})*
            binding.effectorFrame(left ? EffectorId::LeftGrip:EffectorId::RightGrip,p.globals));
    }
    // Used by CLI, desktop and CPU tests. Every mutation is a public BodyCommand.
    class InteractionDemo
    {
    public:
        unsigned stage=0,sample=0;
        embodiment::ArmSide arm=embodiment::ArmSide::Left;
        embodiment::RootPose root;
        void start(embodiment::BodyInstance& body)
        {
            using namespace embodiment;
            body.enqueue(SetDirectMode{true}); body.enqueue(SetRoot{root});
            body.enqueue(CreateInteractionSphere{1,interactionDemoPose(body.control()->binding(),arm,sample,root)});
            stage=1;
        }
        void drive(embodiment::BodyInstance& body)
        {
            using namespace embodiment;
            const auto& s=body.snapshot(); if (!s.object) return;
            if (stage==1) { body.enqueue(GraspObject{s.object->handle,arm}); stage=2; }
            else if (stage==2 && s.interaction.phase==InteractionPhase::Holding) {
                const auto target=interactionDemoPose(body.control()->binding(),arm,sample+1,root);
                body.enqueue(MoveHeldObject{s.object->handle,target}); stage=3;
            } else if (stage==3 && s.interaction.phase==InteractionPhase::Holding && s.interaction.result=="Moved") {
                body.enqueue(ReleaseObject{s.object->handle}); stage=4;
            }
        }
    };
    inline void interactionTrace(std::ostream& out,const embodiment::BodySnapshot& s)
    {
        const auto& i=s.interaction; const auto side=i.arm==embodiment::ArmSide::Left ? 0u:1u;
        out << std::setprecision(17) << s.tick << ',' << static_cast<int>(i.phase) << ',' << i.action << ',' << i.object.id << ',' << i.object.generation
            << ',' << static_cast<int>(i.hold) << ',' << s.hands[side].closure << ',' << i.frameError << ',' << i.angularError;
        if (s.object) { const auto& o=*s.object; const auto& p=o.worldPose;
            out << ',' << static_cast<int>(o.state) << ',' << o.owner << ',' << p.position.x << ',' << p.position.y << ',' << p.position.z
                << ',' << p.rotation.w << ',' << p.rotation.x << ',' << p.rotation.y << ',' << p.rotation.z;
        } else out << ",-1,0,0,0,0,1,0,0,0";
        for (const auto q:s.coordinates) out << ',' << q.x << ',' << q.y << ',' << q.z;
        const auto& r=s.reaches[side];
        out << ',' << static_cast<int>(r.state) << ',' << r.planning.ikIterations << ',' << r.planning.queries << ',' << r.planning.poses
            << ',' << i.phaseCheckPoses << ',' << i.phaseCheckQueries << ',' << i.result << ',' << i.reason << '\n';
    }
}
