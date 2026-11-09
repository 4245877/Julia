#pragma once
#include "StanceReachDemo.hpp"
#include "InteractionDemo.hpp"

namespace julia::app
{
    inline embodiment::ReachTarget preparationPregrip(embodiment::ArmSide arm,embodiment::RigidPose grip)
    {
        using namespace embodiment;
        const auto pre=rigidPose(rigidMatrix(grip)*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
        ReachTarget t; t.arm=arm; t.effector=arm==ArmSide::Left ? EffectorId::LeftGrip:EffectorId::RightGrip;
        t.task=ReachTask::PositionAndOrientation; t.space=TargetSpace::World; t.position=pre.position; t.orientation=pre.rotation;
        t.strategy=MovementStrategy::AllowDetour; return t;
    }
    // One command protocol for CPU, FPS replay, native desktop and GPU verification.
    class StancePreparationDemo
    {
    public:
        unsigned stage=0;
        embodiment::ArmSide arm=embodiment::ArmSide::Left;
        bool cancelAtReady=false;
        std::uint64_t epoch=0,source=0,action=0,since=0;
        void start(embodiment::BodyInstance& b)
        {
            using namespace embodiment;
            if (!b.snapshot().direct || b.snapshot().support.enabled || b.snapshot().object)
                throw std::invalid_argument("Preparation demo requires idle direct mode, support disabled and no object");
            epoch=b.snapshot().epoch;
            b.enqueue(SetSupportSurface{{1,b.snapshot().support.surface.version+1,b.snapshot().root.position.y,true}});
            b.enqueue(EnableSupport{}); stage=1;
        }
        void drive(embodiment::BodyInstance& b)
        {
            using namespace embodiment;
            const auto& s=b.snapshot(); const auto& o=s.support; const auto& i=s.interaction;
            const auto& r=s.reaches[arm==ArmSide::Left ? 0:1];
            if (s.epoch!=epoch || (stage<8 && (o.phase==SupportPhase::Lost || o.result=="PlanRejected"))) { stage=0; return; }
            if (stage==1 && o.phase==SupportPhase::Unaligned) { b.enqueue(AlignSupport{}); stage=2; }
            else if (stage==2 && o.phase==SupportPhase::Holding) {
                b.enqueue(CreateInteractionSphere{1,interactionDemoPose(b.control()->binding(),arm,0,s.root)}); stage=3;
            } else if (stage==3 && s.object) {
                source=b.enqueue(StartReach{preparationPregrip(arm,s.object->worldPose)}).id; stage=4;
            } else if (stage==4 && r.state==ReachState::Succeeded) {
                action=b.enqueue(PrepareGrasp{s.object->handle,arm,source,epoch}).id; stage=5;
            } else if (stage==5 && i.phase==InteractionPhase::ReadyToGrasp) { since=s.tick; stage=6; }
            else if (stage==6 && s.tick>=since+60) {
                if (cancelAtReady) b.enqueue(CancelInteraction{action}); else b.enqueue(FinishGraspPreparation{action}); stage=7;
            } else if (stage==7 && (i.phase==InteractionPhase::Succeeded || i.phase==InteractionPhase::Cancelled)) {
                b.enqueue(DeleteInteractionObject{s.object->handle});
                b.enqueue(StartReach{stanceReachTarget(b,arm,2,1)}); stage=8;
            } else if (stage==8 && r.state==ReachState::Succeeded && r.action!=source) {
                b.enqueue(DisableSupport{}); auto target=stanceReachTarget(b,arm); target.strategy=MovementStrategy::DirectOnly;
                b.enqueue(StartReach{target}); stage=9;
            } else if (stage==9 && !s.support.enabled && r.state==ReachState::Succeeded) stage=10;
        }
        bool done() const { return stage==10; }
    };
    inline void stancePreparationTrace(std::ostream& out,const embodiment::BodySnapshot& s)
    {
        std::ostringstream base; stanceReachTrace(base,s); auto line=base.str(); line.pop_back(); out << line;
        const auto& i=s.interaction; const auto& r=s.preparationReach; const auto& g=s.preparationGeometry;
        out << std::setprecision(17) << ',' << static_cast<int>(i.phase) << ',' << i.action << ',' << i.sourceReach << ',' << i.sourceEpoch
            << ',' << i.object.id << ',' << i.object.generation << ',' << i.preparation << ',' << i.geometryCurrent
            << ',' << i.frameError << ',' << i.angularError << ',' << i.cancellationPending << ',' << static_cast<int>(i.hold)
            << ',' << i.contactHand << ',' << i.contactObject
            << ',' << i.result << ',' << i.reason;
        for (auto owner:s.armOwners) out << ',' << static_cast<int>(owner);
        for (const auto& h:s.hands) out << ',' << h.enabled << ',' << h.closure << ',' << h.blend << ',' << h.velocity;
        if (s.object) {
            const auto& o=*s.object; const auto p=o.worldPose;
            out << ',' << static_cast<int>(o.state) << ',' << o.owner << ',' << p.position.x << ',' << p.position.y << ',' << p.position.z
                << ',' << p.rotation.w << ',' << p.rotation.x << ',' << p.rotation.y << ',' << p.rotation.z;
        } else out << ",-1,0,0,0,0,1,0,0,0";
        out << ',' << r.action << ',' << static_cast<int>(r.state) << ',' << static_cast<int>(r.reason) << ',' << r.error << ',' << r.angularError
            << ',' << r.planning.ikIterations << ',' << r.planning.queries << ',' << r.planning.poses
            << ',' << i.phaseCheckPoses << ',' << i.phaseCheckQueries << ',' << static_cast<int>(i.geometryCheck.status);
        for (const auto& d:{i.geometryCheck.dependencies,g.certificate,g.current}) out << ',' << d.epoch << ',' << d.scene << ',' << d.supportSurface
            << ',' << d.supportVersion << ',' << d.frozenGeometry << ',' << d.calibration << ',' << d.handShapeVersion << ',' << d.contactPolicyVersion << ',' << d.interactionRevision;
        out << ',' << g.dependenciesMatch << ',' << g.executable << '\n';
    }
}
