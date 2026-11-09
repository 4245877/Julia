#pragma once
#include "SupportDemo.hpp"
#include <sstream>

namespace julia::app
{
    inline bool reachActive(embodiment::ReachState s)
    { using embodiment::ReachState; return s==ReachState::Planning || s==ReachState::Moving || s==ReachState::Settling; }
    inline embodiment::ReachTarget stanceReachTarget(const embodiment::BodyInstance& body,
        embodiment::ArmSide side,unsigned effector=2,unsigned variant=0)
    {
        using namespace embodiment;
        DirectBodyControl source(*body.control());
        const auto ids=side==ArmSide::Left ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
            std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist};
        const std::array<float,6> q=variant==0 ? std::array{.7f,.5f,.1f,.9f,.1f,-.04f}:std::array{.9f,.4f,.1f,1.1f,.15f,.05f};
        for (unsigned k=0;k<6;++k) source.setCoordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4,q[k]);
        animation::Pose pose(body.asset().skeleton()); source.apply(pose.locals);
        animation::computeGlobalTransforms(body.asset().skeleton(),pose.locals,pose.globals);
        ReachTarget t; t.arm=side; t.effector=static_cast<EffectorId>((side==ArmSide::Left ? 0:4)+effector);
        const auto f=source.binding().effectorFrame(t.effector,pose.globals);
        t.position=glm::vec3{f[3]}; t.orientation=glm::quat_cast(glm::mat3{f}); t.task=ReachTask::PositionAndOrientation;
        t.strategy=MovementStrategy::AllowDetour;
        return t;
    }
    class StanceReachDemo
    {
    public:
        unsigned stage=0;
        embodiment::ArmSide arm=embodiment::ArmSide::Left;
        embodiment::RootPose origin;
        std::uint64_t epoch=0, action=0, since=0;
        void start(embodiment::BodyInstance& b)
        {
            using namespace embodiment;
            if (!b.snapshot().direct || b.snapshot().support.enabled) throw std::invalid_argument("Stance Reach demo requires idle direct mode, support disabled");
            origin=b.snapshot().root; epoch=b.snapshot().epoch;
            b.enqueue(SetSupportSurface{{1,b.snapshot().support.surface.version+1,origin.position.y,true}});
            b.enqueue(EnableSupport{}); stage=1;
        }
        void drive(embodiment::BodyInstance& b)
        {
            using namespace embodiment;
            const auto& s=b.snapshot(); const auto& o=s.support; const auto& r=s.reaches[arm==ArmSide::Left ? 0:1];
            if (s.epoch!=epoch || o.phase==SupportPhase::Lost || o.result=="PlanRejected") { stage=0; return; }
            if (stage==1 && o.phase==SupportPhase::Unaligned) { b.enqueue(AlignSupport{}); stage=2; }
            else if (stage==2 && o.phase==SupportPhase::Holding) {
                action=b.enqueue(StartReach{stanceReachTarget(b,arm)}).id; stage=3;
            } else if (stage==3 && r.state==ReachState::Moving) {
                since=s.tick; b.enqueue(RetargetReach{action,stanceReachTarget(b,arm,2,1)}); stage=4;
            } else if (stage==4 && s.tick>=since+30 && reachActive(r.state)) {
                b.enqueue(CancelReach{arm,action}); stage=5;
            } else if (stage==5 && r.state==ReachState::Cancelled) {
                action=b.enqueue(StartReach{stanceReachTarget(b,arm,2,1)}).id; stage=6;
            } else if (stage==6 && r.state==ReachState::Succeeded) {
                b.enqueue(MoveSupportRoot{origin.position+origin.rotation*glm::vec3{.008f,0,0}}); stage=7;
            } else if (stage==7 && o.phase==SupportPhase::Holding && o.result=="Succeeded") {
                action=b.enqueue(StartReach{stanceReachTarget(b,arm)}).id; stage=8;
            } else if (stage==8 && r.state==ReachState::Succeeded) stage=9;
        }
        bool done() const { return stage==9; }
    };
    inline void stanceReachTrace(std::ostream& out,const embodiment::BodySnapshot& s)
    {
        std::ostringstream base; supportTrace(base,s); auto line=base.str(); line.pop_back(); out << std::setprecision(17) << line;
        for (unsigned side=0;side<2;++side) {
            const auto& r=s.reaches[side]; const auto& g=s.reachGeometry[side];
            out << ',' << r.action << ',' << static_cast<int>(r.state) << ',' << static_cast<int>(r.reason) << ',' << r.error << ',' << r.angularError
                << ',' << r.collisionChecked << ',' << r.cancellationPending << ',' << r.replanningDeferred << ',' << r.planning.queries << ',' << r.planning.poses
                << ',' << static_cast<int>(r.planning.state) << ',' << g.certificate.scene << ',' << g.certificate.frozenGeometry << ',' << g.current.frozenGeometry
                << ',' << g.certificate.supportVersion << ',' << g.dependenciesMatch << ',' << g.executable;
        }
        for (const auto& r:s.reachGeometry) out << ',' << r.current.scene;
        out << '\n';
    }
}
