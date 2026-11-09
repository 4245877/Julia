#pragma once
#include "julia/embodiment/ArmReach.hpp"
#include "julia/animation/Pose.hpp"
namespace julia::app
{
    // Three procedural goals: identical wrist position, different wrist rotations.
    // Only constructs a command target; the production solver still finds all q.
    inline embodiment::ReachTarget spatialDemo(const embodiment::BodyRigBinding& binding,unsigned sample=0,
        embodiment::ArmSide arm=embodiment::ArmSide::Left,unsigned effector=0)
    {
        using namespace embodiment;
        if (sample>2 || effector>2) throw std::invalid_argument("Spatial demo sample/effector requires 0..2");
        DirectBodyControl source(binding); source.setEnabled(true);
        const bool left=arm==ArmSide::Left;
        const auto shoulder=left ? JointId::LeftShoulder:JointId::RightShoulder;
        const auto elbow=left ? JointId::LeftElbow:JointId::RightElbow,wrist=left ? JointId::LeftWrist:JointId::RightWrist;
        source.setCoordinate(shoulder,0,.8f); source.setCoordinate(shoulder,1,.6f); source.setCoordinate(shoulder,2,.1f);
        source.setCoordinate(elbow,0,1.2f);
        source.setCoordinate(wrist,0,-.3f+static_cast<float>(sample)*.3f);
        source.setCoordinate(wrist,1,-.12f+static_cast<float>(sample)*.12f);
        animation::Pose pose(binding.asset().skeleton()); source.apply(pose.locals);
        animation::computeGlobalTransforms(binding.asset().skeleton(),pose.locals,pose.globals);
        ReachTarget target; target.arm=arm; target.effector=static_cast<EffectorId>((left ? 0:4)+effector);
        const auto frame=binding.effectorFrame(target.effector,pose.globals);
        target.position=glm::vec3{frame[3]}; target.orientation=normalizedReachOrientation(glm::quat_cast(glm::mat3{frame}));
        target.task=ReachTask::PositionAndOrientation; return target;
    }
}
