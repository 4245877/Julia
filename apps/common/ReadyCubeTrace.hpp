#pragma once
#include "ReadyCubeDemo.hpp"
#include "ReadySphereTrace.hpp"

namespace julia::app
{
    inline void readyCubeTraceHeader(std::ostream& out)
    {
        out << "tick,completed,idle,idle_seconds,support,reach,interaction_reach,interaction,action,result,left_owner,"
            "object_state,generation,object_owner,x_m,y_m,z_m,qw,qx,qy,qz,opacity,closure,gaze_yaw_deg,gaze_pitch_deg,"
            "left_foot_error_m,right_foot_error_m,left_sole_gap_m,right_sole_gap_m,path,body_form,scene_object,state_hash\n"
            << std::setprecision(17);
    }

    inline void readyCubeTrace(std::ostream& out,const embodiment::BodySnapshot& s,const ReadyCubeDemo& demo)
    {
        using namespace embodiment;
        const auto& i=s.interaction;
        const auto* o=s.object ? &*s.object:nullptr;
        const auto pose=o ? o->worldPose:RigidPose{};
        out << s.tick << ',' << demo.completed() << ',' << idlePhaseName(s.idle.phase) << ',' << s.idle.phaseSeconds
            << ',' << supportPhaseName(s.support.phase) << ',' << static_cast<unsigned>(s.reaches[0].state)
            << ',' << static_cast<unsigned>(s.preparationReach.state) << ',' << interactionPhaseName(i.phase)
            << ',' << i.action << ',' << i.result << ',' << static_cast<unsigned>(s.armOwners[0])
            << ',' << (o ? (o->state==ObjectState::Attached ? "Attached":"Free"):"Absent")
            << ',' << (o ? o->handle.generation:0) << ',' << (o ? o->owner:0);
        out << ',' << pose.position.x << ',' << pose.position.y << ',' << pose.position.z
            << ',' << pose.rotation.w << ',' << pose.rotation.x << ',' << pose.rotation.y << ',' << pose.rotation.z
            << ',' << (o ? demo.opacity(static_cast<double>(s.tick)):0)
            << ',' << s.hands[0].closure << ',' << s.eyes.gazeDegrees.x << ',' << s.eyes.gazeDegrees.y;
        for (const auto& a:s.support.anchors) out << ',' << a.positionError;
        for (const auto& a:s.support.anchors) out << ',' << std::max(-a.minGap,a.maxGap);
        out << ',' << static_cast<unsigned>(s.pathCheck.status) << ',' << s.pathCheck.bodyForm << ',' << s.pathCheck.object
            << ',' << std::hex << bodyPoseHash(s) << std::dec << '\n';
    }
}
