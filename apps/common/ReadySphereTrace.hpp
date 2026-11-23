#pragma once
#include "julia/embodiment/BodyInstance.hpp"
#include <bit>
#include <iomanip>
#include <ostream>

namespace julia::app
{
    inline void readySphereTraceHeader(std::ostream& out)
    {
        out << "tick,idle,idle_seconds,support,interaction,action,result,left_owner,object_state,generation,object_owner,"
            "x_m,y_m,z_m,left_foot_error_m,right_foot_error_m,left_sole_gap_m,right_sole_gap_m,state_hash\n"
            << std::setprecision(17);
    }

    inline std::uint64_t bodyPoseHash(const embodiment::BodySnapshot& s)
    {
        using namespace embodiment;
        std::uint64_t hash=14695981039346656037ull;
        const auto add=[&](std::uint64_t value) { hash=(hash^value)*1099511628211ull; };
        const auto scalar=[&](float value) { add(std::bit_cast<std::uint32_t>(value)); };
        const auto vec=[&](const auto& value) { for (int k=0;k<value.length();++k) scalar(value[k]); };
        const auto matrix=[&](const auto& value) { for (int k=0;k<4;++k) vec(value[k]); };
        add(s.epoch); vec(s.root.position); vec(s.root.rotation);
        for (const auto& q:s.coordinates) vec(q);
        for (const auto& m:s.pose.palette) matrix(m);
        for (const auto& a:s.support.anchors) {
            add(a.bound); add(a.surface); add(a.version);
            matrix(a.desiredWorld); matrix(a.actualWorld);
            scalar(a.positionError); scalar(a.angularError); scalar(a.minGap); scalar(a.maxGap);
        }
        for (const auto& h:s.hands) { add(h.enabled); scalar(h.closure); scalar(h.velocity); scalar(h.blend); }
        for (const auto* r:{&s.reaches[0],&s.preparationReach}) {
            add(r->action); add(static_cast<unsigned>(r->state)); add(static_cast<unsigned>(r->reason));
            add(r->planning.poses); add(r->planning.queries); scalar(r->error); scalar(r->angularError);
        }
        const auto& i=s.interaction;
        add(i.contactHand); add(i.contactObject); add(i.geometryCurrent);
        const auto& o=s.object;
        if (o) { add(o->handle.id); vec(o->worldPose.rotation); matrix(o->effectorFromObject); }
        return hash;
    }

    inline void readySphereTrace(std::ostream& out,const embodiment::BodySnapshot& s)
    {
        using namespace embodiment;
        const auto& i=s.interaction;
        const auto& o=s.object;
        const auto p=o ? o->worldPose.position:glm::vec3{0};
        out << s.tick << ',' << idlePhaseName(s.idle.phase) << ',' << s.idle.phaseSeconds << ',' << supportPhaseName(s.support.phase)
            << ',' << interactionPhaseName(i.phase) << ',' << i.action << ',' << i.result << ',' << static_cast<unsigned>(s.armOwners[0])
            << ',' << (o ? (o->state==ObjectState::Attached ? "Attached":"Free"):"Absent")
            << ',' << (o ? o->handle.generation:0) << ',' << (o ? o->owner:0) << ',' << p.x << ',' << p.y << ',' << p.z;
        for (const auto& a:s.support.anchors) out << ',' << a.positionError;
        for (const auto& a:s.support.anchors) out << ',' << std::max(-a.minGap,a.maxGap);
        out << ',' << std::hex << bodyPoseHash(s) << std::dec << '\n';
    }
}
