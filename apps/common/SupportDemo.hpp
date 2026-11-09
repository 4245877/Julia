#pragma once
#include "julia/embodiment/BodyInstance.hpp"
#include <ostream>
#include <iomanip>
#include <stdexcept>

namespace julia::app
{
    class SupportDemo
    {
    public:
        unsigned stage=0;
        std::uint64_t epoch=0;
        embodiment::RootPose origin;
        void start(embodiment::BodyInstance& body)
        {
            using namespace embodiment;
            origin=body.snapshot().root;
            epoch=body.snapshot().epoch;
            if (!body.snapshot().direct) throw std::invalid_argument("Support demo requires explicit direct mode first; current pose is preserved");
            body.enqueue(SetSupportSurface{{1,body.snapshot().support.surface.version+1,origin.position.y,true}});
            body.enqueue(EnableSupport{}); stage=1;
        }
        void drive(embodiment::BodyInstance& body)
        {
            using namespace embodiment;
            const auto& s=body.snapshot().support;
            if (body.snapshot().epoch!=epoch || s.phase==SupportPhase::Lost || s.result=="PlanRejected" ||
                (stage>1 && s.phase==SupportPhase::Disabled)) { stage=0; return; }
            if (stage==1 && s.phase==SupportPhase::Unaligned) { body.enqueue(AlignSupport{}); stage=2; }
            else if (stage>=2 && stage<=5 && s.phase==SupportPhase::Holding && s.result=="Succeeded") {
                constexpr std::array<glm::vec3,4> offsets{glm::vec3{.008f,0,0},glm::vec3{.004f,-.006f,.006f},glm::vec3{-.004f,-.003f,.003f},glm::vec3{0}};
                body.enqueue(MoveSupportRoot{origin.position+origin.rotation*offsets[stage-2]}); ++stage;
            }
        }
        bool done(const embodiment::BodySnapshot& s) const
        { return stage==6 && s.support.phase==embodiment::SupportPhase::Holding && s.support.result=="Succeeded"; }
    };
    inline void supportTrace(std::ostream& out,const embodiment::BodySnapshot& s)
    {
        const auto& o=s.support;
        out << std::setprecision(17) << s.tick << ',' << s.epoch << ',' << static_cast<int>(o.phase) << ',' << static_cast<int>(o.kind)
            << ',' << o.action << ',' << s.root.position.x << ',' << s.root.position.y << ',' << s.root.position.z;
        for (const auto& a:o.anchors) {
            out << ',' << a.surface << ',' << a.version << ',' << a.bound << ',' << a.positionError << ',' << a.angularError << ',' << a.minGap << ',' << a.maxGap;
            for (int k=0;k<3;++k) out << ',' << a.actualWorld[3][k] << ',' << a.desiredWorld[3][k];
        }
        for (const auto q:s.coordinates) out << ',' << q.x << ',' << q.y << ',' << q.z;
        out << ',' << o.iterations << ',' << o.evaluations << ',' << o.fkChecks << ',' << o.plannedStates << ',' << o.executedStates
            << ',' << o.tickIterations << ',' << o.tickEvaluations << ',' << o.tickFkChecks << ',' << o.result << ',' << o.reason << '\n';
    }
}
