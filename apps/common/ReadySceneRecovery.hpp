#pragma once
#include "julia/embodiment/BodyInstance.hpp"

namespace julia::app
{
    inline bool canRecoverReadyScene(const embodiment::BodySnapshot& s,std::uint64_t epoch,
        embodiment::ObjectHandle handle)
    {
        using namespace embodiment;
        const auto& i=s.interaction;
        if (s.epoch!=epoch || s.idle.phase!=IdlePhase::Ready || s.support.phase!=SupportPhase::Holding ||
            !s.object || s.object->handle!=handle || i.cancellationPending)
            return false;
        if (s.object->state==ObjectState::Attached)
            return s.armOwners[0]==ArmOwner::HandInteraction &&
                (i.phase==InteractionPhase::Holding || i.phase==InteractionPhase::Failed);
        return s.armOwners[0]==ArmOwner::None && s.object->owner==0 &&
            (i.phase==InteractionPhase::Idle || i.phase==InteractionPhase::Cancelled);
    }
}
