#pragma once
#include "ReadySceneRecovery.hpp"
#include "StancePreparationDemo.hpp"
#include <algorithm>
#include <utility>

namespace julia::app
{
    inline constexpr embodiment::RigidPose readySphereLocalGrip{};
    inline embodiment::RigidPose readySpherePose(const embodiment::BodyInstance& body, unsigned sample)
    {
        using namespace embodiment;
        return interactionDemoPose(body.control()->binding(),ArmSide::Left,sample,body.snapshot().root,SphereGrasp::LeftPalmar);
    }

    class ReadySphereDemo
    {
    public:
        static constexpr unsigned CycleCount=3;
        static constexpr std::uint64_t TickLimit=12000;
        bool done() const { return stage_==Stage::Done; }
        bool failed() const { return stage_==Stage::Failed; }
        unsigned completed() const { return completed_; }
        const std::string& reason() const { return reason_; }
        void requestRecovery() { recover_=true; }
        bool canRecover(const embodiment::BodySnapshot& s) const
        {
            return failed() && canRecoverReadyScene(s,epoch_,handle_);
        }

        // Called once before each simulation tick, while the previous results are available.
        void drive(embodiment::BodyInstance& body)
        {
            using namespace embodiment;
            const auto& s=body.snapshot();
            const auto& i=s.interaction;
            constexpr auto arm=ArmSide::Left;
            if (done()) return;
            if (failed()) {
                if (!std::exchange(recover_,false) || !canRecover(s)) return;
                recovering_=true;
                startTick_=s.tick;
                reason_.clear();
                if (s.object->state==ObjectState::Attached) send(body,ReleaseObject{handle_},Stage::Released);
                else send(body,DeleteInteractionObject{handle_},Stage::Deleted);
                return;
            }
            if (pending_) {
                const auto result=std::find_if(body.results().begin(),body.results().end(),[&](const auto& r) {
                    return r.id==pending_ && r.epoch==epoch_;
                });
                if (result==body.results().end() || result->status!=BodyCommandStatus::Applied) {
                    fail(result==body.results().end() ? "Missing command acknowledgement":result->reason);
                    return;
                }
                pending_=0;
            }
            if (stage_!=Stage::Start) {
                if (s.epoch!=epoch_ || s.idle.phase==IdlePhase::Fault) { fail("Idle controller or body epoch changed"); return; }
                if (s.tick-startTick_>=TickLimit) { fail("Ready sphere tick limit reached"); return; }
            }
            if (i.cancellationPending) return;
            switch (stage_) {
            case Stage::Start:
                epoch_=s.epoch;
                startTick_=s.tick;
                body.setIdleView(IdleView{{0,0,1},true});
                send(body,StartIdle{},Stage::Idle);
                break;
            case Stage::Idle:
                if (s.idle.phase!=IdlePhase::Idle) break;
                if (!idleUntil_) idleUntil_=s.tick+60;
                if (s.tick>=idleUntil_) send(body,SetIdleStance{true},Stage::Ready);
                break;
            case Stage::Ready:
                if (s.idle.phase==IdlePhase::Ready && s.support.phase==SupportPhase::Holding)
                    send(body,CreateInteractionSphere{1,readySpherePose(body,0),.022f,readySphereLocalGrip,SphereGrasp::LeftPalmar},Stage::Created);
                break;
            case Stage::Created:
                if (!s.object || s.object->state!=ObjectState::Free) { fail("Expected a Free sphere"); break; }
                handle_=s.object->handle;
                send(body,StartReach{graspPregripTarget(arm,*s.object)},Stage::Reached);
                break;
            case Stage::Reached: {
                const auto& r=s.reaches[0];
                if (r.action!=action_ || r.epoch!=epoch_) { fail("Unexpected Reach action"); break; }
                if (r.state==ReachState::Succeeded && r.collisionChecked)
                    send(body,PrepareGrasp{handle_,arm,r.action,r.epoch},Stage::Prepared);
                else if (!reachActive(r.state)) fail(reachReasonName(r.reason));
                break;
            }
            case Stage::Prepared:
                if (!interactionMatches(i)) break;
                if (i.phase==InteractionPhase::ReadyToGrasp) {
                    if (!i.geometryCurrent) fail("ReadyToGrasp geometry is stale");
                    else send(body,ConfirmGrasp{handle_,i.action,i.epoch},Stage::Grasped);
                }
                break;
            case Stage::Grasped:
                if (interactionMatches(i) && i.phase==InteractionPhase::Holding && i.result=="Grasped" && ownsSphere(body))
                    send(body,MoveHeldObject{handle_,readySpherePose(body,3)},Stage::Moved);
                break;
            case Stage::Moved:
                if (interactionMatches(i) && i.phase==InteractionPhase::Holding && i.result=="Moved" && ownsSphere(body))
                    send(body,ReleaseObject{handle_},Stage::Released);
                break;
            case Stage::Released:
                if (interactionMatches(i) && i.phase==InteractionPhase::Succeeded && i.result=="Released") {
                    if (!s.object || s.object->state!=ObjectState::Free || s.object->owner || s.armOwners[0]!=ArmOwner::None) {
                        fail("Release did not clear sphere and arm ownership"); break;
                    }
                    send(body,DeleteInteractionObject{handle_},Stage::Deleted);
                }
                break;
            case Stage::Deleted:
                if (s.object) { fail("Delete did not clear the scene"); break; }
                send(body,SetIdleStance{false},Stage::Resumed);
                break;
            case Stage::Resumed:
                if (s.idle.phase==IdlePhase::Idle) {
                    if (!recovering_) ++completed_;
                    recovering_=false;
                    stage_=completed_==CycleCount ? Stage::Done:Stage::Idle;
                    idleUntil_=s.tick+60;
                }
                break;
            case Stage::Done: case Stage::Failed: break;
            }
        }

    private:
        enum class Stage { Start, Idle, Ready, Created, Reached, Prepared, Grasped, Moved, Released, Deleted, Resumed, Done, Failed };
        Stage stage_=Stage::Start;
        unsigned completed_=0;
        std::uint64_t pending_=0, action_=0, epoch_=0, startTick_=0, idleUntil_=0;
        embodiment::ObjectHandle handle_;
        bool recover_=false, recovering_=false;
        std::string reason_;

        void send(embodiment::BodyInstance& body,embodiment::BodyOperation operation,Stage next)
        {
            const auto queued=body.enqueue(std::move(operation));
            if (queued.status!=embodiment::BodyCommandStatus::Queued) { fail(queued.reason); return; }
            pending_=action_=queued.id;
            stage_=next;
        }
        void fail(std::string reason) { stage_=Stage::Failed; reason_=std::move(reason); }
        bool interactionMatches(const embodiment::InteractionObservation& i)
        {
            using namespace embodiment;
            if (i.action!=action_ || i.epoch!=epoch_ || i.object!=handle_) fail("Unexpected sphere interaction action");
            else if (i.phase==InteractionPhase::Failed || i.phase==InteractionPhase::PreparationBlocked ||
                i.phase==InteractionPhase::ReleaseBlocked || i.phase==InteractionPhase::Cancelled || i.result=="CancelledHolding")
                fail(i.result+" ("+i.reason+")");
            return !failed();
        }
        bool ownsSphere(const embodiment::BodyInstance& body)
        {
            using namespace embodiment;
            const auto& s=body.snapshot();
            if (s.object && s.object->state==ObjectState::Attached && s.object->owner==body.id() &&
                s.object->arm==ArmSide::Left && s.armOwners[0]==ArmOwner::HandInteraction) return true;
            fail("Expected an attached sphere owned by the left hand");
            return false;
        }
    };
}
