#pragma once
#include "ReadySceneRecovery.hpp"
#include "StancePreparationDemo.hpp"
#include <algorithm>
#include <string_view>
#include <utility>

namespace julia::app
{
    inline constexpr embodiment::IdleView readyCubeReplayView{{0,0,1},true};

    inline embodiment::RigidPose readyCubePose(const embodiment::BodyInstance& body,
        const embodiment::RootPose& root,bool inspecting=false)
    {
        using namespace embodiment;
        const auto& binding=body.control()->binding();
        DirectBodyControl q(binding);
        q.setEnabled(true);
        q.setCoordinate(JointId::LeftShoulder,0,inspecting ? .77f:.65f);
        q.setCoordinate(JointId::LeftShoulder,1,inspecting ? .15f:.13f);
        q.setCoordinate(JointId::LeftShoulder,2,inspecting ? .36f:.32f);
        q.setCoordinate(JointId::LeftElbow,0,.80f);
        q.setCoordinate(JointId::LeftWrist,0,inspecting ? -.18f:-.32f);
        q.setCoordinate(JointId::LeftWrist,1,inspecting ? .095f:.12f);
        animation::Pose pose(binding.asset().skeleton());
        q.apply(pose.locals);
        animation::computeGlobalTransforms(binding.asset().skeleton(),pose.locals,pose.globals);
        return rigidPose(rigidMatrix({root.position,root.rotation})*
            binding.effectorFrame(EffectorId::LeftGrip,pose.globals)*glm::inverse(rigidMatrix(interactionCubeLocalGrip)));
    }

    class ReadyCubeDemo
    {
    public:
        static constexpr unsigned CycleCount=3;
        static constexpr std::uint64_t TickLimit=12000;
        bool done() const { return stage_==Stage::Done; }
        bool failed() const { return stage_==Stage::Stopped; }
        unsigned completed() const { return completed_; }
        const std::string& reason() const { return reason_; }
        void requestRecovery() { recover_=true; }

        float opacity(double tick) const
        {
            const auto u=std::clamp((tick-static_cast<double>(fadeTick_))/fadeTicks_,0.,1.);
            return glm::mix(fadeFrom_,fadeTo_,static_cast<float>(u*u*(3-2*u)));
        }

        bool canRecover(const embodiment::BodySnapshot& s) const
        {
            return failed() && canRecoverReadyScene(s,epoch_,handle_);
        }

        // One call per body tick. Camera input is supplied by the host.
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
                if (s.object->state==ObjectState::Attached) send(body,ReleaseObject{handle_},Stage::Releasing);
                else stage_=Stage::Releasing;
                return;
            }
            if (pending_) {
                const auto result=std::find_if(body.results().begin(),body.results().end(),[&](const auto& r) {
                    return r.id==pending_ && r.epoch==epoch_;
                });
                pending_=0;
                if (result==body.results().end() || result->status!=BodyCommandStatus::Applied) {
                    stop(s,result==body.results().end() ? "Missing command acknowledgement":result->reason);
                    return;
                }
            }
            if (stage_!=Stage::Start && (epoch_!=s.epoch || s.idle.phase==IdlePhase::Fault || s.tick-startTick_>=TickLimit)) {
                stop(s,"Cube scene interrupted or tick limit reached");
                return;
            }
            if (s.object && stage_>=Stage::Looking)
                body.setIdleFocus(s.object->worldPose.position);
            if (i.cancellationPending) return;

            switch (stage_) {
            case Stage::Start:
                epoch_=s.epoch;
                startTick_=s.tick;
                send(body,StartIdle{},Stage::Idle);
                break;
            case Stage::Idle:
                if (s.idle.phase!=IdlePhase::Idle) break;
                if (!until_) {
                    if (!completed_ && !recovering_) referenceRoot_=s.root;
                    until_=s.tick+72;
                }
                if (s.tick>=until_) {
                    fade(1,s.tick+1,AppearanceTicks);
                    send(body,CreateInteractionBox{1,readyCubePose(body,referenceRoot_)},Stage::Appearing);
                }
                break;
            case Stage::Appearing:
                if (!s.object || s.object->state!=ObjectState::Free) { stop(s,"Expected a free cube"); break; }
                handle_=s.object->handle;
                if (s.tick>=fadeTick_+AppearanceTicks/3) {
                    body.setIdleFocus(s.object->worldPose.position);
                    until_=s.tick+28;
                    stage_=Stage::Looking;
                }
                break;
            case Stage::Looking:
                if (!s.object || s.object->handle!=handle_) { stop(s,"Cube disappeared before Ready"); break; }
                if (s.tick>=until_) send(body,SetIdleStance{true,s.object->worldPose.position},Stage::Ready);
                break;
            case Stage::Ready:
                if (!s.object || s.object->handle!=handle_) { stop(s,"Cube disappeared before Reach"); break; }
                if (s.idle.phase==IdlePhase::Ready && s.support.phase==SupportPhase::Holding)
                    send(body,StartReach{preparationPregrip(arm,
                        rigidPose(rigidMatrix(s.object->worldPose)*rigidMatrix(s.object->localGrip)))},Stage::Reaching);
                break;
            case Stage::Reaching: {
                const auto& r=s.reaches[0];
                if (r.action!=action_ || r.epoch!=epoch_) { stop(s,"Unexpected cube Reach action"); break; }
                if (r.state==ReachState::Succeeded && r.collisionChecked)
                    send(body,PrepareGrasp{handle_,arm,r.action,r.epoch},Stage::Preparing);
                else if (!reachActive(r.state)) stop(s,reachReasonName(r.reason));
                break;
            }
            case Stage::Preparing:
                if (interactionMatches(s) && i.phase==InteractionPhase::ReadyToGrasp) {
                    if (!i.geometryCurrent) stop(s,"ReadyToGrasp geometry is stale");
                    else send(body,ConfirmGrasp{handle_,i.action,i.epoch},Stage::Grasping);
                }
                break;
            case Stage::Grasping:
                if (interactionMatches(s) && held(body,"Grasped")) {
                    until_=s.tick+48;
                    stage_=Stage::Viewing;
                }
                break;
            case Stage::Viewing:
                if (interactionMatches(s) && held(body,"Grasped") && s.tick>=until_)
                    send(body,MoveHeldObject{handle_,readyCubePose(body,referenceRoot_,true)},Stage::Turning);
                break;
            case Stage::Turning:
                if (interactionMatches(s) && held(body,"Moved")) {
                    until_=s.tick+72;
                    stage_=Stage::Viewed;
                }
                break;
            case Stage::Viewed:
                if (interactionMatches(s) && held(body,"Moved") && s.tick>=until_)
                    send(body,ReleaseObject{handle_},Stage::Releasing);
                break;
            case Stage::Releasing: {
                const bool cancelledFree=recovering_ && s.object && s.object->state==ObjectState::Free &&
                    canRecoverReadyScene(s,epoch_,handle_);
                if (!cancelledFree && !interactionMatches(s)) break;
                if ((i.phase==InteractionPhase::Retreat || cancelledFree) && fadeTo_!=0) fade(0,s.tick,96);
                const bool retreated=i.phase==InteractionPhase::Succeeded && i.result=="Released";
                if ((retreated || cancelledFree) && s.tick>=fadeTick_+fadeTicks_) {
                    if (!s.object || s.object->state!=ObjectState::Free || s.object->owner || s.armOwners[0]!=ArmOwner::None) {
                        stop(s,"Cube and hand did not separate"); break;
                    }
                    send(body,DeleteInteractionObject{handle_},Stage::Deleted);
                }
                break;
            }
            case Stage::Deleted:
                if (s.object) { stop(s,"Delete did not clear the cube"); break; }
                body.setIdleFocus(std::nullopt);
                send(body,SetIdleStance{false},Stage::Resuming);
                break;
            case Stage::Resuming:
                if (s.idle.phase!=IdlePhase::Idle) break;
                if (!recovering_) ++completed_;
                recovering_=false;
                until_=0;
                stage_=completed_==CycleCount ? Stage::Done:Stage::Idle;
                break;
            case Stage::Done: case Stage::Stopped: break;
            }
        }

    private:
        enum class Stage { Start, Idle, Appearing, Looking, Ready, Reaching, Preparing, Grasping,
            Viewing, Turning, Viewed, Releasing, Deleted, Resuming, Done, Stopped };
        static constexpr unsigned AppearanceTicks=84;
        Stage stage_=Stage::Start;
        embodiment::ObjectHandle handle_;
        embodiment::RootPose referenceRoot_;
        unsigned completed_=0, fadeTicks_=1;
        std::uint64_t pending_=0, action_=0, epoch_=0, startTick_=0, until_=0, fadeTick_=0;
        float fadeFrom_=0, fadeTo_=0;
        bool recover_=false, recovering_=false;
        std::string reason_;

        void fade(float to,std::uint64_t tick,unsigned duration)
        {
            fadeFrom_=opacity(static_cast<double>(tick));
            fadeTo_=to;
            fadeTick_=tick;
            fadeTicks_=duration;
        }
        void send(embodiment::BodyInstance& body,embodiment::BodyOperation operation,Stage next)
        {
            const auto queued=body.enqueue(std::move(operation));
            if (queued.status!=embodiment::BodyCommandStatus::Queued) { stop(body.snapshot(),queued.reason); return; }
            pending_=action_=queued.id;
            stage_=next;
        }
        void stop(const embodiment::BodySnapshot& s,std::string reason)
        {
            stage_=Stage::Stopped;
            reason_=std::move(reason);
            if (s.object) fade(1,s.tick,48);
        }
        bool interactionMatches(const embodiment::BodySnapshot& s)
        {
            using namespace embodiment;
            const auto& i=s.interaction;
            if (i.action!=action_ || i.epoch!=epoch_ || i.object!=handle_) stop(s,"Unexpected cube interaction action");
            else if (i.phase==InteractionPhase::Failed || i.phase==InteractionPhase::PreparationBlocked ||
                i.phase==InteractionPhase::ReleaseBlocked || i.phase==InteractionPhase::Cancelled || i.result=="CancelledHolding")
                stop(s,i.reason.empty() ? i.result:i.result+" ("+i.reason+")");
            return !failed();
        }
        bool held(const embodiment::BodyInstance& body,std::string_view result)
        {
            using namespace embodiment;
            const auto& s=body.snapshot();
            if (s.interaction.phase!=InteractionPhase::Holding || s.interaction.result!=result) return false;
            if (s.object && s.object->handle==handle_ && s.object->state==ObjectState::Attached && s.object->owner==body.id() &&
                s.object->arm==ArmSide::Left && s.armOwners[0]==ArmOwner::HandInteraction) return true;
            stop(s,"Expected the cube attached to the left hand");
            return false;
        }
    };
}
