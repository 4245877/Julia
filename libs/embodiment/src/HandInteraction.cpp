#include "julia/embodiment/HandInteraction.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace julia::embodiment
{
    namespace
    {
        constexpr std::uint64_t objectNamespace=1ull<<63;
        std::size_t index(ArmSide arm)
        { if (arm!=ArmSide::Left && arm!=ArmSide::Right) throw std::invalid_argument("Invalid hand"); return arm==ArmSide::Left ? 0:1; }
        EffectorId effector(ArmSide arm) { return arm==ArmSide::Left ? EffectorId::LeftGrip:EffectorId::RightGrip; }
        bool active(ReachState s) { return s==ReachState::Moving || s==ReachState::Planning || s==ReachState::Settling; }
        glm::mat4 proper(const glm::mat4& m)
        {
            const auto y=glm::normalize(glm::vec3{m[1]});
            const auto z=glm::normalize(glm::vec3{m[2]}-glm::dot(glm::vec3{m[2]},y)*y);
            glm::mat4 f{1}; f[0]=glm::vec4{glm::cross(y,z),0}; f[1]=glm::vec4{y,0}; f[2]=glm::vec4{z,0}; f[3]=m[3]; return f;
        }
        RigidPose checked(RigidPose p)
        {
            for (int k=0;k<3;++k) if (!std::isfinite(p.position[k])) throw std::invalid_argument("Nonfinite object pose");
            if (glm::length(p.position)>100) throw std::invalid_argument("Object pose outside 100 metre range");
            p.rotation=normalizedReachOrientation(p.rotation); return p;
        }
    }
    glm::mat4 rigidMatrix(const RigidPose& p) { return glm::translate(glm::mat4{1},p.position)*glm::mat4_cast(p.rotation); }
    RigidPose rigidPose(const glm::mat4& m) { return {glm::vec3{m[3]},normalizedReachOrientation(glm::quat_cast(glm::mat3{m}))}; }
    const char* interactionPhaseName(InteractionPhase p)
    {
        switch(p) {
        case InteractionPhase::Idle:return "Idle"; case InteractionPhase::Opening:return "Opening";
        case InteractionPhase::Approach:return "Approach"; case InteractionPhase::Align:return "Align";
        case InteractionPhase::Closing:return "Closing"; case InteractionPhase::Holding:return "Holding";
        case InteractionPhase::Transport:return "Transport"; case InteractionPhase::ReleaseOpening:return "ReleaseOpening";
        case InteractionPhase::Retreat:return "Retreat"; case InteractionPhase::Cancelling:return "Cancelling";
        case InteractionPhase::Succeeded:return "Succeeded"; case InteractionPhase::Failed:return "Failed";
        case InteractionPhase::Cancelled:return "Cancelled"; case InteractionPhase::ReleaseBlocked:return "ReleaseBlocked";
        case InteractionPhase::Handoff:return "Handoff"; case InteractionPhase::ReadyToGrasp:return "ReadyToGrasp";
        case InteractionPhase::PreparationBlocked:return "PreparationBlocked";
        } return "Invalid";
    }
    HandLayer::HandLayer(const BodyRigBinding& binding)
    {
        const auto& nodes=binding.asset().nodes();
        nodeCount_=nodes.size();
        const auto& skeleton=binding.asset().skeleton();
        std::vector<glm::mat4> bind;
        for (const auto& node:skeleton.nodes()) bind.push_back(node.globalBindTransform);
        for (unsigned side=0;side<2;++side)
        {
            const auto wrist=side==0 ? JointId::LeftWrist:JointId::RightWrist;
            const auto wristNode=binding.jointNode(wrist);
            const auto palm=binding.jointFrame(wrist,bind);
            for (unsigned finger=0;finger<5;++finger)
            {
                const std::array<const char*,5> names{"index","middle","ring","pinky","thumb"};
                std::uint32_t parent=wristNode;
                auto parentOpen=binding.profile().bodyFromResource*bind[parent],parentClosed=parentOpen;
                for (unsigned phalanx=0;phalanx<3;++phalanx)
                {
                    const auto prefix=std::string{"c_"}+names[finger]+std::to_string(phalanx+1)+(side==0 ? ".l_":".r_");
                    auto node=animation::NoNode;
                    for (std::uint32_t n=0;n<nodes.size();++n) if (nodes[n].name.starts_with(prefix))
                    { if (node!=animation::NoNode) throw std::invalid_argument("Ambiguous finger node"); node=n; }
                    if (node==animation::NoNode || nodes[node].parent!=parent) throw std::invalid_argument("Unsupported finger chain");
                    const auto global=binding.profile().bodyFromResource*bind[node];
                    const auto frame=proper(global);
                    const auto before=glm::inverse(binding.profile().bodyFromResource*bind[parent])*frame;
                    const auto after=glm::inverse(frame)*global;
                    const float spread=finger==4 ? (side==0 ? -.65f:.65f):
                        (static_cast<float>(finger)-1.4f)*(side==0 ? .045f:-.045f);
                    const auto target=[&](bool closed,const glm::mat4& parentGlobal)
                    {
                        const float bend=closed ? (finger==4 ? .50f+static_cast<float>(phalanx)*.45f:
                            std::array<float,3>{1.40f,2.65f,3.65f}[phalanx]):0;
                        const float opposition=closed && finger==4 ? (side==0 ? 1.15f:-1.15f):spread;
                        auto f=palm*glm::rotate(glm::mat4{1},opposition,glm::vec3{0,0,1})*
                            glm::rotate(glm::mat4{1},bend,glm::vec3{1,0,0});
                        f[3]=parentGlobal*nodes[node].localTransform[3];
                        return f*after;
                    };
                    const auto openGlobal=target(false,parentOpen),closedGlobal=target(true,parentClosed);
                    const auto rotation=[&](const glm::mat4& targetGlobal,const glm::mat4& parentGlobal)
                    { return normalizedReachOrientation(glm::quat_cast(glm::mat3{glm::inverse(before)*glm::inverse(parentGlobal)*targetGlobal*glm::inverse(after)})); };
                    joints_.push_back({node,side,before,after,rotation(openGlobal,parentOpen),rotation(closedGlobal,parentClosed)});
                    parent=node; parentOpen=openGlobal; parentClosed=closedGlobal;
                }
            }
        }
    }
    void HandLayer::set(ArmSide arm,float closure,bool enabled)
    {
        const auto s=index(arm);
        if (!std::isfinite(closure) || closure<0 || closure>1) throw std::invalid_argument("Hand closure outside [0,1]");
        if (!ready(arm)) throw std::invalid_argument("Hand trajectory active; wait for rest");
        if (!enabled) { states_[s]={}; from_[s]=goal_[s]=0; elapsed_[s]=duration_[s]=0; return; }
        blendFrom_[s]=states_[s].blend;
        states_[s].enabled=enabled; from_[s]=states_[s].closure; goal_[s]=closure; elapsed_[s]=0;
        // Quintic: |dc/dt| <= 1.875/s, |d2c/dt2| <= 5.78/s2 for a full stroke.
        // One second for every stroke is conservative even for partial travel.
        duration_[s]=from_[s]==closure && states_[s].blend==1 ? 0:120;
    }
    void HandLayer::advance()
    {
        for (std::size_t s=0;s<2;++s) if (elapsed_[s]<duration_[s])
        {
            const double t=static_cast<double>(++elapsed_[s])/duration_[s];
            const auto old=states_[s].closure;
            states_[s].closure=static_cast<float>(from_[s]+(goal_[s]-from_[s])*(t*t*t*(10+t*(-15+6*t))));
            states_[s].blend=static_cast<float>(blendFrom_[s]+(1-blendFrom_[s])*(t*t*t*(10+t*(-15+6*t))));
            states_[s].velocity=(states_[s].closure-old)*120;
        } else states_[s].velocity=0;
    }
    bool HandLayer::ready(ArmSide arm) const
    { const auto s=index(arm); return elapsed_[s]>=duration_[s] && states_[s].velocity==0; }
    void HandLayer::apply(std::span<glm::mat4> locals,const std::array<HandState,2>& states) const
    {
        if (locals.size()!=nodeCount_) throw std::invalid_argument("Hand layer local buffer size mismatch");
        for (const auto& s:states) if (!std::isfinite(s.closure) || !std::isfinite(s.blend) ||
            s.closure<0 || s.closure>1 || s.blend<0 || s.blend>1) throw std::invalid_argument("Invalid hand layer state");
        for (const auto& j:joints_) if (states[j.side].enabled)
            locals[j.node]=j.before*glm::mat4_cast(glm::slerp(glm::quat{1,0,0,0},
                glm::slerp(j.open,j.closed,states[j.side].closure),states[j.side].blend))*j.after;
    }
    HandInteraction::HandInteraction(std::uint64_t owner,DirectBodyControl& control,ArmReach& reach)
        :hand(control.binding()),owner_(owner),control_(control),reach_(reach) {}
    bool HandInteraction::locked() const
    {
        if (object_ && object_->state==ObjectState::Attached) return true;
        const auto p=observation_.phase;
        return p==InteractionPhase::Opening || p==InteractionPhase::Approach || p==InteractionPhase::Align ||
            p==InteractionPhase::Handoff || p==InteractionPhase::ReadyToGrasp || p==InteractionPhase::PreparationBlocked ||
            p==InteractionPhase::Closing || p==InteractionPhase::ReleaseOpening || p==InteractionPhase::Retreat || p==InteractionPhase::Cancelling;
    }
    void HandInteraction::requireObject(ObjectHandle h) const
    { if (!object_ || object_->handle!=h) throw std::invalid_argument("Missing object or stale generation"); }
    void HandInteraction::phase(InteractionPhase p,bool geometry)
    {
        observation_.phase=p;
        if (geometry) {
            contactAllowed_=p==InteractionPhase::Align || p==InteractionPhase::Closing || p==InteractionPhase::Holding ||
                p==InteractionPhase::Transport || p==InteractionPhase::ReleaseOpening || p==InteractionPhase::Retreat || p==InteractionPhase::ReadyToGrasp;
            ++geometryRevision_;
            if (geometryChanged) geometryChanged();
        }
    }
    void HandInteraction::create(const CreateInteractionSphere& c)
    {
        if (object_ || locked() || reach_.anyActive()) throw std::invalid_argument("One object only; creation requires idle body");
        if (!c.id || c.id>=objectNamespace || !std::isfinite(c.radius) || std::abs(c.radius-.022f)>1e-7f)
            throw std::invalid_argument("Demo supports exactly one 22 mm radius sphere");
        const auto p=checked(c.pose),g=checked(c.localGrip);
        if (glm::length(g.position)>.005f) throw std::invalid_argument("Grip must be within 5 mm of sphere centre");
        if (generation_==std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Object generations exhausted");
        object_=InteractiveObject{{c.id,++generation_},p,c.radius,g};
        observation_={}; observation_.object=object_->handle; phase(InteractionPhase::Idle,true);
    }
    void HandInteraction::erase(ObjectHandle h)
    {
        requireObject(h);
        if (locked() || reach_.anyActive()) throw std::invalid_argument("Cannot delete an owned/active object");
        object_.reset(); observation_={}; phase(InteractionPhase::Idle,true);
    }
    void HandInteraction::resetFree(std::uint64_t epoch)
    {
        if (locked()) throw std::invalid_argument("Cannot reset owned interaction");
        hand.reset(); observation_={}; observation_.epoch=epoch;
        if (object_) observation_.object=object_->handle;
        cancelBeforeAttachment_=contactAllowed_=false; reachAction_=0;
    }
    void HandInteraction::startReach(const RigidPose& goal,const glm::mat4& root)
    {
        ReachTarget t; t.arm=observation_.arm; t.effector=effector(t.arm); t.space=TargetSpace::World;
        t.task=ReachTask::PositionAndOrientation; t.position=goal.position; t.orientation=goal.rotation;
        t.strategy=MovementStrategy::AllowDetour; t.detour.maxConfigurations=1;
        reachAction_=observation_.action;
        if (observation_.preparation && freezeReach) freezeReach(observation_.arm);
        reach_.start(reachAction_,observation_.epoch,t,root,false,reachAction_);
    }
    void HandInteraction::prepare(std::uint64_t action,std::uint64_t epoch,const PrepareGrasp& c,PathCheck check)
    {
        // BodyInstance has transactionally checked source, ownership and prospective envelope.
        requireObject(c.object); (void)index(c.arm);
        observation_={}; observation_.action=action; observation_.epoch=epoch; observation_.object=c.object; observation_.arm=c.arm;
        observation_.preparation=true; observation_.sourceReach=c.reach; observation_.sourceEpoch=c.reachEpoch;
        observation_.geometryCheck=std::move(check);
        observation_.phaseCheckPoses=observation_.geometryCheck.poses; observation_.phaseCheckQueries=observation_.geometryCheck.queries;
        observation_.phaseCheckMicroseconds=observation_.geometryCheck.microseconds;
        observation_.phaseCheckScratchBytes=observation_.geometryCheck.scratchBytes;
        grip_=rigidPose(rigidMatrix(object_->worldPose)*rigidMatrix(object_->localGrip));
        pregrip_=rigidPose(rigidMatrix(grip_)*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
        cancelBeforeAttachment_=true; finishingPreparation_=false; handoffWait_=true;
        phase(InteractionPhase::Handoff,true);
    }
    void HandInteraction::finishPreparation(std::uint64_t action)
    {
        if (!observation_.preparation || observation_.phase!=InteractionPhase::ReadyToGrasp)
            throw std::invalid_argument("Finish requires ReadyToGrasp");
        cancel(action); finishingPreparation_=true;
    }
    void HandInteraction::abandonPreparation()
    {
        if (!observation_.preparation || !object_ || object_->state!=ObjectState::Free)
            throw std::invalid_argument("Only Free preparation may be reset");
        observation_.cancellationPending=false; observation_.result="Reset";
        phase(InteractionPhase::Cancelled,true);
    }
    void HandInteraction::grasp(std::uint64_t action,std::uint64_t epoch,const GraspObject& c,const glm::mat4& root)
    {
        requireObject(c.object); (void)index(c.arm);
        if (!control_.enabled() || locked() || reach_.anyActive() || !hand.ready(ArmSide::Left) || !hand.ready(ArmSide::Right)) throw std::invalid_argument("Grasp requires idle direct body and hand");
        observation_={}; observation_.action=action; observation_.epoch=epoch; observation_.object=c.object; observation_.arm=c.arm;
        cancelBeforeAttachment_=false;
        grip_=rigidPose(rigidMatrix(object_->worldPose)*rigidMatrix(object_->localGrip));
        pregrip_=rigidPose(rigidMatrix(grip_)*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
        phase(InteractionPhase::Opening,true);
        if (!restClear(collisionContext())) { fail("Opening envelope intersects scene/body/object"); return; }
        hand.set(c.arm,0);
        (void)root;
    }
    void HandInteraction::move(std::uint64_t action,std::uint64_t epoch,const MoveHeldObject& c,const glm::mat4& root)
    {
        requireObject(c.object); const auto p=checked(c.targetWorld);
        if (object_->state!=ObjectState::Attached || reach_.anyActive() || (observation_.phase!=InteractionPhase::Holding && observation_.phase!=InteractionPhase::Failed))
            throw std::invalid_argument("Move requires a stationary attached object");
        const auto goal=rigidPose(rigidMatrix(p)*glm::inverse(object_->effectorFromObject));
        const auto previous=observation_; const auto previousReachAction=reachAction_;
        observation_.action=action; observation_.epoch=epoch; observation_.result.clear(); observation_.reason.clear();
        try { startReach(goal,root); phase(InteractionPhase::Transport); }
        catch (...) { observation_=previous; reachAction_=previousReachAction; throw; }
    }
    void HandInteraction::release(std::uint64_t action,std::uint64_t epoch,ObjectHandle h)
    {
        requireObject(h);
        if (object_->state!=ObjectState::Attached || reach_.anyActive() || !hand.ready(observation_.arm) ||
            (observation_.phase!=InteractionPhase::Holding && observation_.phase!=InteractionPhase::Failed))
            throw std::invalid_argument("Release requires a stationary attached object");
        observation_.action=action; observation_.epoch=epoch; observation_.result.clear(); observation_.reason.clear();
        hand.set(observation_.arm,0); phase(InteractionPhase::ReleaseOpening);
    }
    void HandInteraction::cancel(std::uint64_t action)
    {
        if (!action || observation_.action!=action || !locked()) throw std::invalid_argument("Stale interaction action");
        if (observation_.phase==InteractionPhase::ReleaseOpening || observation_.phase==InteractionPhase::Retreat)
            throw std::invalid_argument("Release is atomic through retreat; cancellation unsupported");
        observation_.cancellationPending=true;
        finishingPreparation_=false;
        cancelBeforeAttachment_=object_->state==ObjectState::Free;
        const auto& r=reach_.observations()[index(observation_.arm)];
        if (active(r.state)) reach_.cancel(observation_.arm,reachAction_);
        phase(InteractionPhase::Cancelling);
    }
    void HandInteraction::fail(const std::string& why)
    {
        observation_.result="Failed"; observation_.reason=why;
        observation_.cancellationPending=false;
        phase(observation_.preparation ? InteractionPhase::PreparationBlocked:InteractionPhase::Failed,true);
    }
    bool HandInteraction::restClear(CollisionContext context)
    {
        if (!validateRest) return false;
        const auto check=validateRest(observation_.arm,std::move(context));
        observation_.phaseCheckPoses+=check.poses; observation_.phaseCheckQueries+=check.queries;
        observation_.phaseCheckMicroseconds+=check.microseconds;
        observation_.phaseCheckScratchBytes=check.scratchBytes;
        observation_.geometryCheck=check;
        return check.status==PathStatus::Clear;
    }
    CollisionContext HandInteraction::collisionContext() const
    {
        CollisionContext c;
        for (std::size_t k=0;k<2;++k) c.handEnvelopes[k]=hand.states()[k].enabled;
        if (!object_) return c;
        if (locked()) c.handEnvelopes[index(observation_.arm)]=true;
        if (contactAllowed_ || object_->state==ObjectState::Attached) {
            c.contactHand=observation_.arm==ArmSide::Left ? 3u:6u; c.contactObject=objectNamespace|object_->handle.id;
        }
        if (object_->state==ObjectState::Attached)
            c.carried=CarriedSphere{objectNamespace|object_->handle.id,effector(object_->arm),object_->effectorFromObject,object_->radius};
        return c;
    }
    std::optional<world::SceneObject> HandInteraction::freeCollider() const
    {
        if (!object_ || object_->state==ObjectState::Attached) return {};
        world::SceneObject c; c.id=objectNamespace|object_->handle.id; c.position=object_->worldPose.position;
        c.rotation=object_->worldPose.rotation; c.shape=world::Sphere{object_->radius}; return c;
    }
    void HandInteraction::advance(std::span<const glm::mat4> globals,const glm::mat4& root)
    {
        if (!object_) return;
        const auto s=index(observation_.arm); const auto f=root*control_.binding().effectorFrame(effector(observation_.arm),globals);
        if (object_->state==ObjectState::Attached) object_->worldPose=rigidPose(f*object_->effectorFromObject);
        observation_.hold=object_->state==ObjectState::Attached ? HoldType::AssistedAttachment:HoldType::None;
        const auto goal=rigidMatrix(object_->worldPose)*rigidMatrix(object_->localGrip);
        observation_.frameError=glm::length(glm::vec3{goal[3]-f[3]});
        observation_.angularError=static_cast<float>(glm::length(orientationResidual(rigidPose(goal).rotation,rigidPose(f).rotation)));
        const auto r=reach_.observations()[s];
        const auto p=observation_.phase;
        try
        {
            if (observation_.preparation && locked() && preparationValid && !preparationValid()) {
                if (active(r.state)) return; // ArmReach's context gate stops invalid execution first.
                if (p!=InteractionPhase::PreparationBlocked) fail("Preparation support/object/frozen geometry changed");
                return;
            }
            if (p==InteractionPhase::Handoff) {
                if (handoffWait_) { handoffWait_=false; return; }
                if (!restClear(collisionContext())) { fail("Opening envelope no longer clear"); return; }
                phase(InteractionPhase::Opening); hand.set(observation_.arm,0);
            } else if (p==InteractionPhase::Opening && hand.ready(observation_.arm)) {
                phase(InteractionPhase::Approach); startReach(pregrip_,root);
            } else if (p==InteractionPhase::Approach || p==InteractionPhase::Align || p==InteractionPhase::Transport) {
                if (active(r.state)) return;
                if (r.action!=reachAction_ || r.state!=ReachState::Succeeded) { fail(reachReasonName(r.reason)); return; }
                if (p==InteractionPhase::Approach) {
                    phase(InteractionPhase::Align,true); startReach(grip_,root);
                } else if (p==InteractionPhase::Align) {
                    if (observation_.preparation) {
                        if (!hand.ready(observation_.arm) || !hand.states()[s].enabled || hand.states()[s].closure!=0 ||
                            hand.states()[s].blend!=1 || observation_.frameError>.002f || observation_.angularError>glm::radians(1.f) ||
                            !r.collisionChecked || object_->state!=ObjectState::Free || !restClear(collisionContext()))
                        { fail("ReadyToGrasp conditions not reached"); return; }
                        // Same geometry/contact policy as Align; no hidden context mutation.
                        observation_.result="ReadyToGrasp"; phase(InteractionPhase::ReadyToGrasp);
                    } else { hand.set(observation_.arm,1); phase(InteractionPhase::Closing); }
                } else {
                    observation_.result="Moved"; phase(InteractionPhase::Holding);
                }
            } else if (p==InteractionPhase::Closing && hand.ready(observation_.arm)) {
                // Timers/closure alone can NEVER attach. Recheck actual FK, settled reach,
                // calibrated envelope against every unpermitted pair and object availability.
                if (observation_.frameError>.002f || observation_.angularError>glm::radians(1.f) ||
                    r.state!=ReachState::Succeeded || !r.collisionChecked || hand.states()[s].closure<.9999f || object_->state!=ObjectState::Free)
                { fail("Attachment conditions not reached"); return; }
                // The free sphere was checked against the body during Align. Verify
                // the prospective payload against the environment too, before committing.
                auto context=collisionContext();
                context.carried=CarriedSphere{objectNamespace|object_->handle.id,effector(observation_.arm),
                    glm::inverse(f)*rigidMatrix(object_->worldPose),object_->radius};
                if (!restClear(context)) { fail("Attachment geometry rejected"); return; }
                object_->effectorFromObject=glm::inverse(f)*rigidMatrix(object_->worldPose);
                object_->state=ObjectState::Attached; object_->owner=owner_; object_->arm=observation_.arm;
                observation_.hold=HoldType::AssistedAttachment; observation_.result="Grasped";
                phase(InteractionPhase::Holding,true);
            } else if (p==InteractionPhase::ReleaseOpening && hand.ready(observation_.arm)) {
                // Pose above is the actual current pose; no snap to the ideal grip.
                object_->state=ObjectState::Free; object_->owner=0; observation_.hold=HoldType::None;
                const auto retreat=rigidPose(f*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
                phase(InteractionPhase::Retreat,true); startReach(retreat,root);
            } else if (p==InteractionPhase::Retreat && !active(r.state)) {
                auto context=collisionContext(); context.contactHand=0; context.contactObject=0;
                const bool separated=r.state==ReachState::Succeeded && restClear(context);
                observation_.result=separated ? (finishingPreparation_ ? "Prepared":cancelBeforeAttachment_ ? "Cancelled":"Released"):"ReleaseBlocked";
                observation_.reason=separated ? "":(r.state==ReachState::Succeeded ? "Contact remains after retreat":reachReasonName(r.reason));
                observation_.cancellationPending=false;
                phase(separated ? (finishingPreparation_ ? InteractionPhase::Succeeded:cancelBeforeAttachment_ ? InteractionPhase::Cancelled:InteractionPhase::Succeeded):
                    observation_.preparation ? InteractionPhase::PreparationBlocked:InteractionPhase::ReleaseBlocked,true);
            } else if (p==InteractionPhase::Cancelling && !active(r.state) && hand.ready(observation_.arm)) {
                if (!cancelBeforeAttachment_) {
                    observation_.result="CancelledHolding"; observation_.cancellationPending=false; phase(InteractionPhase::Holding);
                } else {
                    if (hand.states()[s].closure>0) { hand.set(observation_.arm,0); return; }
                    auto context=collisionContext(); context.contactHand=0; context.contactObject=0;
                    if (restClear(context)) {
                        observation_.result=finishingPreparation_ ? "Prepared":"Cancelled"; observation_.cancellationPending=false;
                        phase(finishingPreparation_ ? InteractionPhase::Succeeded:InteractionPhase::Cancelled,true); return;
                    }
                    phase(InteractionPhase::Retreat,true); startReach(pregrip_,root);
                }
            }
        }
        catch (const std::exception& e) {
            if (p==InteractionPhase::ReleaseOpening || p==InteractionPhase::Retreat) {
                observation_.result="ReleaseBlocked"; observation_.reason=e.what(); observation_.cancellationPending=false;
                phase(observation_.preparation ? InteractionPhase::PreparationBlocked:InteractionPhase::ReleaseBlocked,true);
            } else fail(e.what());
        }
    }
}
