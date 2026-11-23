#include "../apps/common/ReadySphereDemo.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "hand_skin_audit.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace
{
    using namespace julia;
    using namespace julia::embodiment;
    constexpr auto arm=ArmSide::Left;
    constexpr std::array armJoints{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist};

    void require(bool condition,const std::string& reason)
    { if (!condition) throw std::runtime_error(reason); }

    std::uint64_t command(BodyInstance& b,BodyOperation operation,BodyCommandStatus expected=BodyCommandStatus::Applied)
    {
        const auto id=b.enqueue(std::move(operation)).id;
        b.step();
        const auto& result=b.results().front();
        require(result.id==id && result.status==expected,"Unexpected command outcome: "+result.reason);
        return id;
    }

    template<class Condition> void until(BodyInstance& b,Condition condition)
    {
        for (unsigned t=0;t<2800 && !condition(b.snapshot());++t) b.step();
        const auto& s=b.snapshot();
        require(condition(s),std::string("Timed out in ")+interactionPhaseName(s.interaction.phase)+" "+s.interaction.reason+
            " " + reachReasonName(s.reaches[0].reason));
    }

    glm::mat4 rootMatrix(const RootPose& r) { return rigidMatrix({r.position,r.rotation}); }
    template<class PoseState> glm::mat4 grip(const BodyInstance& b,const PoseState& s)
    { return rootMatrix(s.root)*b.control()->binding().effectorFrame(EffectorId::LeftGrip,s.pose.globals); }

    float rotationError(glm::quat a,glm::quat b)
    { return static_cast<float>(glm::length(orientationResidual(a,b))); }

    void startReady(BodyInstance& b)
    {
        command(b,StartIdle{});
        until(b,[](const auto& s) { return s.idle.phase==IdlePhase::Idle; });
        command(b,SetIdleStance{true});
        until(b,[](const auto& s) { return s.idle.phase==IdlePhase::Ready && s.support.phase==SupportPhase::Holding; });
    }

    ConfirmGrasp prepare(BodyInstance& b)
    {
        const auto& o=*b.snapshot().object;
        const auto handle=o.handle;
        const auto source=command(b,StartReach{app::preparationPregrip(arm,rigidPose(rigidMatrix(o.worldPose)*rigidMatrix(o.localGrip)))});
        until(b,[](const auto& s) { return s.reaches[0].state==ReachState::Succeeded; });
        command(b,PrepareGrasp{handle,arm,source,b.snapshot().epoch});
        until(b,[](const auto& s) { return s.interaction.phase==InteractionPhase::ReadyToGrasp; });
        const auto& s=b.snapshot();
        require(s.interaction.geometryCurrent && s.armOwners[0]==ArmOwner::GraspPreparation,"Invalid ReadyToGrasp authority");
        return {handle,s.interaction.action,s.interaction.epoch};
    }

    void create(BodyInstance& b)
    { command(b,CreateInteractionSphere{1,app::readySpherePose(b,0),.022f,app::readySphereLocalGrip}); }

    void grasp(BodyInstance& b)
    {
        create(b);
        command(b,prepare(b));
        until(b,[](const auto& s) { return s.interaction.result=="Grasped"; });
    }

    void release(BodyInstance& b)
    {
        command(b,ReleaseObject{b.snapshot().object->handle});
        until(b,[](const auto& s) { return s.interaction.result=="Released"; });
        const auto& s=b.snapshot();
        require(s.preparationReach.state==ReachState::Succeeded && s.interaction.geometryCheck.status==PathStatus::Clear &&
            s.object->state==ObjectState::Free && s.object->owner==0 && s.armOwners[0]==ArmOwner::None,"Release lacks checked separation");
    }

    void resume(BodyInstance& b)
    {
        command(b,DeleteInteractionObject{b.snapshot().object->handle});
        command(b,SetIdleStance{false});
        until(b,[](const auto& s) { return s.idle.phase==IdlePhase::Idle; });
        require(!b.snapshot().object && !b.snapshot().hands[0].enabled,"Idle did not restore the empty scene and finger pose");
    }

    struct Measurements
    {
        std::array<glm::mat4,2> desired{},held{};
        std::array<float,2> footError{},footAngle{},gap{};
        float jump=0,jumpAngle=0,attachmentError=0,attachmentAngle=0,resumeStep=0;
        unsigned attached=0,released=0,resumed=0;
        std::vector<double> planning,transportPlanning,transportMotion;

        void checkFeet(const SupportObservation& s)
        {
            for (std::size_t k=0;k<2;++k) {
                const auto& a=s.anchors[k];
                require(a.bound && a.desiredWorld==desired[k] && a.positionError<=SupportPositionTolerance &&
                    a.angularError<=SupportAngularTolerance && a.minGap>=-SupportSurfaceTolerance &&
                    a.maxGap<=SupportSurfaceTolerance,"Lost or shifted planted foot");
                footError[k]=std::max(footError[k],a.positionError);
                footAngle[k]=std::max(footAngle[k],a.angularError);
                gap[k]=std::max({gap[k],-a.minGap,a.maxGap});
            }
        }

        template<class PoseState> void checkAttachment(const BodyInstance& b,const PoseState& s)
        {
            if (s.object && s.object->state==ObjectState::Attached) {
                const auto expected=rigidPose(grip(b,s)*s.object->effectorFromObject);
                attachmentError=std::max(attachmentError,glm::length(expected.position-s.object->worldPose.position));
                attachmentAngle=std::max(attachmentAngle,rotationError(expected.rotation,s.object->worldPose.rotation));
                require(s.object->owner==b.id() && s.object->arm==arm && s.armOwners[0]==ArmOwner::HandInteraction &&
                    s.interaction.hold==HoldType::AssistedAttachment && s.interaction.contactHand==3 &&
                    s.interaction.contactObject==((1ull<<63)|s.object->handle.id),"Attachment ownership or contact changed");
            }
        }
    };

    void reportTiming(std::ostream& out,const char* label,std::vector<double>& samples)
    {
        require(!samples.empty(),"Missing tick timing coverage");
        std::sort(samples.begin(),samples.end());
        out << label << " samples=" << samples.size() << " median_us=" << samples[samples.size()/2]
            << " p95_us=" << samples[(samples.size()-1)*95/100] << " max_us=" << samples.back() << '\n';
    }

    void cycles(BodyInstance& b,std::ostream& report)
    {
        app::ReadySphereDemo demo;
        Measurements m;
        hand_skin_audit::Surface skin(b.asset());
        float minimumGap=std::numeric_limits<float>::infinity();
        unsigned skinSamples=0,skinIntersections=0;
        const auto measureSkin=[&](const auto& frame) {
            const auto sample=skin.measure(b,frame);
            ++skinSamples;
            if (sample.intersects) ++skinIntersections;
            minimumGap=std::min(minimumGap,std::ranges::min(sample.vertexGaps));
            return sample.vertexGaps;
        };
        glm::vec3 initialSphere{0};
        while (b.snapshot().tick<app::ReadySphereDemo::TickLimit) {
            demo.drive(b);
            if (demo.done() || demo.failed()) break;
            const auto start=std::chrono::steady_clock::now();
            b.step();
            const auto elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            const auto& s=b.snapshot(); const auto& p=b.previousSnapshot();
            if (s.tick==1) for (std::size_t k=0;k<2;++k) m.desired[k]=s.support.anchors[k].desiredWorld;
            m.checkFeet(s.support);
            if (s.idle.phase==IdlePhase::Ready) {
                require(s.support.phase==SupportPhase::Holding,"Ready lost Holding");
                if (p.idle.phase!=IdlePhase::Ready) for (std::size_t k=0;k<2;++k) m.held[k]=s.support.anchors[k].actualWorld;
                for (std::size_t k=0;k<2;++k) require(s.support.anchors[k].actualWorld==m.held[k],"Foot moved during arm interaction");
                if (p.idle.phase==IdlePhase::Ready) {
                    require(s.root.position==p.root.position && s.root.rotation==p.root.rotation &&
                        s.idle.phaseSeconds==p.idle.phaseSeconds,"Ready root or Idle time changed");
                    const auto& joints=b.control()->binding().profile().joints;
                    for (std::size_t j=0;j<joints.size();++j)
                        if (std::find(armJoints.begin(),armJoints.end(),joints[j].id)==armJoints.end())
                            require(s.coordinates[j]==p.coordinates[j],"Frozen joint moved during interaction");
                }
            }
            const bool transport=s.interaction.phase==InteractionPhase::Transport || p.interaction.phase==InteractionPhase::Transport;
            const bool internalPlanning=s.preparationReach.state==ReachState::Planning || p.preparationReach.state==ReachState::Planning;
            if (transport) (internalPlanning ? m.transportPlanning:m.transportMotion).push_back(elapsed);
            else if (internalPlanning || s.reaches[0].state==ReachState::Planning || p.reaches[0].state==ReachState::Planning)
                m.planning.push_back(elapsed);

            m.checkAttachment(b,s);
            if (s.object && s.hands[0].enabled) {
                const auto gaps=measureSkin(s);
                if (p.object && p.object->state==ObjectState::Free && s.object->state==ObjectState::Attached) {
                    report << "grasp_tick=" << s.tick << " skin_gaps_m_index_middle_ring_pinky_thumb_palm=";
                    for (float gap:gaps) report << ' ' << gap;
                    report << '\n';
                    require(*std::max_element(gaps.begin(),gaps.end()-1)<.006f,"Skin pads did not approach the sphere");
                }
            }
            if (s.object && p.object && s.object->handle==p.object->handle) {
                const auto& a=*s.object; const auto& before=*p.object;
                if (a.state!=before.state) {
                    m.jump=std::max(m.jump,glm::length(a.worldPose.position-before.worldPose.position));
                    m.jumpAngle=std::max(m.jumpAngle,rotationError(a.worldPose.rotation,before.worldPose.rotation));
                    if (a.state==ObjectState::Attached) { ++m.attached; initialSphere=a.worldPose.position; }
                    else {
                        ++m.released;
                        require(a.owner==0 && s.armOwners[0]==ArmOwner::HandInteraction &&
                            glm::length(a.worldPose.position-initialSphere)>.08f,"Sphere was not transferred and released at target");
                    }
                } else if (a.state==ObjectState::Attached) require(a.effectorFromObject==before.effectorFromObject,"Attachment frame drifted");
                else require(a.worldPose.position==before.worldPose.position && a.worldPose.rotation==before.worldPose.rotation,"Free sphere moved");
            }
            if (!s.object && p.object) require(p.interaction.result=="Released" && p.armOwners[0]==ArmOwner::None,"Sphere deleted before checked retreat");
            if (s.idle.phase==IdlePhase::Resuming && p.idle.phase==IdlePhase::Ready) {
                ++m.resumed;
                require(s.idle.phaseSeconds==p.idle.phaseSeconds+runtime::BodyStepSeconds && p.hands[0].blend==1 &&
                    s.hands[0].blend<1 && s.hands[0].blend>.999f,"Resume skipped the saved Idle phase or finger blend");
                for (JointId joint:armJoints) {
                    const auto j=b.control()->binding().jointIndex(joint);
                    m.resumeStep=std::max(m.resumeStep,glm::length(s.coordinates[j]-p.coordinates[j]));
                }
            }
            for (double alpha:{.19,.53,.87}) {
                const auto d=b.display(alpha);
                m.checkFeet(d.support);
                m.checkAttachment(b,d);
                if (d.object && d.hands[0].enabled) measureSkin(d);
            }
        }
        require(demo.done(),"Shared replay failed: "+demo.reason());
        require(m.attached==3 && m.released==3 && m.resumed==3 && !b.snapshot().hands[0].enabled,"Incomplete cycle boundaries");
        require(m.jump<1e-6f && m.jumpAngle<1e-5f && m.attachmentError<1e-6f && m.attachmentAngle<1e-5f &&
            m.resumeStep<2e-5f,"Discontinuous attachment, release or Idle resume");
        report << "cycles=" << demo.completed() << " ticks=" << b.snapshot().tick << " exact_ready_feet=PASS\n";
        for (std::size_t k=0;k<2;++k)
            report << (k==0 ? "left":"right") << " foot_error_m=" << m.footError[k] << " angle_rad=" << m.footAngle[k] << " gap_m=" << m.gap[k] << '\n';
        report << "max_transition_jump_m=" << m.jump << " max_transition_jump_rad=" << m.jumpAngle
            << " max_attachment_error_m=" << m.attachmentError << " max_attachment_error_rad=" << m.attachmentAngle
            << " first_resume_step_rad=" << m.resumeStep << '\n'
            << "min_sampled_skin_vertex_gap_m=" << minimumGap << " skin_triangle_samples=" << skinSamples
            << " intersecting_samples=" << skinIntersections << '\n';
        require(skinSamples>0 && minimumGap>0 && skinIntersections==0,"Visible hand skin entered the sphere");
        reportTiming(report,"reach_and_retreat_planning",m.planning);
        reportTiming(report,"transport_planning",m.transportPlanning);
        reportTiming(report,"transport_motion",m.transportMotion);
    }

    void cancellations(BodyInstance& b,std::ostream& report)
    {
        constexpr auto rejected=BodyCommandStatus::Rejected;
        command(b,ResetBody{});
        startReady(b);
        const auto scene=b.snapshot().collisionScene;
        command(b,CreateInteractionSphere{1,{{2,.01f,0}}},rejected);
        require(!b.snapshot().object && b.snapshot().collisionScene==scene &&
            b.snapshot().support.phase==SupportPhase::Holding,"Rejected sphere creation changed floor/ownership");
        report << "sphere_over_floor=Rejected radius_m=.022 center_y_m=.010 scene_unchanged=PASS\n";
        create(b);
        const auto stale=b.snapshot().object->handle;
        command(b,DeleteInteractionObject{stale});
        create(b);
        const auto confirmation=prepare(b);
        const auto ready=b.snapshot();
        command(b,ConfirmGrasp{confirmation.object,confirmation.action-1,confirmation.epoch},rejected);
        command(b,ConfirmGrasp{confirmation.object,confirmation.action,confirmation.epoch-1},rejected);
        command(b,ConfirmGrasp{stale,confirmation.action,confirmation.epoch},rejected);
        command(b,MoveHeldObject{confirmation.object,ready.object->worldPose},rejected);
        command(b,ReleaseObject{confirmation.object},rejected);
        const auto& unchanged=b.snapshot();
        require(unchanged.interaction.phase==InteractionPhase::ReadyToGrasp && unchanged.interaction.geometryCurrent &&
            unchanged.armOwners[0]==ArmOwner::GraspPreparation && unchanged.coordinates==ready.coordinates &&
            unchanged.object->state==ObjectState::Free && unchanged.object->handle==confirmation.object && unchanged.object->owner==0 &&
            unchanged.object->worldPose.position==ready.object->worldPose.position,"Stale confirmation changed Ready ownership or state");
        const auto close=command(b,confirmation);
        command(b,confirmation,rejected);
        until(b,[](const auto& s) { return s.hands[0].closure>.15f; });
        command(b,CancelInteraction{close});
        until(b,[](const auto& s) {
            require(s.object && s.object->state==ObjectState::Free && !s.object->owner,"Pre-attachment cancel attached the sphere");
            return s.interaction.phase==InteractionPhase::Cancelled;
        });
        require(b.snapshot().armOwners[0]==ArmOwner::None && b.snapshot().interaction.contactHand==0 &&
            b.snapshot().object->worldPose.position==ready.object->worldPose.position,"Pre-attachment cancel lost separation or sphere pose");
        resume(b);
        report << "stale_action_epoch_generation=Rejected owner=GraspPreparation sphere=Free\n"
            << "cancel_during_closing=Cancelled owner=None sphere=Free returned_to=Idle\n";

        command(b,SetIdleStance{true});
        until(b,[](const auto& s) { return s.idle.phase==IdlePhase::Ready; });
        grasp(b);
        const auto held=*b.snapshot().object;
        command(b,ResetBody{},rejected);
        const auto move=command(b,MoveHeldObject{held.handle,app::readySpherePose(b,3)});
        until(b,[&](const auto& s) { return s.preparationReach.state==ReachState::Moving &&
            glm::length(s.object->worldPose.position-held.worldPose.position)>.005f; });
        command(b,CancelInteraction{move});
        until(b,[](const auto& s) { return s.interaction.result=="CancelledHolding"; });
        const auto& s=b.snapshot();
        require(s.interaction.phase==InteractionPhase::Holding && s.armOwners[0]==ArmOwner::HandInteraction &&
            s.object->state==ObjectState::Attached && s.object->owner==b.id() && s.object->effectorFromObject==held.effectorFromObject &&
            s.preparationReach.state==ReachState::Cancelled,"Cancelling motion changed the attachment");
        command(b,SetIdleStance{false},rejected);
        command(b,DeleteInteractionObject{held.handle},rejected);
        release(b);
        resume(b);
        report << "cancel_during_transport=CancelledHolding owner=HandInteraction sphere=Attached released_then_returned_to=Idle\n";
    }

    void demoRecovery(BodyInstance& b,std::ostream& report)
    {
        app::ReadySphereDemo demo;
        const auto step=[&] { demo.drive(b); b.step(); };
        const auto wait=[&](const auto& condition) {
            for (unsigned k=0;k<2800 && !condition(b.snapshot());++k) step();
            require(condition(b.snapshot()),"Sphere demo recovery timed out: "+demo.reason());
        };
        wait([](const auto& s) {
            return s.interaction.phase==InteractionPhase::Transport && s.preparationReach.state==ReachState::Moving;
        });
        const auto held=*b.snapshot().object;
        command(b,CancelInteraction{b.snapshot().interaction.action});
        wait([&](const auto&) { return demo.failed(); });
        const auto& s=b.snapshot();
        require(demo.canRecover(s) && s.interaction.result=="CancelledHolding" && s.object &&
            s.object->state==ObjectState::Attached && s.object->owner==b.id() &&
            s.object->effectorFromObject==held.effectorFromObject && s.armOwners[0]==ArmOwner::HandInteraction &&
            s.hands[0].closure==1 && s.support.phase==SupportPhase::Holding,
            "Stopped sphere demo lost its grip or recovery");
        demo.requestRecovery();
        wait([](const auto& frame) { return frame.idle.phase==IdlePhase::Idle; });
        require(!b.snapshot().object && !b.snapshot().hands[0].enabled && !demo.completed(),
            "Sphere demo did not restore Idle after interruption");
        wait([](const auto& frame) { return frame.interaction.result=="Grasped"; });
        require(b.snapshot().object->handle.generation>held.handle.generation,"Sphere retry reused its previous object");
        report << "demo_cancel_transport=CancelledHolding recover=Released+Idle retry_grasp=PASS\n";
    }

    void staleGeometry(BodyInstance& b,std::ostream& report)
    {
        command(b,SetIdleStance{true});
        until(b,[](const auto& s) { return s.idle.phase==IdlePhase::Ready; });
        create(b);
        const auto confirmation=prepare(b);
        // Fault injection exercises the confirmation gate with a once-valid ReadyToGrasp certificate.
        auto& control=const_cast<DirectBodyControl&>(*b.control());
        const auto saved=control.coordinate(JointId::RightElbow,0);
        control.setCoordinate(JointId::RightElbow,0,saved+.02f);
        command(b,confirmation,BodyCommandStatus::Rejected);
        const auto& s=b.snapshot();
        require(s.interaction.phase==InteractionPhase::PreparationBlocked && !s.interaction.geometryCurrent &&
            s.armOwners[0]==ArmOwner::GraspPreparation && s.object->state==ObjectState::Free && s.object->owner==0 &&
            s.hands[0].closure==0,"Stale Ready geometry closed the hand or released ownership");
        control.setCoordinate(JointId::RightElbow,0,saved);
        command(b,CancelInteraction{confirmation.action});
        until(b,[](const auto& settled) { return settled.interaction.phase==InteractionPhase::Cancelled; });
        resume(b);
        report << "stale_ready_geometry=Rejected owner=GraspPreparation sphere=Free checked_cancel_then_returned_to=Idle\n";
    }

    void setArm(DirectBodyControl& c,const ArmCoordinates& q)
    {
        for (std::size_t k=0;k<3;++k) require(c.setCoordinate(JointId::LeftShoulder,k,q[k]).status==CommandStatus::Applied,"Shoulder fixture limit");
        require(c.setCoordinate(JointId::LeftElbow,0,q[3]).status==CommandStatus::Applied,"Elbow fixture limit");
        for (std::size_t k=0;k<2;++k) require(c.setCoordinate(JointId::LeftWrist,k,q[k+4]).status==CommandStatus::Applied,"Wrist fixture limit");
    }

    glm::mat4 effectorAt(const BodyInstance& b,ArmCoordinates q,EffectorId id)
    {
        DirectBodyControl c=*b.control();
        setArm(c,q);
        animation::Pose pose(b.asset().skeleton());
        c.apply(pose.locals);
        animation::computeGlobalTransforms(b.asset().skeleton(),pose.locals,pose.globals);
        return rootMatrix(b.snapshot().root)*c.binding().effectorFrame(id,pose.globals);
    }

    void sweptCollision(BodyInstance& b,std::ostream& report)
    {
        startReady(b);
        const auto& s=b.snapshot();
        const auto root=rootMatrix(s.root);
        const auto& control=*b.control();
        const ArmCoordinates middle{0,.60f,0,.15f,0,0};
        auto from=middle, to=middle;
        from[4]=-.42f; to[4]=.42f;
        const auto wrist=effectorAt(b,middle,EffectorId::LeftWrist);
        const auto axis=glm::vec3{wrist[0]};
        const auto downward=glm::normalize(glm::vec3{0,-1,0}+axis*axis.y);
        // A deliberately large offset exercises a mid-interval collision caused only by wrist rotation.
        const auto worldOffset=downward*((wrist[3].y-.016f)/-downward.y);
        const auto wristOffset=glm::vec3{glm::inverse(wrist)*glm::vec4{worldOffset,0}};
        const auto center=glm::dvec3{wrist*glm::vec4{wristOffset,1}};
        CollisionContext hand;
        hand.handEnvelopes[0]=true;
        hand.fixedLowerForms=lowerBodyColliders(control.binding(),s.pose.globals,root,calibrateSoleContacts(control.binding()));
        hand.soleContacts={std::pair{14u,SupportFloorObject},std::pair{17u,SupportFloorObject}};
        auto payload=hand;
        payload.contactHand=3; payload.contactObject=(1ull<<63)|1;
        payload.carried=CarriedObject{payload.contactObject,EffectorId::LeftWrist,
            glm::translate(glm::mat4{1},wristOffset),world::Sphere{.022}};
        const std::array path{from,to};
        const auto check=[&](const world::CollisionScene& scene,std::span<const ArmCoordinates> route,CollisionContext context) {
            return checkArmPath(control,0,route,root,scene,{},std::move(context));
        };
        const auto requireSweep=[&](const world::CollisionScene& scene,CollisionContext armContext,CollisionContext carriedContext,
            std::uint32_t bodyForm,std::uint64_t sceneObject) {
            require(check(scene,path,armContext).status==PathStatus::Clear,"Arm-only sweep intercepted obstacle");
            require(check(scene,std::span{&from,1},carriedContext).status==PathStatus::Clear &&
                check(scene,std::span{&to,1},carriedContext).status==PathStatus::Clear,"Payload endpoints are not clear");
            const auto hit=check(scene,path,carriedContext);
            require(hit.status==PathStatus::Blocked && hit.bodyForm==10 && hit.otherBodyForm==bodyForm && hit.object==sceneObject,
                "Wrong payload sweep witness: "+std::to_string(hit.bodyForm)+"/"+std::to_string(hit.otherBodyForm)+" "+pathStatusName(hit.status));
            report << "arm_and_payload_endpoints=Clear sweep=Blocked body_form=" << hit.bodyForm
                << " other_body_form=" << hit.otherBodyForm << " object=" << hit.object << " gap_m=" << hit.contact.gap
                << " interval=" << hit.fromTick << ':' << hit.toTick << '\n';
        };
        auto floor=s.collisionScene->objects().front();
        floor.filter.mask=0; // The canonical floor stays active even when the user mask excludes it.
        requireSweep(world::CollisionScene(2,{floor}),hand,payload,0,SupportFloorObject);

        const world::CollisionScene empty(3,{});
        const auto form=std::find_if(hand.fixedLowerForms.begin(),hand.fixedLowerForms.end(),[](const auto& c) { return c.id==13; });
        // Relocate one lower proxy in the fixture to isolate its pair from the floor.
        form->capsule={center,center,.006};
        payload.fixedLowerForms=hand.fixedLowerForms;
        requireSweep(empty,hand,payload,13,payload.contactObject);
    }

    void blockedTransfer(BodyInstance& b,std::ostream& report)
    {
        const RigidPose localGrip{{.002f,0,-.004f}};
        const auto sphereAt=[&](unsigned sample) {
            return rigidPose(rigidMatrix(app::interactionDemoPose(b.control()->binding(),arm,sample,b.snapshot().root))*
                glm::inverse(rigidMatrix(localGrip)));
        };
        const auto target=sphereAt(4);
        const world::SceneObject obstacle{80,glm::dvec3{rigidMatrix(target)*glm::vec4{0,0,.0215f,1}},
            {1,0,0,0},{},world::Sphere{.0002}};
        command(b,SetDirectMode{true});
        command(b,SetCollisionScene{std::make_shared<const world::CollisionScene>(1,std::vector{obstacle}),{}});
        command(b,EnableSupport{});
        command(b,AlignSupport{});
        until(b,[](const auto& s) { return s.support.phase==SupportPhase::Holding; });
        command(b,CreateInteractionSphere{1,sphereAt(0),.022f,localGrip});
        command(b,prepare(b));
        until(b,[](const auto& s) { return s.interaction.result=="Grasped"; });
        const auto held=b.snapshot();
        const auto& c=*b.control();
        CollisionContext hand;
        hand.handEnvelopes[0]=true;
        hand.fixedLowerForms=lowerBodyColliders(c.binding(),held.pose.globals,rootMatrix(held.root),calibrateSoleContacts(c.binding()));
        hand.soleContacts={std::pair{14u,SupportFloorObject},std::pair{17u,SupportFloorObject}};
        const ArmCoordinates from{c.coordinate(JointId::LeftShoulder,0),c.coordinate(JointId::LeftShoulder,1),
            c.coordinate(JointId::LeftShoulder,2),c.coordinate(JointId::LeftElbow,0),c.coordinate(JointId::LeftWrist,0),c.coordinate(JointId::LeftWrist,1)};
        const ArmCoordinates to{.97f,.45f,0,.9f,.12f,0};
        require(checkArmPath(c,0,std::array{from,to},rootMatrix(held.root),*held.collisionScene,{},hand).status==PathStatus::Clear,
            "Transfer obstacle also blocks the arm-only route");
        command(b,MoveHeldObject{held.object->handle,target});
        until(b,[](const auto& s) { return s.interaction.phase==InteractionPhase::Failed; });
        const auto& failure=b.snapshot();
        const auto& hit=failure.preparationReach.planning.directCheck;
        require(hit.status==PathStatus::Blocked && hit.bodyForm==10 && hit.object==obstacle.id,
            std::string("Rejected transfer witness: ")+pathStatusName(hit.status)+" "+std::to_string(hit.bodyForm)+" object="+std::to_string(hit.object));
        require(failure.coordinates==held.coordinates && failure.armOwners[0]==ArmOwner::HandInteraction &&
            failure.object->state==ObjectState::Attached && failure.object->owner==b.id() &&
            failure.object->effectorFromObject==held.object->effectorFromObject &&
            glm::length(failure.object->worldPose.position-held.object->worldPose.position)<1e-6f,
            "Rejected transfer changed pose or attachment");
        report << "blocked_transfer=" << failure.interaction.result << " owner=HandInteraction sphere=Attached body_form="
            << hit.bodyForm << " object=" << hit.object << " gap_m=" << hit.contact.gap << '\n';
        command(b,MoveHeldObject{held.object->handle,sphereAt(1)});
        until(b,[](const auto& s) { return s.interaction.result=="Moved"; });
        release(b);
        command(b,DeleteInteractionObject{held.object->handle});
        require(!b.snapshot().object && b.snapshot().support.phase==SupportPhase::Holding,"Blocked transfer did not recover");
        report << "blocked_transfer_retry_move_and_release=PASS\n";
    }

    void blockedRetreat(BodyInstance& b,std::ostream& report)
    {
        startReady(b);
        grasp(b);
        const auto object=*b.snapshot().object;
        const ArmCoordinates q{1.1f,.65f,0,.15f,.50f,.20f};
        const auto target=rigidPose(effectorAt(b,q,EffectorId::LeftGrip)*object.effectorFromObject);
        command(b,MoveHeldObject{object.handle,target});
        until(b,[](const auto& s) { return s.interaction.result=="Moved"; });
        const auto held=b.snapshot().object->worldPose;
        const auto action=command(b,ReleaseObject{object.handle});
        command(b,CancelInteraction{action},BodyCommandStatus::Rejected);
        until(b,[](const auto& s) { return s.interaction.phase==InteractionPhase::ReleaseBlocked; });
        const auto& s=b.snapshot();
        require(s.interaction.result=="ReleaseBlocked" && !s.interaction.reason.empty() && !s.interaction.contactHand &&
            s.object->state==ObjectState::Free && !s.object->owner && s.armOwners[0]==ArmOwner::None &&
            glm::length(s.object->worldPose.position-held.position)<1e-6f && s.support.phase==SupportPhase::Holding,
            "Blocked retreat misreported ownership, support or pose");
        report << "blocked_retreat=" << s.interaction.result << " owner=None sphere=Free reason=" << s.interaction.reason << '\n';
        command(b,SetIdleStance{false},BodyCommandStatus::Rejected);
        // Explicit recovery from a blocked retreat removes the Free object before moving the arm again.
        command(b,DeleteInteractionObject{object.handle});
        auto away=app::preparationPregrip(arm,app::interactionDemoPose(b.control()->binding(),arm,3,b.snapshot().root));
        command(b,StartReach{away});
        until(b,[](const auto& settled) { return settled.reaches[0].state==ReachState::Succeeded; });
        command(b,SetIdleStance{false});
        until(b,[](const auto& resumed) { return resumed.idle.phase==IdlePhase::Idle; });
        report << "blocked_retreat_explicit_recovery=PASS\n";
    }
}

int main(int argc,char** argv)
{
    try {
        if (argc!=2) throw std::invalid_argument("Expected output directory");
        const std::filesystem::path output{argv[1]};
        std::filesystem::create_directories(output);
        std::ofstream report(output/"ready-sphere.txt");
        if (!report) throw std::runtime_error("Cannot open Ready sphere report");
        report << std::setprecision(10);
        const auto asset=model_import::ModelImporter::loadFromFile(
            std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx",
            {.rebuildSmoothNormals=true,.omitPreparedGloves=true});
        const auto profile=raphtaliaBodyProfile();
        BodyInstance replay{1,asset,&profile}, cancel{2,asset,&profile}, sweep{3,asset,&profile};
        BodyInstance blockedMove{4,asset,&profile}, blockedExit{5,asset,&profile};
        BodyInstance demoCancel{6,asset,&profile};
        cycles(replay,report);
        cancellations(cancel,report);
        demoRecovery(demoCancel,report);
        staleGeometry(cancel,report);
        sweptCollision(sweep,report);
        blockedTransfer(blockedMove,report);
        blockedRetreat(blockedExit,report);
        report << "result=PASS\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
