#include "../apps/common/ReadyCubeDemo.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "hand_skin_audit.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace
{
    using namespace julia;
    using namespace julia::embodiment;
    constexpr auto left=ArmSide::Left;
    constexpr auto cubeShape=world::Box{glm::dvec3{InteractionCubeHalfExtent}};

    void require(bool condition,std::string reason)
    {
        if (!condition) throw std::runtime_error(std::move(reason));
    }

    std::uint64_t command(BodyInstance& b,BodyOperation op,BodyCommandStatus status=BodyCommandStatus::Applied)
    {
        const auto id=b.enqueue(std::move(op)).id;
        b.step();
        require(b.results().size()==1 && b.results().front().id==id && b.results().front().status==status,
            b.results().empty() ? "Missing acknowledgement":b.results().front().reason);
        return id;
    }

    template<class Step,class Predicate> void until(BodyInstance& b,Step step,Predicate predicate)
    {
        for (unsigned k=0;k<5000 && !predicate(b.snapshot());++k) step();
        const auto& s=b.snapshot();
        require(predicate(s),std::string("Timed out in ")+interactionPhaseName(s.interaction.phase)+" "+
            s.interaction.reason+" "+reachReasonName(s.preparationReach.reason));
    }

    template<class Predicate> void until(BodyInstance& b,Predicate predicate)
    { until(b,[&] { b.step(); },predicate); }

    glm::mat4 rootFrame(const RootPose& r) { return rigidMatrix({r.position,r.rotation}); }

    double rotationError(glm::quat a,glm::quat b) { return glm::length(orientationResidual(a,b)); }

    void timing(std::ostream& out,const char* name,std::vector<double>& values)
    {
        require(!values.empty(),"Missing timing samples");
        std::sort(values.begin(),values.end());
        out << name << " n=" << values.size() << " p95_us=" << values[(values.size()*95+99)/100-1]
            << " max_us=" << values.back() << '\n';
    }

    void cycles(BodyInstance& b,std::ostream& out)
    {
        app::ReadyCubeDemo demo;
        b.setIdleView(app::readyCubeReplayView);
        std::vector<double> planning,transferPlanning,transfer;
        std::array<float,2> foot{},sole{},footAngle{};
        std::array<glm::mat4,2> anchors{};
        hand_skin_audit::Surface skin(b.asset());
        float signedSkinGap=std::numeric_limits<float>::infinity(), closestClosure=0;
        std::size_t closestFinger=0;
        double closestTick=0;
        unsigned skinSamples=0,skinIntersections=0;
        const auto measure=[&](const auto& frame,double tick) {
            const auto sample=skin.measure(b,frame);
            ++skinSamples;
            if (sample.intersects) ++skinIntersections;
            const auto gap=std::ranges::min_element(sample.vertexGaps);
            if (*gap<signedSkinGap) {
                signedSkinGap=*gap;
                closestFinger=static_cast<std::size_t>(gap-sample.vertexGaps.begin());
                closestClosure=frame.hands[0].closure;
                closestTick=tick;
            }
            return sample.vertexGaps;
        };
        float attachJump=0,detachJump=0,relative=0,moveStep=0,returnStep=0;
        double attachAngle=0,detachAngle=0,relativeAngle=0,moveStepAngle=0;
        unsigned grasped=0,released=0,enteredIdle=0;
        glm::vec2 eyesAtSpawn{0};
        RigidPose initialPose;
        std::uint64_t spawned=0,settled=0,moved=0,detached=0,separated=0;
        double savedIdlePhase=0;
        while (b.snapshot().tick<app::ReadyCubeDemo::TickLimit) {
            demo.drive(b);
            if (demo.done() || demo.failed()) break;
            const auto start=std::chrono::steady_clock::now();
            b.step();
            const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            const auto& s=b.snapshot(); const auto& p=b.previousSnapshot();
            const auto& i=s.interaction;
            const bool plan=s.preparationReach.state==ReachState::Planning || p.preparationReach.state==ReachState::Planning;
            if (plan || s.reaches[0].state==ReachState::Planning || p.reaches[0].state==ReachState::Planning)
                planning.push_back(us);
            if (i.phase==InteractionPhase::Transport || p.interaction.phase==InteractionPhase::Transport) {
                if (plan) transferPlanning.push_back(us);
                else transfer.push_back(us);
            }
            if (s.tick==1) for (std::size_t k=0;k<2;++k) anchors[k]=s.support.anchors[k].desiredWorld;
            for (std::size_t k=0;k<2;++k) {
                const auto& a=s.support.anchors[k];
                require(a.bound && a.desiredWorld==anchors[k] && a.positionError<=SupportPositionTolerance &&
                    a.angularError<=SupportAngularTolerance && a.minGap>=-SupportSurfaceTolerance &&
                    a.maxGap<=SupportSurfaceTolerance,"Drift or loss of support in cube scene");
                foot[k]=std::max(foot[k],a.positionError);
                footAngle[k]=std::max(footAngle[k],a.angularError);
                sole[k]=std::max({sole[k],-a.minGap,a.maxGap});
                if (s.idle.phase==IdlePhase::Ready && p.idle.phase==IdlePhase::Ready)
                    require(a.actualWorld==p.support.anchors[k].actualWorld,"Ready foot changed after planning");
            }
            if (s.object && !p.object) {
                require(s.idle.phase==IdlePhase::Idle && demo.opacity(static_cast<double>(s.tick))==0,"Spawn was visible in one tick");
                spawned=s.tick;
                eyesAtSpawn=s.eyes.gazeDegrees;
                initialPose=s.object->worldPose;
                const auto it=std::find_if(s.collisionScene->objects().begin(),s.collisionScene->objects().end(),
                    [&](const auto& o) { return o.id==s.object->collider().id; });
                require(it!=s.collisionScene->objects().end() && std::get<world::Box>(it->shape)==cubeShape,
                    "Free cube metric dimensions disagree");
            }
            if (s.idle.phase==IdlePhase::Settling && p.idle.phase==IdlePhase::Idle) {
                require(glm::length(p.eyes.gazeDegrees-eyesAtSpawn)>2 && p.tick>spawned &&
                    demo.opacity(static_cast<double>(p.tick))>0,"Head preparation led the eyes");
                savedIdlePhase=p.idle.phaseSeconds;
            }
            if (s.idle.phase==IdlePhase::Ready && p.idle.phase==IdlePhase::Settling) settled=s.tick;
            if (s.armOwners[0]!=ArmOwner::None)
                require(settled && s.tick>settled && s.idle.phase==IdlePhase::Ready && s.support.phase==SupportPhase::Holding,
                    "Cube arm acquired before frozen Ready and Holding");
            if (s.object && s.hands[0].enabled) {
                const auto gaps=measure(s,static_cast<double>(s.tick));
                if (p.object) measure(b.display(.5),static_cast<double>(s.tick)-.5);
                if (p.object && p.object->state==ObjectState::Free && s.object->state==ObjectState::Attached) {
                    out << "grasp_tick=" << s.tick << " skin_gaps_m_index_middle_ring_pinky_thumb_palm=";
                    for (float gap:gaps) out << ' ' << gap;
                    out << '\n';
                    require(*std::max_element(gaps.begin(),gaps.end()-1)<.006f,"Skin pads did not approach cube");
                }
            }
            if (s.idle.phase==IdlePhase::Ready && p.idle.phase==IdlePhase::Ready) {
                for (const auto joint:{JointId::Head,JointId::Neck,JointId::Thorax,JointId::Chest}) {
                    const auto k=b.control()->binding().jointIndex(joint);
                    require(s.coordinates[k]==p.coordinates[k],"Gaze changed frozen body geometry");
                }
                require(s.root.position==p.root.position,"Ready root changed");
            }
            if (s.object && s.object->state==ObjectState::Attached) {
                const auto gripFor=[&](const auto& frame) {
                    return rootFrame(frame.root)*b.control()->binding().effectorFrame(EffectorId::LeftGrip,frame.pose.globals);
                };
                require(p.object && s.object->owner==b.id() && s.armOwners[0]==ArmOwner::HandInteraction &&
                    s.hands[0].grip==HandGrip::Cube,"Held cube lost ownership or hand pose");
                if (p.object->state==ObjectState::Free) {
                    ++grasped;
                    const auto from=p.object->worldPose,to=s.object->worldPose;
                    attachJump=std::max(attachJump,glm::length(to.position-from.position));
                    attachAngle=std::max(attachAngle,rotationError(to.rotation,from.rotation));
                }
                if (i.phase==InteractionPhase::Transport) {
                    moveStep=std::max(moveStep,glm::length(p.object->worldPose.position-s.object->worldPose.position));
                    moveStepAngle=std::max(moveStepAngle,rotationError(p.object->worldPose.rotation,s.object->worldPose.rotation));
                }
                const auto check=[&](const auto& frame) {
                    const auto actual=rigidPose(glm::inverse(gripFor(frame))*rigidMatrix(frame.object->worldPose));
                    const auto saved=rigidPose(frame.object->effectorFromObject);
                    relative=std::max(relative,glm::length(actual.position-saved.position));
                    relativeAngle=std::max(relativeAngle,rotationError(actual.rotation,saved.rotation));
                };
                check(s);
                check(b.display(.5));
                if (i.result=="Moved" && p.interaction.result!="Moved") {
                    moved=s.tick;
                    const auto angle=rotationError(initialPose.rotation,s.object->worldPose.rotation);
                    require(angle>.06 && angle<.6,"Cube did not make a small checked turn");
                    out << "move_tick=" << moved << " distance_m=" << glm::length(s.object->worldPose.position-initialPose.position)
                        << " angle_rad=" << angle << '\n';
                }
            }
            if (s.object && s.object->state==ObjectState::Free && p.object && p.object->state==ObjectState::Attached) {
                detached=s.tick;
                detachJump=std::max(detachJump,glm::length(p.object->worldPose.position-s.object->worldPose.position));
                detachAngle=std::max(detachAngle,rotationError(p.object->worldPose.rotation,s.object->worldPose.rotation));
            }
            if (i.result=="Released" && p.interaction.result!="Released") {
                ++released; separated=s.tick;
                require(s.object && s.object->state==ObjectState::Free && s.object->owner==0 && s.armOwners[0]==ArmOwner::None &&
                    s.preparationReach.collisionChecked && i.geometryCheck.status==PathStatus::Clear,"Missing checked separation");
            }
            if (!s.object && p.object)
                require(p.interaction.result=="Released" && demo.opacity(static_cast<double>(p.tick))==0,"Cube deleted before fade or separation");
            if (s.idle.phase==IdlePhase::Resuming && p.idle.phase==IdlePhase::Ready) {
                require(std::abs(s.idle.phaseSeconds-savedIdlePhase-runtime::BodyStepSeconds)<1e-11,"Idle phase restarted");
                for (std::size_t k=0;k<s.coordinates.size();++k)
                    returnStep=std::max(returnStep,glm::length(s.coordinates[k]-p.coordinates[k]));
            }
            if (s.idle.phase==IdlePhase::Idle && p.idle.phase==IdlePhase::Resuming && separated) {
                ++enteredIdle;
                out << "cycle=" << enteredIdle << " spawned=" << spawned << " ready=" << settled << " moved=" << moved
                    << " detached=" << detached << " separated=" << separated << " idle=" << s.tick << '\n';
            }
        }
        require(demo.done() && grasped==3 && released==3 && enteredIdle==3,demo.reason());
        require(!b.snapshot().object && !b.snapshot().hands[0].enabled && glm::length(b.snapshot().eyes.gazeDegrees)<2,
            "Idle did not restore the camera and resting hand");
        require(attachJump<1e-6f && detachJump<1e-6f && attachAngle<2e-6 && detachAngle<2e-6 &&
            relative<1e-6f && relativeAngle<2e-6 && returnStep<1e-4f,"Visible frame discontinuity");
        out << "max_foot_error_m_left_right=" << foot[0] << ' ' << foot[1] << "\nmax_foot_angle_rad_left_right="
            << footAngle[0] << ' ' << footAngle[1] << "\nmax_sole_gap_m_left_right=" << sole[0] << ' ' << sole[1]
            << "\nattach_jump_m_rad=" << attachJump << ' ' << attachAngle << "\ndetach_jump_m_rad=" << detachJump << ' ' << detachAngle
            << "\nmax_attachment_residual_m_rad=" << relative << ' ' << relativeAngle
            << "\nmax_transport_step_m_rad=" << moveStep << ' ' << moveStepAngle << "\nresume_joint_step_rad=" << returnStep
            << "\nmin_sampled_skin_vertex_gap_m=" << signedSkinGap << " tick=" << closestTick
            << " closure=" << closestClosure << " region=" << hand_skin_audit::regions[closestFinger]
            << "\nskin_triangle_samples=" << skinSamples << " intersecting_samples=" << skinIntersections << '\n';
        require(skinSamples>0 && signedSkinGap>0 && skinIntersections==0,"Visible hand skin entered the cube");
        timing(out,"planning",planning);
        timing(out,"transfer_planning",transferPlanning);
        timing(out,"transfer_motion",transfer);
    }

    glm::mat4 wristAt(const BodyInstance& b,const ArmCoordinates& q)
    {
        DirectBodyControl c=*b.control();
        for (int k=0;k<3;++k) c.setCoordinate(JointId::LeftShoulder,k,q[k]);
        c.setCoordinate(JointId::LeftElbow,0,q[3]);
        for (int k=0;k<2;++k) c.setCoordinate(JointId::LeftWrist,k,q[k+4]);
        animation::Pose pose(b.asset().skeleton());
        c.apply(pose.locals);
        animation::computeGlobalTransforms(b.asset().skeleton(),pose.locals,pose.globals);
        return rootFrame(b.snapshot().root)*c.binding().effectorFrame(EffectorId::LeftWrist,pose.globals);
    }

    void cubeSweep(BodyInstance& b,std::ostream& out)
    {
        command(b,StartIdle{});
        until(b,[](const auto& s) { return s.idle.phase==IdlePhase::Idle; });
        command(b,SetIdleStance{true});
        until(b,[](const auto& s) { return s.idle.phase==IdlePhase::Ready; });
        const auto& s=b.snapshot();
        const auto root=rootFrame(s.root);
        const auto& c=*b.control();
        CollisionContext hand;
        hand.handEnvelopes[0]=true;
        hand.fixedLowerForms=lowerBodyColliders(c.binding(),s.pose.globals,root,calibrateSoleContacts(c.binding()));
        hand.soleContacts={std::pair{14u,SupportFloorObject},std::pair{17u,SupportFloorObject}};
        const ArmCoordinates middle{0,.60f,0,.15f,0,0};
        auto from=middle,to=middle;
        from[4]=-.42f; to[4]=.42f;
        const auto wrist=wristAt(b,middle);
        const auto axis=glm::vec3{wrist[0]};
        const auto downward=glm::normalize(glm::vec3{0,-1,0}+axis*axis.y);
        const auto offset=downward*((wrist[3].y-static_cast<float>(glm::length(cubeShape.halfExtent))+.003f)/-downward.y);
        const auto center=glm::vec3{wrist[3]}+offset;
        const auto cornerDir=glm::normalize(glm::vec3{1});
        const glm::vec3 down{0,-1,0};
        const auto q=glm::normalize(glm::quat{1+glm::dot(cornerDir,down),glm::cross(cornerDir,down)});
        auto payload=hand;
        payload.contactHand=3; payload.contactObject=(1ull<<63)|1;
        payload.carried=CarriedObject{payload.contactObject,EffectorId::LeftWrist,
            glm::inverse(wrist)*rigidMatrix({center,q}),cubeShape};
        const std::array path{from,to};
        const auto check=[&](const world::CollisionScene& scene,std::span<const ArmCoordinates> route,const CollisionContext& ctx) {
            return checkArmPath(c,0,route,root,scene,{},ctx);
        };
        const auto corner=glm::dvec3{center+q*glm::vec3{cubeShape.halfExtent}};
        const auto requireCornerHit=[&](const world::CollisionScene& scene,std::uint64_t object,std::uint32_t bodyForm) {
            require(check(scene,path,hand).status==PathStatus::Clear,"Arm hit the corner fixture");
            require(check(scene,std::span{&from,1},payload).status==PathStatus::Clear &&
                check(scene,std::span{&to,1},payload).status==PathStatus::Clear,"Corner fixture endpoints blocked");
            auto sphere=payload;
            sphere.carried->shape=world::Sphere{InteractionCubeHalfExtent};
            require(check(scene,path,sphere).status==PathStatus::Clear,"Inscribed sphere hit: fixture does not isolate the corner");
            const auto hit=check(scene,path,payload);
            require(hit.status==PathStatus::Blocked && hit.object==object && hit.bodyForm==10 && hit.otherBodyForm==bodyForm,
                "Expected cube-only corner hit, got "+std::string(pathStatusName(hit.status))+" body="+std::to_string(hit.bodyForm)+
                " other="+std::to_string(hit.otherBodyForm)+" object="+std::to_string(hit.object));
            out << "corner_between_ticks=Blocked arm_and_endpoints=Clear sphere=Clear gap_m=" << hit.contact.gap
                << " body=" << bodyForm << " object=" << object << " interval=" << hit.fromTick << ':' << hit.toTick << '\n';
        };
        auto floor=s.collisionScene->objects().front();
        floor.filter.mask=0;
        requireCornerHit(world::CollisionScene(2,{floor}),SupportFloorObject,0);
        world::SceneObject box{81,corner,glm::dquat{q},{},world::Box{glm::dvec3{.001}}};
        requireCornerHit(world::CollisionScene(3,{box}),box.id,0);
        const auto calf=std::find_if(hand.fixedLowerForms.begin(),hand.fixedLowerForms.end(),[](const auto& f) { return f.id==13; });
        calf->capsule={corner,corner,.001};
        payload.fixedLowerForms=hand.fixedLowerForms;
        const world::CollisionScene empty(4,{});
        requireCornerHit(empty,payload.contactObject,13);

        const auto upper=bodyColliders(c.binding(),s.pose.globals,root);
        const auto head=upper.back().capsule.b;
        payload.carried->effectorFromObject=glm::inverse(wrist)*rigidMatrix({glm::vec3{head},q});
        const auto hit=check(empty,std::span{&middle,1},payload);
        require(hit.status==PathStatus::InitialIntersection && hit.bodyForm==10 && hit.otherBodyForm==9,"Upper head form missed cube");
        out << "head_cube=InitialIntersection gap_m=" << hit.contact.gap << '\n';
    }

    void cancelAndRecover(BodyInstance& b,std::ostream& out)
    {
        app::ReadyCubeDemo demo;
        b.setIdleView(app::readyCubeReplayView);
        const auto step=[&] { demo.drive(b); b.step(); };
        until(b,step,[](const auto& s) { return s.interaction.phase==InteractionPhase::Transport && s.preparationReach.state==ReachState::Moving; });
        const auto held=*b.snapshot().object;
        command(b,CancelInteraction{b.snapshot().interaction.action});
        until(b,step,[&](const auto&) { return demo.failed(); });
        const auto& s=b.snapshot();
        require(s.interaction.result=="CancelledHolding" && s.object && s.object->owner==b.id() &&
            s.object->state==ObjectState::Attached && s.object->effectorFromObject==held.effectorFromObject &&
            s.armOwners[0]==ArmOwner::HandInteraction && demo.canRecover(s),"Cube cancellation corrupted ownership");
        command(b,DeleteInteractionObject{held.handle},BodyCommandStatus::Rejected);
        command(b,SetIdleStance{false},BodyCommandStatus::Rejected);
        demo.requestRecovery();
        until(b,step,[](const auto& state) { return state.idle.phase==IdlePhase::Idle; });
        require(!b.snapshot().object && !b.snapshot().hands[0].enabled && demo.completed()==0,"Interrupted cycle counted or retained cube");
        until(b,step,[](const auto& state) { return state.interaction.result=="Grasped"; });
        require(b.snapshot().object->handle.generation>held.handle.generation,"Retry did not create a new generation");
        command(b,ReleaseObject{held.handle},BodyCommandStatus::Rejected);
        out << "cancel_transport=CancelledHolding attachment=preserved recover=Released+Idle retry_grasp=PASS stale_generation=Rejected\n";
    }

    void cancelReachAndRecover(BodyInstance& b,std::ostream& out)
    {
        app::ReadyCubeDemo demo;
        b.setIdleView(app::readyCubeReplayView);
        const auto step=[&] { demo.drive(b); b.step(); };
        until(b,step,[](const auto& s) {
            return s.armOwners[0]==ArmOwner::Reach && s.reaches[0].state==ReachState::Moving;
        });
        const auto free=*b.snapshot().object;
        command(b,CancelReach{left,b.snapshot().reaches[0].action});
        until(b,step,[&](const auto&) { return demo.failed(); });
        const auto& s=b.snapshot();
        require(demo.canRecover(s) && s.reaches[0].state==ReachState::Cancelled &&
            s.armOwners[0]==ArmOwner::None && !s.hands[0].enabled && s.support.phase==SupportPhase::Holding &&
            s.object && s.object->state==ObjectState::Free && s.object->owner==0 &&
            s.object->worldPose.position==free.worldPose.position,"Early cube cancellation left no recovery");
        demo.requestRecovery();
        until(b,step,[](const auto& frame) { return frame.idle.phase==IdlePhase::Idle; });
        require(!b.snapshot().object && !b.snapshot().hands[0].enabled && !demo.completed(),
            "Early cube cancellation did not restore Idle");
        until(b,step,[](const auto& frame) { return frame.interaction.result=="Grasped"; });
        require(b.snapshot().object->handle.generation>free.handle.generation,"Early cube retry reused its previous object");
        out << "cancel_initial_reach=Cancelled cube=Free recover=Idle retry_grasp=PASS\n";
    }

    void blockedTransfer(BodyInstance& b,std::ostream& out)
    {
        const auto target=app::readyCubePose(b,{},true);
        const float h=static_cast<float>(InteractionCubeHalfExtent);
        const auto corner=glm::dvec3{rigidMatrix(target)*glm::vec4{h-.001f,.012f,h-.001f,1}};
        const world::SceneObject obstruction{82,corner,{1,0,0,0},{},world::Sphere{.0005}};
        command(b,SetDirectMode{true});
        command(b,SetCollisionScene{std::make_shared<const world::CollisionScene>(1,std::vector{obstruction}),{}});
        command(b,EnableSupport{});
        command(b,AlignSupport{});
        until(b,[](const auto& s) { return s.support.phase==SupportPhase::Holding; });
        const auto start=app::readyCubePose(b,b.snapshot().root);
        const auto scene=b.snapshot().collisionScene;
        command(b,CreateInteractionBox{1,start,world::Box{{.012,.015,.012}}},BodyCommandStatus::Rejected);
        require(!b.snapshot().object && b.snapshot().collisionScene==scene,"Rejected dimensions changed scene");
        command(b,CreateInteractionBox{1,start});
        const auto handle=b.snapshot().object->handle;
        const auto source=command(b,StartReach{app::preparationPregrip(left,
            rigidPose(rigidMatrix(start)*rigidMatrix(interactionCubeLocalGrip)))});
        until(b,[](const auto& s) { return s.reaches[0].state==ReachState::Succeeded; });
        const auto prep=command(b,PrepareGrasp{handle,left,source,b.snapshot().epoch});
        until(b,[](const auto& s) { return s.interaction.phase==InteractionPhase::ReadyToGrasp; });
        const auto epoch=b.snapshot().epoch;
        command(b,ConfirmGrasp{handle,prep-1,epoch},BodyCommandStatus::Rejected);
        command(b,ConfirmGrasp{handle,prep,epoch+1},BodyCommandStatus::Rejected);
        require(b.snapshot().interaction.geometryCurrent && b.snapshot().armOwners[0]==ArmOwner::GraspPreparation &&
            b.snapshot().object->state==ObjectState::Free,"Stale confirmation corrupted cube preparation");
        command(b,ConfirmGrasp{handle,prep,epoch});
        until(b,[](const auto& s) { return s.interaction.result=="Grasped"; });
        const auto held=b.snapshot();
        command(b,MoveHeldObject{handle,target});
        until(b,[](const auto& s) { return s.interaction.phase==InteractionPhase::Failed; });
        const auto& s=b.snapshot();
        const auto& witness=s.preparationReach.planning.directCheck;
        require(witness.status==PathStatus::Blocked && witness.bodyForm==10 && witness.object==obstruction.id,
            "Cube transfer failed without a payload witness: "+std::to_string(witness.bodyForm)+" "+s.interaction.reason);
        require(s.coordinates==held.coordinates && s.armOwners[0]==ArmOwner::HandInteraction &&
            s.object->state==ObjectState::Attached && s.object->owner==b.id() &&
            s.object->effectorFromObject==held.object->effectorFromObject &&
            glm::length(s.object->worldPose.position-held.object->worldPose.position)<1e-6f &&
            rotationError(s.object->worldPose.rotation,held.object->worldPose.rotation)<1e-6,
            "Blocked move changed cube or hand");
        out << "rejected_transfer=" << s.interaction.reason << " object=" << witness.object
            << " body_form=" << witness.bodyForm << " gap_m=" << witness.contact.gap << '\n';
        const auto& pose=held.object->worldPose;
        command(b,MoveHeldObject{handle,{glm::mix(pose.position,target.position,.22f),
            glm::slerp(pose.rotation,target.rotation,.22f)}});
        until(b,[](const auto& state) { return state.interaction.result=="Moved"; });
        command(b,ReleaseObject{handle});
        until(b,[](const auto& state) { return state.interaction.result=="Released"; });
        command(b,DeleteInteractionObject{handle});
        require(!b.snapshot().object && b.snapshot().support.phase==SupportPhase::Holding,"Failed recovery after blocked move");
        out << "bad_dimensions+stale_action+stale_epoch=Rejected retry_move_and_release=PASS\n";
    }
}

int main(int argc,char** argv)
{
    try {
        if (argc!=2) throw std::invalid_argument("Expected output directory");
        const std::filesystem::path output{argv[1]};
        std::filesystem::create_directories(output);
        std::ofstream report(output/"ready-cube.txt");
        if (!report) throw std::runtime_error("Cannot write cube report");
        report << std::setprecision(10);
        const auto asset=model_import::ModelImporter::loadFromFile(
            std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx",
            {.rebuildSmoothNormals=true,.omitPreparedGloves=true});
        const auto profile=raphtaliaBodyProfile();
        BodyInstance replay{1,asset,&profile}, sweep{2,asset,&profile}, cancel{3,asset,&profile}, blocked{4,asset,&profile};
        BodyInstance earlyCancel{5,asset,&profile};
        cycles(replay,report);
        cubeSweep(sweep,report);
        cancelAndRecover(cancel,report);
        cancelReachAndRecover(earlyCancel,report);
        blockedTransfer(blocked,report);
        report << "result=PASS\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
