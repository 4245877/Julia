#include "body_reach_test_support.hpp"
#include "../apps/common/StancePreparationDemo.hpp"
#include <sstream>
using namespace body_reach_test;
namespace
{
    void require(bool ok,const std::string& why) { if (!ok) throw std::runtime_error(why); }
    std::uint64_t command(BodyInstance& b,BodyOperation op,bool accept=true)
    {
        const auto id=b.enqueue(std::move(op)).id; b.step();
        require(b.results().size()==1 && b.results().back().status==(accept ? BodyCommandStatus::Applied:BodyCommandStatus::Rejected),
            b.results().empty() ? "No command result":b.results().back().reason);
        return id;
    }
    void until(BodyInstance& b,const std::function<bool(const BodySnapshot&)>& predicate)
    {
        for (unsigned t=0;t<10000 && !predicate(b.snapshot());++t) {
            b.step();
            require(b.snapshot().interaction.phase!=InteractionPhase::PreparationBlocked,
                "Preparation blocked: "+b.snapshot().interaction.reason);
        }
        require(predicate(b.snapshot()),"Timeout phase="+std::string(interactionPhaseName(b.snapshot().interaction.phase)));
    }
    void aligned(BodyInstance& b,const RootPose& root={})
    {
        command(b,SetRoot{root}); command(b,SetDirectMode{true});
        command(b,SetSupportSurface{{1,b.snapshot().support.surface.version+1,root.position.y,true}}); command(b,EnableSupport{}); command(b,AlignSupport{});
        until(b,[](const auto& s) { return s.support.phase==SupportPhase::Holding; });
    }
    std::uint64_t sourceReach(BodyInstance& b,ArmSide arm)
    {
        command(b,CreateInteractionSphere{1,app::interactionDemoPose(b.control()->binding(),arm,0,b.snapshot().root)});
        const auto id=command(b,StartReach{app::preparationPregrip(arm,b.snapshot().object->worldPose)});
        until(b,[&](const auto& s) { return s.reaches[arm==ArmSide::Left ? 0:1].state==ReachState::Succeeded; }); return id;
    }
    PrepareGrasp request(const BodyInstance& b,ArmSide arm,std::uint64_t source)
    { return {b.snapshot().object->handle,arm,source,b.snapshot().epoch}; }
    struct Measures
    {
        std::vector<double> ticks,checks,preparations,contexts;
        std::size_t scratch=0,context=0; unsigned queries=0,poses=0;
        double soleP=0,soleA=0,soleGap=0,speed=0,acceleration=0;
        void invariants(BodyInstance& b,const BodySnapshot& frozen,unsigned side)
        {
            const auto& s=b.snapshot(); const auto& old=b.previousSnapshot(); const auto ids=joints(side);
            require(s.root.position==frozen.root.position && s.root.rotation==frozen.root.rotation,"Preparation moved root");
            for (std::size_t j=0;j<s.coordinates.size();++j) {
                const auto& definition=b.control()->binding().profile().joints[j];
                if (std::find(ids.begin(),ids.end(),definition.id)==ids.end()) require(s.coordinates[j]==frozen.coordinates[j],"Preparation changed frozen q");
                for (std::size_t k=0;k<definition.coordinates.size();++k) require(s.coordinates[j][static_cast<int>(k)]>=definition.coordinates[k].lower &&
                    s.coordinates[j][static_cast<int>(k)]<=definition.coordinates[k].upper,"Joint limit");
            }
            require(s.object && s.object->state==ObjectState::Free && s.object->owner==0 && s.object->handle==frozen.object->handle &&
                s.object->worldPose.position==frozen.object->worldPose.position && s.object->worldPose.rotation==frozen.object->worldPose.rotation,"Free object moved/attached");
            require(s.interaction.phase!=InteractionPhase::Closing && s.interaction.phase!=InteractionPhase::Holding && s.interaction.hold==HoldType::None,"Automatic attachment/closing");
            require(s.reaches[side].action==frozen.reaches[side].action && s.reaches[side].state==ReachState::Succeeded &&
                s.reaches[side].reason==ReachReason::Reached && s.reaches[side].error==frozen.reaches[side].error,"Preparation replaced source Reach result");
            require(s.reachGeometry[side].certificate==frozen.reachGeometry[side].certificate && !s.reachGeometry[side].executable,"Source certificate rewritten/reused");
            const auto& i=s.interaction;
            require(i.phaseCheckPoses<=2 && i.phaseCheckQueries<=2048,"Phase budget exceeded");
            const auto& p=s.preparationReach.planning;
            require(p.tickIterations<=4 && p.tickIkEvaluations<=128 && p.tickPoses<=24 && p.tickQueries<=2048 && p.tickCandidates<=1,"Planner tick budget exceeded");
            if (active(s.preparationReach.state)) require(s.armOwners[side]==ArmOwner::GraspPreparation,"Internal Reach stole arm ownership");
            if (s.preparationReach.state==ReachState::Moving || s.preparationReach.state==ReachState::Settling)
                require(s.preparationGeometry.executable,"Executing segment has stale certificate");
            require(s.preparationReach.maxMeasuredSpeed<=1.2f && s.preparationReach.maxMeasuredAcceleration<=4.f,"Arm speed/acceleration");
            speed=std::max(speed,static_cast<double>(s.preparationReach.maxMeasuredSpeed));
            acceleration=std::max(acceleration,static_cast<double>(s.preparationReach.maxMeasuredAcceleration));
            require(i.contactHand==0 || (i.contactHand==side*3+3 && i.contactObject==((1ull<<63)|s.object->handle.id)),"Permission broadened");
            if (i.phase==InteractionPhase::Handoff || i.phase==InteractionPhase::Opening || i.phase==InteractionPhase::Approach ||
                i.phase==InteractionPhase::Succeeded || i.phase==InteractionPhase::Cancelled || i.phase==InteractionPhase::PreparationBlocked)
                require(i.contactHand==0 && i.contactObject==0,"Contact leaked outside allowed phases");
            for (unsigned k=0;k<2;++k) require(s.support.anchors[k].desiredWorld==frozen.support.anchors[k].desiredWorld &&
                s.support.anchors[k].surfaceFromSole==frozen.support.anchors[k].surfaceFromSole,"Anchor recaptured");
            const auto saved=hash(s); const auto q=s.coordinates;
            for (double alpha:{0.,.1,.23,.5,.77,.9,1.}) {
                const auto d=b.display(alpha); const auto again=b.display(alpha);
                require(d.pose.palette==again.pose.palette && d.colliders.size()==16,"Display not pure or lost lower forms");
                if (d.armOwners[side]==ArmOwner::GraspPreparation) require(d.colliders[side*3+2].capsule.radius==.045,"Handoff display lost prospective envelope");
                require(d.object->worldPose.position==frozen.object->worldPose.position,"Display moved Free object");
                for (const auto& a:d.support.anchors) {
                    soleP=std::max(soleP,static_cast<double>(a.positionError)); soleA=std::max(soleA,static_cast<double>(a.angularError));
                    soleGap=std::max(soleGap,static_cast<double>(std::max(std::abs(a.minGap),std::abs(a.maxGap))));
                    require(a.bound && a.positionError<=SupportPositionTolerance && a.angularError<=SupportAngularTolerance &&
                        a.minGap>=-SupportSurfaceTolerance && a.maxGap<=SupportSurfaceTolerance,"Display sole slipped");
                }
            }
            require(hash(b.snapshot())==saved && b.snapshot().coordinates==q && old.tick+1==s.tick,"Display mutated simulation");
            scratch=std::max({scratch,i.phaseCheckScratchBytes,p.scratchBytes}); context=std::max(context,s.preparationGeometry.contextBytes);
            queries=std::max(queries,i.phaseCheckQueries); poses=std::max(poses,i.phaseCheckPoses);
            if (i.phaseCheckQueries) checks.push_back(i.phaseCheckMicroseconds);
            if (s.preparationGeometry.preparationMicroseconds) contexts.push_back(s.preparationGeometry.preparationMicroseconds);
        }
        void step(BodyInstance& b,const BodySnapshot& frozen,unsigned side) {
            const auto start=Clock::now(); b.step(); ticks.push_back(std::chrono::duration<double,std::micro>(Clock::now()-start).count());
            invariants(b,frozen,side);
        }
    };
    void cycles(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        Measures m;
        for (unsigned side=0;side<2;++side) for (unsigned transformed=0;transformed<2;++transformed) {
            BodyInstance b(1,asset,&profile,false); const auto arm=side==0 ? ArmSide::Left:ArmSide::Right;
            RootPose root; if (transformed) root={{1.2f,.3f,-.4f},glm::angleAxis(.8f,glm::vec3{0,1,0})};
            aligned(b,root); const auto source=sourceReach(b,arm); const auto frozen=b.snapshot();
            const auto start=Clock::now(); const auto action=command(b,request(b,arm,source));
            m.preparations.push_back(std::chrono::duration<double,std::micro>(Clock::now()-start).count()); m.invariants(b,frozen,side);
            require(b.snapshot().interaction.phase==InteractionPhase::Handoff,"Missing handoff boundary");
            require(!b.snapshot().reachGeometry[side].dependenciesMatch,"Old Reach authorized new hand shape");
            for (unsigned t=0;t<4000 && b.snapshot().interaction.phase!=InteractionPhase::ReadyToGrasp;++t) m.step(b,frozen,side);
            const auto ready=b.snapshot(); require(ready.interaction.phase==InteractionPhase::ReadyToGrasp,"Ready not reached: "+ready.interaction.reason);
            require(ready.interaction.geometryCurrent && ready.hands[side].closure==0 && ready.hands[side].velocity==0 && ready.hands[side].blend==1 &&
                ready.interaction.frameError<=.002f && ready.interaction.angularError<=glm::radians(1.f),"False Ready");
            const auto readyQ=ready.coordinates; for (unsigned t=0;t<150;++t) { m.step(b,frozen,side); require(b.snapshot().coordinates==readyQ,"Ready drift"); }
            if (transformed) command(b,CancelInteraction{action}); else command(b,FinishGraspPreparation{action});
            for (unsigned t=0;t<4000 && b.snapshot().armOwners[side]!=ArmOwner::None;++t) m.step(b,frozen,side);
            require(b.snapshot().armOwners[side]==ArmOwner::None && b.snapshot().interaction.contactHand==0,"End did not release arm/contact");
            require(b.snapshot().interaction.result==(transformed ? "Cancelled":"Prepared"),"End result");
            command(b,request(b,arm,source),false); // Consumed capability cannot be reused.
            const auto retained=b.snapshot().coordinates; command(b,DeleteInteractionObject{b.snapshot().object->handle});
            require(b.snapshot().coordinates==retained && b.snapshot().hands[side].enabled,"Exit snapped to bind");
            command(b,StartReach{app::stanceReachTarget(b,arm,2,1)}); until(b,[&](const auto& s) { return !active(s.reaches[side].state); });
            require(b.snapshot().reaches[side].state==ReachState::Succeeded,"Reach after preparation failed");
            command(b,DisableSupport{}); auto t=app::stanceReachTarget(b,arm); t.strategy=MovementStrategy::DirectOnly;
            command(b,StartReach{t}); until(b,[&](const auto& s) { return !active(s.reaches[side].state); });
            require(b.snapshot().reaches[side].state==ReachState::Succeeded,"Standalone Reach regression");
            out << "cycle=" << side << ',' << transformed << " ready_errors=" << ready.interaction.frameError << ',' << ready.interaction.angularError << '\n';
        }
        const auto stats=[&](const char* label,std::vector<double> v) {
            require(!v.empty(),"Missing measurements"); std::sort(v.begin(),v.end());
            out << label << "_n_mean_p95_max=" << v.size() << ',' << std::accumulate(v.begin(),v.end(),0.)/v.size() << ',' << v[v.size()*95/100] << ',' << v.back() << '\n';
        };
        stats("tick_us",m.ticks); stats("phase_check_us",m.checks); stats("preparation_us",m.preparations); stats("context_us",m.contexts);
        out << "max_sole_p_angle_gap=" << m.soleP << ',' << m.soleA << ',' << m.soleGap << " max_arm_v_a=" << m.speed << ',' << m.acceleration
            << " scratch_context_bytes=" << m.scratch << ',' << m.context << " phase_queries_poses=" << m.queries << ',' << m.poses << '\n';
    }
    void conflictsAndCancel(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        for (const auto phase:{InteractionPhase::Handoff,InteractionPhase::Opening,InteractionPhase::Approach,InteractionPhase::Align,InteractionPhase::ReadyToGrasp}) {
            BodyInstance b(1,asset,&profile,false); aligned(b); const auto source=sourceReach(b,ArmSide::Left);
            const auto frozen=b.snapshot(); const auto action=command(b,request(b,ArmSide::Left,source));
            until(b,[&](const auto& s) { return s.interaction.phase==phase &&
                ((phase!=InteractionPhase::Approach && phase!=InteractionPhase::Align) || s.preparationReach.state==ReachState::Moving); });
            if (phase==InteractionPhase::Opening) for (unsigned t=0;t<15;++t) b.step();
            const auto checkpoint=b.snapshot();
            for (BodyOperation op:std::vector<BodyOperation>{MoveSupportRoot{{.008f,0,0}},SetSupportSurface{{1,2,0,true}},DisableSupport{},
                EnableSupport{},AlignSupport{},CancelSupportMotion{},SetCoordinate{JointId::RightElbow,0,.2f},SetHandClosure{ArmSide::Right,1},
                SetCollisionScene{scene(1000),{}},DeleteInteractionObject{checkpoint.object->handle},CreateInteractionSphere{2,{}},
                GraspObject{checkpoint.object->handle,ArmSide::Left},MoveHeldObject{},ReleaseObject{},
                StartReach{app::stanceReachTarget(b,ArmSide::Right)},CancelReach{ArmSide::Left,source},
                RetargetReach{source,app::stanceReachTarget(b,ArmSide::Left)},SetDirectMode{false},SetRoot{{}},ResetCoordinates{},
                PrepareGrasp{checkpoint.object->handle,ArmSide::Right,source,checkpoint.epoch},FinishGraspPreparation{action+1},CancelInteraction{action+1}}) {
                // Batch at one boundary so a short Approach cannot end during the rejection matrix.
                b.enqueue(std::move(op));
            }
            const auto wasActive=active(b.snapshot().preparationReach.state); b.enqueue(CancelInteraction{action}); b.step();
            for (std::size_t k=0;k+1<b.results().size();++k) require(b.results()[k].status==BodyCommandStatus::Rejected,"Conflict accepted: "+b.results()[k].reason);
            require(b.results().back().status==BodyCommandStatus::Applied,"Cancel rejected");
            if (wasActive) require(b.snapshot().interaction.cancellationPending,"Cancel stopped a segment immediately");
            until(b,[](const auto& s) { return s.interaction.phase==InteractionPhase::Cancelled; });
            require(b.snapshot().armOwners[0]==ArmOwner::None && b.snapshot().interaction.contactHand==0 &&
                b.snapshot().reaches[0].action==source && b.snapshot().reaches[0].state==ReachState::Succeeded &&
                b.snapshot().reachGeometry[0].certificate==frozen.reachGeometry[0].certificate,"Cancel damaged source/owner/contact");
            require(b.snapshot().root.position==frozen.root.position && b.snapshot().support.kind==SupportKind::KinematicSupport,"Cancel damaged support");
            out << "cancel_phase=" << interactionPhaseName(phase) << " result=PASS\n";
        }
        // ResetBody is the sole diagnostic discontinuity allowed to abort preparation.
        for (const auto phase:{InteractionPhase::Handoff,InteractionPhase::Opening,InteractionPhase::Approach,InteractionPhase::Align,InteractionPhase::ReadyToGrasp}) {
            BodyInstance b(1,asset,&profile,false); aligned(b); const auto source=sourceReach(b,ArmSide::Left);
            command(b,request(b,ArmSide::Left,source)); until(b,[&](const auto& s) { return s.interaction.phase==phase; });
            const auto object=*b.snapshot().object; const auto epoch=b.snapshot().epoch; command(b,ResetBody{});
            require(b.snapshot().epoch==epoch+1 && !b.snapshot().support.enabled && b.snapshot().armOwners[0]==ArmOwner::None &&
                b.snapshot().interaction.contactHand==0 && b.snapshot().object->handle==object.handle &&
                b.snapshot().object->worldPose.position==object.worldPose.position && b.snapshot().object->worldPose.rotation==object.worldPose.rotation &&
                b.previousSnapshot().tick==b.snapshot().tick,"Reset left stale preparation");
            require(b.submit({999,1,b.snapshot().tick+1,epoch,CancelInteraction{source}}).status==BodyCommandStatus::Rejected,"Old epoch accepted");
        }
        out << "reset_all_phases=PASS\n";
        BodyInstance standalone(3,asset,&profile,false); aligned(standalone); const auto sr=sourceReach(standalone,ArmSide::Left);
        const auto a=command(standalone,request(standalone,ArmSide::Left,sr));
        until(standalone,[](const auto& s) { return s.interaction.phase==InteractionPhase::ReadyToGrasp; });
        command(standalone,FinishGraspPreparation{a}); until(standalone,[](const auto& s) { return s.interaction.phase==InteractionPhase::Succeeded; });
        command(standalone,MoveSupportRoot{{.008f,0,0}}); until(standalone,[](const auto& s) { return s.support.phase==SupportPhase::Holding; });
        command(standalone,DisableSupport{}); command(standalone,GraspObject{standalone.snapshot().object->handle,ArmSide::Left});
        until(standalone,[](const auto& s) { return s.interaction.phase==InteractionPhase::Holding || s.interaction.phase==InteractionPhase::Failed; });
        require(standalone.snapshot().interaction.phase==InteractionPhase::Holding && standalone.snapshot().object->state==ObjectState::Attached &&
            !standalone.snapshot().interaction.preparation,"Standalone grasp after preparation failed");
        command(standalone,ReleaseObject{standalone.snapshot().object->handle});
        until(standalone,[](const auto& s) { return s.interaction.phase==InteractionPhase::Succeeded || s.interaction.phase==InteractionPhase::ReleaseBlocked; });
        require(standalone.snapshot().interaction.phase==InteractionPhase::Succeeded,"Standalone release regression");
        out << "return_to_support_motion_and_full_standalone_grasp_release=PASS\n";
    }
    void admission(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        BodyInstance b(1,asset,&profile,false); aligned(b);
        command(b,CreateInteractionSphere{1,app::interactionDemoPose(b.control()->binding(),ArmSide::Left)});
        const auto source=command(b,StartReach{app::preparationPregrip(ArmSide::Left,b.snapshot().object->worldPose)});
        require(active(b.snapshot().reaches[0].state),"Expected unfinished source fixture");
        command(b,request(b,ArmSide::Left,source),false);
        until(b,[](const auto& s) { return s.reaches[0].state==ReachState::Succeeded; });
        const auto original=b.snapshot(); auto bad=request(b,ArmSide::Left,source);
        bad.reach=source+10; command(b,bad,false); bad=request(b,ArmSide::Left,source); ++bad.reachEpoch; command(b,bad,false);
        bad=request(b,ArmSide::Left,source); ++bad.object.generation; command(b,bad,false);
        bad=request(b,ArmSide::Right,source); command(b,bad,false);
        require(b.submit({999,2,b.snapshot().tick+1,b.snapshot().epoch,request(b,ArmSide::Left,source)}).status==BodyCommandStatus::Rejected,"Wrong BodyId accepted");
        require(b.snapshot().coordinates==original.coordinates && b.snapshot().reaches[0].state==ReachState::Succeeded,"Refusal changed source q/result");
        command(b,SetSupportSurface{{1,b.snapshot().support.surface.version+1,0,true}}); command(b,request(b,ArmSide::Left,source),false);
        command(b,EnableSupport{}); command(b,AlignSupport{}); until(b,[](const auto& s) { return s.support.phase==SupportPhase::Holding; });
        command(b,request(b,ArmSide::Left,source),false); // Historical certificate refers to surface v1.
        const auto handle=b.snapshot().object->handle; const auto objectPose=b.snapshot().object->worldPose;
        command(b,DeleteInteractionObject{handle}); command(b,CreateInteractionSphere{1,objectPose});
        bad=request(b,ArmSide::Left,source); bad.object=handle; command(b,bad,false);
        command(b,request(b,ArmSide::Left,source),false); // New handle cannot refresh an old source certificate.
        const auto fresh=command(b,StartReach{app::preparationPregrip(ArmSide::Left,b.snapshot().object->worldPose)});
        until(b,[](const auto& s) { return s.reaches[0].state==ReachState::Succeeded; });
        const auto q=b.snapshot().coordinates; const auto action=b.enqueue(request(b,ArmSide::Left,fresh)).id;
        b.enqueue(CancelInteraction{action}); b.step();
        for (const auto& result:b.results()) require(result.status==BodyCommandStatus::Applied,"Same-tick handoff cancellation rejected");
        require(b.snapshot().interaction.phase==InteractionPhase::Cancelled && b.snapshot().armOwners[0]==ArmOwner::None &&
            b.snapshot().coordinates==q && !b.snapshot().hands[0].enabled && b.snapshot().reaches[0].action==fresh &&
            b.snapshot().reaches[0].state==ReachState::Succeeded,"Same-tick cancel changed q/source/hand");
        command(b,request(b,ArmSide::Left,fresh),false);
        out << "admission_unfinished_body_arm_action_epoch_handle_surface_certificate=PASS\n";
    }
    CollisionContext fixedContext(const BodyInstance& b,unsigned side,bool envelope=true)
    {
        CollisionContext c; c.handEnvelopes[side]=envelope;
        c.fixedLowerForms=lowerBodyColliders(b.control()->binding(),b.snapshot().pose.globals,matrix(b.snapshot().root),calibrateSoleContacts(b.control()->binding()));
        c.soleContacts={std::pair{14u,SupportFloorObject},std::pair{17u,SupportFloorObject}}; return c;
    }
    ArmCoordinates armQ(const BodyInstance& b,unsigned side)
    {
        ArmCoordinates q{}; const auto ids=joints(side);
        for (unsigned k=0;k<6;++k) q[k]=b.control()->coordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4); return q;
    }
    void geometry(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        BodyInstance b(1,asset,&profile,false); aligned(b); sourceReach(b,ArmSide::Left);
        const auto wrist=matrix(b.snapshot().root)*b.control()->binding().jointFrame(JointId::LeftWrist,b.snapshot().pose.globals);
        world::SceneObject obstacle{55,glm::dvec3{wrist*glm::vec4{0,.192f,0,1}},{1,0,0,0},{},world::Sphere{.002}};
        auto objects=b.snapshot().collisionScene->objects(); objects.push_back(obstacle); const world::CollisionScene augmented(100,objects);
        const auto q=armQ(b,0); const auto small=checkArmPath(*b.control(),0,std::span{&q,1},matrix(b.snapshot().root),augmented,{},fixedContext(b,0,false));
        const auto large=checkArmPath(*b.control(),0,std::span{&q,1},matrix(b.snapshot().root),augmented,{},fixedContext(b,0));
        require(small.status==PathStatus::Clear && large.status!=PathStatus::Clear && large.object==55,"Envelope-only obstacle fixture");
        const auto target=rigidPose(matrix(b.snapshot().root)*b.control()->binding().effectorFrame(EffectorId::LeftGrip,b.snapshot().pose.globals));
        command(b,DisableSupport{}); command(b,SetCollisionScene{scene(100,{obstacle}),{}}); command(b,EnableSupport{}); command(b,AlignSupport{});
        until(b,[](const auto& s) { return s.support.phase==SupportPhase::Holding; });
        auto t=app::preparationPregrip(ArmSide::Left,target); t.position=target.position; t.orientation=target.rotation; t.strategy=MovementStrategy::DirectOnly;
        const auto source=command(b,StartReach{t}); until(b,[](const auto& s) { return s.reaches[0].state==ReachState::Succeeded; });
        const auto before=b.snapshot(); command(b,request(b,ArmSide::Left,source),false);
        require(b.snapshot().coordinates==before.coordinates && b.snapshot().hands[0].enabled==before.hands[0].enabled &&
            b.snapshot().reaches[0].state==ReachState::Succeeded && b.snapshot().interaction.phase==InteractionPhase::Idle,"Rejected opening mutated accepted state");
        require(b.snapshot().pathCheck.object==55 && b.snapshot().pathCheck.status!=PathStatus::Clear,"Rejected envelope lost diagnostic witness");
        out << "ordinary_clear_enlarged_blocked=PASS object=" << large.object << '\n';
        BodyInstance ready(2,asset,&profile,false); aligned(ready); const auto sr=sourceReach(ready,ArmSide::Left);
        command(ready,request(ready,ArmSide::Left,sr)); until(ready,[](const auto& s) { return s.interaction.phase==InteractionPhase::ReadyToGrasp; });
        const auto rq=armQ(ready,0); auto c=fixedContext(ready,0); const auto& sceneNow=*ready.snapshot().collisionScene;
        const auto check=[&](const world::CollisionScene& sceneValue,CollisionContext context,const glm::mat4& root) {
            return checkArmPath(*ready.control(),0,std::span{&rq,1},root,sceneValue,{},std::move(context)); };
        const auto root=matrix(ready.snapshot().root); const auto blocked=check(sceneNow,c,root);
        require(blocked.status!=PathStatus::Clear && blocked.bodyForm==3 && blocked.object==((1ull<<63)|1),"Target was not an obstacle");
        c.contactHand=3; c.contactObject=(1ull<<63)|1;
        require(check(sceneNow,c,root).status==PathStatus::Clear,"Named Align contact not admitted");
        const auto forms=ready.display(1).colliders;
        for (const auto formIndex:{1u,5u,6u}) {
            auto relocated=sceneNow.objects();
            for (auto& object:relocated) if (object.id==c.contactObject) object.position=(forms[formIndex].capsule.a+forms[formIndex].capsule.b)*.5;
            const auto result=check(world::CollisionScene(103+formIndex,relocated),c,root);
            require(result.status!=PathStatus::Clear && result.object==c.contactObject && result.bodyForm==forms[formIndex].id,
                "Named contact masked forearm/other hand/torso");
        }
        c.contactObject=(1ull<<63)|2; require(check(sceneNow,c,root).status!=PathStatus::Clear,"Permission covered another object"); c.contactObject=(1ull<<63)|1;
        auto wrong=c; wrong.contactHand=6; bool rejected=false;
        try { (void)check(sceneNow,wrong,root); } catch (const std::invalid_argument&) { rejected=true; } require(rejected,"Wrong-hand permission accepted");
        auto lower=c.fixedLowerForms;
        const auto hand=ready.display(1).colliders[2].capsule; const auto center=(hand.a+hand.b)*.5;
        for (auto form:lower) {
            form.capsule={center,center,.01}; auto injected=c; injected.fixedLowerForms={form};
            const auto result=check(sceneNow,injected,root);
            require(result.status!=PathStatus::Clear && result.otherBodyForm==form.id,"Named contact masked lower form");
        }
        auto shifted=root; shifted[3].y-=1.5f; const auto floor=check(sceneNow,c,shifted);
        require(floor.status!=PathStatus::Clear && floor.object==SupportFloorObject && floor.bodyForm!=14 && floor.bodyForm!=17,"Shoe allowance disabled arm-floor");
        auto more=sceneNow.objects(); more.push_back({56,center,{1,0,0,0},{},world::Sphere{.003}});
        const auto other=check(world::CollisionScene(101,more),c,root); require(other.status!=PathStatus::Clear && other.object==56,"Target permission covered obstacle");
        more=sceneNow.objects(); more.push_back({57,lower[0].capsule.a,{1,0,0,0},{},world::Sphere{.003}});
        const auto bottom=check(world::CollisionScene(102,more),c,root); require(bottom.status!=PathStatus::Clear && bottom.bodyForm>=11 && bottom.object==57,"Lower scene pairs disappeared");
        out << "named_pair_only_lower_11_to_17_floor_other_object=PASS\n";
    }
    void failures(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        BodyInstance b(1,asset,&profile,false); aligned(b); command(b,CreateInteractionSphere{1,{{10,1,0},{1,0,0,0}}});
        const auto source=command(b,StartReach{app::stanceReachTarget(b,ArmSide::Left)});
        until(b,[](const auto& s) { return s.reaches[0].state==ReachState::Succeeded; }); const auto sourceState=b.snapshot();
        const auto action=command(b,request(b,ArmSide::Left,source));
        for (unsigned t=0;t<4000 && b.snapshot().interaction.phase!=InteractionPhase::PreparationBlocked;++t) b.step();
        require(b.snapshot().interaction.phase==InteractionPhase::PreparationBlocked && b.snapshot().interaction.contactHand==0 &&
            b.snapshot().armOwners[0]==ArmOwner::GraspPreparation && b.snapshot().reaches[0].state==ReachState::Succeeded &&
            b.snapshot().reaches[0].action==source && b.snapshot().coordinates==sourceState.coordinates,"Failed preparation replaced source/lost owner");
        command(b,CancelInteraction{action}); until(b,[](const auto& s) { return s.interaction.phase==InteractionPhase::Cancelled; });
        // Independently corrupt a frozen dependency to exercise the runtime gate,
        // rather than relying solely on public command rejection.
        BodyInstance stale(2,asset,&profile,false); aligned(stale); const auto sr=sourceReach(stale,ArmSide::Left);
        command(stale,request(stale,ArmSide::Left,sr));
        until(stale,[](const auto& s) { return s.interaction.phase==InteractionPhase::Align && s.preparationReach.state==ReachState::Moving; });
        const auto before=stale.snapshot();
        auto& control=const_cast<DirectBodyControl&>(*stale.control()); control.setCoordinate(JointId::RightElbow,0,.05f);
        stale.step(); require(stale.snapshot().interaction.phase==InteractionPhase::PreparationBlocked &&
            stale.snapshot().preparationReach.reason==ReachReason::GeometryContextChanged && !stale.snapshot().preparationGeometry.executable &&
            stale.snapshot().interaction.contactHand==0 && stale.snapshot().reaches[0].state==ReachState::Succeeded,"Stale context executed");
        for (const auto id:joints(0)) require(control.coordinates()[control.binding().jointIndex(id)]==before.coordinates[control.binding().jointIndex(id)],"Stale segment wrote q");
        const auto revision=stale.snapshot().collisionScene->version(); stale.step();
        require(stale.snapshot().collisionScene->version()==revision,"Blocked state keeps mutating geometry"); command(stale,ResetBody{});
        out << "failed_approach_historical_source_and_runtime_stale_context_gate=PASS\n";
        // Fault injection at the planner boundary: every return job exhausts its
        // query quota. Real production HandInteraction/ArmReach execute all phases.
        // No source, support or geometry admission is bypassed in the BodyInstance tests above.
        BodyRigBinding binding(profile,*asset); DirectBodyControl c(binding); c.setEnabled(true); ArmReach reach(c); HandInteraction interaction(3,c,reach);
        const auto objectPose=app::interactionDemoPose(binding,ArmSide::Left); const auto pre=app::preparationPregrip(ArmSide::Left,objectPose);
        const auto solution=solveArmPosition(c,pre,pre.position,pre.orientation); require(solution.reason==ReachReason::Reached,"Fault fixture IK");
        ArmCoordinates q{}; for (unsigned k=0;k<6;++k) q[k]=static_cast<float>(solution.coordinates[k]); set(c,0,q);
        std::uint64_t version=1;
        interaction.geometryChanged=[&] {
            require(!reach.anyActive(),"Phase mutated a moving segment");
            const auto sceneValue=scene(++version,{*interaction.freeCollider()}); const auto context=interaction.collisionContext();
            reach.setPathValidator([&,sceneValue,context](std::size_t side,std::span<const ArmCoordinates> path,unsigned) {
                return checkArmPath(c,side,path,glm::mat4{1},*sceneValue,{},context).status; },version);
            reach.setPlanningValidator([&,sceneValue,context](std::size_t side,std::span<const ArmCoordinates> path) {
                CollisionSettings settings; if (interaction.observation().phase==InteractionPhase::Retreat) settings.maxQueries=1;
                return std::make_unique<ArmPathCheckJob>(c,side,path,glm::mat4{1},*sceneValue,settings,context);
            },[](PathCheck) {});
        };
        interaction.validateRest=[&](ArmSide,CollisionContext context) {
            const auto coordinates=ArmCoordinates{c.coordinate(JointId::LeftShoulder,0),c.coordinate(JointId::LeftShoulder,1),c.coordinate(JointId::LeftShoulder,2),
                c.coordinate(JointId::LeftElbow,0),c.coordinate(JointId::LeftWrist,0),c.coordinate(JointId::LeftWrist,1)};
            return checkArmPath(c,0,std::span{&coordinates,1},glm::mat4{1},*scene(version,{*interaction.freeCollider()}),{},context);
        };
        interaction.preparationValid=[] { return true; }; interaction.create({1,objectPose});
        auto prospective=interaction.collisionContext(); prospective.handEnvelopes[0]=true;
        auto opening=interaction.validateRest(ArmSide::Left,prospective); require(opening.status==PathStatus::Clear,"Fault fixture envelope");
        interaction.prepare(10,0,{interaction.object()->handle,ArmSide::Left,9,0},opening);
        std::uint64_t tick=0;
        const auto step=[&] {
            interaction.beginTick(); interaction.hand.advance(); reach.advance(++tick);
            auto p=pose(c); interaction.hand.apply(p.locals); animation::computeGlobalTransforms(asset->skeleton(),p.locals,p.globals);
            reach.measure(p.globals,glm::mat4{1}); interaction.advance(p.globals,glm::mat4{1});
        };
        for (unsigned t=0;t<4000 && interaction.observation().phase!=InteractionPhase::ReadyToGrasp;++t) step();
        require(interaction.observation().phase==InteractionPhase::ReadyToGrasp,"Fault fixture never Ready");
        interaction.cancel(10);
        for (unsigned t=0;t<4000 && interaction.observation().phase!=InteractionPhase::PreparationBlocked;++t) step();
        require(interaction.observation().phase==InteractionPhase::PreparationBlocked && interaction.observation().result=="ReleaseBlocked" &&
            interaction.locked() && interaction.collisionContext().contactHand==0 && interaction.object()->state==ObjectState::Free &&
            interaction.object()->worldPose.position==objectPose.position,"Unchecked retreat released owner/contact/object");
        interaction.abandonPreparation(); require(!interaction.locked(),"Reset did not release blocked owner");
        out << "retreat_incomplete_fault_injection_blocked_owner_retained_permission_removed=PASS\n";
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        for (unsigned mode=0;mode<2;++mode) {
            const auto run=[&](int fps) {
                BodyInstance b(1,asset,&profile,false); RootPose root;
                if (mode) root={{1.2f,.3f,-.4f},glm::angleAxis(.8f,glm::vec3{0,1,0})};
                b.enqueue(SetRoot{root}); b.enqueue(SetDirectMode{true}); b.step();
                app::StancePreparationDemo demo; demo.arm=mode ? ArmSide::Right:ArmSide::Left; demo.cancelAtReady=mode!=0; demo.start(b);
                std::vector<std::string> history;
                const auto tick=[&] {
                    demo.drive(b); b.step(); std::ostringstream line; app::stancePreparationTrace(line,b.snapshot()); line << hash(b.snapshot());
                    for (const auto& r:b.results()) { require(r.status!=BodyCommandStatus::Rejected,r.reason); line << ',' << r.id << ':' << static_cast<int>(r.status) << ':' << r.reason; }
                    history.push_back(line.str());
                };
                if (!fps) for (unsigned t=0;t<8400;++t) tick();
                else { runtime::FixedStepScheduler scheduler; for (int f=0;f<70*fps;++f) { scheduler.advance(1./fps,tick); (void)b.display(scheduler.alpha()); } }
                require(demo.done(),"Replay incomplete stage="+std::to_string(demo.stage)); return history;
            };
            const auto reference=run(0);
            for (int fps:{30,60,144}) { require(run(fps)==reference,"FPS history mismatch"); out << "replay_mode_fps=" << mode << ',' << fps << " ticks=8400_identical\n"; }
        }
    }
}
int main(int argc,char** argv)
{
    try {
        const auto output=argc>1 ? std::filesystem::path{argv[1]}:std::filesystem::path{"build/diagnostics/stage6i/cpu"};
        std::filesystem::create_directories(output); std::ofstream out(output/"stance-preparation.txt"); out << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx"); const auto profile=raphtaliaBodyProfile();
        const auto before=memory(); cycles(asset,profile,out); admission(asset,profile,out); conflictsAndCancel(asset,profile,out); geometry(asset,profile,out);
        failures(asset,profile,out); replay(asset,profile,out);
        out << "private_usage_before_after=" << before << ',' << memory() << " sizeof_snapshot=" << sizeof(BodySnapshot) << '\n';
        out << "result=PASS\n"; return JULIA_TEST_RESULT();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
