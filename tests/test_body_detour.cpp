#include "body_reach_test_support.hpp"
using namespace body_reach_test;
int main(int argc,char** argv)
{
    try
    {
        const std::filesystem::path folder=argc>1 ? argv[1]:"build/diagnostics/stage6b/cpu";
        std::filesystem::create_directories(folder); std::ofstream report(folder/"body_detour.txt"); report << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile(); const BodyRigBinding binding(profile,*asset); const auto f=fixture(binding);
        // DirectOnly is the unchanged synchronous 6A contract. AllowDetour first tries that same path.
        BodyInstance direct(1,asset,&profile,false); prepare(direct,f.empty); auto directTarget=f.target; directTarget.strategy=MovementStrategy::DirectOnly;
        direct.enqueue(StartReach{directTarget}); direct.step(); finish(direct); JULIA_CHECK(direct.snapshot().reaches[0].state==ReachState::Succeeded);
        BodyInstance free(1,asset,&profile,false); prepare(free,f.empty); free.enqueue(StartReach{f.target}); free.step();
        JULIA_CHECK(free.snapshot().reaches[0].state==ReachState::Planning); finish(free);
        JULIA_CHECK(free.snapshot().reaches[0].state==ReachState::Succeeded && free.snapshot().reaches[0].planning.segmentCount==1);
        JULIA_CHECK(free.snapshot().reaches[0].planning.directCheck.status==PathStatus::Clear && free.snapshot().reaches[0].planning.candidates==0);
        JULIA_CHECK(free.snapshot().coordinates==direct.snapshot().coordinates); report << "free_direct_compatibility=PASS\n";
        BodyInstance blockedDirect(1,asset,&profile,false); prepare(blockedDirect,f.blocked); const auto before=blockedDirect.snapshot().coordinates;
        blockedDirect.enqueue(StartReach{directTarget}); blockedDirect.step();
        JULIA_CHECK(blockedDirect.results().back().status==BodyCommandStatus::Rejected && blockedDirect.snapshot().pathCheck.status==PathStatus::Blocked && blockedDirect.snapshot().coordinates==before);
        const auto memoryBefore=memory(); BodyInstance detour(1,asset,&profile,false); prepare(detour,f.blocked);
        detour.enqueue(StartReach{f.target}); Metrics metrics; metrics.step(detour); finish(detour,0,&metrics);
        const auto positive=detour.snapshot().reaches[0]; const auto& p=positive.planning;
        JULIA_CHECK(positive.state==ReachState::Succeeded && positive.error<=positive.target.tolerance);
        JULIA_CHECK(p.directCheck.status==PathStatus::Blocked && p.segmentCount==2 && p.candidates>0);
        JULIA_CHECK(p.segmentChecks[0].status==PathStatus::Clear && p.segmentChecks[1].status==PathStatus::Clear);
        JULIA_CHECK(p.segmentChecks[0].sceneVersion==1 && p.segmentChecks[1].sceneVersion==1);
        JULIA_CHECK(p.segmentChecks[0].toTick==p.segmentDurations[0] && p.segmentChecks[1].toTick==p.segmentDurations[1]);
        JULIA_CHECK(positive.collisionChecked && positive.collisionSceneVersion==1);
        JULIA_CHECK(p.ticks>1 && p.queries<=f.target.detour.maxTotalQueries);
        report << "positive_direct=" << pathStatusName(p.directCheck.status) << "\npositive_state=" << reachStateName(positive.state)
            << "\npositive_error=" << positive.error << "\nplanning_ticks=" << p.ticks << "\ncandidates=" << p.candidates << "\nqueries=" << p.queries << "\nposes=" << p.poses
            << "\nselected_candidate=" << p.selectedCandidate << "\nsegment_durations=" << p.segmentDurations[0] << ',' << p.segmentDurations[1] << "\nwaypoint=";
        for (float q:p.waypoint) report << q << ',';
        report << "\nmax_speed=" << metrics.maxSpeed << "\nmax_acceleration=" << metrics.maxAcceleration << "\nmax_queries_per_tick=" << metrics.maxQueriesPerTick
            << "\nmax_poses_per_tick=" << metrics.maxPosesPerTick << "\nscratch_accounted_bytes=" << metrics.scratchBytes << "\nprivate_before=" << memoryBefore << "\nprivate_after=" << memory() << '\n';
        stats(report,"tick_us",metrics.ticks); stats(report,"planning_tick_us",metrics.planningTicks);
        // Recheck the exact old DirectOnly proposal after successful execution with identical geometry/settings.
        BodyInstance witness(1,asset,&profile,false); prepare(witness,f.blocked); witness.enqueue(StartReach{directTarget}); witness.step();
        JULIA_CHECK(witness.snapshot().pathCheck.status==PathStatus::Blocked && witness.snapshot().pathCheck.object==p.directCheck.object);
        // A free waypoint alone says nothing about its incoming or outgoing edge.
        DirectBodyControl source(binding); source.setEnabled(true); bool freeWaypointBlockedEdge=false;
        for (float weight:{.1f,.2f,.8f,.9f})
        {
            ArmCoordinates waypoint{}; for (std::size_t k=0;k<6;++k) waypoint[k]=weight*f.solved[k];
            const auto point=checkArmPath(source,0,std::span{&waypoint,1},glm::mat4{1},*f.blocked,{});
            if (point.status!=PathStatus::Clear) continue;
            const auto first=checkArmPath(source,0,std::array{ArmCoordinates{},waypoint},glm::mat4{1},*f.blocked,{});
            const auto second=checkArmPath(source,0,std::array{waypoint,f.solved},glm::mat4{1},*f.blocked,{});
            freeWaypointBlockedEdge|=first.status==PathStatus::Blocked || second.status==PathStatus::Blocked;
        }
        JULIA_CHECK(freeWaypointBlockedEdge); report << "free_waypoint_blocked_edge=PASS\n";
        // Both arms, all supported effectors, body/world targets and a nonidentity root.
        for (std::size_t side=0;side<2;++side) for (unsigned effector=0;effector<3;++effector)
        {
            const RootPose root{{1.2f,.3f,-.4f},glm::angleAxis(.8f,glm::vec3{0,1,0})};
            const auto sample=fixture(binding,side,effector,root,effector!=0); BodyInstance supported(1,asset,&profile,false);
            prepare(supported,sample.blocked,root); supported.enqueue(StartReach{sample.target}); supported.step(); Metrics sampleMetrics;
            finish(supported,side,&sampleMetrics); const auto& result=supported.snapshot().reaches[side];
            report << "arm=" << side << " effector=" << effector << " state=" << reachStateName(result.state) << " direct=" << pathStatusName(result.planning.directCheck.status) << " segments=" << result.planning.segmentCount << '\n';
            JULIA_CHECK(result.state==ReachState::Succeeded && result.error<=result.target.tolerance && result.planning.segmentCount==2);
            JULIA_CHECK(result.planning.directCheck.status==PathStatus::Blocked);
        }
        // Goal collision makes every route fail; a finite candidate set is not a global proof.
        set(source,0,f.solved); const auto goalForms=bodyColliders(binding,pose(source).globals,glm::mat4{1}); source.reset();
        world::SceneObject endpoint; endpoint.id=201; endpoint.position=(goalForms[2].capsule.a+goalForms[2].capsule.b)*.5; endpoint.shape=world::Sphere{.008};
        BodyInstance exhausted(1,asset,&profile,false); prepare(exhausted,scene(1,{endpoint})); exhausted.enqueue(StartReach{f.target}); exhausted.step(); finish(exhausted);
        JULIA_CHECK(exhausted.snapshot().reaches[0].planning.state==PlanningState::Exhausted && exhausted.snapshot().reaches[0].state==ReachState::Failed);
        JULIA_CHECK(exhausted.snapshot().reaches[0].planning.candidates==f.target.detour.maxCandidates);
        auto tiny=f.target; tiny.detour.maxTotalQueries=1;
        BodyInstance budget(1,asset,&profile,false); prepare(budget,f.blocked); budget.enqueue(StartReach{tiny}); budget.step(); finish(budget);
        JULIA_CHECK(budget.snapshot().reaches[0].planning.state==PlanningState::BudgetExhausted && budget.snapshot().reaches[0].state==ReachState::Failed);
        CollisionSettings incomplete; incomplete.maxQueries=1;
        BodyInstance uncertain(1,asset,&profile,false); prepare(uncertain,f.blocked,{},incomplete); uncertain.enqueue(StartReach{f.target}); uncertain.step(); finish(uncertain);
        JULIA_CHECK(uncertain.snapshot().reaches[0].planning.state==PlanningState::CheckIncomplete);
        JULIA_CHECK(uncertain.snapshot().reaches[0].planning.directCheck.status==PathStatus::Incomplete);
        const auto initialForms=bodyColliders(binding,pose(source).globals,glm::mat4{1}); endpoint.position=initialForms[2].capsule.a;
        BodyInstance intersecting(1,asset,&profile,false); prepare(intersecting,scene(1,{endpoint})); intersecting.enqueue(StartReach{f.target}); intersecting.step(); finish(intersecting);
        JULIA_CHECK(intersecting.snapshot().reaches[0].planning.state==PlanningState::InvalidInitial);
        JULIA_CHECK(intersecting.snapshot().reaches[0].planning.directCheck.status==PathStatus::InitialIntersection);
        JULIA_CHECK(intersecting.snapshot().coordinates==before);
        report << "exhausted_budget_incomplete_initial=PASS\n";
        // Retarget during search, and while an approved segment is executing.
        BodyInstance retargetPlanning(1,asset,&profile,false); prepare(retargetPlanning,f.blocked); retargetPlanning.enqueue(StartReach{f.target}); retargetPlanning.step();
        const auto planningAction=retargetPlanning.snapshot().reaches[0].action;
        retargetPlanning.enqueue(RetargetReach{planningAction,f.home}); retargetPlanning.step(); finish(retargetPlanning);
        JULIA_CHECK(retargetPlanning.snapshot().reaches[0].state==ReachState::Succeeded && retargetPlanning.snapshot().reaches[0].target.position==f.home.position);
        BodyInstance ikRejected(1,asset,&profile,false); prepare(ikRejected,f.blocked); ikRejected.enqueue(StartReach{f.target}); ikRejected.step();
        const auto ikAction=ikRejected.snapshot().reaches[0].action; auto impossible=f.target; impossible.position={5,5,5};
        ikRejected.enqueue(RetargetReach{ikAction,impossible}); ikRejected.step();
        JULIA_CHECK(ikRejected.results().back().status==BodyCommandStatus::Rejected);
        JULIA_CHECK(ikRejected.snapshot().reaches[0].target.position==f.target.position);
        finish(ikRejected); JULIA_CHECK(ikRejected.snapshot().reaches[0].state==ReachState::Succeeded);
        BodyInstance retargetMoving(1,asset,&profile,false); prepare(retargetMoving,f.blocked); retargetMoving.enqueue(StartReach{f.target}); retargetMoving.step(); waitRoute(retargetMoving);
        const auto movingAction=retargetMoving.snapshot().reaches[0].action;
        retargetMoving.enqueue(RetargetReach{movingAction,f.home}); retargetMoving.step(); finish(retargetMoving);
        JULIA_CHECK(retargetMoving.snapshot().reaches[0].state==ReachState::Succeeded && retargetMoving.snapshot().reaches[0].error<=.002f);
        // A failed asynchronous replacement restores the previously authorized route.
        BodyInstance transactional(1,asset,&profile,false); prepare(transactional,f.blocked); transactional.enqueue(StartReach{f.target}); transactional.step(); waitRoute(transactional);
        const auto oldAction=transactional.snapshot().reaches[0].action; auto failure=f.home; failure.detour.maxTotalQueries=1;
        auto savedRoute=record(transactional.snapshot().reaches[0].route);
        transactional.enqueue(RetargetReach{oldAction,failure}); transactional.step();
        JULIA_CHECK(record(transactional.snapshot().reaches[0].route)==savedRoute);
        for (unsigned tick=0;tick<16000 && transactional.snapshot().reaches[0].planning.state!=PlanningState::BudgetExhausted;++tick) transactional.step();
        JULIA_CHECK(transactional.snapshot().reaches[0].planning.state==PlanningState::BudgetExhausted);
        JULIA_CHECK(record(transactional.snapshot().reaches[0].route)==savedRoute);
        const auto atWaypoint=transactional.snapshot().coordinates; const auto previousSecondStart=savedRoute.starts[1];
        transactional.step(); // The resumed second segment begins with the required stationary splice tick.
        JULIA_CHECK(transactional.snapshot().coordinates==atWaypoint);
        JULIA_CHECK(transactional.snapshot().reaches[0].route.currentSegment==1);
        savedRoute.current=1; savedRoute.starts[1]=transactional.snapshot().tick;
        JULIA_CHECK(savedRoute.starts[1]>previousSecondStart && record(transactional.snapshot().reaches[0].route)==savedRoute);
        finish(transactional);
        JULIA_CHECK(transactional.snapshot().reaches[0].action==oldAction && transactional.snapshot().reaches[0].state==ReachState::Succeeded);
        JULIA_CHECK(transactional.snapshot().reaches[0].target.position==f.target.position && transactional.snapshot().reaches[0].error<=.002f);
        BodyInstance replacement(1,asset,&profile,false); prepare(replacement,f.blocked); replacement.enqueue(StartReach{f.target}); replacement.step(); waitRoute(replacement);
        const auto replacedAction=replacement.snapshot().reaches[0].action;
        const auto replacedRoute=record(replacement.snapshot().reaches[0].route);
        replacement.enqueue(StartReach{failure}); replacement.step(); finish(replacement);
        JULIA_CHECK(replacement.snapshot().reaches[0].action==replacedAction && replacement.snapshot().reaches[0].state==ReachState::Succeeded);
        JULIA_CHECK(replacement.snapshot().reaches[0].target.position==f.target.position);
        const auto& resumed=replacement.snapshot().reaches[0].route;
        JULIA_CHECK(resumed.command==replacedRoute.command && resumed.segmentCount==replacedRoute.count && resumed.waypoint==replacedRoute.waypoint);
        JULIA_CHECK(resumed.segmentChecks[0].sceneVersion==replacedRoute.versions[0] && resumed.segmentChecks[1].sceneVersion==replacedRoute.versions[1]);
        // Changing strategy at a certified segment boundary must preserve the old samples.
        for (unsigned segment:{0u,1u})
        {
            BodyInstance crossStrategy(1,asset,&profile,false); prepare(crossStrategy,f.blocked);
            crossStrategy.enqueue(StartReach{f.target}); crossStrategy.step(); Metrics crossMetrics; waitRoute(crossStrategy,segment,&crossMetrics);
            auto ordinaryHome=f.home; ordinaryHome.strategy=MovementStrategy::DirectOnly;
            const auto action=crossStrategy.snapshot().reaches[0].action;
            crossStrategy.enqueue(RetargetReach{action,ordinaryHome}); crossMetrics.step(crossStrategy);
            const bool accepted=crossStrategy.results().back().status==BodyCommandStatus::Applied;
            finish(crossStrategy,0,&crossMetrics);
            const auto& result=crossStrategy.snapshot().reaches[0];
            JULIA_CHECK(result.state==ReachState::Succeeded && result.error<=result.target.tolerance);
            JULIA_CHECK(result.target.position==(accepted ? f.home.position:f.target.position));
        }
        report << "retarget_and_transactional_fallback=PASS\n";
        // Cancel before waypoint keeps only segment 0; after transition it finishes only segment 1.
        for (unsigned segment:{0u,1u})
        {
            BodyInstance cancelled(1,asset,&profile,false); prepare(cancelled,f.blocked); cancelled.enqueue(StartReach{f.target}); cancelled.step(); waitRoute(cancelled,segment);
            const auto action=cancelled.snapshot().reaches[0].action; const auto waypoint=cancelled.snapshot().reaches[0].planning.waypoint;
            const auto counters=record(cancelled.snapshot().reaches[0].route);
            cancelled.enqueue(CancelReach{ArmSide::Left,action}); cancelled.step(); finish(cancelled);
            JULIA_CHECK(cancelled.snapshot().reaches[0].state==ReachState::Cancelled);
            JULIA_CHECK(cancelled.snapshot().reaches[0].route.segmentCount==segment+1 && cancelled.snapshot().reaches[0].route.currentSegment==segment);
            JULIA_CHECK(cancelled.snapshot().reaches[0].planning.queries==counters.queries && cancelled.snapshot().reaches[0].planning.poses==counters.poses);
            if (segment==0)
            {
                const auto ids=joints(0);
                for (std::size_t k=0;k<6;++k) JULIA_CHECK(std::abs(cancelled.control()->coordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4)-waypoint[k])<1e-6f);
            }
            else JULIA_CHECK(cancelled.snapshot().reaches[0].error<=.002f);
            const auto held=cancelled.snapshot().coordinates; for (int n=0;n<5;++n) cancelled.step(); JULIA_CHECK(cancelled.snapshot().coordinates==held);
        }
        BodyInstance cancelSearch(1,asset,&profile,false); prepare(cancelSearch,f.blocked); cancelSearch.enqueue(StartReach{f.target}); cancelSearch.step();
        cancelSearch.enqueue(CancelReach{ArmSide::Left,cancelSearch.snapshot().reaches[0].action}); cancelSearch.step(); finish(cancelSearch);
        JULIA_CHECK(cancelSearch.snapshot().reaches[0].state==ReachState::Cancelled && cancelSearch.snapshot().coordinates==before);
        BodyInstance cancelWaiting(1,asset,&profile,false); prepare(cancelWaiting,f.blocked); cancelWaiting.enqueue(StartReach{f.target}); cancelWaiting.step(); waitRoute(cancelWaiting);
        const auto oldWaypoint=cancelWaiting.snapshot().reaches[0].planning.waypoint;
        const auto replacementAction=cancelWaiting.enqueue(StartReach{f.home}).id; cancelWaiting.step();
        JULIA_CHECK(cancelWaiting.snapshot().reaches[0].planning.state==PlanningState::WaitingForStop);
        cancelWaiting.enqueue(CancelReach{ArmSide::Left,replacementAction}); cancelWaiting.step(); finish(cancelWaiting);
        JULIA_CHECK(cancelWaiting.snapshot().reaches[0].state==ReachState::Cancelled && cancelWaiting.snapshot().reaches[0].action==replacementAction);
        const auto leftIds=joints(0);
        for (std::size_t k=0;k<6;++k) JULIA_CHECK(std::abs(cancelWaiting.control()->coordinate(leftIds[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4)-oldWaypoint[k])<1e-6f);
        report << "cancel_search_before_after_waypoint=PASS\n";
        // Planning owns scene, the other arm and geometry; explicit discontinuities invalidate it.
        BodyInstance conflicts(1,asset,&profile,false); prepare(conflicts,f.blocked); conflicts.enqueue(StartReach{f.target}); conflicts.step();
        auto other=f.home; other.arm=ArmSide::Right; other.effector=EffectorId::RightWrist;
        conflicts.enqueue(SetCollisionScene{scene(2),{}}); conflicts.enqueue(SetCoordinate{JointId::Head,0,.1f}); conflicts.enqueue(StartReach{other}); conflicts.step();
        for (const auto& result:conflicts.results()) JULIA_CHECK(result.status==BodyCommandStatus::Rejected);
        for (const auto& operation:std::array<BodyOperation,3>{ResetCoordinates{},SetRoot{{{1,0,0},{1,0,0,0}}},SetDirectMode{false}})
        {
            BodyInstance interrupted(1,asset,&profile,false); prepare(interrupted,f.blocked); interrupted.enqueue(StartReach{f.target}); interrupted.step();
            interrupted.enqueue(operation); interrupted.step(); JULIA_CHECK(interrupted.snapshot().reaches[0].state==ReachState::Cancelled);
            JULIA_CHECK(interrupted.display(.5).fromTick==interrupted.display(.5).toTick);
        }
        BodyInstance reset(1,asset,&profile,false); prepare(reset,f.blocked); reset.enqueue(StartReach{f.target}); reset.step(); reset.enqueue(ResetBody{}); reset.step();
        JULIA_CHECK(reset.snapshot().epoch==1 && reset.snapshot().collisionScene->version()==1 && reset.snapshot().reaches[0].state==ReachState::Idle);
        JULIA_CHECK(reset.submit({100,1,reset.snapshot().tick+1,0,StartReach{f.target}}).status==BodyCommandStatus::Rejected);
        reset.enqueue(SetCollisionScene{f.blocked,{}}); reset.step(); JULIA_CHECK(reset.results().back().status==BodyCommandStatus::Rejected);
        reset.enqueue(SetCollisionScene{scene(2),{}}); reset.step(); JULIA_CHECK(reset.results().back().status==BodyCommandStatus::Applied);
        report << "conflicts_reset_epoch_versions=PASS\n";
        replay(asset,profile,f,report);
        report << "result=" << (JULIA_TEST_RESULT() ? "FAIL":"PASS") << '\n';
        std::cout << "stage6b report=" << folder.string() << '\n';
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return JULIA_TEST_RESULT();
}
