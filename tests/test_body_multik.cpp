#include "body_reach_test_support.hpp"
#include "../apps/common/CollisionDemo.hpp"
using namespace body_reach_test;
int main(int argc,char** argv)
{
    try {
        const std::filesystem::path folder=argc>1 ? argv[1]:"build/diagnostics/stage6c/cpu";
        std::filesystem::create_directories(folder); std::ofstream report(folder/"body_multik.txt"); report << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile(); const BodyRigBinding binding(profile,*asset); auto f=fixture(binding);
        const auto demo=app::multiConfigurationDemo(binding,1); auto target=demo.target;
        target.strategy=MovementStrategy::AllowDetour; target.detour.maxConfigurations=4;
        DirectBodyControl source(binding); source.setEnabled(true);
        ArmPositionSearchJob ik(source,target,target.position);
        while (!ik.done()) {
            ik.advance(3,1); JULIA_CHECK(ik.observation().tickIterations<=3 && ik.observation().tickSeeds<=1);
        }
        JULIA_CHECK(ik.solutions().size()==4 && ik.observation().duplicates>0 && !ik.budgetExhausted());
        const auto legacy=solveArmPosition(source,target,target.position);
        for (std::size_t k=0;k<6;++k) JULIA_CHECK(static_cast<float>(legacy.coordinates[k])==ik.solutions()[0].coordinates[k]);
        for (std::size_t i=0;i<ik.solutions().size();++i) {
            const auto& solution=ik.solutions()[i]; ArmCoordinates q{};
            for (std::size_t k=0;k<6;++k) q[k]=static_cast<float>(solution.coordinates[k]);
            set(source,0,q); JULIA_CHECK(glm::length(glm::vec3{binding.effectorFrame(target.effector,pose(source).globals)[3]}-target.position)<=target.tolerance);
            for (std::size_t j=0;j<i;++j) {
                double distance=0; for (std::size_t k=0;k<4;++k) distance=std::max(distance,std::abs(solution.coordinates[k]-ik.solutions()[j].coordinates[k]));
                JULIA_CHECK(distance>=.08);
            }
        }
        source.reset(); source.setCoordinate(JointId::LeftWrist,0,.1f); source.setCoordinate(JointId::LeftWrist,1,-.2f);
        ArmPositionSearchJob wrist(source,target,target.position); while (!wrist.done()) wrist.advance(16,1);
        for (const auto& solution:wrist.solutions()) JULIA_CHECK(solution.coordinates[4]==static_cast<double>(.1f) && solution.coordinates[5]==static_cast<double>(-.2f));
        source.reset();
        auto single=target; single.detour.maxConfigurations=1;
        BodyInstance baseline(1,asset,&profile,false); prepare(baseline,demo.scene); baseline.enqueue(StartReach{single}); Metrics baselineMetrics; baselineMetrics.step(baseline); finish(baseline,0,&baselineMetrics);
        JULIA_CHECK(baseline.snapshot().reaches[0].state==ReachState::Failed && baseline.snapshot().reaches[0].planning.state==PlanningState::Exhausted);
        JULIA_CHECK(baseline.snapshot().reaches[0].planning.candidates==32 && baseline.snapshot().reaches[0].planning.configurations==1);
        report << "single_same_scene=Exhausted_32_candidates\n";
        const auto memoryBefore=memory(); BodyInstance direct(1,asset,&profile,false); prepare(direct,demo.scene); direct.enqueue(StartReach{target}); Metrics directMetrics;
        directMetrics.step(direct); finish(direct,0,&directMetrics); const auto positive=direct.snapshot().reaches[0];
        JULIA_CHECK(positive.state==ReachState::Succeeded && positive.error<=target.tolerance && positive.route.selectedConfiguration>1 && positive.route.segmentCount==1);
        JULIA_CHECK(positive.route.solutions[0].directStatus==PathStatus::Blocked && positive.route.segmentChecks[0].status==PathStatus::Clear);
        JULIA_CHECK(positive.route.solutions[positive.route.selectedConfiguration-1].detail==pathStatusName(PathStatus::Clear));
        JULIA_CHECK(positive.route.candidates==0 && positive.route.ikIterations==ik.observation().ikIterations);
        report << "alternative_direct_selected=" << positive.route.selectedConfiguration << "\nalternative_direct_error=" << positive.error
            << "\nalternative_direct_ticks=" << positive.route.ticks << "\nalternative_direct_queries=" << positive.route.queries << "\nalternative_direct_poses=" << positive.route.poses << '\n';
        stats(report,"direct_tick_us",directMetrics.ticks);
        for (std::size_t i=0;i<4;++i) stats(report,("direct_stage"+std::to_string(i)+"_us").c_str(),directMetrics.stages[i]);
        f.target.detour.maxConfigurations=4;
        BodyInstance waypoint(1,asset,&profile,false); prepare(waypoint,f.blocked); waypoint.enqueue(StartReach{f.target}); Metrics metrics; metrics.step(waypoint);
        waitRoute(waypoint,0,&metrics); const auto accepted=waypoint.snapshot().reaches[0].route; finish(waypoint,0,&metrics);
        const auto& w=waypoint.snapshot().reaches[0];
        JULIA_CHECK(w.state==ReachState::Succeeded && w.error<=w.target.tolerance && w.route.selectedConfiguration>1 && w.route.segmentCount==2);
        JULIA_CHECK(w.route.segmentChecks[0].status==PathStatus::Clear && w.route.segmentChecks[1].status==PathStatus::Clear);
        JULIA_CHECK(w.route.finalCoordinates==accepted.finalCoordinates && w.route.selectedConfiguration==accepted.selectedConfiguration);
        report << "alternative_waypoint_selected=" << w.route.selectedConfiguration << "\nwaypoint_candidate=" << w.route.selectedCandidate
            << "\nwaypoint_planning_ticks=" << w.route.ticks << "\nwaypoint_queries=" << w.route.queries << "\nwaypoint_poses=" << w.route.poses
            << "\nwaypoint_error=" << w.error << "\nIK_seeds=" << w.route.seedAttempts << "\nIK_iterations=" << w.route.ikIterations
            << "\nIK_duplicates=" << w.route.duplicates << "\nIK_fk_checks=" << w.route.fkChecks << "\nscratch=" << metrics.scratchBytes
            << "\nprivate_before=" << memoryBefore << "\nprivate_after=" << memory() << "\nmax_speed=" << metrics.maxSpeed << "\nmax_acceleration=" << metrics.maxAcceleration
            << "\nmax_queries_per_tick=" << metrics.maxQueriesPerTick << "\nmax_poses_per_tick=" << metrics.maxPosesPerTick << '\n';
        stats(report,"waypoint_tick_us",metrics.ticks); stats(report,"planning_tick_us",metrics.planningTicks);
        for (std::size_t i=0;i<4;++i) stats(report,("waypoint_stage"+std::to_string(i)+"_us").c_str(),metrics.stages[i]);
        report << "stage_cost_ik_us=" << w.route.ikMicroseconds << "\nstage_cost_segments_us=" << w.route.segmentMicroseconds << "\nstage_cost_path_us=" << w.route.pathMicroseconds << '\n';
        // Locked active coordinates produce duplicates to the finite set's end;
        // failure to fill four slots is ordinary finite completion, not a budget error.
        auto lockedProfile=profile;
        for (auto& joint:lockedProfile.joints) if (joint.id==JointId::LeftShoulder || joint.id==JointId::LeftElbow)
            for (auto& coordinate:joint.coordinates) { coordinate.lower=-.00001f; coordinate.upper=.00001f; }
        BodyRigBinding lockedBinding(lockedProfile,*asset); DirectBodyControl locked(lockedBinding); locked.setEnabled(true);
        auto home=f.home; home.detour.maxConfigurations=4;
        ArmPositionSearchJob duplicate(locked,home,home.position); while (!duplicate.done()) duplicate.advance(16,1);
        JULIA_CHECK(duplicate.solutions().size()==1 && duplicate.observation().duplicates==21 && !duplicate.budgetExhausted());
        JULIA_CHECK(duplicate.observation().ikCompletion==IkSearchCompletion::SeedSetExhausted);
        auto legacyFailure=f.target; legacyFailure.detour.maxConfigurations=1;
        ArmPositionSearchJob singleFailure(locked,legacyFailure,legacyFailure.position); while (!singleFailure.done()) singleFailure.advance(16,1);
        JULIA_CHECK(singleFailure.solutions().empty() && singleFailure.observation().seedAttempts==9 && !singleFailure.budgetExhausted());
        BodyInstance noIK(1,asset,&lockedProfile,false); prepare(noIK,f.empty); noIK.enqueue(StartReach{f.target}); noIK.step(); finish(noIK);
        JULIA_CHECK(noIK.snapshot().reaches[0].planning.state==PlanningState::IkFailed && noIK.snapshot().reaches[0].planning.configurations==0);
        JULIA_CHECK(noIK.snapshot().reaches[0].planning.seedAttempts==30 && noIK.snapshot().reaches[0].planning.queries==0);
        // A wrist-centred obstacle rejects every endpoint without altering the target.
        world::SceneObject endpoint; endpoint.id=501; endpoint.position=f.target.position; endpoint.shape=world::Sphere{.015};
        auto allRejected=f.target; allRejected.detour.maxCandidates=4;
        BodyInstance rejected(1,asset,&profile,false); prepare(rejected,scene(1,{endpoint})); rejected.enqueue(StartReach{allRejected}); Metrics rejectedMetrics; rejectedMetrics.step(rejected); finish(rejected,0,&rejectedMetrics);
        const auto& reject=rejected.snapshot().reaches[0].planning;
        report << "all_rejected_state=" << planningStateName(reject.state) << " candidates=" << reject.candidates << " direct=" << pathStatusName(reject.directCheck.status) << " form=" << reject.directCheck.bodyForm << '\n';
        JULIA_CHECK(reject.state==PlanningState::Exhausted && reject.configurations==4 && reject.candidates==4);
        for (unsigned i=0;i<4;++i) {
            report << "rejected_config=" << i+1 << " direct=" << pathStatusName(reject.solutions[i].directStatus) << " candidates=" << reject.solutions[i].candidates << '\n';
            JULIA_CHECK(reject.solutions[i].directStatus==PathStatus::Blocked && reject.solutions[i].candidates==1);
        }
        report << "duplicates_finite_noIK_all_configurations_rejected=PASS\n";
        // Budgets stop at IK, direct checks, and either waypoint connector.
        for (unsigned type=0;type<6;++type) {
            auto limited=f.target;
            if (type==0) limited.detour.maxIkIterations=1;
            if (type==1) limited.detour.maxSeedAttempts=1;
            if (type==2) limited.detour.maxTotalQueries=1;
            if (type==3) limited.detour.maxTotalPoses=1;
            if (type==4) { limited.detour.maxConfigurations=1; limited.detour.maxTotalQueries=26675; }
            if (type==5) { limited.detour.maxConfigurations=1; limited.detour.maxTotalQueries=w.route.directCheck.queries+1; }
            BodyInstance body(1,asset,&profile,false); prepare(body,f.blocked); body.enqueue(StartReach{limited}); Metrics check; check.step(body); finish(body,0,&check);
            const auto& p=body.snapshot().reaches[0].planning;
            JULIA_CHECK(p.state==PlanningState::BudgetExhausted && !p.budgetReason.empty() && body.snapshot().reaches[0].route.segmentCount==0);
            if (type<2) JULIA_CHECK(p.stage==PlanningState::GeneratingIK && p.queries==0);
            if (type==4) JULIA_CHECK(p.stage==PlanningState::CheckingSecond);
            if (type==5) JULIA_CHECK(p.stage==PlanningState::CheckingFirst);
            report << "budget_type=" << type << " stage=" << planningStateName(p.stage) << " reason=" << p.budgetReason << " queries=" << p.queries << '\n';
        }
        // CheckIncomplete remains distinct from the outer operation budget.
        CollisionSettings incomplete; incomplete.maxQueries=1;
        BodyInstance uncertain(1,asset,&profile,false); prepare(uncertain,f.blocked,{},incomplete); uncertain.enqueue(StartReach{f.target}); uncertain.step(); finish(uncertain);
        JULIA_CHECK(uncertain.snapshot().reaches[0].planning.state==PlanningState::CheckIncomplete);
        for (unsigned i=0;i<4;++i) JULIA_CHECK(uncertain.snapshot().reaches[0].planning.solutions[i].directStatus==PathStatus::Incomplete);
        // Each side/effector/space uses the same production generator and transformed root.
        for (std::size_t side=0;side<2;++side) for (unsigned effector=0;effector<3;++effector) for (bool world:{false,true}) {
            const RootPose root{{1.2f,.3f,-.4f},glm::angleAxis(.8f,glm::vec3{0,1,0})};
            auto sample=fixture(binding,side,effector,root,world); sample.target.detour.maxConfigurations=4;
            BodyInstance body(1,asset,&profile,false); prepare(body,sample.blocked,root); body.enqueue(StartReach{sample.target}); Metrics check; check.step(body,side); finish(body,side,&check);
            const auto& o=body.snapshot().reaches[side];
            JULIA_CHECK(o.state==ReachState::Succeeded && o.error<=o.target.tolerance && o.route.configurations==4);
            report << "supported=" << side << ',' << effector << ',' << world << " selected=" << o.route.selectedConfiguration << " segments=" << o.route.segmentCount << '\n';
        }
        const auto reachStage=[&](BodyInstance& body,PlanningState stage) {
            for (unsigned i=0;i<12000 && body.snapshot().reaches[0].planning.stage!=stage && active(body.snapshot().reaches[0].state);++i) body.step();
            JULIA_CHECK(body.snapshot().reaches[0].planning.stage==stage);
        };
        for (auto stage:{PlanningState::GeneratingIK,PlanningState::CheckingDirect,PlanningState::CheckingFirst,PlanningState::CheckingSecond}) {
            BodyInstance cancel(1,asset,&profile,false); prepare(cancel,f.blocked); cancel.enqueue(StartReach{f.target}); cancel.step(); reachStage(cancel,stage);
            const auto held=cancel.snapshot().coordinates; cancel.enqueue(CancelReach{ArmSide::Left,cancel.snapshot().reaches[0].action}); cancel.step(); finish(cancel);
            JULIA_CHECK(cancel.snapshot().reaches[0].state==ReachState::Cancelled && cancel.snapshot().coordinates==held);
            BodyInstance retarget(1,asset,&profile,false); prepare(retarget,f.blocked); retarget.enqueue(StartReach{f.target}); retarget.step(); reachStage(retarget,stage);
            retarget.enqueue(RetargetReach{retarget.snapshot().reaches[0].action,home}); retarget.step(); finish(retarget);
            JULIA_CHECK(retarget.snapshot().reaches[0].state==ReachState::Succeeded && retarget.snapshot().reaches[0].target.position==home.position);
            for (const auto& op:std::array<BodyOperation,4>{ResetCoordinates{},SetRoot{{{1,0,0},{1,0,0,0}}},SetDirectMode{false},ResetBody{}}) {
                BodyInstance body(1,asset,&profile,false); prepare(body,f.blocked); body.enqueue(StartReach{f.target}); body.step(); reachStage(body,stage);
                body.enqueue(op); body.step(); JULIA_CHECK(!active(body.snapshot().reaches[0].state));
                JULIA_CHECK(body.snapshot().collisionScene==f.blocked && body.snapshot().pathCheck.status==PathStatus::Invalidated);
            }
        }
        // Failed IK replacement restores the exact admitted alternative pose/certificates.
        BodyInstance restore(1,asset,&profile,false); prepare(restore,f.blocked); restore.enqueue(StartReach{f.target}); restore.step(); waitRoute(restore);
        const auto action=restore.snapshot().reaches[0].action; const auto oldRoute=record(restore.snapshot().reaches[0].route);
        auto failure=home; failure.detour.maxIkIterations=1;
        restore.enqueue(RetargetReach{action,failure}); restore.step(); JULIA_CHECK(restore.snapshot().reaches[0].planning.state==PlanningState::WaitingForStop);
        for (unsigned i=0;i<12000 && restore.snapshot().reaches[0].planning.state!=PlanningState::BudgetExhausted;++i) restore.step();
        JULIA_CHECK(record(restore.snapshot().reaches[0].route)==oldRoute); finish(restore);
        JULIA_CHECK(restore.snapshot().reaches[0].action==action && restore.snapshot().reaches[0].state==ReachState::Succeeded);
        for (unsigned segment:{0u,1u}) {
            BodyInstance cancel(1,asset,&profile,false); prepare(cancel,f.blocked); cancel.enqueue(StartReach{f.target}); cancel.step(); Metrics check; waitRoute(cancel,segment,&check);
            cancel.enqueue(CancelReach{ArmSide::Left,cancel.snapshot().reaches[0].action}); check.step(cancel); finish(cancel,0,&check);
            JULIA_CHECK(cancel.snapshot().reaches[0].state==ReachState::Cancelled && cancel.snapshot().reaches[0].route.segmentCount==segment+1);
        }
        BodyInstance conflicts(1,asset,&profile,false); prepare(conflicts,f.blocked); conflicts.enqueue(StartReach{f.target}); conflicts.step();
        auto other=home; other.arm=ArmSide::Right; other.effector=EffectorId::RightWrist;
        auto directTarget=f.target; directTarget.strategy=MovementStrategy::DirectOnly;
        conflicts.enqueue(StartReach{other}); conflicts.enqueue(StartReach{directTarget}); conflicts.enqueue(SetCoordinate{JointId::Head,0,.1f}); conflicts.enqueue(SetCollisionScene{scene(2),{}}); conflicts.step();
        for (const auto& result:conflicts.results()) JULIA_CHECK(result.status==BodyCommandStatus::Rejected);
        conflicts.enqueue(ResetBody{}); conflicts.step(); JULIA_CHECK(conflicts.snapshot().epoch==1);
        JULIA_CHECK(conflicts.submit({100,1,conflicts.snapshot().tick+1,0,StartReach{f.target}}).status==BodyCommandStatus::Rejected);
        report << "retarget_cancel_restore_reset_epochs_conflicts=PASS\n";
        replay(asset,profile,f,report);
        auto directReplay=f; directReplay.target=target; directReplay.blocked=demo.scene;
        replay(asset,profile,directReplay,report);
        report << "result=" << (JULIA_TEST_RESULT() ? "FAIL":"PASS") << '\n';
        std::cout << "stage6c report=" << folder.string() << '\n';
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return JULIA_TEST_RESULT();
}
