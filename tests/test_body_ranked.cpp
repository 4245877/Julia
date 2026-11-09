#include "body_reach_test_support.hpp"
#include "../apps/common/CollisionDemo.hpp"
#include <limits>
using namespace body_reach_test;

namespace
{
    ReachTarget ranked(ReachTarget target,glm::vec3 direction={1,0,0})
    {
        target.strategy=MovementStrategy::AllowDetour; target.selection=RouteSelection::Ranked;
        target.detour.maxConfigurations=4; target.detour.maxCandidates=0;
        target.elbowPreference=ElbowPreference{direction,TargetSpace::Body}; return target;
    }
    void verifyScore(const ReachObservation& o)
    {
        const auto& p=o.route; const auto& score=p.score;
        JULIA_CHECK(p.segmentCount>0 && p.scoreEvaluations>0);
        for (unsigned i=0;i<p.segmentCount;++i) JULIA_CHECK(p.segmentChecks[i].status==PathStatus::Clear);
        JULIA_CHECK(o.state==ReachState::Succeeded && o.error<=o.target.tolerance);
        JULIA_CHECK(glm::length(score.elbow.elbow-o.elbowBody)<1e-6f);
        JULIA_CHECK(std::abs(score.elbow.penalty-o.actualElbow.penalty)<1e-10);
        JULIA_CHECK(std::abs(score.total-(p.ranking.elbowWeight*score.elbow.penalty+p.ranking.travelWeight*score.travel+p.ranking.durationWeight*score.duration))<1e-12);
        JULIA_CHECK(score.key==std::llround(score.total*1e9));
        for (unsigned i=0;i<p.scoreEvaluations;++i) JULIA_CHECK(score.key<=p.evaluations[i].score.key);
    }
}
int main(int argc,char** argv)
{
    try {
        const std::filesystem::path folder=argc>1 ? argv[1]:"build/diagnostics/stage6d/cpu";
        std::filesystem::create_directories(folder); std::ofstream report(folder/"body_ranked.txt"); report << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile(); const BodyRigBinding binding(profile,*asset); auto f=fixture(binding);
        // Geometry contracts do not depend on the visual bind-zero elbow angle.
        const glm::vec3 s{0},e{.5f,.2f,0},w{1,0,0};
        const auto absent=evaluateElbowPreference(s,e,w,std::nullopt);
        const auto aligned=evaluateElbowPreference(s,e,w,glm::vec3{0,1,0});
        const auto opposite=evaluateElbowPreference(s,e,w,glm::vec3{0,-1,0});
        JULIA_CHECK(absent.status==ElbowPreferenceStatus::Absent && absent.penalty==0);
        JULIA_CHECK(aligned.status==ElbowPreferenceStatus::Defined && aligned.penalty==0 && opposite.penalty==1);
        for (float offset:{-1e-7f,0.0f,1e-7f}) {
            const auto parallel=evaluateElbowPreference(s,e,w,glm::vec3{1,offset,0});
            const auto straight=evaluateElbowPreference(s,{.5f,offset,0},w,glm::vec3{0,1,0});
            JULIA_CHECK(parallel.status==ElbowPreferenceStatus::ParallelDirection && parallel.penalty==1);
            JULIA_CHECK(straight.status==ElbowPreferenceStatus::StraightArm && straight.penalty==1);
        }
        JULIA_CHECK(evaluateElbowPreference(s,e,s,glm::vec3{0,1,0}).status==ElbowPreferenceStatus::CollapsedAxis);
        JULIA_CHECK(evaluateElbowPreference(s,e,w,glm::vec3{0}).penalty==1);
        JULIA_CHECK(evaluateElbowPreference(s,e,w,glm::vec3{std::numeric_limits<float>::quiet_NaN()}).penalty==1);
        report << "geometry_absent_straight_parallel_collapsed=PASS\n";

        // Default and explicit FirstValid retain production ordering and all counters.
        auto legacy=f.target; legacy.detour.maxConfigurations=4;
        BodyInstance defaultBody(1,asset,&profile,false),explicitBody(1,asset,&profile,false);
        prepare(defaultBody,f.blocked); prepare(explicitBody,f.blocked);
        defaultBody.enqueue(StartReach{legacy}); legacy.selection=RouteSelection::FirstValid; explicitBody.enqueue(StartReach{legacy});
        defaultBody.step(); explicitBody.step(); finish(defaultBody); finish(explicitBody);
        JULIA_CHECK(record(defaultBody.snapshot().reaches[0].route)==record(explicitBody.snapshot().reaches[0].route));
        JULIA_CHECK(defaultBody.snapshot().reaches[0].route.selectedConfiguration==3 && defaultBody.snapshot().reaches[0].route.selectedCandidate==2);
        JULIA_CHECK(defaultBody.snapshot().reaches[0].route.scoreEvaluations==0);
        report << "FirstValid_default_explicit_6C_waypoint=identical\n";

        const auto memoryBefore=memory();
        std::array<ReachObservation,2> oppositeResults;
        for (unsigned sign=0;sign<2;++sign) {
            auto target=ranked(f.target,{sign==0 ? 1.0f:-1.0f,0,0});
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); body.enqueue(StartReach{target}); Metrics metrics; metrics.step(body);
            while (body.snapshot().reaches[0].state==ReachState::Planning) {
                const auto before=body.snapshot().coordinates; metrics.step(body);
                JULIA_CHECK(body.snapshot().coordinates==before); // No motion on admission tick either.
                if (body.snapshot().reaches[0].state==ReachState::Planning) JULIA_CHECK(body.snapshot().reaches[0].route.segmentCount==0);
            }
            const auto admitted=record(body.snapshot().reaches[0].route); finish(body,0,&metrics);
            const auto& o=body.snapshot().reaches[0]; verifyScore(o); oppositeResults[sign]=o;
            JULIA_CHECK(record(o.route)==admitted && o.route.scoreEvaluations==4 && o.route.completion==SelectionCompletion::FiniteSet);
            report << "sign=" << sign << " configuration=" << o.route.selectedConfiguration << " clear_routes=" << o.route.scoreEvaluations
                << " ticks=" << o.route.ticks << " first_clear=" << o.route.firstClearTick << " queries=" << o.route.queries << " poses=" << o.route.poses
                << " score=" << o.route.score.total << " E=" << o.route.score.elbow.penalty << " J=" << o.route.score.travel << " T=" << o.route.score.duration
                << " error_m=" << o.error << " elbow=" << o.elbowBody.x << ',' << o.elbowBody.y << ',' << o.elbowBody.z << '\n';
            stats(report,("sign"+std::to_string(sign)+"_tick_us").c_str(),metrics.ticks);
            stats(report,("sign"+std::to_string(sign)+"_planning_us").c_str(),metrics.planningTicks);
            report << "score_us=" << o.route.scoreMicroseconds << " scratch_bytes=" << metrics.scratchBytes << " max_v=" << metrics.maxSpeed << " max_a=" << metrics.maxAcceleration << '\n';
            report << "seeds=" << o.route.seedAttempts << " iterations=" << o.route.ikIterations << " duplicates=" << o.route.duplicates << " fk_checks=" << o.route.fkChecks
                << " ik_us=" << o.route.ikMicroseconds << " segments_us=" << o.route.segmentMicroseconds << " path_us=" << o.route.pathMicroseconds
                << " max_tick_queries=" << metrics.maxQueriesPerTick << " max_tick_poses=" << metrics.maxPosesPerTick << '\n';
        }
        JULIA_CHECK(oppositeResults[0].route.selectedConfiguration!=oppositeResults[1].route.selectedConfiguration);
        JULIA_CHECK(glm::length(oppositeResults[0].elbowBody-oppositeResults[1].elbowBody)>.02f);
        report << "opposite_preference_changes_verified_final_elbow=PASS\nprivate_before=" << memoryBefore << "\nprivate_after=" << memory() << '\n';

        // Degenerate preference evaluated through real generation, validation and commands.
        auto parallel=ranked(f.target); parallel.detour.maxConfigurations=1;
        const auto& endpoint0=oppositeResults[0].route.evaluations[0].score.elbow;
        parallel.elbowPreference=ElbowPreference{endpoint0.wrist-endpoint0.shoulder,TargetSpace::Body};
        BodyInstance parallelBody(1,asset,&profile,false); prepare(parallelBody,f.empty); parallelBody.enqueue(StartReach{parallel}); parallelBody.step(); finish(parallelBody);
        verifyScore(parallelBody.snapshot().reaches[0]);
        JULIA_CHECK(parallelBody.snapshot().reaches[0].route.score.elbow.status==ElbowPreferenceStatus::ParallelDirection);
        for (std::size_t side=0;side<2;++side) {
            DirectBodyControl source(binding); source.setEnabled(true); const auto ids=joints(side);
            const auto& limit=profile.joints[binding.jointIndex(ids[1])].coordinates[0];
            float bestDistance=0,bestAngle=0; glm::vec3 extension{0};
            for (unsigned k=0;k<=1000;++k) {
                const float angle=limit.lower+(limit.upper-limit.lower)*static_cast<float>(k)/1000;
                source.setCoordinate(ids[1],0,angle); const auto actual=pose(source);
                const auto a=glm::vec3{binding.jointFrame(ids[0],actual.globals)[3]},wrist=glm::vec3{binding.jointFrame(ids[2],actual.globals)[3]};
                if (glm::length(wrist-a)>bestDistance) { bestDistance=glm::length(wrist-a); bestAngle=angle; extension=wrist; }
            }
            ReachTarget target; target.arm=side==0 ? ArmSide::Left:ArmSide::Right; target.effector=side==0 ? EffectorId::LeftWrist:EffectorId::RightWrist;
            target=ranked(target); target.position=extension; target.detour.maxConfigurations=1;
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); body.enqueue(SetCoordinate{ids[1],0,bestAngle}); body.step();
            body.enqueue(StartReach{target}); body.step(); finish(body,side); verifyScore(body.snapshot().reaches[side]);
            const auto& evaluation=body.snapshot().reaches[side].route.score.elbow;
            // The engineering profile may stop before exact collinearity. A maximum
            // extension must remain finite; exactly straight geometry is covered above.
            JULIA_CHECK(std::isfinite(evaluation.penalty) && evaluation.penalty>=0 && evaluation.penalty<=1);
            const auto axis=glm::normalize(evaluation.wrist-evaluation.shoulder),bend=evaluation.elbow-evaluation.shoulder;
            report << "max_extension_side=" << side << " distance=" << bestDistance
                << " bend_m=" << glm::length(bend-axis*glm::dot(bend,axis)) << " status=" << elbowPreferenceStatusName(evaluation.status) << '\n';
        }
        report << "production_parallel_and_both_maximally_extended_arms=PASS\n";

        // All-zero weights create exact equal keys. Earlier Clear route wins, independent of later work.
        auto tie=ranked(f.target); tie.elbowPreference.reset(); tie.ranking.elbowWeight=tie.ranking.travelWeight=tie.ranking.durationWeight=0;
        BodyInstance equal(1,asset,&profile,false); prepare(equal,f.empty); equal.enqueue(StartReach{tie}); equal.step(); finish(equal);
        verifyScore(equal.snapshot().reaches[0]); JULIA_CHECK(equal.snapshot().reaches[0].route.selectedConfiguration==1);
        JULIA_CHECK(equal.snapshot().reaches[0].route.score.elbow.status==ElbowPreferenceStatus::Absent);
        auto absentTarget=ranked(f.target); absentTarget.elbowPreference.reset();
        BodyInstance noPreference(1,asset,&profile,false); prepare(noPreference,f.empty); noPreference.enqueue(StartReach{absentTarget}); noPreference.step(); finish(noPreference);
        verifyScore(noPreference.snapshot().reaches[0]); JULIA_CHECK(noPreference.snapshot().reaches[0].route.score.elbow.penalty==0);

        // A blocked direct route with the best soft endpoint score cannot enter the comparison.
        auto blocked=ranked(f.target);
        const auto& first=oppositeResults[0].route.evaluations[0].score.elbow;
        blocked.elbowPreference=ElbowPreference{first.elbow-first.shoulder,TargetSpace::Body};
        blocked.ranking.travelWeight=blocked.ranking.durationWeight=0;
        const auto demo=app::multiConfigurationDemo(binding,1);
        BodyInstance guarded(1,asset,&profile,false); prepare(guarded,demo.scene); guarded.enqueue(StartReach{blocked}); guarded.step(); finish(guarded);
        const auto& guardedResult=guarded.snapshot().reaches[0]; verifyScore(guardedResult);
        JULIA_CHECK(guardedResult.route.solutions[0].directStatus==PathStatus::Blocked && guardedResult.route.selectedConfiguration!=1);
        const auto forbidden=evaluateElbowPreference(first.shoulder,first.elbow,first.wrist,first.elbow-first.shoulder);
        JULIA_CHECK(forbidden.penalty<guardedResult.route.score.elbow.penalty);
        for (unsigned i=0;i<guardedResult.route.scoreEvaluations;++i) JULIA_CHECK(guardedResult.route.evaluations[i].configuration>2);
        report << "blocked_soft_best_excluded=PASS\n";

        // General budgets, evaluation budget and a deterministic post-Clear tick limit.
        for (unsigned type=0;type<6;++type) {
            auto limited=ranked(f.target); limited.detour.maxCandidates=32; limited.ranking.additionalTicks=4096;
            if (type==0) limited.detour.maxTotalQueries=1;
            if (type==1) limited.detour.maxTotalQueries=4*oppositeResults[0].route.segmentChecks[0].queries;
            if (type==2) limited.ranking.maxEvaluations=1;
            if (type==3) limited.ranking.additionalTicks=1;
            if (type==4) limited.ranking.additionalTicks=0;
            if (type==5) limited.detour.maxTotalPoses=1;
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); body.enqueue(StartReach{limited}); Metrics metrics; metrics.step(body); finish(body,0,&metrics);
            const auto& o=body.snapshot().reaches[0];
            if (type==0 || type==5) {
                JULIA_CHECK(o.state==ReachState::Failed && o.route.segmentCount==0 && o.planning.scoreEvaluations==0);
                JULIA_CHECK(o.planning.completion==SelectionCompletion::BudgetExhausted);
            } else {
                verifyScore(o);
                JULIA_CHECK(o.route.completion==(type==3 || type==4 ? SelectionCompletion::RankingLimit:SelectionCompletion::BudgetExhausted));
                if (type>=2) JULIA_CHECK(o.route.scoreEvaluations==1 && o.route.selectedConfiguration==1);
                if (type==3) JULIA_CHECK(o.route.queries>o.route.segmentChecks[0].queries); // Unfinished alternative never executes.
            }
            report << "budget_case=" << type << " completion=" << selectionCompletionName(o.planning.completion) << " scores=" << o.planning.scoreEvaluations
                << " queries=" << o.planning.queries << " state=" << reachStateName(o.state) << '\n';
        }
        // No Clear at finite completion, and no Clear on incomplete checks.
        world::SceneObject endpoint; endpoint.id=501; endpoint.position=f.target.position; endpoint.shape=world::Sphere{.015};
        for (bool incomplete:{false,true}) {
            BodyInstance body(1,asset,&profile,false); CollisionSettings settings; if (incomplete) settings.maxQueries=1;
            prepare(body,incomplete ? f.empty:scene(1,{endpoint}),{},settings); body.enqueue(StartReach{ranked(f.target)}); body.step(); finish(body);
            const auto& o=body.snapshot().reaches[0]; JULIA_CHECK(o.state==ReachState::Failed && o.route.segmentCount==0 && o.planning.scoreEvaluations==0);
            JULIA_CHECK(o.planning.completion==(incomplete ? SelectionCompletion::CheckIncomplete:SelectionCompletion::FiniteSet));
        }

        // Both arms, actual wrist geometry for palm/grip, transformed root, both preference spaces.
        const RootPose root{{1.2f,.3f,-.4f},glm::angleAxis(.8f,glm::vec3{0,1,0})};
        for (std::size_t side=0;side<2;++side) for (unsigned effector=0;effector<3;++effector) for (bool world:{false,true}) {
            auto sample=fixture(binding,side,effector,root,world); auto target=ranked(sample.target);
            if (world) target.elbowPreference=ElbowPreference{glm::vec3{matrix(root)*glm::vec4{1,0,0,0}},TargetSpace::World};
            BodyInstance body(1,asset,&profile,false); prepare(body,sample.empty,root); body.enqueue(StartReach{target}); Metrics metrics; metrics.step(body,side); finish(body,side,&metrics);
            const auto& o=body.snapshot().reaches[side]; verifyScore(o);
            JULIA_CHECK(o.elbowDirectionBody && glm::length(*o.elbowDirectionBody-glm::vec3{1,0,0})<1e-6f);
            if (effector) JULIA_CHECK(glm::length(o.actualElbow.wrist-o.actualBody)>.01f);
            report << "supported=" << side << ',' << effector << ',' << world << " configuration=" << o.route.selectedConfiguration << " scores=" << o.route.scoreEvaluations << '\n';
        }

        // Invalid payloads are rejected before starting or replacing a search.
        for (unsigned type=0;type<11;++type) {
            auto invalid=ranked(f.target);
            if (type==0) invalid.elbowPreference->direction={0,0,0};
            if (type==1) invalid.elbowPreference->direction.x=std::numeric_limits<float>::quiet_NaN();
            if (type==2) invalid.elbowPreference->direction.y=std::numeric_limits<float>::infinity();
            if (type==3) invalid.elbowPreference->space=static_cast<TargetSpace>(7);
            if (type==4) invalid.ranking.elbowWeight=-1;
            if (type==5) invalid.ranking.travelWeight=std::numeric_limits<double>::quiet_NaN();
            if (type==6) invalid.ranking.durationWeight=std::numeric_limits<double>::infinity();
            if (type==7) invalid.ranking.maxEvaluations=0;
            if (type==8) invalid.ranking.additionalTicks=4097;
            if (type==9) invalid.selection=static_cast<RouteSelection>(7);
            if (type==10) invalid.strategy=MovementStrategy::DirectOnly;
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); body.enqueue(StartReach{invalid}); body.step();
            JULIA_CHECK(body.results().back().status==BodyCommandStatus::Rejected && !active(body.snapshot().reaches[0].state));
        }

        const auto waitBest=[](BodyInstance& body) {
            for (unsigned i=0;i<16000 && body.snapshot().reaches[0].planning.scoreEvaluations==0;++i) body.step();
            JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Planning && body.snapshot().reaches[0].planning.scoreEvaluations>0);
        };
        auto pending=ranked(f.target); pending.detour.maxCandidates=32; pending.ranking.additionalTicks=128;
        // Commands while a Clear best exists but has NOT been admitted.
        for (unsigned operation=0;operation<6;++operation) {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); body.enqueue(StartReach{pending}); body.step(); waitBest(body);
            const auto held=body.snapshot().coordinates; const auto action=body.snapshot().reaches[0].action;
            if (operation==0) body.enqueue(CancelReach{ArmSide::Left,action});
            if (operation==1) body.enqueue(ResetCoordinates{});
            if (operation==2) body.enqueue(SetRoot{root});
            if (operation==3) body.enqueue(SetDirectMode{false});
            if (operation==4) body.enqueue(ResetBody{});
            if (operation==5) {
                auto replacement=ranked(f.target,{-1,0,0}); body.enqueue(RetargetReach{action,replacement});
            }
            body.step(); finish(body);
            if (operation==5) JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Succeeded && body.snapshot().reaches[0].action==action &&
                body.snapshot().reaches[0].route.selectedConfiguration==oppositeResults[1].route.selectedConfiguration);
            else JULIA_CHECK(body.snapshot().reaches[0].route.segmentCount==0 && !active(body.snapshot().reaches[0].state));
            if (operation==0) JULIA_CHECK(body.snapshot().coordinates==held && body.snapshot().reaches[0].state==ReachState::Cancelled);
            if (operation==4) {
                JULIA_CHECK(body.snapshot().epoch==1 && body.snapshot().collisionScene==f.empty);
                JULIA_CHECK(body.submit({100,1,body.snapshot().tick+1,0,StartReach{pending}}).status==BodyCommandStatus::Rejected);
            }
        }
        // Rejected payload leaves the saved best intact; external settings are value-owned.
        BodyInstance owned(1,asset,&profile,false); prepare(owned,f.empty); auto external=pending; owned.enqueue(StartReach{external});
        external.elbowPreference->direction={-1,0,0}; external.ranking.elbowWeight=0; owned.step(); waitBest(owned);
        const auto before=owned.snapshot().reaches[0].planning;
        auto invalid=pending; invalid.ranking.elbowWeight=-1; owned.enqueue(RetargetReach{owned.snapshot().reaches[0].action,invalid}); owned.step();
        JULIA_CHECK(owned.results().back().status==BodyCommandStatus::Rejected);
        JULIA_CHECK(owned.snapshot().reaches[0].planning.scoreEvaluations>=before.scoreEvaluations && owned.snapshot().reaches[0].target.ranking.elbowWeight==1);
        finish(owned); verifyScore(owned.snapshot().reaches[0]);

        // Failure of a replacement restores the admitted route and its scoring parameters.
        for (bool retarget:{false,true}) {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); auto start=ranked(f.target); body.enqueue(StartReach{start}); body.step();
            while (body.snapshot().reaches[0].state==ReachState::Planning) body.step();
            const auto admitted=record(body.snapshot().reaches[0].route); const auto action=body.snapshot().reaches[0].action;
            auto failure=ranked(f.home,{-1,0,0}); failure.detour.maxIkIterations=1;
            if (retarget) body.enqueue(RetargetReach{action,failure}); else body.enqueue(StartReach{failure});
            body.step(); JULIA_CHECK(body.snapshot().reaches[0].planning.state==PlanningState::WaitingForStop);
            for (unsigned t=0;t<16000 && body.snapshot().reaches[0].planning.state!=PlanningState::BudgetExhausted;++t) body.step();
            JULIA_CHECK(record(body.snapshot().reaches[0].route)==admitted);
            finish(body); JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Succeeded && body.snapshot().reaches[0].action==action);
        }
        BodyInstance conflict(1,asset,&profile,false); prepare(conflict,f.empty); conflict.enqueue(StartReach{pending}); conflict.step(); waitBest(conflict);
        auto right=pending; right.arm=ArmSide::Right; right.effector=EffectorId::RightWrist;
        auto direct=f.target; direct.strategy=MovementStrategy::DirectOnly;
        conflict.enqueue(StartReach{right}); conflict.enqueue(StartReach{direct}); conflict.enqueue(SetCoordinate{JointId::Head,0,.1f}); conflict.enqueue(SetCollisionScene{scene(2),{}}); conflict.step();
        for (const auto& result:conflict.results()) JULIA_CHECK(result.status==BodyCommandStatus::Rejected);
        conflict.enqueue(CancelReach{ArmSide::Left,conflict.snapshot().reaches[0].action}); conflict.step(); finish(conflict);
        report << "transactions_value_ownership_cancel_restore_reset_epochs_conflicts=PASS\n";

        // Ranked waypoint search still uses both real connectors; no endpoint substitution.
        auto waypoint=f; waypoint.target=ranked(f.target); waypoint.target.detour.maxCandidates=32;
        BodyInstance via(1,asset,&profile,false); prepare(via,f.blocked); via.enqueue(StartReach{waypoint.target}); Metrics viaMetrics; viaMetrics.step(via); finish(via,0,&viaMetrics);
        verifyScore(via.snapshot().reaches[0]); JULIA_CHECK(via.snapshot().reaches[0].route.segmentCount==2);
        report << "waypoint_scores=" << via.snapshot().reaches[0].route.scoreEvaluations << " selected=" << via.snapshot().reaches[0].route.selectedConfiguration
            << ',' << via.snapshot().reaches[0].route.selectedCandidate << " completion=" << selectionCompletionName(via.snapshot().reaches[0].route.completion) << '\n';
        const auto& viaRoute=via.snapshot().reaches[0].route;
        report << "waypoint_ticks=" << viaRoute.ticks << " first_clear=" << viaRoute.firstClearTick << " queries=" << viaRoute.queries << " poses=" << viaRoute.poses
            << " score=" << viaRoute.score.total << " E=" << viaRoute.score.elbow.penalty << " J=" << viaRoute.score.travel << " T=" << viaRoute.score.duration
            << " score_us=" << viaRoute.scoreMicroseconds << " scratch_bytes=" << viaMetrics.scratchBytes << '\n';
        stats(report,"waypoint_tick_us",viaMetrics.ticks); stats(report,"waypoint_planning_us",viaMetrics.planningTicks);
        for (unsigned segment:{0u,1u}) {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.blocked); body.enqueue(StartReach{waypoint.target}); body.step(); Metrics metrics; waitRoute(body,segment,&metrics);
            const auto route=body.snapshot().reaches[0].route;
            body.enqueue(CancelReach{ArmSide::Left,body.snapshot().reaches[0].action}); metrics.step(body); finish(body,0,&metrics);
            JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Cancelled && body.snapshot().reaches[0].route.segmentCount==segment+1);
            JULIA_CHECK(body.snapshot().reaches[0].route.score.key==route.score.key && body.snapshot().reaches[0].route.finalCoordinates==route.finalCoordinates);
        }
        {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.blocked); body.enqueue(StartReach{waypoint.target}); body.step(); waitRoute(body);
            const auto old=body.snapshot().reaches[0].route; const auto action=body.snapshot().reaches[0].action;
            auto failure=ranked(f.home,{-1,0,0}); failure.detour.maxIkIterations=1;
            body.enqueue(RetargetReach{action,failure}); body.step();
            for (unsigned t=0;t<16000 && body.snapshot().reaches[0].planning.state!=PlanningState::BudgetExhausted;++t) body.step();
            JULIA_CHECK(record(body.snapshot().reaches[0].route)==record(old));
            finish(body); const auto& restored=body.snapshot().reaches[0];
            JULIA_CHECK(restored.state==ReachState::Succeeded && restored.action==action && restored.route.currentSegment==1);
            JULIA_CHECK(restored.route.score.key==old.score.key && restored.route.finalCoordinates==old.finalCoordinates && restored.route.ranking.elbowWeight==old.ranking.elbowWeight);
            JULIA_CHECK(restored.route.segmentChecks[1].segmentStartTick>old.segmentChecks[1].segmentStartTick);
        }
        replay(asset,profile,waypoint,report);
        auto directReplay=f; directReplay.target=ranked(f.target); directReplay.blocked=f.empty; replay(asset,profile,directReplay,report);
        auto tieReplay=f; tieReplay.target=tie; tieReplay.blocked=f.empty; replay(asset,profile,tieReplay,report);
        auto budgetReplay=directReplay; budgetReplay.target.detour.maxTotalQueries=1; replay(asset,profile,budgetReplay,report);
        budgetReplay=directReplay; budgetReplay.target.ranking.maxEvaluations=1; replay(asset,profile,budgetReplay,report);
        auto commandsReplay=directReplay; commandsReplay.target=pending; auto replacement=ranked(f.home,{-1,0,0});
        replay(asset,profile,commandsReplay,report,{{7,1,22,0,RetargetReach{3,replacement}},
            {8,1,45,0,CancelReach{ArmSide::Left,3}},{9,1,80,0,StartReach{pending}},
            {10,1,290,0,RetargetReach{9,replacement}},{11,1,360,0,CancelReach{ArmSide::Left,9}},
            {12,1,650,0,StartReach{pending}}});
        report << "result=" << (JULIA_TEST_RESULT() ? "FAIL":"PASS") << '\n';
        std::cout << "stage6d report=" << folder.string() << '\n';
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return JULIA_TEST_RESULT();
}
