#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "julia_test.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#ifdef _WIN32
#include <Windows.h>
#include <psapi.h>
#endif

using namespace julia;
using namespace julia::embodiment;
namespace
{
    using Clock = std::chrono::steady_clock;
    constexpr float stepSeconds=static_cast<float>(runtime::BodyStepSeconds);
    std::array<JointId,3> joints(std::size_t side)
    { return side==0 ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
        std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist}; }
    void set(DirectBodyControl& control,std::size_t side,const ArmCoordinates& q)
    {
        const auto ids=joints(side);
        for (std::size_t k=0;k<6;++k)
            JULIA_CHECK(control.setCoordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4,q[k]).status==CommandStatus::Applied);
    }
    animation::Pose pose(const DirectBodyControl& control)
    {
        animation::Pose value(control.binding().asset().skeleton()); control.apply(value.locals);
        animation::computeGlobalTransforms(control.binding().asset().skeleton(),value.locals,value.globals); return value;
    }
    glm::mat4 matrix(const RootPose& root)
    { return glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation); }
    std::shared_ptr<const world::CollisionScene> scene(std::uint64_t version,std::vector<world::SceneObject> objects={})
    { return std::make_shared<const world::CollisionScene>(version,std::move(objects)); }
    bool active(ReachState state)
    { return state==ReachState::Planning || state==ReachState::Moving || state==ReachState::Settling; }
    struct RouteRecord
    {
        PlanningState state;
        std::uint64_t command;
        unsigned count,current,selected,candidates,queries,poses;
        ArmCoordinates waypoint;
        std::array<unsigned,2> durations,checkPoses,checkQueries;
        std::array<PathStatus,2> statuses;
        std::array<std::uint64_t,2> versions,starts,checkCommands,objects;
        std::array<std::uint32_t,2> forms,otherForms;
        std::array<double,2> from,to,gaps;
        bool operator==(const RouteRecord&) const = default;
    };
    RouteRecord record(const PlanningObservation& route)
    {
        RouteRecord result{}; result.state=route.state; result.command=route.command; result.count=route.segmentCount;
        result.current=route.currentSegment; result.selected=route.selectedCandidate; result.candidates=route.candidates;
        result.queries=route.queries; result.poses=route.poses; result.waypoint=route.waypoint; result.durations=route.segmentDurations;
        for (std::size_t i=0;i<2;++i)
        {
            const auto& check=route.segmentChecks[i]; result.checkPoses[i]=check.poses; result.checkQueries[i]=check.queries;
            result.statuses[i]=check.status; result.versions[i]=check.sceneVersion; result.starts[i]=check.segmentStartTick;
            result.checkCommands[i]=check.command; result.objects[i]=check.object; result.forms[i]=check.bodyForm;
            result.otherForms[i]=check.otherBodyForm; result.from[i]=check.fromTick; result.to[i]=check.toTick; result.gaps[i]=check.contact.gap;
        }
        return result;
    }
    struct Fixture
    {
        ReachTarget target,home;
        ArmCoordinates solved{};
        std::shared_ptr<const world::CollisionScene> blocked,empty;
    };
    Fixture fixture(const BodyRigBinding& binding,std::size_t side=0,unsigned effector=0,const RootPose& root={},bool world=false)
    {
        DirectBodyControl source(binding); source.setEnabled(true); Fixture value;
        value.target.arm=side==0 ? ArmSide::Left:ArmSide::Right;
        value.target.effector=static_cast<EffectorId>(side*4+effector);
        value.home=value.target;
        value.home.position=glm::vec3{binding.effectorFrame(value.home.effector,pose(source).globals)[3]};
        set(source,side,{.8f,.6f,.1f,1.2f,0,0});
        value.target.position=glm::vec3{binding.effectorFrame(value.target.effector,pose(source).globals)[3]};
        source.reset(); const auto solved=solveArmPosition(source,value.target,value.target.position);
        JULIA_CHECK(solved.reason==ReachReason::Reached);
        ArmCoordinates middle{};
        for (std::size_t k=0;k<6;++k) { value.solved[k]=static_cast<float>(solved.coordinates[k]); middle[k]=value.solved[k]*.5f; }
        set(source,side,middle);
        const auto forms=bodyColliders(binding,pose(source).globals,matrix(root));
        const auto& forearm=forms[side*3+1].capsule;
        world::SceneObject obstacle; obstacle.id=101; obstacle.position=(forearm.a+forearm.b)*.5;
        obstacle.rotation=glm::normalize(glm::dquat{root.rotation}); obstacle.shape=world::Box{{.001,.008,.008}};
        value.blocked=scene(1,{obstacle}); value.empty=scene(1);
        value.target.strategy=MovementStrategy::AllowDetour; value.home.strategy=MovementStrategy::AllowDetour;
        if (world)
        {
            value.target.space=TargetSpace::World; value.home.space=TargetSpace::World;
            value.target.position=glm::vec3{matrix(root)*glm::vec4{value.target.position,1}};
            value.home.position=glm::vec3{matrix(root)*glm::vec4{value.home.position,1}};
        }
        return value;
    }
    void prepare(BodyInstance& body,const std::shared_ptr<const world::CollisionScene>& geometry,const RootPose& root={},CollisionSettings settings={})
    {
        body.enqueue(SetDirectMode{true}); body.enqueue(SetRoot{root}); body.enqueue(SetCollisionScene{geometry,settings}); body.step();
        for (const auto& result:body.results()) JULIA_CHECK(result.status==BodyCommandStatus::Applied);
    }
    struct Metrics
    {
        std::vector<double> ticks,planningTicks;
        std::vector<glm::vec3> velocity;
        float maxSpeed=0,maxAcceleration=0;
        std::size_t scratchBytes=0;
        unsigned maxQueriesPerTick=0,maxPosesPerTick=0;
        void step(BodyInstance& body,std::size_t side=0)
        {
            const auto before=body.snapshot().coordinates;
            const auto previous=body.snapshot().reaches[side].planning;
            const auto previousRouteCount=body.snapshot().reaches[side].route.segmentCount;
            const auto previousSegment=body.snapshot().reaches[side].route.currentSegment;
            const bool planning=body.snapshot().reaches[side].state==ReachState::Planning;
            const auto start=Clock::now(); body.step();
            const auto elapsed=std::chrono::duration<double,std::micro>(Clock::now()-start).count();
            ticks.push_back(elapsed); if (planning) planningTicks.push_back(elapsed);
            const auto& observation=body.snapshot().reaches[side]; const auto& current=observation.planning;
            if (planning && observation.state==ReachState::Planning) JULIA_CHECK(body.snapshot().coordinates==before);
            if (previousRouteCount==2 && previousSegment==0 && observation.route.currentSegment==1)
            {
                JULIA_CHECK(body.snapshot().coordinates==before);
                JULIA_CHECK(observation.route.segmentChecks[1].segmentStartTick==body.snapshot().tick);
            }
            scratchBytes=std::max(scratchBytes,current.scratchBytes);
            if (previous.command==current.command && previous.queries<=current.queries && previous.poses<=current.poses)
            {
                maxQueriesPerTick=std::max(maxQueriesPerTick,current.queries-previous.queries);
                maxPosesPerTick=std::max(maxPosesPerTick,current.poses-previous.poses);
                JULIA_CHECK(current.queries-previous.queries<=observation.target.detour.queriesPerTick);
                JULIA_CHECK(current.poses-previous.poses<=observation.target.detour.posesPerTick);
            }
            if (velocity.empty()) velocity.resize(before.size(),glm::vec3{0});
            const auto& profile=body.control()->binding().profile();
            for (std::size_t j=0;j<before.size();++j)
            {
                const auto v=(body.snapshot().coordinates[j]-before[j])/stepSeconds;
                const auto a=(v-velocity[j])/stepSeconds; velocity[j]=v;
                for (int k=0;k<3;++k) { maxSpeed=std::max(maxSpeed,std::abs(v[k])); maxAcceleration=std::max(maxAcceleration,std::abs(a[k])); }
                for (std::size_t k=0;k<profile.joints[j].coordinates.size();++k)
                {
                    const auto q=body.snapshot().coordinates[j][static_cast<int>(k)]; const auto& limit=profile.joints[j].coordinates[k];
                    JULIA_CHECK(q>=limit.lower-1e-6f && q<=limit.upper+1e-6f);
                }
            }
            JULIA_CHECK(maxSpeed<=observation.target.maxSpeed+.0001f);
            JULIA_CHECK(maxAcceleration<=observation.target.maxAcceleration+.03f);
        }
    };
    void finish(BodyInstance& body,std::size_t side=0,Metrics* metrics=nullptr)
    {
        for (unsigned tick=0;tick<16000 && active(body.snapshot().reaches[side].state);++tick)
            if (metrics) metrics->step(body,side); else body.step();
        JULIA_CHECK(!active(body.snapshot().reaches[side].state));
    }
    void waitRoute(BodyInstance& body,unsigned segment=0,Metrics* metrics=nullptr)
    {
        for (unsigned tick=0;tick<16000;++tick)
        {
            const auto& o=body.snapshot().reaches[0];
            if (o.state==ReachState::Moving && o.route.segmentCount==2 && o.route.currentSegment==segment) return;
            if (!active(o.state)) break;
            if (metrics) metrics->step(body); else body.step();
        }
        JULIA_CHECK(false);
    }
    std::size_t memory()
    {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS_EX value{}; value.cb=sizeof(value);
        GetProcessMemoryInfo(GetCurrentProcess(),reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&value),sizeof(value)); return value.PrivateUsage;
#else
        return 0;
#endif
    }
    void stats(std::ostream& out,const char* name,std::vector<double> values)
    {
        if (values.empty()) { out << name << "_count=0\n"; return; }
        std::sort(values.begin(),values.end());
        out << name << "_count=" << values.size() << '\n' << name << "_mean=" << std::accumulate(values.begin(),values.end(),0.0)/static_cast<double>(values.size())
            << '\n' << name << "_p95=" << values[static_cast<std::size_t>(.95*static_cast<double>(values.size()-1))] << '\n' << name << "_max=" << values.back() << '\n';
    }
    std::uint64_t hash(const BodySnapshot& value)
    {
        std::uint64_t h=1469598103934665603ull;
        for (const auto& m:value.pose.palette) for (int c=0;c<4;++c) for (int r=0;r<4;++r)
        { h^=std::bit_cast<std::uint32_t>(m[c][r]); h*=1099511628211ull; }
        return h;
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const Fixture& f,std::ostream& report)
    {
        struct State
        {
            std::vector<glm::vec3> q; std::uint64_t palette;
            ReachState reach; ReachReason reason; PlanningState planning; PathStatus direct;
            std::array<PathStatus,2> segmentStatuses;
            unsigned candidates,queries,poses,segment,selected; ArmCoordinates waypoint;
            RouteRecord route;
            std::vector<BodyCommandResult> results;
        };
        const auto schedule=[&](BodyInstance& body)
        {
            for (const auto& command:std::vector<BodyCommand>{{1,1,1,0,SetDirectMode{true}},
                {2,1,1,0,SetCollisionScene{f.blocked,{}}},{3,1,2,0,StartReach{f.target}},
                {4,1,3,0,SetCoordinate{JointId::Head,0,.1f}},{5,1,3,0,SetCollisionScene{scene(2),{}}},
                {6,1,1800,0,ResetCoordinates{}}})
                JULIA_CHECK(body.submit(command).status==BodyCommandStatus::Queued);
        };
        constexpr unsigned total=2400;
        BodyInstance exact(1,asset,&profile,false); schedule(exact); std::vector<State> history; history.reserve(total);
        for (unsigned t=0;t<total;++t)
        {
            exact.step(); const auto& s=exact.snapshot(); const auto& p=s.reaches[0].planning;
            history.push_back({s.coordinates,hash(s),s.reaches[0].state,s.reaches[0].reason,p.state,p.directCheck.status,
                {p.segmentChecks[0].status,p.segmentChecks[1].status},p.candidates,p.queries,p.poses,p.currentSegment,p.selectedCandidate,p.waypoint,record(s.reaches[0].route),exact.results()});
        }
        for (int fps:{30,60,144})
        {
            BodyInstance body(1,asset,&profile,false); schedule(body); runtime::FixedStepScheduler scheduler; double elapsed=0;
            while (elapsed<20-1e-12)
            {
                const double frame=std::min(20-elapsed,1.0/fps);
                scheduler.advance(frame,[&]
                {
                    body.step(); const auto& s=body.snapshot(); const auto& p=s.reaches[0].planning;
                    const auto& expected=history[static_cast<std::size_t>(s.tick-1)];
                    JULIA_CHECK(s.coordinates==expected.q && hash(s)==expected.palette && s.reaches[0].state==expected.reach && s.reaches[0].reason==expected.reason);
                    JULIA_CHECK(p.state==expected.planning && p.directCheck.status==expected.direct && p.candidates==expected.candidates && p.queries==expected.queries && p.poses==expected.poses);
                    JULIA_CHECK(p.currentSegment==expected.segment && p.selectedCandidate==expected.selected && p.waypoint==expected.waypoint && body.results().size()==expected.results.size());
                    JULIA_CHECK(p.segmentChecks[0].status==expected.segmentStatuses[0] && p.segmentChecks[1].status==expected.segmentStatuses[1]);
                    JULIA_CHECK(record(s.reaches[0].route)==expected.route);
                    for (std::size_t k=0;k<body.results().size();++k)
                        JULIA_CHECK(body.results()[k].status==expected.results[k].status && body.results()[k].reason==expected.results[k].reason);
                });
                elapsed+=frame; const auto tick=body.snapshot().tick;
                const auto display=body.display(scheduler.alpha()); const auto again=body.display(scheduler.alpha());
                JULIA_CHECK(tick==body.snapshot().tick && display.pose.palette==again.pose.palette);
            }
            JULIA_CHECK(body.snapshot().tick==total); report << "replay_fps_" << fps << "=identical_2400_ticks\n";
        }
    }
}
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
