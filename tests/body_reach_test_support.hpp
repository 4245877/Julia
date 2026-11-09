#pragma once
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
namespace body_reach_test
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
    std::vector<std::uint64_t> accounting(const PlanningObservation& p)
    {
        std::vector<std::uint64_t> values{static_cast<unsigned>(p.stage),static_cast<unsigned>(p.budgetStage),p.seedAttempts,p.seedsProcessed,p.ikIterations,p.duplicates,p.failedSeeds,p.fkChecks,
            p.configurations,p.currentConfiguration,p.selectedConfiguration,p.tickIterations,p.tickSeeds,p.tickQueries,p.tickPoses,p.tickCandidates,
            p.budgetConfigurations,p.budgetCandidates,static_cast<unsigned>(p.lastSeedOutcome),static_cast<unsigned>(p.ikCompletion)};
        values.insert(values.end(),{p.ikEvaluations,p.tickIkEvaluations});
        for (float q:p.finalCoordinates) values.push_back(std::bit_cast<std::uint32_t>(q));
        for (const auto& c:p.solutions) {
            values.insert(values.end(),{static_cast<unsigned>(c.directStatus),static_cast<unsigned>(c.lastStatus),static_cast<unsigned>(c.outcome),c.queries,c.poses,c.candidates,std::bit_cast<std::uint32_t>(c.error)});
            values.insert(values.end(),{c.object,c.bodyForm,c.otherBodyForm,std::bit_cast<std::uint64_t>(c.gap)});
            values.push_back(std::bit_cast<std::uint32_t>(c.angularError));
            for (char v:c.detail) values.push_back(static_cast<unsigned char>(v));
            for (float q:c.coordinates) values.push_back(std::bit_cast<std::uint32_t>(q));
        }
        for (char c:p.budgetReason) values.push_back(static_cast<unsigned char>(c));
        values.insert(values.end(),{static_cast<unsigned>(p.selection),static_cast<unsigned>(p.completion),p.scoreEvaluations,p.tickScoreEvaluations,p.firstClearTick,
            p.ranking.additionalTicks,p.ranking.maxEvaluations});
        for (double v:{p.ranking.elbowWeight,p.ranking.travelWeight,p.ranking.durationWeight}) values.push_back(std::bit_cast<std::uint64_t>(v));
        values.push_back(p.elbowDirectionBody.has_value());
        if (p.elbowDirectionBody) for (int k=0;k<3;++k) values.push_back(std::bit_cast<std::uint32_t>((*p.elbowDirectionBody)[k]));
        const auto score=[&](const RouteScore& v) {
            values.push_back(static_cast<unsigned>(v.elbow.status));
            for (double d:{v.elbow.penalty,v.travel,v.duration,v.total}) values.push_back(std::bit_cast<std::uint64_t>(d));
            values.push_back(static_cast<std::uint64_t>(v.key));
            for (auto point:{v.elbow.shoulder,v.elbow.elbow,v.elbow.wrist})
                for (int k=0;k<3;++k) values.push_back(std::bit_cast<std::uint32_t>(point[k]));
        };
        score(p.score);
        for (unsigned i=0;i<p.scoreEvaluations;++i) {
            const auto& e=p.evaluations[i]; values.insert(values.end(),{e.configuration,e.candidate,e.segments}); score(e.score);
        }
        return values;
    }
    struct RouteRecord
    {
        std::vector<std::uint64_t> accounting;
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
        RouteRecord result{}; result.accounting=accounting(route); result.state=route.state; result.command=route.command; result.count=route.segmentCount;
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
        std::array<std::vector<double>,4> stages;
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
            const auto stage=previous.stage;
            if (planning) stages[stage==PlanningState::GeneratingIK ? 0:stage==PlanningState::CheckingDirect ? 1:stage==PlanningState::CheckingFirst ? 2:3].push_back(elapsed);
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
            JULIA_CHECK(current.tickIterations<=observation.target.detour.iterationsPerTick);
            if (observation.target.task==ReachTask::PositionAndOrientation)
                JULIA_CHECK(current.tickIterations<=4 && current.tickIkEvaluations<=128);
            JULIA_CHECK(current.tickSeeds<=observation.target.detour.seedsPerTick);
            JULIA_CHECK(current.tickCandidates<=1);
            JULIA_CHECK(current.tickScoreEvaluations<=1 && current.scoreEvaluations<=observation.target.ranking.maxEvaluations);
            if (previous.command==current.command && previous.ticks==current.ticks)
                JULIA_CHECK(current.tickIterations==0 && current.tickSeeds==0 && current.tickQueries==0 && current.tickPoses==0 && current.tickCandidates==0 && current.tickScoreEvaluations==0);
            JULIA_CHECK(current.configurations<=observation.target.detour.maxConfigurations && current.configurations<=4);
            JULIA_CHECK(current.ikIterations<=observation.target.detour.maxIkIterations && current.seedAttempts<=observation.target.detour.maxSeedAttempts);
            JULIA_CHECK(current.queries<=observation.target.detour.maxTotalQueries && current.poses<=observation.target.detour.maxTotalPoses);
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
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const Fixture& f,std::ostream& report,
        const std::vector<BodyCommand>& followups={})
    {
        struct State
        {
            std::vector<glm::vec3> q; std::uint64_t palette;
            ReachState reach; ReachReason reason; PlanningState planning; PathStatus direct;
            std::array<PathStatus,2> segmentStatuses;
            unsigned candidates,queries,poses,segment,selected; ArmCoordinates waypoint;
            RouteRecord route; std::vector<std::uint64_t> accounting;
            std::vector<BodyCommandResult> results;
            ReachTask task; glm::quat goalOrientation,actualOrientation; float positionError,angularError;
        };
        const auto schedule=[&](BodyInstance& body)
        {
            for (const auto& command:std::vector<BodyCommand>{{1,1,1,0,SetDirectMode{true}},
                {2,1,1,0,SetCollisionScene{f.blocked,{}}},{3,1,2,0,StartReach{f.target}},
                {4,1,3,0,SetCoordinate{JointId::Head,0,.1f}},{5,1,3,0,SetCollisionScene{scene(2),{}}},
                {6,1,1800,0,ResetCoordinates{}}})
                JULIA_CHECK(body.submit(command).status==BodyCommandStatus::Queued);
            for (const auto& command:followups) JULIA_CHECK(body.submit(command).status==BodyCommandStatus::Queued);
        };
        constexpr unsigned total=2400;
        BodyInstance exact(1,asset,&profile,false); schedule(exact); std::vector<State> history; history.reserve(total);
        for (unsigned t=0;t<total;++t)
        {
            exact.step(); const auto& s=exact.snapshot(); const auto& p=s.reaches[0].planning;
            history.push_back({s.coordinates,hash(s),s.reaches[0].state,s.reaches[0].reason,p.state,p.directCheck.status,
                {p.segmentChecks[0].status,p.segmentChecks[1].status},p.candidates,p.queries,p.poses,p.currentSegment,p.selectedCandidate,p.waypoint,record(s.reaches[0].route),accounting(p),exact.results(),
                s.reaches[0].target.task,s.reaches[0].goalOrientationBody,s.reaches[0].actualOrientationBody,s.reaches[0].error,s.reaches[0].angularError});
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
                    JULIA_CHECK(s.reaches[0].target.task==expected.task && s.reaches[0].goalOrientationBody==expected.goalOrientation &&
                        s.reaches[0].actualOrientationBody==expected.actualOrientation && s.reaches[0].error==expected.positionError && s.reaches[0].angularError==expected.angularError);
                    JULIA_CHECK(accounting(p)==expected.accounting);
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
