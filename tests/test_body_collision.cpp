#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "pose_test_support.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <iostream>
#include <bit>
#ifdef _WIN32
#include <Windows.h>
#include <psapi.h>
#endif
using namespace julia;
using namespace julia::embodiment;
namespace
{
    std::array<JointId,3> joints{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist};
    void set(DirectBodyControl& c,const ArmCoordinates& q)
    { for (std::size_t k=0;k<6;++k) c.setCoordinate(joints[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4,q[k]); }
    animation::Pose pose(const DirectBodyControl& c)
    { animation::Pose p(c.binding().asset().skeleton()); c.apply(p.locals); animation::computeGlobalTransforms(c.binding().asset().skeleton(),p.locals,p.globals); return p; }
    ReachTarget target(const DirectBodyControl& c)
    { ReachTarget t; t.position=glm::vec3{c.binding().effectorFrame(EffectorId::LeftWrist,pose(c).globals)[3]}; return t; }
    std::shared_ptr<const world::CollisionScene> scene(std::uint64_t version,std::vector<world::SceneObject> objects={})
    { return std::make_shared<const world::CollisionScene>(version,std::move(objects)); }
    bool moving(const BodyInstance& b)
    { const auto s=b.snapshot().reaches[0].state; return s==ReachState::Moving || s==ReachState::Settling; }
    void finish(BodyInstance& b) { for (int t=0;t<2000 && moving(b);++t) b.step(); JULIA_CHECK(!moving(b)); }
    void stats(std::ostream& out,const char* name,std::vector<double> values)
    {
        std::sort(values.begin(),values.end());
        out << name << "_count=" << values.size() << "\n" << name << "_mean=" << std::accumulate(values.begin(),values.end(),0.0)/static_cast<double>(values.size())
            << "\n" << name << "_p95=" << values[static_cast<std::size_t>(.95*static_cast<double>(values.size()-1))] << "\n" << name << "_max=" << values.back() << '\n';
    }
    std::size_t memory()
    {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS_EX m{}; m.cb=sizeof(m);
        GetProcessMemoryInfo(GetCurrentProcess(),reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&m),sizeof(m)); return m.PrivateUsage;
#else
        return 0;
#endif
    }
    void geometry()
    {
        world::Capsule c{{-1,0,0},{1,0,0},.1};
        world::SceneObject b; b.id=1; b.shape=world::Box{{.001,.1,.1}};
        JULIA_CHECK(world::clearance(c,b).gap<0);
        b.position={0,.3,0}; JULIA_CHECK(std::abs(world::clearance(c,b).gap-.1)<1e-10);
        b.rotation=glm::angleAxis(.7,glm::dvec3{0,0,1}); JULIA_CHECK(std::isfinite(world::clearance(c,b).gap));
        b.position={0,.2,0}; b.rotation={1,0,0,0}; b.shape=world::Sphere{.1};
        JULIA_CHECK(std::abs(world::clearance(c,b).gap)<1e-12);
        b.position.y=.202; JULIA_CHECK(std::abs(world::clearance(c,b).gap-.002)<1e-12);
        b.position.y=.203; JULIA_CHECK(world::clearance(c,b).gap>.002);
        b.position={0,-.1,0}; b.shape=world::Plane{}; JULIA_CHECK(std::abs(world::clearance(c,b).gap)<1e-12);
        JULIA_CHECK(!world::interacts({1,2},{4,1})); JULIA_CHECK(world::interacts({1,2},{2,1}));
        JULIA_CHECK(checksSelfPair(1,3)); JULIA_CHECK(!checksSelfPair(1,2)); JULIA_CHECK(checksSelfPair(2,7)); JULIA_CHECK(checksSelfPair(3,6));
    }
    std::uint64_t hash(const BodySnapshot& s)
    {
        std::uint64_t h=1469598103934665603ull;
        for (const auto& m:s.pose.palette) for (int c=0;c<4;++c) for (int r=0;r<4;++r) { h^=std::bit_cast<std::uint32_t>(m[c][r]); h*=1099511628211ull; }
        return h;
    }
    void resumeParity(DirectBodyControl& control,std::span<const ArmCoordinates> path,
        const world::CollisionScene& collisionScene,const CollisionSettings& settings)
    {
        const auto expected=checkArmPath(control,0,path,glm::mat4{1},collisionScene,settings);
        for (const auto budget:std::array{std::array{1u,1u},std::array{1u,17u},std::array{2u,31u},std::array{11u,257u}})
        {
            auto source=std::vector<ArmCoordinates>(path.begin(),path.end());
            auto sceneCopy=std::make_unique<world::CollisionScene>(collisionScene.version(),collisionScene.objects());
            ArmPathCheckJob job(control,0,source,glm::mat4{1},*sceneCopy,settings);
            // Inputs are snapshots. Later source edits and scene destruction do not change the job.
            source.clear(); sceneCopy.reset();
            JULIA_CHECK(!job.done() && job.result().status==PathStatus::NotChecked);
            JULIA_CHECK(!job.advance(0,1) && !job.advance(1,0));
            unsigned calls=0;
            while (!job.done())
            {
                const auto poses=job.result().poses,queries=job.result().queries;
                const bool complete=job.advance(budget[0],budget[1]);
                JULIA_CHECK(job.result().poses-poses<=budget[0] && job.result().queries-queries<=budget[1]);
                JULIA_CHECK(complete==job.done());
                if (!complete) JULIA_CHECK(job.result().status==PathStatus::NotChecked);
                JULIA_CHECK(++calls<300000);
            }
            const auto& actual=job.result();
            JULIA_CHECK(actual.status==expected.status && actual.sceneVersion==expected.sceneVersion && actual.detail==expected.detail);
            JULIA_CHECK(actual.poses==expected.poses && actual.queries==expected.queries);
            JULIA_CHECK(actual.fromTick==expected.fromTick && actual.toTick==expected.toTick);
            JULIA_CHECK(actual.bodyForm==expected.bodyForm && actual.otherBodyForm==expected.otherBodyForm && actual.object==expected.object);
            JULIA_CHECK(actual.contact.gap==expected.contact.gap && actual.contact.bodyPoint==expected.contact.bodyPoint &&
                actual.contact.otherPoint==expected.contact.otherPoint && actual.contact.normal==expected.contact.normal && actual.trace==expected.trace);
        }
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const ReachTarget& goal,
        const ReachTarget& home,const std::vector<world::SceneObject>& obstacles,std::ostream& report)
    {
        struct State { std::vector<glm::vec3> q; std::uint64_t hash; ReachState state; PathStatus status; unsigned queries; std::vector<BodyCommandResult> results; };
        const auto prepare=[&](BodyInstance& b) {
            std::vector<BodyCommand> commands{
                {1,1,1,0,SetDirectMode{true}},{2,1,1,0,SetCollisionScene{scene(1,{obstacles[0]}),{}}},
                {3,1,2,0,StartReach{goal}},{4,1,3,0,SetCollisionScene{scene(2),{}}},{5,1,4,0,StartReach{goal}},
                {6,1,10,0,SetCoordinate{JointId::Head,0,.1f}},{7,1,10,0,SetCollisionScene{scene(3),{}}},
                {8,1,60,0,RetargetReach{5,home}},{9,1,90,0,CancelReach{ArmSide::Left,5}},
                {10,1,500,0,SetRoot{{{1,.2f,-.4f},glm::angleAxis(.5f,glm::vec3{0,1,0})}}}};
            for (const auto& cmd:commands) JULIA_CHECK(b.submit(cmd).status==BodyCommandStatus::Queued);
        };
        BodyInstance exact(1,asset,&profile,false); prepare(exact); std::vector<State> history;
        for (int t=0;t<1200;++t) { exact.step(); const auto& s=exact.snapshot(); history.push_back({s.coordinates,hash(s),s.reaches[0].state,s.pathCheck.status,s.pathCheck.queries,exact.results()}); }
        for (int fps:{30,60,144}) {
            BodyInstance b(1,asset,&profile,false); prepare(b); runtime::FixedStepScheduler scheduler;
            double elapsed=0;
            while (elapsed<10-1e-12) {
                const double frame=std::min(10-elapsed,1.0/fps);
                scheduler.advance(frame,[&] {
                    b.step(); const auto& s=b.snapshot(); const auto& h=history[static_cast<std::size_t>(s.tick-1)];
                    JULIA_CHECK(s.coordinates==h.q && hash(s)==h.hash && s.reaches[0].state==h.state && s.pathCheck.status==h.status && s.pathCheck.queries==h.queries);
                    JULIA_CHECK(b.results().size()==h.results.size());
                    for (std::size_t k=0;k<b.results().size();++k) JULIA_CHECK(b.results()[k].status==h.results[k].status && b.results()[k].reason==h.results[k].reason);
                }); elapsed+=frame;
                const auto tick=b.snapshot().tick; const auto display=b.display(scheduler.alpha()); const auto again=b.display(scheduler.alpha());
                JULIA_CHECK(tick==b.snapshot().tick && display.pose.palette==again.pose.palette);
                if (display.direct && display.collisionScene) {
                    const auto w=glm::translate(glm::mat4{1},display.root.position)*glm::mat4_cast(display.root.rotation);
                    const auto expected=bodyColliders(b.control()->binding(),display.pose.globals,w);
                    JULIA_CHECK(display.colliders.size()==expected.size());
                    for (std::size_t k=0;k<expected.size();++k) JULIA_CHECK(display.colliders[k].capsule.a==expected[k].capsule.a);
                }
            }
            JULIA_CHECK(b.snapshot().tick==1200); report << "replay_fps_" << fps << "=identical_1200_ticks\n";
        }
    }
}
int main(int argc,char** argv)
{
    try {
        const auto folder=std::filesystem::path{argc>1 ? argv[1]:"build/diagnostics/stage6a/cpu"}; std::filesystem::create_directories(folder);
        std::ofstream report(folder/"body_collision.txt"); report << std::setprecision(12);
        geometry();
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile(); BodyRigBinding binding(profile,*asset);
        DirectBodyControl c(binding); c.setEnabled(true);
        const ArmCoordinates end{.8f,.6f,.1f,1.2f,0,0},zero{};
        set(c,end); const auto goal=target(c); set(c,zero);
        const auto solution=solveArmPosition(c,goal,goal.position);
        ArmCoordinates solved{}; for (std::size_t k=0;k<6;++k) solved[k]=static_cast<float>(solution.coordinates[k]);
        const std::array path{zero,solved}; const auto empty=scene(1); CollisionSettings settings;
        auto clear=checkArmPath(c,0,path,glm::mat4{1},*empty,settings);
        report << "free=" << pathStatusName(clear.status) << " gap=" << clear.contact.gap << " forms=" << clear.bodyForm << ',' << clear.otherBodyForm << '\n';
        JULIA_CHECK(clear.status==PathStatus::Clear);
        resumeParity(c,path,*empty,settings);
        std::ofstream calibration(folder/"calibration.csv"); calibration << "id,segment,first,second,from,to,radius,localA_y,localB_y\n";
        for (const auto& f:bodyColliderCalibration()) calibration << f.id << ',' << static_cast<int>(f.segment) << ',' << static_cast<int>(f.first) << ',' << static_cast<int>(f.second) << ',' << f.from << ',' << f.to << ',' << f.radius << ',' << f.localA.y << ',' << f.localB.y << '\n';
        ArmCoordinates half{}; for (std::size_t k=0;k<6;++k) half[k]=solved[k]*.5f;
        set(c,half); const auto middle=bodyColliders(binding,pose(c).globals,glm::mat4{1}); set(c,zero);
        std::vector<world::SceneObject> obstacles;
        for (std::size_t form : {std::size_t{1},std::size_t{2}})
        {
            world::SceneObject o; o.id=100+form; o.position=(middle[form].capsule.a+middle[form].capsule.b)*.5;
            o.shape=world::Box{{.001,.008,.008}}; obstacles.push_back(o);
            const auto blocked=scene(2+form,{o});
            const auto startFree=checkArmPath(c,0,std::array<ArmCoordinates,1>{zero},glm::mat4{1},*blocked,settings);
            const auto endFree=checkArmPath(c,0,std::array<ArmCoordinates,1>{solved},glm::mat4{1},*blocked,settings);
            const auto hit=checkArmPath(c,0,path,glm::mat4{1},*blocked,settings);
            report << "obstacle_form=" << form << " endpoints=" << pathStatusName(startFree.status) << ',' << pathStatusName(endFree.status)
                << " path=" << pathStatusName(hit.status) << " hit=" << hit.bodyForm << " object=" << hit.object << " interval=" << hit.fromTick << ',' << hit.toTick << " gap=" << hit.contact.gap << '\n';
            JULIA_CHECK(startFree.status==PathStatus::Clear); JULIA_CHECK(endFree.status==PathStatus::Clear); JULIA_CHECK(hit.status==PathStatus::Blocked);
            JULIA_CHECK(hit.object==o.id);
            resumeParity(c,path,*blocked,settings);
            if (form==1) { JULIA_CHECK(hit.bodyForm==2); JULIA_CHECK(world::clearance(middle[2].capsule,o).gap>.002); }
            auto filtered=o; filtered.filter.mask=0;
            JULIA_CHECK(checkArmPath(c,0,path,glm::mat4{1},*scene(9,{filtered}),settings).status==PathStatus::Clear);
        }
        auto forms=bodyColliders(binding,pose(c).globals,glm::mat4{1});
        ArmCoordinates quarter{}; for (std::size_t k=0;k<6;++k) quarter[k]=solved[k]*.27f;
        set(c,quarter); const auto offsetForms=bodyColliders(binding,pose(c).globals,glm::mat4{1}); set(c,zero);
        world::SceneObject offMid; offMid.id=104; offMid.position=(offsetForms[2].capsule.a+offsetForms[2].capsule.b)*.5; offMid.shape=world::Sphere{.001};
        const auto offMidScene=scene(8,{offMid});
        for (const auto& sample:std::array{zero,half,solved}) JULIA_CHECK(checkArmPath(c,0,std::span{&sample,1},glm::mat4{1},*offMidScene,settings).status==PathStatus::Clear);
        const auto offMidHit=checkArmPath(c,0,path,glm::mat4{1},*offMidScene,settings);
        JULIA_CHECK(offMidHit.status==PathStatus::Blocked); report << "off_midpoint_hit_interval=" << offMidHit.fromTick << ',' << offMidHit.toTick << '\n';
        world::SceneObject initial; initial.id=500; initial.position=forms[2].capsule.a; initial.shape=world::Sphere{.01};
        JULIA_CHECK(checkArmPath(c,0,path,glm::mat4{1},*scene(10,{initial}),settings).status==PathStatus::InitialIntersection);
        resumeParity(c,path,*scene(10,{initial}),settings);
        double lowest=1e9; for (const auto& f:forms) lowest=std::min({lowest,f.capsule.a.y-f.capsule.radius,f.capsule.b.y-f.capsule.radius});
        world::SceneObject floor; floor.id=501; floor.shape=world::Plane{};
        for (double gap : {0.0,.002,.0021,.004}) {
            floor.position.y=lowest-gap;
            const auto hit=checkArmPath(c,0,std::array<ArmCoordinates,1>{zero},glm::mat4{1},*scene(11,{floor}),settings);
            JULIA_CHECK(gap>.00203 ? hit.status==PathStatus::Clear:hit.status!=PathStatus::Clear);
        }
        settings.maxQueries=1;
        resumeParity(c,path,*empty,settings);
        JULIA_CHECK(checkArmPath(c,0,path,glm::mat4{1},*empty,settings).status==PathStatus::Incomplete); settings={};
        settings.maxPoses=1; resumeParity(c,path,*empty,settings); JULIA_CHECK(checkArmPath(c,0,path,glm::mat4{1},*empty,settings).status==PathStatus::Incomplete); settings={};
        settings.maxDepth=0; resumeParity(c,path,*empty,settings); JULIA_CHECK(checkArmPath(c,0,path,glm::mat4{1},*empty,settings).status==PathStatus::Incomplete); settings={};
        const auto root=glm::translate(glm::mat4{1},glm::vec3{1.2f,.3f,-.4f})*glm::rotate(glm::mat4{1},.8f,glm::vec3{0,1,0});
        auto transformed=obstacles[0]; transformed.position=glm::dvec3{root*glm::vec4{glm::vec3{transformed.position},1}}; transformed.rotation=glm::normalize(glm::dquat{glm::quat_cast(root)});
        JULIA_CHECK(checkArmPath(c,0,path,root,*scene(12,{transformed}),settings).status==PathStatus::Blocked);
        bool self=false;
        for (float flex : {-.3f,.3f,.8f,1.1f}) for (float roll : {-.5f,0.f,.5f}) for (float elbow : {.3f,1.f,1.7f}) {
            const ArmCoordinates q{flex,-.17f,roll,elbow,0,0};
            const auto hit=checkArmPath(c,0,std::array<ArmCoordinates,1>{q},glm::mat4{1},*empty,settings);
            if (hit.status==PathStatus::InitialIntersection) { self=true; report << "self=" << flex << ',' << roll << ',' << elbow << " forms=" << hit.bodyForm << ',' << hit.otherBodyForm << '\n'; }
        }
        JULIA_CHECK(self);
        const auto home=target(c);
        replay(asset,profile,goal,home,obstacles,report);
        // A geometric rejection of either replacement operation preserves the old certified action.
        BodyInstance protectedBody(1,asset,&profile,false);
        protectedBody.enqueue(SetDirectMode{true}); protectedBody.enqueue(SetCollisionScene{scene(1,{obstacles[0]}),{}});
        protectedBody.enqueue(StartReach{home}); protectedBody.step();
        const auto protectedAction=protectedBody.snapshot().reaches[0].action;
        for (const auto& op:std::array<BodyOperation,2>{RetargetReach{protectedAction,goal},StartReach{goal}}) {
            protectedBody.enqueue(op); protectedBody.step();
            JULIA_CHECK(protectedBody.results().back().status==BodyCommandStatus::Rejected);
            JULIA_CHECK(protectedBody.snapshot().pathCheck.status==PathStatus::Blocked);
            JULIA_CHECK(protectedBody.snapshot().reaches[0].action==protectedAction && moving(protectedBody));
            JULIA_CHECK(protectedBody.snapshot().reaches[0].target.position==home.position);
        }
        finish(protectedBody); JULIA_CHECK(protectedBody.snapshot().reaches[0].state==ReachState::Succeeded);
        // Exercise real deferred replanning rather than a test-only bypass of the validator.
        bool deferred=false;
        for (int reversalTick : {10,25,40,55,70,85,100,115,130,150,180,210,240}) {
            BodyInstance reversal(1,asset,&profile,false); reversal.enqueue(SetDirectMode{true}); reversal.enqueue(SetCollisionScene{scene(1),{}});
            reversal.enqueue(StartReach{goal}); reversal.step();
            const auto a=reversal.snapshot().reaches[0].action;
            for (int t=1;t<reversalTick;++t) reversal.step();
            ReachTarget stopHere; stopHere.position=reversal.snapshot().reaches[0].actualBody;
            reversal.enqueue(RetargetReach{a,stopHere}); reversal.step();
            if (reversal.snapshot().reaches[0].replanningDeferred) {
                deferred=true; report << "deferred_at=" << reversalTick << '\n';
                finish(reversal); JULIA_CHECK(reversal.snapshot().reaches[0].state==ReachState::Succeeded);
                JULIA_CHECK(reversal.snapshot().reaches[0].collisionChecked);
                JULIA_CHECK(reversal.snapshot().pathCheck.command==0 && reversal.snapshot().pathCheck.status==PathStatus::Clear);
                report << "deferred_rechecked_start_tick=" << reversal.snapshot().pathCheck.segmentStartTick << '\n'; break;
            }
        }
        report << "deferred_exercised=" << deferred << '\n';
        JULIA_CHECK(deferred);
        BodyInstance pendingCancel(1,asset,&profile,false);
        pendingCancel.enqueue(SetDirectMode{true}); pendingCancel.enqueue(SetCollisionScene{scene(1),{}}); pendingCancel.enqueue(StartReach{goal}); pendingCancel.step();
        const auto pendingAction=pendingCancel.snapshot().reaches[0].action;
        for (int t=0;t<20;++t) pendingCancel.step();
        pendingCancel.enqueue(RetargetReach{pendingAction,home}); pendingCancel.step();
        JULIA_CHECK(pendingCancel.snapshot().reaches[0].replanningDeferred);
        const auto deferredCheck=pendingCancel.snapshot().pathCheck;
        JULIA_CHECK(deferredCheck.segmentStartTick>pendingCancel.snapshot().tick);
        pendingCancel.enqueue(CancelReach{ArmSide::Left,pendingAction}); pendingCancel.step();
        JULIA_CHECK(!pendingCancel.snapshot().reaches[0].replanningDeferred); finish(pendingCancel);
        JULIA_CHECK(pendingCancel.snapshot().reaches[0].state==ReachState::Cancelled);
        JULIA_CHECK(glm::length(pendingCancel.snapshot().reaches[0].actualBody-goal.position)<=.002f);
        // Every explicit geometry discontinuity cancels the certificate and collapses display history.
        for (const auto& op:std::array<BodyOperation,3>{SetRoot{{{1,.2f,-.4f},glm::angleAxis(.7f,glm::vec3{0,1,0})}},SetDirectMode{false},ResetCoordinates{}}) {
            BodyInstance interrupted(1,asset,&profile,false); interrupted.enqueue(SetDirectMode{true}); interrupted.enqueue(SetCollisionScene{scene(1),{}});
            interrupted.enqueue(StartReach{goal}); interrupted.step(); interrupted.enqueue(op); interrupted.step();
            JULIA_CHECK(interrupted.snapshot().reaches[0].state==ReachState::Cancelled);
            JULIA_CHECK(interrupted.display(.5).fromTick==interrupted.display(.5).toTick);
        }
        // Initial overlap and incomplete validation must reject at the command boundary too.
        for (bool budget:{false,true}) {
            BodyInstance rejected(1,asset,&profile,false); CollisionSettings tiny; if (budget) tiny.maxQueries=1;
            rejected.enqueue(SetDirectMode{true}); rejected.enqueue(SetCollisionScene{budget ? scene(1):scene(1,{initial}),tiny}); rejected.step();
            const auto oldQ=rejected.snapshot().coordinates; rejected.enqueue(StartReach{goal}); rejected.step();
            JULIA_CHECK(rejected.results().back().status==BodyCommandStatus::Rejected && rejected.snapshot().coordinates==oldQ);
            JULIA_CHECK(rejected.snapshot().pathCheck.status==(budget ? PathStatus::Incomplete:PathStatus::InitialIntersection));
        }
        BodyInstance body(1,asset,&profile,false);
        const auto blockedScene=scene(20,{obstacles[0]});
        body.enqueue(SetDirectMode{true}); body.enqueue(SetCollisionScene{blockedScene,{}}); body.step();
        const auto before=body.snapshot().coordinates;
        body.enqueue(StartReach{goal}); body.step();
        JULIA_CHECK(body.results().back().status==BodyCommandStatus::Rejected); JULIA_CHECK(body.snapshot().coordinates==before);
        JULIA_CHECK(body.snapshot().pathCheck.status==PathStatus::Blocked);
        body.enqueue(SetCollisionScene{scene(21),{}}); body.enqueue(StartReach{goal}); body.step();
        const auto action=body.snapshot().reaches[0].action; JULIA_CHECK(moving(body));
        body.enqueue(SetCollisionScene{scene(22),{}}); body.enqueue(SetCoordinate{JointId::Head,0,.1f});
        ReachTarget right; right.arm=ArmSide::Right; right.effector=EffectorId::RightWrist;
        body.enqueue(StartReach{right}); body.step(); for (const auto& r:body.results()) JULIA_CHECK(r.status==BodyCommandStatus::Rejected);
        ReachTarget impossible=goal; impossible.position={5,5,5};
        body.enqueue(RetargetReach{action,impossible}); body.step();
        JULIA_CHECK(body.results().back().status==BodyCommandStatus::Rejected); JULIA_CHECK(body.snapshot().reaches[0].action==action); JULIA_CHECK(moving(body));
        JULIA_CHECK(body.snapshot().pathCheck.status==PathStatus::Unreachable);
        body.enqueue(CancelReach{ArmSide::Left,action}); body.step(); finish(body);
        JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Cancelled);
        const auto held=body.snapshot().coordinates; body.step(); JULIA_CHECK(body.snapshot().coordinates==held);
        body.enqueue(SetCollisionScene{scene(22),{}}); body.step(); JULIA_CHECK(body.results().back().status==BodyCommandStatus::Applied);
        body.enqueue(SetCollisionScene{scene(22),{}}); body.step(); JULIA_CHECK(body.results().back().status==BodyCommandStatus::Rejected);
        body.enqueue(ResetBody{}); body.step(); JULIA_CHECK(body.snapshot().epoch==1); JULIA_CHECK(body.snapshot().collisionScene->version()==22);
        body.enqueue(SetDirectMode{true}); body.enqueue(StartReach{goal}); body.step(); finish(body);
        JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Succeeded); JULIA_CHECK(body.snapshot().reaches[0].error<=.002f);
        body.enqueue(SetCollisionScene{nullptr,{}}); body.enqueue(StartReach{impossible}); body.step();
        JULIA_CHECK(body.results().back().status==BodyCommandStatus::Applied);
        for (int side=0;side<2;++side) for (int effector=0;effector<3;++effector) {
            DirectBodyControl source(binding); source.setEnabled(true);
            source.setCoordinate(side==0 ? JointId::LeftShoulder:JointId::RightShoulder,0,.5f);
            source.setCoordinate(side==0 ? JointId::LeftShoulder:JointId::RightShoulder,1,.4f);
            source.setCoordinate(side==0 ? JointId::LeftElbow:JointId::RightElbow,0,.8f);
            ReachTarget t; t.arm=side==0 ? ArmSide::Left:ArmSide::Right; t.effector=static_cast<EffectorId>(side*4+effector);
            t.position=glm::vec3{binding.effectorFrame(t.effector,pose(source).globals)[3]};
            BodyInstance supported(1,asset,&profile,false); supported.enqueue(SetDirectMode{true}); supported.enqueue(SetCollisionScene{scene(1),{}}); supported.enqueue(StartReach{t}); supported.step();
            JULIA_CHECK(supported.results().back().status==BodyCommandStatus::Applied);
            for (int n=0;n<1200;++n) { supported.step(); if (supported.snapshot().reaches[static_cast<std::size_t>(side)].state==ReachState::Succeeded) break; }
            const auto& r=supported.snapshot().reaches[static_cast<std::size_t>(side)];
            JULIA_CHECK(r.state==ReachState::Succeeded && r.error<=.002f && r.collisionChecked && r.collisionSceneVersion==1);
        }
        report << "checked_arms_effectors=left,right x wrist,palm,grip\n";
        std::vector<double> checks,ticks,queries,productionChecks,commandTicks,productionQueries; std::size_t scratchBytes=0; const auto memoryBefore=memory();
        for (int n=0;n<30;++n) {
            const auto check=checkArmPath(c,0,path,glm::mat4{1},*empty,{}); JULIA_CHECK(check.status==PathStatus::Clear);
            checks.push_back(check.microseconds); queries.push_back(check.queries); scratchBytes=std::max(scratchBytes,check.scratchBytes);
        }
        BodyInstance bench(1,asset,&profile,false); bench.enqueue(SetDirectMode{true}); bench.enqueue(SetCollisionScene{scene(1),{}}); bench.enqueue(StartReach{goal});
        std::vector<glm::vec3> previousVelocity(profile.joints.size(),glm::vec3{0}); float maxSpeed=0,maxAcceleration=0;
        for (int n=0;n<1200;++n) {
            const auto beforeQ=bench.snapshot().coordinates; const auto start=std::chrono::steady_clock::now(); bench.step();
            ticks.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
            for (std::size_t j=0;j<beforeQ.size();++j) {
                const auto v=(bench.snapshot().coordinates[j]-beforeQ[j])/static_cast<float>(runtime::BodyStepSeconds);
                const auto a=(v-previousVelocity[j])/static_cast<float>(runtime::BodyStepSeconds); previousVelocity[j]=v;
                for (int k=0;k<3;++k) { maxSpeed=std::max(maxSpeed,std::abs(v[k])); maxAcceleration=std::max(maxAcceleration,std::abs(a[k])); }
                for (std::size_t k=0;k<profile.joints[j].coordinates.size();++k) {
                    const auto q=bench.snapshot().coordinates[j][static_cast<int>(k)]; const auto& limit=profile.joints[j].coordinates[k];
                    JULIA_CHECK(q>=limit.lower-1e-6f && q<=limit.upper+1e-6f);
                }
            }
        }
        JULIA_CHECK(maxSpeed<=1.2001f && maxAcceleration<=4.03f);
        report << "checked_motion_max_speed=" << maxSpeed << "\nchecked_motion_max_acceleration=" << maxAcceleration << '\n';
        stats(report,"check_us",checks); stats(report,"tick_us",ticks); stats(report,"queries",queries);
        for (int n=0;n<30;++n) {
            BodyInstance commandBench(1,asset,&profile,false); commandBench.enqueue(SetDirectMode{true});
            commandBench.enqueue(SetCollisionScene{n%2 ? scene(1):scene(1,{obstacles[0]}),{}}); commandBench.enqueue(StartReach{goal});
            const auto start=std::chrono::steady_clock::now(); commandBench.step();
            commandTicks.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
            const auto& check=commandBench.snapshot().pathCheck;
            productionChecks.push_back(check.microseconds); productionQueries.push_back(check.queries); scratchBytes=std::max(scratchBytes,check.scratchBytes);
        }
        stats(report,"production_check_us",productionChecks); stats(report,"command_tick_us",commandTicks); stats(report,"production_queries",productionQueries);
        report << "scratch_accounted_bytes=" << scratchBytes << "\nprivate_before=" << memoryBefore << "\nprivate_after=" << memory() << '\n';
        report << "result=" << (JULIA_TEST_RESULT() ? "FAIL":"PASS") << '\n';
        std::cout << "stage6a report=" << folder.string() << '\n';
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
    return JULIA_TEST_RESULT();
}

