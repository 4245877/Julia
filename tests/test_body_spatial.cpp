#include "body_reach_test_support.hpp"
#include <limits>
using namespace body_reach_test;
namespace
{
    ReachTarget targetFrom(const BodyRigBinding& binding,std::size_t side,unsigned effector,ArmCoordinates q)
    {
        DirectBodyControl source(binding); source.setEnabled(true); set(source,side,q);
        ReachTarget target; target.arm=side==0 ? ArmSide::Left:ArmSide::Right;
        target.effector=static_cast<EffectorId>(side*4+effector); target.task=ReachTask::PositionAndOrientation;
        const auto frame=binding.effectorFrame(target.effector,pose(source).globals);
        target.position=glm::vec3{frame[3]}; target.orientation=normalizedReachOrientation(glm::quat_cast(glm::mat3{frame}));
        return target;
    }
    void success(const BodyInstance& body,std::size_t side)
    {
        const auto& o=body.snapshot().reaches[side];
        if (o.state!=ReachState::Succeeded) std::cerr << "spatial failure side=" << side << " state=" << reachStateName(o.state)
            << " reason=" << reachReasonName(o.reason) << " p=" << o.error << " r=" << o.angularError << '\n';
        JULIA_CHECK(o.state==ReachState::Succeeded && o.error<=o.target.tolerance && o.angularError<=o.target.angularTolerance);
    }
    void math()
    {
        const glm::quat identity{1,0,0,0};
        for (float angle:{0.f,1e-7f,.01f,3.1415925f,3.14159265f}) {
            const auto q=glm::angleAxis(angle,glm::normalize(glm::vec3{1,2,-3}));
            const auto error=orientationResidual(q,identity);
            JULIA_CHECK(std::abs(glm::length(error)-angle)<3e-7);
            JULIA_CHECK(glm::length(error-orientationResidual(-q,identity))<1e-12);
            JULIA_CHECK(glm::length(error-orientationResidual(q,-identity))<1e-12);
            JULIA_CHECK(glm::length(orientationResidual(q,normalizedReachOrientation(q*3.7f)))<2e-7);
        }
        JULIA_CHECK(orientationResidual({0,1,0,0},identity)==orientationResidual({0,-1,0,0},identity));
    }
    void affine(std::ostream& out)
    {
        model::ModelAsset asset;
        auto residual=glm::scale(glm::mat4{1},glm::vec3{-2,3,4}); residual[1][0]=.7f;
        asset.addNode({"root",animation::NoNode,residual});
        std::array<glm::mat4,4> globals;
        for (unsigned k=0;k<4;++k) globals[k]=glm::translate(glm::mat4{1},glm::vec3{0,1-static_cast<float>(k)*.25f,0})*residual;
        asset.addNode({"shoulder",0,glm::inverse(residual)*globals[0]});
        asset.addNode({"elbow",1,glm::inverse(globals[0])*globals[1]});
        asset.addNode({"wrist",2,glm::inverse(globals[1])*globals[2]});
        asset.addNode({"tip",3,glm::inverse(globals[2])*globals[3]});
        BodyProfile profile; profile.name="spatial affine"; profile.rigFingerprint=bodyRigFingerprint(asset);
        const std::array<std::string,4> names{"shoulder","elbow","wrist","tip"};
        for (std::size_t j=0;j<3;++j) {
            std::vector<JointCoordinate> coordinates;
            const unsigned count=j==0 ? 3:j==1 ? 1:2;
            for (unsigned k=0;k<count;++k) { glm::vec3 axis{0}; axis[k]=1; coordinates.push_back({"q"+std::to_string(k),"positive",axis,-1,1.5f}); }
            const auto frame=glm::translate(glm::mat4{1},glm::vec3{globals[j][3]});
            profile.joints.push_back({joints(0)[j],j==0 ? SegmentId::LeftUpperArm:j==1 ? SegmentId::LeftForearm:SegmentId::LeftHand,names[j],
                {names[j],j==0 ? "root":names[j-1]},frame,coordinates,{names[j+1],names[j]}});
        }
        profile.effectors.push_back({EffectorId::LeftWrist,"wrist",{"wrist","elbow"},glm::translate(glm::mat4{1},glm::vec3{globals[2][3]})});
        BodyRigBinding binding(profile,asset); auto t=targetFrom(binding,0,0,{.3f,.2f,.1f,.5f,.2f,-.1f});
        DirectBodyControl control(binding); control.setEnabled(true); const auto solution=solveArmPosition(control,t,t.position);
        JULIA_CHECK(solution.reason==ReachReason::Reached && solution.error<t.tolerance && solution.angularError<t.angularTolerance);
        ArmCoordinates q{}; for (std::size_t k=0;k<6;++k) q[k]=static_cast<float>(solution.coordinates[k]); set(control,0,q);
        const auto frame=binding.effectorFrame(t.effector,pose(control).globals);
        JULIA_CHECK(std::abs(glm::determinant(glm::mat3{frame})-1)<1e-5f);
        out << "affine_reflection_shear_nonuniform=PASS\n";
    }
    void reachable(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        BodyRigBinding binding(profile,*asset); float maxP=0,maxR=0; unsigned count=0;
        for (std::size_t side=0;side<2;++side) for (unsigned effector=0;effector<3;++effector) for (unsigned sample=0;sample<3;++sample) {
            auto target=targetFrom(binding,side,effector,sample==0 ? ArmCoordinates{.35f,.25f,-.1f,.5f,.15f,-.1f}:
                sample==1 ? ArmCoordinates{.8f,.6f,.1f,1.2f,-.2f,.16f}:ArmCoordinates{.15f,.1f,.15f,.3f,.25f,.1f});
            BodyInstance body(1,asset,&profile,false); body.enqueue(SetDirectMode{true}); body.step();
            // A distinct non-bind initial configuration, not just a copied solution.
            if (sample==2) { body.enqueue(SetCoordinate{joints(side)[0],0,.22f}); body.enqueue(SetCoordinate{joints(side)[1],0,.15f}); body.step(); }
            if (sample==1) {
                RootPose root{{1,.3f,-2},glm::angleAxis(.7f,glm::normalize(glm::vec3{.2f,1,.1f}))};
                body.enqueue(SetRoot{root}); body.step();
                target.space=TargetSpace::World; target.position=glm::vec3{matrix(root)*glm::vec4{target.position,1}};
                target.orientation=root.rotation*target.orientation;
            }
            Metrics metrics; body.enqueue(StartReach{target}); metrics.step(body,side); finish(body,side,&metrics); success(body,side);
            const auto& o=body.snapshot().reaches[side]; maxP=std::max(maxP,o.error); maxR=std::max(maxR,o.angularError); ++count;
            for (std::size_t j=0;j<profile.joints.size();++j) if (profile.joints[j].id!=joints(side)[0] && profile.joints[j].id!=joints(side)[1] && profile.joints[j].id!=joints(side)[2])
                JULIA_CHECK(body.snapshot().coordinates[j]==glm::vec3{0});
        }
        out << "reachable_cases=" << count << "\nmax_position_m=" << maxP << "\nmax_rotation_rad=" << maxR << '\n';
        // Wrist position is unchanged by its two rotations. Demonstrate three such targets.
        glm::vec3 fixed{0}; glm::quat previous{1,0,0,0};
        for (unsigned k=0;k<3;++k) {
            const auto target=targetFrom(binding,0,0,{.4f,.3f,.1f,.7f,-.3f+static_cast<float>(k)*.3f,-.12f+static_cast<float>(k)*.12f});
            if (k) { JULIA_CHECK(glm::length(target.position-fixed)<1e-6f); JULIA_CHECK(glm::length(orientationResidual(target.orientation,previous))>.25); }
            fixed=target.position; previous=target.orientation;
            BodyInstance body(1,asset,&profile,false); body.enqueue(SetDirectMode{true}); body.enqueue(StartReach{target}); body.step(); finish(body); success(body,0);
        }
        // q and -q take precisely the same production path.
        auto t=targetFrom(binding,0,0,{.4f,.3f,.1f,.7f,.2f,.1f});
        BodyInstance a(1,asset,&profile,false),b(1,asset,&profile,false);
        a.enqueue(SetDirectMode{true}); b.enqueue(SetDirectMode{true}); a.enqueue(StartReach{t}); t.orientation=-t.orientation; b.enqueue(StartReach{t});
        for (unsigned k=0;k<500;++k) { a.step(); b.step(); JULIA_CHECK(a.snapshot().coordinates==b.snapshot().coordinates && hash(a.snapshot())==hash(b.snapshot())); }
        success(a,0); success(b,0);
        out << "same_wrist_position_orientations=3\nantipodal_history=identical_500_ticks\n";
        for (std::size_t side=0;side<2;++side) for (bool upper:{false,true}) {
            const auto& wrist=profile.joints[binding.jointIndex(joints(side)[2])];
            const auto target=targetFrom(binding,side,0,{.4f,.3f,.1f,.7f,upper ? wrist.coordinates[0].upper:wrist.coordinates[0].lower,
                upper ? wrist.coordinates[1].upper:wrist.coordinates[1].lower});
            DirectBodyControl start(binding); start.setEnabled(true); set(start,side,{.4f,.3f,.1f,.7f,0,0});
            const auto solution=solveArmPosition(start,target,target.position);
            JULIA_CHECK(solution.reason==ReachReason::Reached && solution.error<=target.tolerance && solution.angularError<=target.angularTolerance);
        }
        out << "wrist_joint_boundary_targets=4\n";
    }
    void invalidAndUnreachable(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        BodyRigBinding binding(profile,*asset); const auto good=targetFrom(binding,0,0,{.35f,.2f,0,.5f,.1f,0});
        DirectBodyControl direct(binding); direct.setEnabled(true); ArmReach reach(direct);
        bool rejectedRoot=false;
        try { reach.start(1,0,good,glm::scale(glm::mat4{1},glm::vec3{2})); }
        catch (const std::invalid_argument&) { rejectedRoot=true; }
        JULIA_CHECK(rejectedRoot && reach.observations()[0].state==ReachState::Idle);
        BodyInstance body(1,asset,&profile,false); body.enqueue(SetDirectMode{true}); body.step(); const auto initial=body.snapshot().coordinates;
        for (unsigned k=0;k<12;++k) {
            auto bad=good;
            switch(k) {
            case 0: bad.orientation={0,0,0,0}; break;
            case 1: bad.orientation.x=std::numeric_limits<float>::quiet_NaN(); break;
            case 2: bad.orientation.w=std::numeric_limits<float>::infinity(); break;
            case 3: bad.orientation={1e-8f,0,0,0}; break;
            case 4: bad.angularTolerance=0; break;
            case 5: bad.angularTolerance=std::numeric_limits<float>::quiet_NaN(); break;
            case 6: bad.task=static_cast<ReachTask>(77); break;
            case 7: bad.space=static_cast<TargetSpace>(77); break;
            case 8: bad.effector=EffectorId::LeftSole; break;
            case 9: bad.position.y=std::numeric_limits<float>::infinity(); break;
            case 10: bad.orientation={1e8f,0,0,0}; break;
            case 11: bad.angularTolerance=std::numeric_limits<float>::infinity(); break;
            }
            body.enqueue(StartReach{bad}); body.step(); JULIA_CHECK(body.results().back().status==BodyCommandStatus::Rejected && body.snapshot().coordinates==initial);
        }
        auto t=targetFrom(binding,0,0,{}); t.orientation=glm::angleAxis(3.14159265f,glm::vec3{0,1,0})*t.orientation;
        body.enqueue(StartReach{t}); body.step(); finish(body);
        JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Failed);
        JULIA_CHECK(body.snapshot().reaches[0].error<t.tolerance && body.snapshot().reaches[0].angularError>t.angularTolerance);
        JULIA_CHECK(body.snapshot().reaches[0].reason==ReachReason::OrientationNotReached);
        out << "invalid_commands=12\nposition_reachable_orientation_failed=no_false_success\n";
        for (std::size_t side=0;side<2;++side) {
            auto extended=targetFrom(binding,side,0,{0,0,0,-.08726646f,0,0});
            BodyInstance singular(1,asset,&profile,false); singular.enqueue(SetDirectMode{true}); singular.enqueue(StartReach{extended}); singular.step(); finish(singular,side);
            JULIA_CHECK(!active(singular.snapshot().reaches[side].state));
            for (const auto& q:singular.snapshot().coordinates) JULIA_CHECK(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z));
        }
    }
    Fixture spatialFixture(const BodyRigBinding& binding)
    {
        Fixture f; f.target=targetFrom(binding,0,0,{.8f,.6f,.1f,1.2f,.15f,-.1f}); f.home=targetFrom(binding,0,0,{});
        DirectBodyControl source(binding); source.setEnabled(true);
        const auto solved=solveArmPosition(source,f.target,f.target.position); JULIA_CHECK(solved.reason==ReachReason::Reached);
        ArmCoordinates middle{}; for (std::size_t k=0;k<6;++k) { f.solved[k]=static_cast<float>(solved.coordinates[k]); middle[k]=f.solved[k]*.5f; }
        set(source,0,middle); const auto forms=bodyColliders(binding,pose(source).globals,glm::mat4{1});
        world::SceneObject obstacle; obstacle.id=601; obstacle.position=(forms[1].capsule.a+forms[1].capsule.b)*.5; obstacle.shape=world::Box{{.001,.008,.008}};
        f.empty=scene(1); f.blocked=scene(1,{obstacle});
        f.target.strategy=f.home.strategy=MovementStrategy::AllowDetour; return f;
    }
    void supportedJobs(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        BodyRigBinding binding(profile,*asset); unsigned cases=0;
        const RootPose root{{1,.3f,-2},glm::angleAxis(.7f,glm::normalize(glm::vec3{.2f,1,.1f}))};
        for (std::size_t side=0;side<2;++side) for (unsigned effector=0;effector<3;++effector) for (bool world:{false,true}) {
            auto target=targetFrom(binding,side,effector,{.4f,.3f,.1f,.7f,.2f,-.1f});
            target.strategy=MovementStrategy::AllowDetour; target.detour.maxConfigurations=4;
            target.detour.maxCandidates=0;
            if (world) {
                target.space=TargetSpace::World; target.position=glm::vec3{matrix(root)*glm::vec4{target.position,1}};
                target.orientation=root.rotation*target.orientation;
                target.selection=RouteSelection::Ranked; target.elbowPreference=ElbowPreference{{1,0,0},TargetSpace::World};
            }
            BodyInstance body(1,asset,&profile,false); prepare(body,scene(1),root); Metrics metrics;
            body.enqueue(StartReach{target}); metrics.step(body,side); finish(body,side,&metrics); success(body,side); ++cases;
            JULIA_CHECK(body.snapshot().reaches[side].route.segmentCount==1);
            out << "job=" << side << ',' << effector << ',' << world << " solutions=" << body.snapshot().reaches[side].planning.configurations << '\n';
        }
        out << "supported_jobs=" << cases << '\n';
    }
    void integration(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& out)
    {
        BodyRigBinding binding(profile,*asset); auto f=spatialFixture(binding);
        const auto memoryBefore=memory();
        for (auto geometry:{f.empty,f.blocked}) {
            BodyInstance body(1,asset,&profile,false); prepare(body,geometry); Metrics metrics;
            body.enqueue(StartReach{f.target}); metrics.step(body); finish(body,0,&metrics); success(body,0);
            const auto& o=body.snapshot().reaches[0];
            JULIA_CHECK(o.route.segmentCount==(geometry==f.empty ? 1u:2u));
            for (unsigned k=0;k<o.planning.configurations;++k) {
                const auto& c=o.planning.solutions[k];
                // Independent frozen control below; do not depend on solver's residual.
                DirectBodyControl check(binding); check.setEnabled(true); set(check,0,c.coordinates);
                const auto frame=binding.effectorFrame(o.target.effector,pose(check).globals);
                JULIA_CHECK(glm::length(glm::vec3{frame[3]}-f.target.position)<=f.target.tolerance);
                JULIA_CHECK(glm::length(orientationResidual(f.target.orientation,glm::quat_cast(glm::mat3{frame})))<=f.target.angularTolerance);
            }
            out << "route=" << o.route.segmentCount << " candidate=" << o.route.selectedCandidate << " p=" << o.error << " r=" << o.angularError
                << " seeds=" << o.planning.seedAttempts << " iterations=" << o.planning.ikIterations << " evaluations=" << o.planning.ikEvaluations
                << " fk=" << o.planning.fkChecks << " poses=" << o.planning.poses << " queries=" << o.planning.queries
                << " ik_us=" << o.planning.ikMicroseconds << " segment_us=" << o.planning.segmentMicroseconds << " path_us=" << o.planning.pathMicroseconds
                << " max_v=" << metrics.maxSpeed << " max_a=" << metrics.maxAcceleration << " scratch=" << metrics.scratchBytes << '\n';
            stats(out,geometry==f.empty ? "direct_tick_us":"waypoint_tick_us",metrics.ticks);
            stats(out,geometry==f.empty ? "direct_planning_us":"waypoint_planning_us",metrics.planningTicks);
            for (unsigned stage=0;stage<4;++stage) stats(out,("stage"+std::to_string(stage)+"_us").c_str(),metrics.stages[stage]);
        }
        // Full endpoint inside hand form: no legal route can bypass a blocked endpoint.
        DirectBodyControl final(binding); final.setEnabled(true); set(final,0,f.solved);
        const auto hand=bodyColliders(binding,pose(final).globals,glm::mat4{1})[2].capsule;
        world::SceneObject blocker; blocker.id=602; blocker.position=hand.b; blocker.shape=world::Sphere{.004};
        BodyInstance blocked(1,asset,&profile,false); prepare(blocked,scene(1,{blocker})); blocked.enqueue(StartReach{f.target}); blocked.step(); finish(blocked);
        JULIA_CHECK(blocked.snapshot().reaches[0].state==ReachState::Failed && blocked.snapshot().reaches[0].route.segmentCount==0);
        bool handHit=false; for (const auto& c:blocked.snapshot().reaches[0].planning.solutions) handHit|=c.bodyForm==3 && c.object==602;
        JULIA_CHECK(handHit); out << "hand_endpoint_blocked=PASS\n";
        // Clear endpoints with a wrist-only sweep through an obstacle BETWEEN ticks.
        ArmCoordinates a{.8f,.6f,.1f,1.2f,-.4f,0},b=a,middle=a; b[4]=.5f; middle[4]=.05f;
        DirectBodyControl sweeping(binding); sweeping.setEnabled(true); set(sweeping,0,middle);
        const auto wristFrame=binding.jointFrame(JointId::LeftWrist,pose(sweeping).globals);
        world::SceneObject between; between.id=603; between.position=glm::dvec3{wristFrame*glm::vec4{0,.14f,0,1}}; between.shape=world::Sphere{.004};
        const auto sweepScene=scene(1,{between}); const std::array<ArmCoordinates,1> oneA{a},oneB{b};
        const std::array<ArmCoordinates,2> jump{a,b};
        const auto start=checkArmPath(sweeping,0,oneA,glm::mat4{1},*sweepScene,{}),end=checkArmPath(sweeping,0,oneB,glm::mat4{1},*sweepScene,{});
        const auto sweep=checkArmPath(sweeping,0,jump,glm::mat4{1},*sweepScene,{});
        JULIA_CHECK(start.status==PathStatus::Clear && end.status==PathStatus::Clear);
        JULIA_CHECK(sweep.status==PathStatus::Blocked && sweep.object==603 && sweep.bodyForm==3 && sweep.fromTick<1);
        out << "wrist_only_between_tick_hand_blocked=" << pathStatusName(sweep.status) << " form=" << sweep.bodyForm << '\n';
        // One-iteration budget cannot admit a weighted compromise.
        auto tiny=f.target; tiny.detour.maxIkIterations=1;
        BodyInstance limited(1,asset,&profile,false); prepare(limited,f.empty); limited.enqueue(StartReach{tiny}); limited.step(); finish(limited);
        JULIA_CHECK(limited.snapshot().reaches[0].planning.state==PlanningState::BudgetExhausted && limited.snapshot().reaches[0].planning.configurations==0);
        // Both modes cross the command boundary without zeroing current coordinates.
        for (bool startReplacement:{false,true}) {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.blocked); body.enqueue(StartReach{f.target}); body.step(); waitRoute(body);
            const auto old=body.snapshot().reaches[0].route; const auto action=body.snapshot().reaches[0].action;
            auto failure=f.home; failure.task=ReachTask::PositionOnly; failure.detour.maxIkIterations=1;
            if (startReplacement) body.enqueue(StartReach{failure}); else body.enqueue(RetargetReach{action,failure}); body.step();
            JULIA_CHECK(body.snapshot().reaches[0].planning.state==PlanningState::WaitingForStop);
            for (unsigned k=0;k<16000 && body.snapshot().reaches[0].planning.state!=PlanningState::BudgetExhausted;++k) body.step();
            JULIA_CHECK(record(body.snapshot().reaches[0].route)==record(old)); finish(body); success(body,0);
            JULIA_CHECK(body.snapshot().reaches[0].action==action && body.snapshot().reaches[0].target.task==ReachTask::PositionAndOrientation);
        }
        for (unsigned segment:{0u,1u}) {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.blocked); body.enqueue(StartReach{f.target}); body.step(); waitRoute(body,segment);
            body.enqueue(CancelReach{ArmSide::Left,body.snapshot().reaches[0].action}); body.step(); finish(body);
            JULIA_CHECK(body.snapshot().reaches[0].state==ReachState::Cancelled && body.snapshot().reaches[0].route.segmentCount==segment+1);
        }
        for (unsigned op=0;op<4;++op) {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); body.enqueue(StartReach{f.target}); body.step();
            if (op==0) body.enqueue(ResetBody{});
            if (op==1) body.enqueue(ResetCoordinates{});
            if (op==2) body.enqueue(SetRoot{{{1,0,0},{1,0,0,0}}});
            if (op==3) body.enqueue(SetDirectMode{false});
            body.step(); JULIA_CHECK(!active(body.snapshot().reaches[0].state) && body.snapshot().reaches[0].route.segmentCount==0);
            if (op==0) JULIA_CHECK(body.snapshot().epoch==1 && body.submit({100,1,body.snapshot().tick+1,0,StartReach{f.target}}).status==BodyCommandStatus::Rejected);
        }
        for (bool beginFull:{false,true}) {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); Metrics metrics;
            auto startTarget=f.target; if (!beginFull) startTarget.task=ReachTask::PositionOnly;
            const auto action=body.enqueue(StartReach{startTarget}).id; metrics.step(body);
            while (body.snapshot().reaches[0].state==ReachState::Planning) metrics.step(body);
            for (unsigned k=0;k<20;++k) metrics.step(body);
            auto next=f.target; if (beginFull) { next=f.home; next.task=ReachTask::PositionOnly; }
            const auto before=body.snapshot().coordinates; body.enqueue(RetargetReach{action,next}); metrics.step(body);
            JULIA_CHECK(body.results().back().status==BodyCommandStatus::Applied && body.snapshot().reaches[0].target.task==next.task);
            JULIA_CHECK(body.snapshot().coordinates[binding.jointIndex(JointId::LeftShoulder)]!=glm::vec3{0});
            JULIA_CHECK(glm::length(body.snapshot().coordinates[binding.jointIndex(JointId::LeftShoulder)]-before[binding.jointIndex(JointId::LeftShoulder)])<.02f);
            finish(body,0,&metrics);
            const auto& o=body.snapshot().reaches[0]; JULIA_CHECK(o.state==ReachState::Succeeded && o.action==action);
            if (!beginFull) success(body,0);
            else JULIA_CHECK(std::abs(body.control()->coordinate(JointId::LeftWrist,0)-f.solved[4])<.002f);
        }
        {
            BodyInstance body(1,asset,&profile,false); prepare(body,f.empty); body.enqueue(StartReach{f.target}); body.step();
            const auto action=body.snapshot().reaches[0].action; const auto held=body.snapshot().coordinates;
            auto right=targetFrom(binding,1,0,{.4f,.3f,.1f,.7f,.2f,0}); right.strategy=MovementStrategy::AllowDetour;
            body.enqueue(StartReach{right}); body.enqueue(SetCoordinate{JointId::LeftWrist,0,.1f}); body.enqueue(SetCollisionScene{scene(2),{}}); body.step();
            for (const auto& result:body.results()) JULIA_CHECK(result.status==BodyCommandStatus::Rejected);
            body.enqueue(CancelReach{ArmSide::Left,action}); body.step(); finish(body);
            JULIA_CHECK(body.snapshot().coordinates==held && body.snapshot().reaches[0].state==ReachState::Cancelled);
        }
        // Ranked operates only on the same hard-admitted endpoints and Clear routes.
        auto ranked=f; ranked.blocked=f.empty; ranked.target.selection=RouteSelection::Ranked;
        ranked.target.detour.maxConfigurations=4; ranked.target.elbowPreference=ElbowPreference{{1,0,0},TargetSpace::Body};
        BodyInstance chosen(1,asset,&profile,false); prepare(chosen,f.empty); chosen.enqueue(StartReach{ranked.target}); chosen.step(); finish(chosen); success(chosen,0);
        JULIA_CHECK(chosen.snapshot().reaches[0].route.scoreEvaluations>0 && chosen.snapshot().reaches[0].planning.configurations<=4);
        out << "ranked_configurations=" << chosen.snapshot().reaches[0].planning.configurations << " scores=" << chosen.snapshot().reaches[0].route.scoreEvaluations << '\n';
        replay(asset,profile,f,out); replay(asset,profile,ranked,out);
        auto position=f.target; position.task=ReachTask::PositionOnly;
        replay(asset,profile,ranked,out,{{7,1,40,0,RetargetReach{3,position}},{8,1,350,0,RetargetReach{3,f.target}},
            {9,1,650,0,CancelReach{ArmSide::Left,3}},{10,1,1000,0,StartReach{ranked.target}}});
        out << "transactions_cancel_restore_reset_epochs_replay=PASS\n";
        out << "sizeof_reach_observation=" << sizeof(ReachObservation) << " sizeof_body_command=" << sizeof(BodyCommand)
            << " private_bytes_before=" << memoryBefore << " private_bytes_after=" << memory() << '\n';
    }
}
int main(int argc,char** argv)
{
    try {
        const std::filesystem::path output=argc>1 ? argv[1]:"build/diagnostics/stage6e/cpu"; std::filesystem::create_directories(output);
        std::ofstream out(output/"spatial.txt"); out << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile(); math(); affine(out); reachable(asset,profile,out); invalidAndUnreachable(asset,profile,out);
        supportedJobs(asset,profile,out); integration(asset,profile,out);
        out << "result=" << (JULIA_TEST_RESULT() ? "FAIL":"PASS") << '\n'; std::cout << "spatial failures=" << JULIA_TEST_RESULT() << '\n'; return JULIA_TEST_RESULT();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
