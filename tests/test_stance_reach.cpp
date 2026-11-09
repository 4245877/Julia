#include "body_reach_test_support.hpp"
#include "../apps/common/StanceReachDemo.hpp"
#include <sstream>
using namespace body_reach_test;
namespace
{
    void require(bool ok,const std::string& why) { if (!ok) throw std::runtime_error(why); }
    std::uint64_t apply(BodyInstance& b,BodyOperation op,bool accept=true)
    {
        const auto id=b.enqueue(std::move(op)).id; b.step();
        require(b.results().back().status==(accept ? BodyCommandStatus::Applied:BodyCommandStatus::Rejected),b.results().back().reason);
        return id;
    }
    void supportRest(BodyInstance& b)
    {
        for (unsigned t=0;t<4000;++t) {
            const auto& o=b.snapshot().support;
            require(o.result!="PlanRejected",o.reason);
            if (o.phase==SupportPhase::Holding) return;
            b.step();
        } throw std::runtime_error("Support timeout");
    }
    void initialize(BodyInstance& b)
    {
        apply(b,SetDirectMode{true}); apply(b,EnableSupport{}); apply(b,AlignSupport{}); supportRest(b);
    }
    struct Measures
    {
        double soleP=0,soleA=0,soleGap=0,speed=0,acceleration=0;
        std::vector<double> ticks,displays,snapshots,checks,preparations,planning;
        std::size_t scratch=0,contextBytes=0; unsigned queries=0,poses=0;
        void soles(const SupportObservation& s) {
            require(s.kind==SupportKind::KinematicSupport,"Lost support during arm Reach");
            for (const auto& a:s.anchors) {
                require(a.bound,"Unbound sole");
                soleP=std::max(soleP,static_cast<double>(a.positionError)); soleA=std::max(soleA,static_cast<double>(a.angularError));
                soleGap=std::max(soleGap,static_cast<double>(std::max(std::abs(a.minGap),std::abs(a.maxGap))));
                require(a.positionError<=SupportPositionTolerance && a.angularError<=SupportAngularTolerance &&
                    a.minGap>=-SupportSurfaceTolerance && a.maxGap<=SupportSurfaceTolerance,"Sole tolerance exceeded");
            }
        }
        void record(BodyInstance& b,const BodySnapshot& frozen,unsigned side) {
            const auto& s=b.snapshot(); const auto& prev=b.previousSnapshot();
            require(s.root.position==frozen.root.position && s.root.rotation==frozen.root.rotation,"Arm changed root");
            const auto ids=joints(side); const auto& binding=b.control()->binding();
            for (std::size_t j=0;j<s.coordinates.size();++j) {
                const auto id=binding.profile().joints[j].id;
                if (std::find(ids.begin(),ids.end(),id)==ids.end()) require(s.coordinates[j]==frozen.coordinates[j],"Controller restored or modified frozen q");
                else for (std::size_t k=0;k<binding.profile().joints[j].coordinates.size();++k) {
                    const auto q=s.coordinates[j][static_cast<int>(k)]; const auto& limit=binding.profile().joints[j].coordinates[k];
                    require(q>=limit.lower && q<=limit.upper,"Arm q limit");
                    const double v=(static_cast<double>(q)-prev.coordinates[j][static_cast<int>(k)])/runtime::BodyStepSeconds;
                    const double oldV=(static_cast<double>(prev.coordinates[j][static_cast<int>(k)])-lastQ[j][static_cast<int>(k)])/runtime::BodyStepSeconds;
                    speed=std::max(speed,std::abs(v)); acceleration=std::max(acceleration,std::abs(v-oldV)/runtime::BodyStepSeconds);
                }
            }
            lastQ=prev.coordinates;
            for (unsigned k=0;k<2;++k) {
                require(s.support.anchors[k].desiredWorld==frozen.support.anchors[k].desiredWorld &&
                    s.support.anchors[k].surfaceFromSole==frozen.support.anchors[k].surfaceFromSole,"Anchor recaptured");
            }
            soles(s.support);
            for (double alpha:{0.,.1,.23,.5,.77,.9,1.}) {
                const auto begin=Clock::now(); const auto d=b.display(alpha);
                displays.push_back(std::chrono::duration<double,std::micro>(Clock::now()-begin).count());
                soles(d.support); require(d.root.position==frozen.root.position,"Display root moved");
                require(d.colliders.size()==16,"Display lacks lower-body proxies");
            }
            const auto begin=Clock::now(); BodySnapshot copy=s;
            snapshots.push_back(std::chrono::duration<double,std::micro>(Clock::now()-begin).count());
            require(copy.tick==s.tick,"Snapshot copy");
            const auto& p=s.pathCheck; scratch=std::max(scratch,p.scratchBytes); queries=std::max(queries,p.queries); poses=std::max(poses,p.poses);
            contextBytes=std::max(contextBytes,s.reachGeometry[side].contextBytes);
        }
        std::vector<glm::vec3> lastQ;
    };
    void finishArm(BodyInstance& b,unsigned side,Measures* m=nullptr,const BodySnapshot* frozen=nullptr)
    {
        for (unsigned t=0;t<4000;++t) {
            if (!active(b.snapshot().reaches[side].state)) return;
            const auto begin=Clock::now(); b.step();
            if (m) {
                const auto elapsed=std::chrono::duration<double,std::micro>(Clock::now()-begin).count(); m->ticks.push_back(elapsed);
                if (b.snapshot().reaches[side].planning.tickPoses || b.snapshot().reaches[side].planning.tickIterations) m->planning.push_back(elapsed);
                m->record(b,*frozen,side);
            }
        } throw std::runtime_error("Arm timeout");
    }
    CollisionContext lowerContext(BodyInstance& b)
    {
        CollisionContext c; c.fixedLowerForms=lowerBodyColliders(b.control()->binding(),b.snapshot().pose.globals,matrix(b.snapshot().root),calibrateSoleContacts(b.control()->binding()));
        c.soleContacts={std::pair{14u,SupportFloorObject},std::pair{17u,SupportFloorObject}}; return c;
    }
    void cycles(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& p,std::ostream& out)
    {
        Measures m; std::vector<double> setup;
        for (unsigned side=0;side<2;++side) for (unsigned effector=0;effector<3;++effector) for (unsigned task=0;task<2;++task) for (unsigned shifted=0;shifted<2;++shifted) {
            BodyInstance b(1,asset,&p,false); initialize(b);
            if (shifted) { apply(b,MoveSupportRoot{{.008f,0,0}}); supportRest(b); }
            const auto frozen=b.snapshot(); m.lastQ=frozen.coordinates;
            auto target=app::stanceReachTarget(b,side==0 ? ArmSide::Left:ArmSide::Right,effector);
            target.task=task==0 ? ReachTask::PositionOnly:ReachTask::PositionAndOrientation;
            target.strategy=(effector==0 ? MovementStrategy::DirectOnly:MovementStrategy::AllowDetour);
            const auto begin=Clock::now(); apply(b,StartReach{target});
            setup.push_back(std::chrono::duration<double,std::micro>(Clock::now()-begin).count());
            m.preparations.push_back(b.snapshot().reachGeometry[side].preparationMicroseconds);
            m.record(b,frozen,side); finishArm(b,side,&m,&frozen);
            m.checks.push_back(b.snapshot().pathCheck.microseconds);
            const auto& r=b.snapshot().reaches[side]; require(r.state==ReachState::Succeeded,"Coordinated Reach failed: "+std::string(reachReasonName(r.reason)));
            require(r.error<=target.tolerance && (task==0 || r.angularError<=target.angularTolerance),"Effector tolerance");
            require(b.snapshot().reachGeometry[side].dependenciesMatch && !b.snapshot().reachGeometry[side].executable,"Historical vs executable certificate");
            require(m.speed<=target.maxSpeed && m.acceleration<=target.maxAcceleration,"Measured arm v/a limit");
            const auto old=b.snapshot().reachGeometry[side].certificate;
            apply(b,MoveSupportRoot{{shifted ? 0.f:.008f,0,0}}); supportRest(b);
            require(!b.snapshot().reachGeometry[side].dependenciesMatch,"Old certificate survived root/leg movement");
            apply(b,StartReach{app::stanceReachTarget(b,target.arm,effector,1)}); finishArm(b,side);
            require(b.snapshot().reaches[side].state==ReachState::Succeeded,"New-position Reach failed");
            require(b.snapshot().reachGeometry[side].certificate.frozenGeometry!=old.frozenGeometry,"New stance reused old context");
            apply(b,DisableSupport{}); auto standalone=app::stanceReachTarget(b,target.arm,effector); standalone.strategy=MovementStrategy::DirectOnly;
            apply(b,StartReach{standalone}); finishArm(b,side);
            require(b.snapshot().reaches[side].state==ReachState::Succeeded,"Standalone Reach regression");
        }
        const auto stats=[&](const char* label,std::vector<double> v) {
            std::sort(v.begin(),v.end()); out << label << "_n_mean_p95_max=" << v.size() << ',' << std::accumulate(v.begin(),v.end(),0.)/v.size() << ',' << v[v.size()*95/100] << ',' << v.back() << '\n';
        };
        stats("setup_us",setup); stats("context_preparation_us",m.preparations); stats("tick_us",m.ticks); stats("planning_tick_us",m.planning);
        stats("display_us",m.displays); stats("snapshot_copy_us",m.snapshots); stats("path_total_us",m.checks);
        out << "cycles=24 sole_p_angle_gap=" << m.soleP << ',' << m.soleA << ',' << m.soleGap << " arm_v_a=" << m.speed << ',' << m.acceleration
            << " path_scratch_queries_poses=" << m.scratch << ',' << m.queries << ',' << m.poses << " context_bytes=" << m.contextBytes << '\n';
    }
    void lifecycle(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& p,std::ostream& out)
    {
        BodyInstance b(1,asset,&p,false); apply(b,SetDirectMode{true}); apply(b,EnableSupport{});
        auto target=app::stanceReachTarget(b,ArmSide::Left);
        apply(b,StartReach{target},false); apply(b,AlignSupport{}); apply(b,StartReach{target},false);
        while (b.snapshot().support.phase==SupportPhase::Planning) b.step();
        require(b.snapshot().support.phase==SupportPhase::Aligning,"Expected Aligning fixture"); apply(b,StartReach{target},false); supportRest(b);
        apply(b,MoveSupportRoot{{.008f,0,0}}); apply(b,StartReach{target},false);
        while (b.snapshot().support.phase==SupportPhase::Planning) b.step();
        apply(b,StartReach{target},false); supportRest(b);
        target=app::stanceReachTarget(b,ArmSide::Left); target.strategy=MovementStrategy::DirectOnly;
        const auto action=apply(b,StartReach{target}); const auto frozen=b.snapshot();
        const auto cert=frozen.reachGeometry[0].certificate;
        for (BodyOperation op:std::vector<BodyOperation>{MoveSupportRoot{{0,0,0}},AlignSupport{},DisableSupport{},CancelSupportMotion{},
            SetSupportSurface{{1,9,0,true}},SetCoordinate{JointId::RightElbow,0,.2f},SetCollisionScene{scene(90),{}},
            GraspObject{{1,1},ArmSide::Left},MoveHeldObject{},ReleaseObject{},SetHandClosure{}}) apply(b,op,false);
        auto distant=target; distant.position.x+=10; apply(b,RetargetReach{action,distant},false);
        require(b.snapshot().reaches[0].action==action && active(b.snapshot().reaches[0].state),"Failed replacement lost old route");
        apply(b,EyeCommand{EyeOperation::Gaze,{10,5,0}});
        require(b.snapshot().reachGeometry[0].certificate==cert && b.snapshot().reachGeometry[0].executable,"Eyes invalidated supported geometry");
        apply(b,StartReach{app::stanceReachTarget(b,ArmSide::Right)},false);
        apply(b,CancelReach{ArmSide::Left,action}); require(active(b.snapshot().reaches[0].state),"Cancel braked immediately");
        finishArm(b,0); require(b.snapshot().reaches[0].state==ReachState::Cancelled,"Cancel result");
        require(b.snapshot().support.kind==SupportKind::KinematicSupport,"Cancel removed supports");
        require(b.snapshot().root.position==frozen.root.position,"Conflict moved root");
        unsigned diagnosticIndex=0;
        for (BodyOperation diagnostic:std::vector<BodyOperation>{SetRoot{{{.02f,0,0},{1,0,0,0}}},ResetCoordinates{},SetDirectMode{true},ResetBody{}}) {
            if (!b.snapshot().direct) apply(b,SetDirectMode{true});
            if (!b.snapshot().support.enabled) { apply(b,SetRoot{{}}); apply(b,EnableSupport{}); apply(b,AlignSupport{}); supportRest(b); }
            auto proposal=app::stanceReachTarget(b,ArmSide::Left);
            if (diagnosticIndex%2==0) proposal.strategy=MovementStrategy::DirectOnly;
            apply(b,StartReach{proposal});
            if (diagnosticIndex==3) while (b.snapshot().reaches[0].state==ReachState::Planning) b.step();
            const auto epoch=b.snapshot().epoch; apply(b,diagnostic);
            require(!b.snapshot().support.enabled && !active(b.snapshot().reaches[0].state),"Diagnostic did not cancel dependent actions");
            require(b.previousSnapshot().tick==b.snapshot().tick,"Diagnostic did not collapse display history");
            require(!b.snapshot().reachGeometry[0].executable,"Stale executable certificate");
            if (b.snapshot().epoch!=epoch) require(b.submit({999,1,b.snapshot().tick+1,epoch,StartReach{target}}).status==BodyCommandStatus::Rejected,"Old epoch accepted");
            ++diagnosticIndex;
        }
        BodyInstance viewer(2,asset,nullptr,false);
        apply(viewer,EnableSupport{},false); apply(viewer,StartReach{target},false); viewer.step();
        require(!viewer.snapshot().pose.palette.empty(),"Uncalibrated viewer stopped working");
        BodyInstance replacement(3,asset,&p,false); initialize(replacement);
        auto oldTarget=app::stanceReachTarget(replacement,ArmSide::Left); const auto oldAction=apply(replacement,StartReach{oldTarget});
        while (replacement.snapshot().reaches[0].state==ReachState::Planning) replacement.step();
        const auto oldCertificate=replacement.snapshot().reachGeometry[0].certificate;
        auto failedTarget=app::stanceReachTarget(replacement,ArmSide::Left,2,1); failedTarget.detour.maxTotalQueries=1;
        apply(replacement,RetargetReach{oldAction,failedTarget}); finishArm(replacement,0);
        const auto& restored=replacement.snapshot();
        require(restored.reaches[0].action==oldAction && restored.reaches[0].state==ReachState::Succeeded &&
            restored.reachGeometry[0].certificate==oldCertificate && restored.support.kind==SupportKind::KinematicSupport,"Failed asynchronous Retarget lost route/support");
        // Simulate a dependency mismatch at the public execution gate. No saved
        // segment may execute even if its earlier geometry check was Clear.
        DirectBodyControl controlled(*replacement.control()); ArmReach gate(controlled); bool valid=true;
        gate.contextValid=[&](std::size_t) { return valid; };
        const auto fixed=lowerContext(replacement);
        gate.setPathValidator([&](std::size_t side,std::span<const ArmCoordinates> path,unsigned) {
            return checkArmPath(controlled,side,path,matrix(restored.root),*restored.collisionScene,{},fixed).status;
        },restored.collisionScene->version());
        oldTarget.strategy=MovementStrategy::DirectOnly; gate.start(1,0,oldTarget,matrix(restored.root));
        require(gate.observations()[0].collisionChecked,"Gate fixture lacks admitted Clear path");
        const auto before=std::vector<glm::vec3>(controlled.coordinates().begin(),controlled.coordinates().end());
        valid=false; gate.advance(1);
        require(!gate.anyActive() && gate.observations()[0].reason==ReachReason::GeometryContextChanged &&
            std::equal(before.begin(),before.end(),controlled.coordinates().begin()),"Stale context executed a segment");
        out << "lifecycle=PASS\n";
    }
    void geometry(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& p,std::ostream& out)
    {
        BodyInstance b(1,asset,&p,false); initialize(b); const auto& binding=b.control()->binding();
        const ArmCoordinates home{},goal{.8f,.6f,.1f,1.2f,0,0}; auto context=lowerContext(b);
        for (const auto& f:context.fixedLowerForms) out << "frozen_form=" << f.id << " radius=" << f.capsule.radius << " a="
            << f.capsule.a.x << ',' << f.capsule.a.y << ',' << f.capsule.a.z << " b=" << f.capsule.b.x << ',' << f.capsule.b.y << ',' << f.capsule.b.z << '\n';
        const auto root=matrix(b.snapshot().root);
        DirectBodyControl trial(*b.control()); set(trial,0,goal); const auto endForms=bodyColliders(binding,pose(trial).globals,root);
        std::array<double,7> minimum; minimum.fill(10);
        for (unsigned sample=0;sample<512;++sample) {
            ArmCoordinates q{}; const auto ids=joints(0);
            for (unsigned k=0;k<6;++k) {
                const auto& limit=p.joints[binding.jointIndex(ids[k<3 ? 0:k==3 ? 1:2])].coordinates[k<3 ? k:k==3 ? 0:k-4];
                const float fraction=sample<64 ? static_cast<float>((sample>>k)&1u):static_cast<float>((sample*(2*k+3))%127)/126.f;
                q[k]=glm::mix(limit.lower,limit.upper,fraction);
            }
            set(trial,0,q); const auto actual=bodyColliders(binding,pose(trial).globals,root);
            for (std::size_t k=0;k<context.fixedLowerForms.size();++k) for (unsigned arm=0;arm<3;++arm)
                minimum[k]=std::min(minimum[k],world::clearance(actual[arm].capsule,context.fixedLowerForms[k].capsule).gap);
        }
        out << "actual_lower_min_gap";
        for (std::size_t k=0;k<context.fixedLowerForms.size();++k) out << ' ' << context.fixedLowerForms[k].id << ':' << minimum[k];
        out << '\n';
        for (unsigned id:{11u,12u,13u,14u,15u,16u,17u}) {
            // Synthetic placement isolates each supported lower-form pair, including
            // shanks unreachable by this rig's hand in a fixed stance.
            auto fixture=context; fixture.fixedLowerForms.clear(); auto capsule=endForms[1].capsule;
            capsule.radius=.02; fixture.fixedLowerForms.push_back({id,SegmentId::Pelvis,capsule});
            const auto c=checkArmPath(*b.control(),0,std::span{&goal,1},root,*scene(1),{},fixture);
            require(c.status!=PathStatus::Clear && c.otherBodyForm==id,"Missing active arm/lower pair "+std::to_string(id));
        }
        set(trial,0,{.4f,.3f,.05f,.6f,0,0}); const auto middle=bodyColliders(binding,pose(trial).globals,root)[1].capsule;
        world::SceneObject obstacle{101,(middle.a+middle.b)*.5,{1,0,0,0},{},world::Sphere{.003}};
        const auto blocked=scene(1,{obstacle});
        const auto a=checkArmPath(*b.control(),0,std::span{&home,1},root,*blocked,{},context);
        const auto z=checkArmPath(*b.control(),0,std::span{&goal,1},root,*blocked,{},context);
        const std::array path{home,goal}; const auto sweep=checkArmPath(*b.control(),0,path,root,*blocked,{},context);
        require(a.status==PathStatus::Clear && z.status==PathStatus::Clear && sweep.status==PathStatus::Blocked,"Unverified intertick obstacle");
        auto incomplete=CollisionSettings{}; incomplete.maxQueries=1;
        require(checkArmPath(*b.control(),0,path,root,*scene(1),incomplete,context).status==PathStatus::Incomplete,"Incomplete weakened");
        // The sole-specific permission cannot exempt the hand from the same plane.
        const auto forms=bodyColliders(binding,b.snapshot().pose.globals,root);
        world::SceneObject floor{SupportFloorObject,{0,forms[2].capsule.a.y,0},{1,0,0,0},{},world::Plane{}};
        CollisionContext sole; sole.fixedLowerForms={{14,SegmentId::LeftFoot,{{0,floor.position.y,0},{0,floor.position.y,0},.04}}};
        sole.soleContacts={std::pair{14u,SupportFloorObject},std::pair{17u,SupportFloorObject}};
        const auto floorCheck=checkArmPath(*b.control(),0,std::span{&home,1},root,*scene(1,{floor}),{},sole);
        require(floorCheck.status!=PathStatus::Clear && floorCheck.bodyForm==3 && floorCheck.object==SupportFloorObject,"Sole contact masked arm/floor");
        CollisionSettings masked; masked.bodyFilter.mask=0;
        require(checkArmPath(*b.control(),0,std::span{&home,1},root,*scene(1,{floor}),masked,sole).status!=PathStatus::Clear,"Filter masked canonical support floor");
        floor.position.y=0;
        require(checkArmPath(*b.control(),0,std::span{&home,1},root,*scene(1,{floor}),{},context).status==PathStatus::Clear,"Sole/floor permission rejected valid stance");
        BodyInstance alias(1,asset,&p,false); apply(alias,SetDirectMode{true});
        floor.position.y=0; floor.id=35; apply(alias,SetCollisionScene{scene(2,{floor}),{}}); apply(alias,EnableSupport{});
        require(alias.snapshot().collisionScene->objects().size()==1 && alias.snapshot().collisionScene->objects()[0].id==SupportFloorObject,"Duplicate floor");
        apply(alias,SetSupportSurface{{1,2,.02f,true}},false);
        require(alias.snapshot().support.surface.height==0,"Rejected floor change mutated surface");
        apply(alias,DisableSupport{}); apply(alias,SetSupportSurface{{1,2,.02f,true}}); apply(alias,EnableSupport{},false);
        require(!alias.snapshot().support.enabled,"Conflicting floor accepted");
        BodyInstance capacity(4,asset,&p,false); apply(capacity,SetDirectMode{true});
        std::vector<world::SceneObject> many;
        for (unsigned k=0;k<63;++k) many.push_back({200+k,{10+k*.1,2,0},{1,0,0,0},{},world::Sphere{.01}});
        apply(capacity,SetCollisionScene{scene(2,many),{}}); apply(capacity,CreateInteractionSphere{800,{{20,3,0},{1,0,0,0}}});
        apply(capacity,EnableSupport{},false); require(!capacity.snapshot().support.enabled,"Scene capacity failure mutated support");
        apply(capacity,DeleteInteractionObject{capacity.snapshot().object->handle}); apply(capacity,EnableSupport{});
        apply(capacity,CreateInteractionSphere{800,{{20,3,0},{1,0,0,0}}},false);
        require(!capacity.snapshot().object,"Scene capacity failure created an object");
        out << "geometry_pairs=7 synthetic_lower_contacts swept_obstacle sole_permission floor_alias incomplete=PASS\n";
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& p,std::ostream& out)
    {
        const auto run=[&](int fps) {
            BodyInstance b(1,asset,&p,false); apply(b,SetDirectMode{true}); app::StanceReachDemo demo; demo.start(b);
            std::vector<std::string> history; history.reserve(12000);
            const auto tick=[&] { demo.drive(b); b.step(); std::ostringstream line; app::stanceReachTrace(line,b.snapshot());
                for (const auto& r:b.results()) { require(r.status!=BodyCommandStatus::Rejected,r.reason); line << r.id << ':' << static_cast<int>(r.status) << ';'; }
                const auto& r=b.snapshot().reaches[0];
                require(r.maxMeasuredSpeed<=r.target.maxSpeed && r.maxMeasuredAcceleration<=r.target.maxAcceleration,"Demo Retarget/Cancel v/a");
                if (active(r.state)) require(b.snapshot().root.position==b.previousSnapshot().root.position,"Demo root changed during active arm");
                if (r.state==ReachState::Moving || r.state==ReachState::Settling)
                    require(b.snapshot().reachGeometry[0].executable,"Retarget/Cancel hid current segment certificate");
                line << hash(b.snapshot()); history.push_back(line.str()); };
            if (!fps) for (unsigned t=0;t<12000;++t) tick();
            else { runtime::FixedStepScheduler scheduler;
                for (int f=0;f<100*fps;++f) { scheduler.advance(1./fps,tick); (void)b.display(scheduler.alpha()); }
            }
            require(demo.done(),"Demo incomplete stage="+std::to_string(demo.stage)); return history;
        };
        const auto reference=run(0);
        for (int fps:{30,60,144}) { require(run(fps)==reference,"FPS history mismatch"); out << "replay_" << fps << "=12000_identical_ticks\n"; }
    }
}
int main(int argc,char** argv)
{
    try {
        const auto output=argc>1 ? std::filesystem::path{argv[1]}:std::filesystem::path{"build/diagnostics/stage6h/cpu"};
        std::filesystem::create_directories(output); std::ofstream out(output/"stance-reach.txt"); out << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx"); const auto p=raphtaliaBodyProfile();
        const auto before=memory(); cycles(asset,p,out); lifecycle(asset,p,out); geometry(asset,p,out); replay(asset,p,out);
        out << "private_usage_before_after=" << before << ',' << memory() << " sizeof_snapshot=" << sizeof(BodySnapshot) << '\n';
        out << "failures=" << JULIA_TEST_RESULT() << '\n'; return JULIA_TEST_RESULT();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
