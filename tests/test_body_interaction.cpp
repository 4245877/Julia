#include "body_reach_test_support.hpp"
#include "../apps/common/InteractionDemo.hpp"
#include <sstream>

using namespace body_reach_test;
namespace
{
    constexpr std::uint64_t objectId=(1ull<<63)|1;
    bool terminal(InteractionPhase p)
    { return p==InteractionPhase::Succeeded || p==InteractionPhase::Failed || p==InteractionPhase::Cancelled || p==InteractionPhase::ReleaseBlocked; }
    void applied(const BodyInstance& body)
    { for (const auto& r:body.results()) JULIA_CHECK(r.status==BodyCommandStatus::Applied); }
    void command(BodyInstance& body,BodyOperation operation)
    { JULIA_CHECK(body.enqueue(std::move(operation)).status==BodyCommandStatus::Queued); body.step(); applied(body); }
    void rejected(BodyInstance& body,BodyOperation operation)
    { body.enqueue(std::move(operation)); body.step(); JULIA_CHECK(body.results().size()==1 && body.results()[0].status==BodyCommandStatus::Rejected); }
    void until(BodyInstance& body,const std::function<bool(const BodySnapshot&)>& predicate)
    {
        for (unsigned t=0;t<12000 && !predicate(body.snapshot());++t) body.step();
        JULIA_CHECK(predicate(body.snapshot()));
    }
    void grasp(BodyInstance& body,ArmSide arm=ArmSide::Left,unsigned sample=0,const RootPose& root={})
    {
        body.enqueue(SetDirectMode{true}); body.enqueue(SetRoot{root});
        body.enqueue(CreateInteractionSphere{1,app::interactionDemoPose(body.control()->binding(),arm,sample,root)});
        body.step(); applied(body);
        command(body,GraspObject{body.snapshot().object->handle,arm});
    }
    void hold(BodyInstance& body)
    {
        until(body,[](const auto& s) { return s.interaction.phase==InteractionPhase::Holding || terminal(s.interaction.phase); });
        JULIA_CHECK(body.snapshot().interaction.phase==InteractionPhase::Holding && body.snapshot().object->state==ObjectState::Attached);
    }
    struct Invariants
    {
        std::vector<glm::vec3> bodyVelocity;
        std::array<float,2> closureVelocity{},blendVelocity{};
        std::vector<glm::dvec3> fingerVelocity;
        float maxFingerSpeed=0,maxFingerAcceleration=0;
        float maxSpeed=0,maxAcceleration=0,maxClosureSpeed=0,maxClosureAcceleration=0;
        double maxSlip=0,maxDisplaySlip=0,maxJump=0;
        std::vector<double> tickTimes,planningTimes;
        std::size_t scratch=0;
        void step(BodyInstance& body)
        {
            const auto old=body.snapshot(); const auto start=Clock::now(); body.step();
            const auto elapsed=std::chrono::duration<double,std::micro>(Clock::now()-start).count(); tickTimes.push_back(elapsed);
            const auto& s=body.snapshot(); applied(body);
            const auto side=s.interaction.arm==ArmSide::Left ? 0u:1u;
            const auto& r=s.reaches[side];
            if (old.reaches[side].state==ReachState::Planning) planningTimes.push_back(elapsed);
            scratch=std::max(scratch,r.planning.scratchBytes);
            JULIA_CHECK(r.planning.tickIterations<=4 && r.planning.tickIkEvaluations<=128);
            JULIA_CHECK(r.planning.tickQueries<=2048 && r.planning.tickPoses<=24 && r.planning.tickCandidates<=1);
            if (bodyVelocity.empty()) bodyVelocity.resize(s.coordinates.size(),glm::vec3{0});
            for (std::size_t j=0;j<s.coordinates.size();++j) {
                const auto v=(s.coordinates[j]-old.coordinates[j])*120.f;
                const auto a=(v-bodyVelocity[j])*120.f; bodyVelocity[j]=v;
                for (int k=0;k<3;++k) { maxSpeed=std::max(maxSpeed,std::abs(v[k])); maxAcceleration=std::max(maxAcceleration,std::abs(a[k])); }
                const auto& definition=body.control()->binding().profile().joints[j];
                for (std::size_t k=0;k<definition.coordinates.size();++k) JULIA_CHECK(s.coordinates[j][static_cast<int>(k)]>=definition.coordinates[k].lower && s.coordinates[j][static_cast<int>(k)]<=definition.coordinates[k].upper);
            }
            for (std::size_t k=0;k<2;++k) {
                const float cv=(s.hands[k].closure-old.hands[k].closure)*120.f,bv=(s.hands[k].blend-old.hands[k].blend)*120.f;
                maxClosureSpeed=std::max({maxClosureSpeed,std::abs(cv),std::abs(bv)});
                maxClosureAcceleration=std::max({maxClosureAcceleration,std::abs(cv-closureVelocity[k])*120.f,std::abs(bv-blendVelocity[k])*120.f});
                closureVelocity[k]=cv; blendVelocity[k]=bv;
                JULIA_CHECK(s.hands[k].closure>=0 && s.hands[k].closure<=1 && s.hands[k].blend>=0 && s.hands[k].blend<=1);
            }
            if (fingerVelocity.empty()) fingerVelocity.resize(body.asset().nodes().size(),glm::dvec3{0});
            for (std::size_t n=0;n<body.asset().nodes().size();++n) if (body.asset().nodes()[n].name.starts_with("c_")) {
                const auto& name=body.asset().nodes()[n].name;
                if (name.find("index")==std::string::npos && name.find("middle")==std::string::npos && name.find("ring")==std::string::npos && name.find("pinky")==std::string::npos && name.find("thumb")==std::string::npos) continue;
                const auto qa=normalizedReachOrientation(glm::quat_cast(glm::mat3{old.pose.locals[n]}));
                const auto qb=normalizedReachOrientation(glm::quat_cast(glm::mat3{s.pose.locals[n]}));
                const auto v=orientationResidual(qb,qa)*120.;
                maxFingerSpeed=std::max(maxFingerSpeed,static_cast<float>(glm::length(v)));
                maxFingerAcceleration=std::max(maxFingerAcceleration,static_cast<float>(glm::length((v-fingerVelocity[n])*120.)));
                fingerVelocity[n]=v;
            }
            JULIA_CHECK(maxFingerSpeed<12.f && maxFingerAcceleration<120.f);
            JULIA_CHECK(s.interaction.phaseCheckPoses<=2 && s.interaction.phaseCheckQueries<=2048);
            JULIA_CHECK(maxSpeed<=1.2001f && maxAcceleration<=4.03f);
            JULIA_CHECK(maxClosureSpeed<=1.876f && maxClosureAcceleration<=5.80f);
            JULIA_CHECK(!s.interaction.gripForce.has_value());
            if (!s.object) return;
            const auto& o=*s.object;
            const auto count=s.collisionScene ? std::count_if(s.collisionScene->objects().begin(),s.collisionScene->objects().end(),[](const auto& c){return c.id==objectId;}):0;
            JULIA_CHECK(count==(o.state==ObjectState::Free ? 1:0));
            if (old.object && old.object->state!=o.state) {
                maxJump=std::max(maxJump,static_cast<double>(glm::length(o.worldPose.position-old.object->worldPose.position)));
                JULIA_CHECK(glm::length(o.worldPose.position-old.object->worldPose.position)<1e-6f);
                JULIA_CHECK(glm::length(orientationResidual(o.worldPose.rotation,old.object->worldPose.rotation))<1e-5);
                if (o.state==ObjectState::Attached) {
                    JULIA_CHECK(s.interaction.frameError<=.002f && s.interaction.angularError<=glm::radians(1.f));
                    JULIA_CHECK(s.hands[side].closure==1 && r.state==ReachState::Succeeded);
                }
            }
            if (old.object && old.object->state==ObjectState::Free && o.state==ObjectState::Free)
                JULIA_CHECK(old.object->worldPose.position==o.worldPose.position && old.object->worldPose.rotation==o.worldPose.rotation);
            if (o.state==ObjectState::Attached) {
                const auto f=matrix(s.root)*body.control()->binding().effectorFrame(side==0 ? EffectorId::LeftGrip:EffectorId::RightGrip,s.pose.globals);
                maxSlip=std::max(maxSlip,static_cast<double>(glm::length(glm::vec3{(f*o.effectorFromObject)[3]}-o.worldPose.position)));
                JULIA_CHECK(maxSlip<1e-6);
                if (old.object && old.object->state==ObjectState::Attached) JULIA_CHECK(old.object->effectorFromObject==o.effectorFromObject);
                for (double alpha:{0.,.23,.5,1.}) {
                    const auto d=body.display(alpha); const auto again=body.display(alpha);
                    JULIA_CHECK(d.pose.palette==again.pose.palette);
                    if (d.object && d.object->state==ObjectState::Attached) {
                        const auto fd=matrix(d.root)*body.control()->binding().effectorFrame(side==0 ? EffectorId::LeftGrip:EffectorId::RightGrip,d.pose.globals);
                        maxDisplaySlip=std::max(maxDisplaySlip,static_cast<double>(glm::length(glm::vec3{(fd*d.object->effectorFromObject)[3]}-d.object->worldPose.position)));
                        JULIA_CHECK(maxDisplaySlip<1e-6);
                    }
                }
            }
        }
    };
    void profileChecks(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const std::filesystem::path& output,std::ostream& report)
    {
        BodyRigBinding binding(profile,*asset); HandLayer hand(binding); JULIA_CHECK(hand.joints().size()==30);
        std::ofstream audit(output/"hand-rig.csv"); audit << std::setprecision(10);
        audit << "node,parent,side,open_w,open_x,open_y,open_z,closed_w,closed_x,closed_y,closed_z,bind_matrix_column_major\n";
        const auto original=asset->nodes();
        for (const auto& j:hand.joints()) {
            audit << asset->nodes()[j.node].name << ',' << asset->nodes()[asset->nodes()[j.node].parent].name << ',' << j.side;
            for (auto q:{j.open,j.closed}) audit << ',' << q.w << ',' << q.x << ',' << q.y << ',' << q.z;
            for (int c=0;c<4;++c) for (int r=0;r<4;++r) audit << ',' << asset->nodes()[j.node].localTransform[c][r]; audit << '\n';
        }
        animation::Pose source(asset->skeleton()),changed(asset->skeleton()); animation::setBindPose(asset->skeleton(),source.locals);
        changed.locals=source.locals; hand.apply(changed.locals); JULIA_CHECK(changed.locals==source.locals);
        std::ofstream channels(output/"finger-channels.csv");
        for (const auto& clip:asset->animations()) {
            unsigned count=0,animated=0;
            for (const auto& j:hand.joints()) for (const auto& c:clip.channels) if (c.nodeIndex==j.node) {
                ++count; bool varies=false;
                if (!c.rotations.empty()) for (const auto& key:c.rotations) varies=varies || glm::length(orientationResidual(key.value,c.rotations.front().value))>1e-5;
                if (varies) ++animated;
                channels << clip.name << ',' << asset->nodes()[j.node].name << ',' << c.rotations.size() << ',' << varies << '\n';
            }
            report << "clip=" << clip.name << " finger_channels=" << count << " varying_rotations=" << animated << '\n';
            animation::sampleAnimation(asset->skeleton(),clip,.37,source.locals); changed.locals=source.locals;
            hand.apply(changed.locals); JULIA_CHECK(changed.locals==source.locals);
        }
        hand.set(ArmSide::Left,0); hand.set(ArmSide::Right,0);
        for (unsigned t=0;t<122;++t) hand.advance();
        changed.locals=source.locals; hand.apply(changed.locals);
        for (std::size_t n=0;n<source.locals.size();++n) {
            const bool finger=std::any_of(hand.joints().begin(),hand.joints().end(),[&](const auto& j) { return j.node==n; });
            if (!finger) JULIA_CHECK(changed.locals[n]==source.locals[n]);
        }
        for (std::size_t n=0;n<asset->nodes().size();++n) JULIA_CHECK(asset->nodes()[n].localTransform==original[n].localTransform);
        BodyInstance a(1,asset,&profile,false),b(2,asset,&profile,false);
        command(a,SetDirectMode{true}); command(b,SetDirectMode{true}); command(a,SetHandClosure{ArmSide::Left,1});
        for (unsigned t=0;t<130;++t) a.step(); JULIA_CHECK(!b.snapshot().hands[0].enabled);
        command(a,SetDirectMode{false}); command(b,SetDirectMode{false});
        command(a,ClipCommand{ClipOperation::Play}); command(b,ClipCommand{ClipOperation::Play});
        JULIA_CHECK(a.snapshot().pose.locals==b.snapshot().pose.locals);
        report << "hand_profile_exact_bind_instance_isolation_clip_priority=PASS\n";
    }
    void cycles(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const std::filesystem::path& output,std::ostream& report)
    {
        Invariants all; BodyRigBinding metricBinding(profile,*asset); HandLayer metricHand(metricBinding);
        const auto memoryBefore=memory();
        for (unsigned side=0;side<2;++side) for (unsigned sample=0;sample<3;++sample) {
            BodyInstance body(1,asset,&profile,false); app::InteractionDemo demo; demo.arm=side==0 ? ArmSide::Left:ArmSide::Right; demo.sample=sample;
            if (sample) demo.root={{1.2f,.3f,-.4f},glm::normalize(glm::angleAxis(.8f,glm::vec3{0,1,0})*glm::angleAxis(.2f,glm::vec3{1,0,0}))};
            demo.start(body); Invariants checks; std::ofstream trace(output/(std::to_string(side)+"_"+std::to_string(sample)+".csv"));
            auto previous=InteractionPhase::Idle;
            for (unsigned t=0;t<8000;++t) {
                demo.drive(body); checks.step(body); const auto& s=body.snapshot(); app::interactionTrace(trace,s);
                if (previous!=s.interaction.phase) { report << "cycle=" << side << ',' << sample << " tick=" << s.tick << " phase=" << interactionPhaseName(s.interaction.phase) << " reason=" << s.interaction.reason << '\n'; report.flush(); previous=s.interaction.phase; }
                if (terminal(s.interaction.phase)) break;
            }
            JULIA_CHECK(body.snapshot().interaction.result=="Released" && body.snapshot().object->state==ObjectState::Free);
            report << "cycle_errors=" << side << ',' << sample << " speed=" << checks.maxSpeed << " acceleration=" << checks.maxAcceleration
                << " finger_speed=" << checks.maxFingerSpeed << " finger_acceleration=" << checks.maxFingerAcceleration
                << " closure_speed=" << checks.maxClosureSpeed << " closure_acceleration=" << checks.maxClosureAcceleration << " jump=" << checks.maxJump << " slip=" << checks.maxSlip << " display_slip=" << checks.maxDisplaySlip << '\n';
            all.tickTimes.insert(all.tickTimes.end(),checks.tickTimes.begin(),checks.tickTimes.end());
            all.planningTimes.insert(all.planningTimes.end(),checks.planningTimes.begin(),checks.planningTimes.end()); all.scratch=std::max(all.scratch,checks.scratch);
        }
        stats(report,"tick_us",all.tickTimes); stats(report,"planning_tick_us",all.planningTimes);
        report << "planner_scratch_max=" << all.scratch << " hand_layer_bytes=" << sizeof(HandLayer)+metricHand.joints().capacity()*sizeof(HandLayer::Joint)
            << " hand_profile_capacity=" << metricHand.joints().capacity() << " object_bytes=" << sizeof(InteractiveObject) << " snapshot_bytes=" << sizeof(BodySnapshot)
            << " private_before=" << memoryBefore << " private_after=" << memory() << '\n';
        // Nonidentity local frame and object orientation; target is derived through that frame.
        BodyInstance body(1,asset,&profile,false); command(body,SetDirectMode{true});
        const RigidPose local{{.002f,-.001f,.003f},glm::angleAxis(.07f,glm::vec3{0,1,0})};
        const auto goal=app::interactionDemoPose(body.control()->binding(),ArmSide::Left);
        command(body,CreateInteractionSphere{1,rigidPose(rigidMatrix(goal)*glm::inverse(rigidMatrix(local))),.022f,local});
        command(body,GraspObject{body.snapshot().object->handle}); hold(body);
        const auto relative=body.snapshot().object->effectorFromObject;
        const auto next=app::interactionDemoPose(body.control()->binding(),ArmSide::Left,1);
        const auto target=rigidPose(rigidMatrix(next)*relative);
        command(body,MoveHeldObject{body.snapshot().object->handle,target}); hold(body);
        JULIA_CHECK(glm::length(body.snapshot().object->worldPose.position-target.position)<.002f);
        JULIA_CHECK(glm::length(orientationResidual(target.rotation,body.snapshot().object->worldPose.rotation))<glm::radians(1.0));
        command(body,ReleaseObject{body.snapshot().object->handle}); until(body,[](const auto& s) { return terminal(s.interaction.phase); });
        JULIA_CHECK(body.snapshot().interaction.result=="Released"); report << "local_grip_transform=PASS\n";
    }
    void negatives(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        for (bool blocked:{false,true}) {
            BodyInstance body(1,asset,&profile,false); command(body,SetDirectMode{true});
            auto p=app::interactionDemoPose(body.control()->binding(),ArmSide::Left);
            if (blocked) {
                world::SceneObject box; box.id=91; box.shape=world::Sphere{.010};
                box.position=rigidPose(rigidMatrix(p)*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f})).position;
                command(body,SetCollisionScene{scene(1,{box}),{}});
            } else p.position={4,2,3};
            command(body,CreateInteractionSphere{1,p}); command(body,GraspObject{body.snapshot().object->handle});
            until(body,[](const auto& s) { return terminal(s.interaction.phase); });
            JULIA_CHECK(body.snapshot().interaction.phase==InteractionPhase::Failed && body.snapshot().object->state==ObjectState::Free);
        }
        BodyInstance body(1,asset,&profile,false); command(body,SetDirectMode{true});
        command(body,CreateInteractionSphere{1,app::interactionDemoPose(body.control()->binding(),ArmSide::Left)});
        const auto original=body.snapshot().object->worldPose; const auto old=body.snapshot().object->handle;
        command(body,SetHandClosure{ArmSide::Left,1}); for (unsigned t=0;t<130;++t) body.step();
        JULIA_CHECK(body.snapshot().hands[0].closure==1 && body.snapshot().object->state==ObjectState::Free && body.snapshot().interaction.hold==HoldType::None);
        command(body,DeleteInteractionObject{old}); command(body,CreateInteractionSphere{1,original});
        JULIA_CHECK(body.snapshot().object->handle.generation!=old.generation); rejected(body,GraspObject{old});
        rejected(body,MoveHeldObject{body.snapshot().object->handle,original}); rejected(body,ReleaseObject{body.snapshot().object->handle});
        command(body,ResetBody{}); JULIA_CHECK(body.snapshot().object->worldPose.position==original.position && body.snapshot().epoch==1 && body.snapshot().interaction.phase==InteractionPhase::Idle && body.snapshot().interaction.epoch==1);
        JULIA_CHECK(body.submit({999,1,body.snapshot().tick+1,0,GraspObject{body.snapshot().object->handle}}).status==BodyCommandStatus::Rejected);
        command(body,SetDirectMode{true}); command(body,GraspObject{body.snapshot().object->handle});
        rejected(body,GraspObject{body.snapshot().object->handle,ArmSide::Right}); hold(body);
        const auto handle=body.snapshot().object->handle; const auto attached=body.snapshot().object->effectorFromObject;
        for (auto op:std::vector<BodyOperation>{ResetBody{},ResetCoordinates{},SetRoot{{{1,0,0},{1,0,0,0}}},SetDirectMode{false},
            DeleteInteractionObject{handle},SetCollisionScene{scene(100),{}},SetCoordinate{JointId::Head,0,.1f},SetHandClosure{ArmSide::Left,0},CancelReach{ArmSide::Left,body.snapshot().interaction.action}}) rejected(body,op);
        JULIA_CHECK(body.snapshot().epoch==1 && body.snapshot().object->state==ObjectState::Attached && body.snapshot().object->effectorFromObject==attached);
        rejected(body,CancelInteraction{99999});
        BodyInstance malformed(1,asset,&profile,false); command(malformed,SetDirectMode{true});
        rejected(malformed,CreateInteractionSphere{0,original}); rejected(malformed,CreateInteractionSphere{1,original,.03f});
        auto invalid=original; invalid.rotation={0,0,0,0}; rejected(malformed,CreateInteractionSphere{1,invalid});
        command(malformed,CreateInteractionSphere{1,original});
        rejected(malformed,SetHandClosure{static_cast<ArmSide>(99),.5f});
        rejected(malformed,SetHandClosure{ArmSide::Left,std::numeric_limits<float>::quiet_NaN()});
        rejected(malformed,CreateInteractionSphere{2,original});
        report << "unreachable_blocked_closure_no_false_attachment_generations_epochs_conflicts=PASS\n";
    }
    void cancellations(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        for (auto phase:{InteractionPhase::Opening,InteractionPhase::Approach,InteractionPhase::Align,InteractionPhase::Closing,InteractionPhase::Holding,InteractionPhase::Transport}) {
            BodyInstance body(1,asset,&profile,false); grasp(body);
            if (phase==InteractionPhase::Transport) {
                hold(body); command(body,MoveHeldObject{body.snapshot().object->handle,app::interactionDemoPose(body.control()->binding(),ArmSide::Left,2)});
                until(body,[](const auto& s) { return s.reaches[0].state==ReachState::Moving || terminal(s.interaction.phase); });
            } else until(body,[&](const auto& s) { return s.interaction.phase==phase || terminal(s.interaction.phase); });
            JULIA_CHECK(body.snapshot().interaction.phase==phase);
            const bool attached=body.snapshot().object->state==ObjectState::Attached;
            command(body,CancelInteraction{body.snapshot().interaction.action});
            until(body,[](const auto& s) { return s.interaction.phase==InteractionPhase::Holding || terminal(s.interaction.phase); });
            if (attached) JULIA_CHECK(body.snapshot().object->state==ObjectState::Attached && body.snapshot().interaction.result=="CancelledHolding");
            else JULIA_CHECK(body.snapshot().object->state==ObjectState::Free && body.snapshot().interaction.phase==InteractionPhase::Cancelled);
            report << "cancel_phase=" << interactionPhaseName(phase) << " outcome=" << body.snapshot().interaction.result << '\n';
        }
    }
    void collisionKernel(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        BodyRigBinding binding(profile,*asset); DirectBodyControl c(binding); c.setEnabled(true);
        const ArmCoordinates from{.65f,.45f,0,.9f,.12f,0},to{.95f,.45f,0,.9f,.12f,0};
        std::vector<ArmCoordinates> path;
        for (unsigned t=0;t<=120;++t) {
            const double u=static_cast<double>(t)/120, w=u*u*u*(10+u*(-15+6*u));
            ArmCoordinates sample{}; for (std::size_t k=0;k<6;++k) sample[k]=static_cast<float>(from[k]+(to[k]-from[k])*w);
            path.push_back(sample);
        }
        CollisionContext payload; payload.contactHand=3; payload.carried=CarriedSphere{objectId,EffectorId::LeftGrip,
            glm::translate(glm::mat4{1},glm::vec3{0,0,.12f}),.022};
        // Large offset deliberately exercises the generic bound independently of the small demo grip.
        ArmCoordinates middle{}; for (std::size_t k=0;k<6;++k) middle[k]=(from[k]+to[k])*.5f;
        set(c,0,middle); const auto pm=pose(c);
        const auto fm=binding.effectorFrame(EffectorId::LeftGrip,pm.globals)*payload.carried->effectorFromObject;
        world::SceneObject obstacle; obstacle.id=71; obstacle.shape=world::Sphere{.0004}; obstacle.position=glm::dvec3{fm[3]};
        const world::CollisionScene wall(1,{obstacle});
        const auto arm=checkArmPath(c,0,path,glm::mat4{1},wall,{});
        const auto carried=checkArmPath(c,0,path,glm::mat4{1},wall,{},payload);
        const auto first=checkArmPath(c,0,std::span{&from,1},glm::mat4{1},wall,{},payload);
        const auto last=checkArmPath(c,0,std::span{&to,1},glm::mat4{1},wall,{},payload);
        JULIA_CHECK(arm.status==PathStatus::Clear && first.status==PathStatus::Clear && last.status==PathStatus::Clear);
        JULIA_CHECK(carried.status==PathStatus::Blocked && carried.bodyForm==10 && carried.object==71);
        ArmPathCheckJob job(c,0,path,glm::mat4{1},wall,{},payload);
        unsigned oldQueries=0,oldPoses=0;
        while (!job.done()) { job.advance(1,3); JULIA_CHECK(job.result().queries-oldQueries<=3 && job.result().poses-oldPoses<=1); oldQueries=job.result().queries; oldPoses=job.result().poses; }
        JULIA_CHECK(job.result().status==carried.status && job.result().queries==carried.queries);
        // The one permitted hand/object pair cannot suppress another scene collider.
        set(c,0,from); const auto pf=pose(c); const auto grip=binding.effectorFrame(EffectorId::LeftGrip,pf.globals);
        world::SceneObject sphere; sphere.id=objectId; sphere.position=glm::vec3{grip[3]}; sphere.shape=world::Sphere{.022};
        CollisionContext allowance; allowance.contactHand=3; allowance.contactObject=objectId;
        const world::CollisionScene allowed(2,{sphere});
        JULIA_CHECK(checkArmPath(c,0,std::span{&from,1},glm::mat4{1},allowed,{}).status!=PathStatus::Clear);
        JULIA_CHECK(checkArmPath(c,0,std::span{&from,1},glm::mat4{1},allowed,{},allowance).status==PathStatus::Clear);
        auto other=sphere; other.id=72; const world::CollisionScene stillBlocked(3,{sphere,other});
        JULIA_CHECK(checkArmPath(c,0,std::span{&from,1},glm::mat4{1},stillBlocked,{},allowance).status!=PathStatus::Clear);
        const auto forms=bodyColliders(binding,pf.globals,glm::mat4{1}); sphere.position=(forms[6].capsule.a+forms[6].capsule.b)*.5;
        const world::CollisionScene torso(4,{sphere});
        JULIA_CHECK(checkArmPath(c,0,std::span{&from,1},glm::mat4{1},torso,{},allowance).status!=PathStatus::Clear);
        sphere.position=(forms[5].capsule.a+forms[5].capsule.b)*.5; const world::CollisionScene opposite(5,{sphere});
        JULIA_CHECK(checkArmPath(c,0,std::span{&from,1},glm::mat4{1},opposite,{},allowance).status!=PathStatus::Clear);
        report << "payload_mid_interval_blocked_clear_arm_and_endpoints=PASS queries=" << carried.queries << " poses=" << carried.poses << '\n';
        report << "contact_allowance_keeps_scene_torso_other_hand_pairs=PASS\n";
    }
    void blockedTransferAndRelease(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        BodyRigBinding binding(profile,*asset);
        const auto start=app::interactionDemoPose(binding,ArmSide::Left);
        const auto target=app::interactionDemoPose(binding,ArmSide::Left,4);
        // Tiny obstacle outside the hand capsule, inside the sphere's outer shell.
        world::SceneObject obstacle; obstacle.id=80; obstacle.shape=world::Sphere{.0002};
        obstacle.position=rigidPose(rigidMatrix(target)*glm::translate(glm::mat4{1},glm::vec3{0,0,.0235f})).position;
        DirectBodyControl endpoint(binding); endpoint.setEnabled(true);
        const ArmCoordinates endpointQ{.97f,.45f,0,.9f,.12f,0}; set(endpoint,0,endpointQ);
        CollisionContext handOnly; handOnly.handEnvelopes[0]=true;
        JULIA_CHECK(checkArmPath(endpoint,0,std::span{&endpointQ,1},glm::mat4{1},*scene(1,{obstacle}),{},handOnly).status==PathStatus::Clear);
        BodyInstance body(1,asset,&profile,false);
        command(body,SetDirectMode{true}); command(body,SetCollisionScene{scene(1,{obstacle}),{}});
        command(body,CreateInteractionSphere{1,start}); command(body,GraspObject{body.snapshot().object->handle}); hold(body);
        const auto relative=body.snapshot().object->effectorFromObject;
        command(body,MoveHeldObject{body.snapshot().object->handle,target});
        until(body,[](const auto& s) { return s.interaction.phase==InteractionPhase::Holding || terminal(s.interaction.phase); });
        JULIA_CHECK(body.snapshot().interaction.phase==InteractionPhase::Failed);
        JULIA_CHECK(body.snapshot().object->state==ObjectState::Attached && body.snapshot().object->effectorFromObject==relative);
        report << "blocked_transfer_result=" << body.snapshot().interaction.result << " reason=" << body.snapshot().interaction.reason << '\n';
        bool found=false;
        // A supported Release may be unable to reach its required retreat even in
        // empty space at a joint boundary. Find a deterministic finite fixture.
        for (float flex:{1.1f,-.3f}) for (float elbow:{.15f,1.65f}) for (float wrist:{-.42f,.50f}) {
            if (found) continue;
            DirectBodyControl source(binding); source.setEnabled(true);
            const ArmCoordinates q{flex,.65f,0,elbow,wrist,.20f}; set(source,0,q);
            const auto end=rigidPose(binding.effectorFrame(EffectorId::LeftGrip,pose(source).globals));
            ReachTarget retreat; retreat.task=ReachTask::PositionAndOrientation; retreat.effector=EffectorId::LeftGrip;
            const auto away=rigidPose(rigidMatrix(end)*glm::translate(glm::mat4{1},glm::vec3{0,0,-.075f}));
            retreat.position=away.position; retreat.orientation=away.rotation;
            if (solveArmPosition(source,retreat,retreat.position).reason==ReachReason::Reached) continue;
            BodyInstance releaseBody(1,asset,&profile,false); grasp(releaseBody); hold(releaseBody);
            command(releaseBody,MoveHeldObject{releaseBody.snapshot().object->handle,end});
            until(releaseBody,[](const auto& s) { return s.interaction.phase==InteractionPhase::Holding || terminal(s.interaction.phase); });
            if (releaseBody.snapshot().interaction.phase!=InteractionPhase::Holding) continue;
            const auto before=releaseBody.snapshot().object->worldPose;
            command(releaseBody,ReleaseObject{releaseBody.snapshot().object->handle});
            rejected(releaseBody,CancelInteraction{releaseBody.snapshot().interaction.action});
            until(releaseBody,[](const auto& s) { return terminal(s.interaction.phase); });
            if (releaseBody.snapshot().interaction.phase!=InteractionPhase::ReleaseBlocked) continue;
            found=true;
            const auto free=releaseBody.snapshot().object->worldPose;
            JULIA_CHECK(releaseBody.snapshot().object->state==ObjectState::Free && releaseBody.snapshot().object->owner==0);
            JULIA_CHECK(glm::length(before.position-free.position)<1e-6f);
            for (unsigned t=0;t<200;++t) releaseBody.step();
            JULIA_CHECK(releaseBody.snapshot().object->worldPose.position==free.position);
            ReachTarget same; same.task=ReachTask::PositionAndOrientation; same.effector=EffectorId::LeftGrip;
            const auto f=binding.effectorFrame(same.effector,releaseBody.snapshot().pose.globals);
            same.position=glm::vec3{f[3]}; same.orientation=rigidPose(f).rotation;
            rejected(releaseBody,StartReach{same});
            report << "release_blocked_q=" << flex << ',' << elbow << ',' << wrist
                << " free_pose_preserved=PASS permission_removed=PASS reason=" << releaseBody.snapshot().interaction.reason << '\n';
        }
        JULIA_CHECK(found);
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        const auto run=[&](int fps) {
            BodyInstance body(1,asset,&profile,false); app::InteractionDemo demo; demo.start(body); std::vector<std::string> records;
            const auto tick=[&] { demo.drive(body); body.step(); std::ostringstream s; app::interactionTrace(s,body.snapshot()); s << hash(body.snapshot());
                for (const auto& r:body.results()) s << ',' << static_cast<int>(r.status) << ',' << r.reason;
                records.push_back(s.str()); };
            if (!fps) for (unsigned t=0;t<1200;++t) tick();
            else {
                runtime::FixedStepScheduler scheduler; double elapsed=0;
                while (elapsed<10-1e-12) { const auto frame=std::min(10-elapsed,1.0/fps); scheduler.advance(frame,tick); elapsed+=frame;
                    const auto before=hash(body.snapshot()); (void)body.display(scheduler.alpha()); JULIA_CHECK(hash(body.snapshot())==before); }
            }
            JULIA_CHECK(body.snapshot().interaction.result=="Released"); return records;
        };
        const auto reference=run(0);
        for (int fps:{30,60,144}) { JULIA_CHECK(run(fps)==reference); report << "replay_" << fps << "=identical_1200_ticks\n"; }
    }
}
int main(int argc,char** argv)
{
    try {
        const auto output=argc>1 ? std::filesystem::path{argv[1]}:std::filesystem::path{"build/diagnostics/stage6f/cpu"};
        std::filesystem::create_directories(output); std::ofstream report(output/"interaction.txt"); report << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile();
        profileChecks(asset,profile,output,report); cycles(asset,profile,output,report); negatives(asset,profile,report);
        cancellations(asset,profile,report); collisionKernel(asset,profile,report); blockedTransferAndRelease(asset,profile,report); replay(asset,profile,report);
        report << "failures=" << JULIA_TEST_RESULT() << '\n'; return JULIA_TEST_RESULT();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
