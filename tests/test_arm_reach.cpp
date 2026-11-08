#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "pose_test_support.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>
#include <bit>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <limits>
#ifdef _WIN32
#include <Windows.h>
#include <psapi.h>
#endif

using namespace julia;
using namespace julia::embodiment;
namespace
{
    constexpr float positionTolerance=.002f, lengthTolerance=3e-5f;
    constexpr double dt=runtime::BodyStepSeconds;
    bool moving(ReachState s) { return s==ReachState::Moving || s==ReachState::Settling; }
    std::array<JointId,3> ids(int side)
    { return side==0 ? std::array{JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist}:
        std::array{JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist}; }
    glm::vec3 point(const DirectBodyControl& control,EffectorId effector)
    {
        animation::Pose pose(control.binding().asset().skeleton()); control.apply(pose.locals);
        animation::computeGlobalTransforms(control.binding().asset().skeleton(),pose.locals,pose.globals);
        return glm::vec3{control.binding().effectorFrame(effector,pose.globals)[3]};
    }
    ReachTarget generated(const BodyRigBinding& binding,int side,unsigned sample,EffectorId effector)
    {
        DirectBodyControl source(binding); source.setEnabled(true); const auto arm=ids(side);
        // Deterministic coverage of configurations unavailable in animation clips.
        for (std::size_t j=0;j<3;++j)
        {
            const auto& definition=binding.profile().joints[binding.jointIndex(arm[j])];
            for (std::size_t k=0;k<definition.coordinates.size();++k)
            {
                const auto& c=definition.coordinates[k];
                const float fraction=sample==0 ? .01f : sample==1 ? .99f : static_cast<float>((sample*(7+2*k+3*j)+5*k+3*j)%101)/100;
                source.setCoordinate(arm[j],k,c.lower+(c.upper-c.lower)*fraction);
            }
        }
        ReachTarget t; t.arm=side==0 ? ArmSide::Left:ArmSide::Right; t.effector=effector; t.position=point(source,effector);
        return t;
    }
    void setSolution(DirectBodyControl& control,int side,const ArmSolution& solution)
    {
        const auto arm=ids(side);
        for (int k=0;k<6;++k) control.setCoordinate(arm[static_cast<std::size_t>(k<3 ? 0:k==3 ? 1:2)],
            static_cast<std::size_t>(k<3 ? k:k==3 ? 0:k-4),static_cast<float>(solution.coordinates[static_cast<std::size_t>(k)]));
    }
    void solverCases(const BodyRigBinding& binding,std::ostream& report)
    {
        std::ofstream csv(std::filesystem::path{JULIA_TEST_STAGE5_DIR}/"solver.csv");
        csv << "side,effector,sample,error,reason,iterations,microseconds\n";
        float worst=0; unsigned count=0; double total=0,maximum=0;
        for (int side=0;side<2;++side)
            for (int e=0;e<3;++e)
            {
                DirectBodyControl c(binding); c.setEnabled(true);
                const auto effector=static_cast<EffectorId>((side==0 ? 0:4)+e);
                for (unsigned sample=0;sample<40;++sample)
                {
                    const auto target=generated(binding,side,sample,effector);
                    const auto begin=std::chrono::steady_clock::now(); const auto solution=solveArmPosition(c,target,target.position);
                    const double micros=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
                    setSolution(c,side,solution);
                    const float measured=glm::length(point(c,effector)-target.position);
                    csv << side << ',' << e << ',' << sample << ',' << measured << ',' << reachReasonName(solution.reason) << ',' << solution.iterations << ',' << micros << '\n';
                    JULIA_CHECK(measured<=positionTolerance); JULIA_CHECK(solution.reason==ReachReason::Reached);
                    worst=std::max(worst,measured); ++count; total+=micros; maximum=std::max(maximum,micros);
                }
            }
        report << "solver_generated_cases=" << count << "\nsolver_max_error_m=" << worst << "\nsolver_mean_us=" << total/count << "\nsolver_max_us=" << maximum << '\n';
        for (int side=0;side<2;++side)
        {
            DirectBodyControl c(binding); c.setEnabled(true);
            auto t=generated(binding,side,4,side==0 ? EffectorId::LeftWrist:EffectorId::RightWrist);
            t.position={0,4,0}; auto result=solveArmPosition(c,t,t.position);
            JULIA_CHECK(result.reason==ReachReason::GeometricRange); setSolution(c,side,result);
            // Within the length shell, but behind/upward outside the calibrated ranges.
            t.position=glm::vec3{binding.profile().joints[binding.jointIndex(ids(side)[0])].frameInBodyBind[3]}+glm::vec3{0,.25f,-.2f};
            result=solveArmPosition(c,t,t.position);
            JULIA_CHECK(result.reason==ReachReason::JointLimited || result.reason==ReachReason::LocalNoConvergence);
            report << "limited_side_" << side << '=' << reachReasonName(result.reason) << ",error=" << result.error << '\n';
            t.position=glm::vec3{binding.profile().joints[binding.jointIndex(ids(side)[0])].frameInBodyBind[3]};
            result=solveArmPosition(c,t,t.position); JULIA_CHECK(std::isfinite(result.error));
            report << "close_side_" << side << '=' << reachReasonName(result.reason) << ",error=" << result.error << '\n';
        }
    }
    struct Measurements
    {
        float speed=0,acceleration=0,lengthError=0,referenceError=0;
        float handSpeed=0,handAcceleration=0;
        std::array<glm::vec3,2> handVelocity{glm::vec3{0},glm::vec3{0}};
        double meanTick=0,maxTick=0;
    };
    void validateTick(const BodyInstance& body,const BodySnapshot& before,const std::vector<glm::vec3>& previousVelocity,Measurements& stats)
    {
        const auto& s=body.snapshot(); const auto& b=body.control()->binding();
        for (std::size_t j=0;j<s.coordinates.size();++j)
            for (std::size_t k=0;k<b.profile().joints[j].coordinates.size();++k)
            {
                const auto& limit=b.profile().joints[j].coordinates[k]; const int d=static_cast<int>(k);
                const float q=s.coordinates[j][d],v=(q-before.coordinates[j][d])/static_cast<float>(dt),a=(v-previousVelocity[j][d])/static_cast<float>(dt);
                JULIA_CHECK(q>=limit.lower-1e-6f && q<=limit.upper+1e-6f);
                stats.speed=std::max(stats.speed,std::abs(v)); stats.acceleration=std::max(stats.acceleration,std::abs(a));
            }
        for (int side=0;side<2;++side)
        {
            const auto arm=ids(side);
            const auto shoulder=glm::vec3{b.jointFrame(arm[0],s.pose.globals)[3]},elbow=glm::vec3{b.jointFrame(arm[1],s.pose.globals)[3]},wrist=glm::vec3{b.jointFrame(arm[2],s.pose.globals)[3]};
            const auto oldWrist=glm::vec3{b.jointFrame(arm[2],before.pose.globals)[3]};
            const auto velocity=(wrist-oldWrist)/static_cast<float>(dt);
            stats.handSpeed=std::max(stats.handSpeed,glm::length(velocity));
            stats.handAcceleration=std::max(stats.handAcceleration,glm::length((velocity-stats.handVelocity[static_cast<std::size_t>(side)])/static_cast<float>(dt)));
            stats.handVelocity[static_cast<std::size_t>(side)]=velocity;
            const auto f0=b.profile().joints[b.jointIndex(arm[0])].frameInBodyBind, f1=b.profile().joints[b.jointIndex(arm[1])].frameInBodyBind, f2=b.profile().joints[b.jointIndex(arm[2])].frameInBodyBind;
            stats.lengthError=std::max({stats.lengthError,std::abs(glm::length(elbow-shoulder)-glm::length(glm::vec3{f1[3]-f0[3]})),std::abs(glm::length(wrist-elbow)-glm::length(glm::vec3{f2[3]-f1[3]}))});
            if (s.reaches[static_cast<std::size_t>(side)].action)
            {
                stats.referenceError=std::max(stats.referenceError,glm::length(s.reaches[static_cast<std::size_t>(side)].referenceBody-s.reaches[static_cast<std::size_t>(side)].actualBody));
                JULIA_CHECK(glm::dot(s.reaches[static_cast<std::size_t>(side)].bendNormalBody,before.reaches[static_cast<std::size_t>(side)].bendNormalBody)>=-1e-6f);
            }
        }
        // Reconstruct via public coordinate controls: includes all calibrated twist followers.
        DirectBodyControl expected(b); expected.setEnabled(true);
        for (std::size_t j=0;j<s.coordinates.size();++j)
            for (std::size_t k=0;k<b.profile().joints[j].coordinates.size();++k)
                expected.setCoordinate(b.profile().joints[j].id,k,s.coordinates[j][static_cast<int>(k)]);
        animation::Pose p(b.asset().skeleton()); expected.apply(p.locals);
        for (const auto& follower:b.profile().followers)
            for (std::size_t n=0;n<b.asset().nodes().size();++n) if (b.asset().nodes()[n].name==follower.node.name)
                JULIA_CHECK(p.locals[n]==s.pose.locals[n]);
    }
    void actionCases(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        BodyRigBinding binding(profile,*asset); Measurements stats;
        std::ofstream csv(std::filesystem::path{JULIA_TEST_STAGE5_DIR}/"motion.csv");
        csv << "case,tick,action,state,goal_x,goal_y,goal_z,ref_x,ref_y,ref_z,actual_x,actual_y,actual_z,error\n";
        for (int side=0;side<2;++side)
        {
            BodyInstance body(1,asset,&profile,false); body.enqueue(SetDirectMode{true}); body.step();
            stats.handVelocity={glm::vec3{0},glm::vec3{0}};
            std::vector<glm::vec3> velocity(profile.joints.size(),glm::vec3{0});
            // Successive starts preserve nonzero endpoint coordinates; not every trial starts at bind.
            for (unsigned sample=0;sample<8;++sample)
            {
                auto t=generated(binding,side,4+sample*3,side==0 ? EffectorId::LeftWrist:EffectorId::RightWrist);
                body.enqueue(StartReach{t});
                for (int tick=0;tick<2600;++tick)
                {
                    const auto before=body.snapshot(); const auto begin=std::chrono::steady_clock::now(); body.step();
                    const double micros=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
                    stats.maxTick=std::max(stats.maxTick,micros); stats.meanTick+=micros;
                    validateTick(body,before,velocity,stats);
                    const auto& s=body.snapshot();
                    for (std::size_t j=0;j<velocity.size();++j) velocity[j]=(s.coordinates[j]-before.coordinates[j])/static_cast<float>(dt);
                    const auto& o=s.reaches[static_cast<std::size_t>(side)];
                    csv << side*10+sample << ',' << s.tick << ',' << o.action << ',' << static_cast<int>(o.state) << ',' << o.goalBody.x << ',' << o.goalBody.y << ',' << o.goalBody.z << ','
                        << o.referenceBody.x << ',' << o.referenceBody.y << ',' << o.referenceBody.z << ',' << o.actualBody.x << ',' << o.actualBody.y << ',' << o.actualBody.z << ',' << o.error << '\n';
                    if (!moving(o.state)) { JULIA_CHECK(o.state==ReachState::Succeeded && o.error<=t.tolerance); break; }
                    JULIA_CHECK(tick<2599);
                }
            }
        }
        JULIA_CHECK(stats.speed<=1.2f+1e-4f); JULIA_CHECK(stats.acceleration<=4+.03f);
        JULIA_CHECK(stats.lengthError<lengthTolerance); JULIA_CHECK(stats.referenceError<3e-5f);
        report << "motion_success_cases=16\nmeasured_max_speed_rad_s=" << stats.speed << "\nmeasured_max_acceleration_rad_s2=" << stats.acceleration <<
            "\nmax_link_length_error_m=" << stats.lengthError << "\nmax_reference_actual_error_m=" << stats.referenceError << "\nmax_tick_including_command_us=" << stats.maxTick << '\n';
        report << "measured_hand_max_speed_m_s=" << stats.handSpeed << "\nmeasured_hand_max_acceleration_m_s2=" << stats.handAcceleration << '\n';
    }
    void protocols(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        BodyRigBinding binding(profile,*asset); BodyInstance b(7,asset,&profile);
        auto t=generated(binding,0,8,EffectorId::LeftWrist);
        b.enqueue(StartReach{t}); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Rejected); JULIA_CHECK(!b.snapshot().direct);
        b.enqueue(SetDirectMode{true});
        b.enqueue(SetRoot{{{1,.3f,-2},glm::angleAxis(.7f,glm::normalize(glm::vec3{.2f,1,.1f}))}}); b.step();
        const auto world=glm::translate(glm::mat4{1},b.snapshot().root.position)*glm::mat4_cast(b.snapshot().root.rotation);
        const auto bodyGoal=t.position; t.space=TargetSpace::World; t.position=glm::vec3{world*glm::vec4{bodyGoal,1}};
        const auto start=b.enqueue(StartReach{t}); b.step();
        JULIA_CHECK(glm::length(b.snapshot().reaches[0].goalBody-bodyGoal)<1e-6f);
        auto bad=t; bad.maxSpeed=.5f; b.enqueue(RetargetReach{start.id,bad}); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Rejected);
        bad=t; bad.position.x=std::numeric_limits<float>::quiet_NaN(); b.enqueue(StartReach{bad}); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Rejected);
        b.enqueue(CancelReach{ArmSide::Left,start.id+100}); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Rejected);
        b.enqueue(SetCoordinate{JointId::LeftElbow,0,.1f}); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Rejected);
        b.enqueue(SetCoordinate{JointId::RightElbow,0,.3f}); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Applied);
        b.enqueue(EyeCommand{EyeOperation::Gaze,{9,3,0}}); b.step(); JULIA_CHECK(glm::length(b.snapshot().eyes.gazeDegrees)>0);
        for (int k=0;k<20;++k) b.step();
        auto next=generated(binding,0,21,EffectorId::LeftWrist); next.space=TargetSpace::World; next.position=glm::vec3{world*glm::vec4{next.position,1}};
        const auto before=b.snapshot(); b.enqueue(RetargetReach{start.id,next}); b.step();
        JULIA_CHECK(b.results().back().status==BodyCommandStatus::Applied); JULIA_CHECK(b.snapshot().reaches[0].action==start.id);
        JULIA_CHECK(glm::length(b.snapshot().coordinates[binding.jointIndex(JointId::LeftShoulder)]-before.coordinates[binding.jointIndex(JointId::LeftShoulder)])<.02f);
        b.enqueue(CancelReach{ArmSide::Left,start.id}); b.step();
        JULIA_CHECK(b.snapshot().reaches[0].cancellationPending);
        for (int k=0;k<2600 && moving(b.snapshot().reaches[0].state);++k) b.step();
        JULIA_CHECK(b.snapshot().reaches[0].state==ReachState::Cancelled && b.snapshot().reaches[0].reason==ReachReason::UserCancelled && !b.snapshot().reaches[0].cancellationPending);
        const auto held=b.snapshot().coordinates; for (int k=0;k<30;++k) b.step(); JULIA_CHECK(held==b.snapshot().coordinates);
        JULIA_CHECK(b.snapshot().coordinates[binding.jointIndex(JointId::LeftShoulder)]!=glm::vec3{0});
        t.space=TargetSpace::Body; t.position=bodyGoal; const auto restart=b.enqueue(StartReach{t}); b.step();
        b.enqueue(SetDirectMode{false}); b.step(); JULIA_CHECK(b.reachEvents().back().reason==ReachReason::ModeChanged);
        b.enqueue(SetDirectMode{true}); b.enqueue(StartReach{next}); b.step();
        b.enqueue(ResetCoordinates{}); b.step(); JULIA_CHECK(b.reachEvents().back().reason==ReachReason::CoordinatesReset);
        b.enqueue(StartReach{t}); b.step();
        b.enqueue(ResetBody{}); b.enqueue(CancelReach{ArmSide::Left,restart.id}); b.step();
        JULIA_CHECK(b.snapshot().epoch==1 && b.snapshot().reaches[0].state==ReachState::Idle);
        JULIA_CHECK(b.results().back().status==BodyCommandStatus::Cancelled);
        JULIA_CHECK(b.submit({100,7,b.snapshot().tick+1,0,StartReach{t}}).status==BodyCommandStatus::Rejected);
        t.effector=EffectorId::RightWrist; b.enqueue(SetDirectMode{true}); b.enqueue(StartReach{t}); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Rejected);
        report << "protocols=clip_rejection,world_transform,conflicts,independent_arm_eyes,retarget,cancel_hold,mode,reset,epochs,invalid_effector\n";
    }
    void edgeCases(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        BodyRigBinding binding(profile,*asset);
        for (int side=0;side<2;++side)
        {
            const auto arm=ids(side); const auto effector=side==0 ? EffectorId::LeftWrist:EffectorId::RightWrist;
            DirectBodyControl source(binding); source.setEnabled(true);
            // Find the most extended pose inside the ACTUAL elbow interval, without assuming q=0 means straight.
            const auto shoulder=glm::vec3{profile.joints[binding.jointIndex(arm[0])].frameInBodyBind[3]};
            float farthest=0; glm::vec3 extension{0};
            const auto& limit=profile.joints[binding.jointIndex(arm[1])].coordinates[0];
            for (int k=0;k<=1000;++k)
            {
                source.setCoordinate(arm[1],0,limit.lower+(limit.upper-limit.lower)*static_cast<float>(k)/1000);
                const auto p=point(source,effector); const auto distance=glm::length(p-shoulder);
                if (distance>farthest) { farthest=distance; extension=p; }
            }
            ReachTarget t; t.arm=side==0 ? ArmSide::Left:ArmSide::Right; t.effector=effector; t.position=extension;
            DirectBodyControl c(binding); c.setEnabled(true); const auto solution=solveArmPosition(c,t,t.position);
            JULIA_CHECK(solution.reason==ReachReason::Reached && solution.error<=positionTolerance);
            report << "extension_side_" << side << "_distance_error_m=" << farthest << ',' << solution.error << '\n';
            BodyInstance b(1,asset,&profile,false); b.enqueue(SetDirectMode{true}); b.step();
            b.enqueue(StartReach{t});
            for (int k=0;k<1200;++k) { b.step(); if (!moving(b.snapshot().reaches[static_cast<std::size_t>(side)].state)) break; }
            JULIA_CHECK(b.snapshot().reaches[static_cast<std::size_t>(side)].state==ReachState::Succeeded);
            for (int failure=0;failure<2;++failure)
            {
                t.position=failure==0 ? glm::vec3{0,4,0}:shoulder+glm::vec3{0,.25f,-.2f};
                b.enqueue(StartReach{t});
                for (int k=0;k<1200;++k) { b.step(); if (!moving(b.snapshot().reaches[static_cast<std::size_t>(side)].state)) break; }
                const auto& r=b.snapshot().reaches[static_cast<std::size_t>(side)];
                JULIA_CHECK(r.state==ReachState::Failed && r.error>t.tolerance);
                JULIA_CHECK(failure ? r.reason==ReachReason::JointLimited || r.reason==ReachReason::LocalNoConvergence:r.reason==ReachReason::GeometricRange);
            }
            // Full world-space action completion, rather than just checking the inverse transform.
            const RootPose root{{1,.2f,-3},glm::angleAxis(.9f,glm::normalize(glm::vec3{.2f,1,.1f}))};
            const auto world=glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation);
            t=generated(binding,side,18,effector); t.position=glm::vec3{world*glm::vec4{t.position,1}}; t.space=TargetSpace::World;
            b.enqueue(SetRoot{root}); b.enqueue(StartReach{t});
            for (int k=0;k<1200;++k) { b.step(); if (!moving(b.snapshot().reaches[static_cast<std::size_t>(side)].state)) break; }
            const auto& r=b.snapshot().reaches[static_cast<std::size_t>(side)];
            JULIA_CHECK(r.state==ReachState::Succeeded && glm::length(r.actualWorld-t.position)<=t.tolerance);
            // Root teleport cancels instead of silently reinterpreting an active world trajectory.
            b.enqueue(StartReach{t}); b.step(); b.enqueue(SetRoot{{}}); b.step();
            JULIA_CHECK(b.reachEvents().back().reason==ReachReason::RootChanged);
            JULIA_CHECK(std::abs(b.snapshot().reaches[static_cast<std::size_t>(side)].error-glm::length(b.snapshot().reaches[static_cast<std::size_t>(side)].actualWorld-t.position))<1e-5f);
        }
        // Same reduced chain works with a nonzero frozen torso and clavicle.
        DirectBodyControl source(binding),control(binding); source.setEnabled(true); control.setEnabled(true);
        for (auto* c:{&source,&control})
        { c->setCoordinate(JointId::Chest,0,.1f); c->setCoordinate(JointId::LeftClavicle,0,.1f); }
        source.setCoordinate(JointId::LeftShoulder,0,.5f); source.setCoordinate(JointId::LeftElbow,0,.8f);
        ReachTarget t; t.position=point(source,EffectorId::LeftWrist);
        const auto result=solveArmPosition(control,t,t.position); setSolution(control,0,result);
        JULIA_CHECK(glm::length(point(control,t.effector)-t.position)<=positionTolerance);
        report << "edge_cases=extension,geometric_and_joint_limited_actions,world_completion,root_cancel,frozen_nonzero_ancestors\n";
    }
    struct History { std::vector<glm::vec3> q; std::array<ReachObservation,2> reach; std::uint64_t hash=0; };
    std::uint64_t hashPose(const BodySnapshot& s)
    {
        std::uint64_t hash=1469598103934665603ull;
        for (const auto& m:s.pose.palette) for (int c=0;c<4;++c) for (int r=0;r<4;++r)
        { hash^=std::bit_cast<std::uint32_t>(m[c][r]); hash*=1099511628211ull; }
        return hash;
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        BodyRigBinding binding(profile,*asset);
        const auto stream=[&](BodyInstance& b)
        {
            auto left=generated(binding,0,7,EffectorId::LeftWrist),right=generated(binding,1,13,EffectorId::RightGrip);
            const auto changed=generated(binding,0,25,EffectorId::LeftWrist);
            std::vector<BodyCommand> commands{{1,1,1,0,SetDirectMode{true}},{2,1,2,0,StartReach{left}},
                {3,1,3,0,StartReach{right}},{4,1,45,0,RetargetReach{2,changed}},{5,1,90,0,CancelReach{ArmSide::Right,3}},
                {6,1,120,0,EyeCommand{EyeOperation::Blink,{1,1,0}}}};
            for (auto it=commands.rbegin();it!=commands.rend();++it) JULIA_CHECK(b.submit(*it).status==BodyCommandStatus::Queued);
        };
        BodyInstance exact(1,asset,&profile); stream(exact); std::vector<History> history;
        for (int k=0;k<1200;++k) { exact.step(); history.push_back({exact.snapshot().coordinates,exact.snapshot().reaches,hashPose(exact.snapshot())}); }
        for (int fps:{30,60,144})
        {
            BodyInstance b(1,asset,&profile); stream(b); runtime::FixedStepScheduler scheduler;
            std::vector<glm::vec3> previousVelocity(profile.joints.size(),glm::vec3{0}); float maxSpeed=0,maxAcceleration=0;
            double elapsed=0;
            while (elapsed<10-1e-12)
            {
                const double frame=std::min(10-elapsed,1.0/fps);
                scheduler.advance(frame,[&] {
                    const auto before=b.snapshot().coordinates; b.step(); const auto& h=history[static_cast<std::size_t>(b.snapshot().tick-1)];
                    for (std::size_t j=0;j<before.size();++j)
                    {
                        const auto v=(b.snapshot().coordinates[j]-before[j])/static_cast<float>(dt),a=(v-previousVelocity[j])/static_cast<float>(dt);
                        for (int d=0;d<3;++d) { maxSpeed=std::max(maxSpeed,std::abs(v[d])); maxAcceleration=std::max(maxAcceleration,std::abs(a[d])); }
                        previousVelocity[j]=v;
                    }
                    JULIA_CHECK(b.snapshot().coordinates==h.q && hashPose(b.snapshot())==h.hash);
                    for (std::size_t s=0;s<2;++s)
                    {
                        const auto& a=b.snapshot().reaches[s]; const auto& e=h.reach[s];
                        JULIA_CHECK(a.action==e.action && a.state==e.state && a.reason==e.reason && a.error==e.error && a.actualBody==e.actualBody && a.referenceBody==e.referenceBody);
                    }
                }); elapsed+=frame;
                const auto before=b.snapshot().coordinates; const auto d=b.display(scheduler.alpha()); const auto again=b.display(scheduler.alpha());
                JULIA_CHECK(d.pose.palette==again.pose.palette && b.snapshot().coordinates==before);
            }
            JULIA_CHECK(b.snapshot().tick==1200);
            JULIA_CHECK(maxSpeed<=1.2f+1e-4f && maxAcceleration<=4+.03f);
            report << "retarget_cancel_fps_" << fps << "_max_speed_acceleration=" << maxSpeed << ',' << maxAcceleration << '\n';
        }
        report << "replay_ticks=1200\nreplay_fps=30,60,144,headless\nreplay_pose_q_action_difference=0\n";
        // Include existing snapshot copies, command ID history and display allocations in timings.
        for (bool reach:{false,true})
        {
            BodyInstance b(1,asset,&profile,false); b.enqueue(SetDirectMode{true}); b.step();
            if (reach) b.enqueue(StartReach{generated(binding,0,24,EffectorId::LeftWrist)});
            std::vector<double> times,activeTimes; times.reserve(1200);
            for (int tick=0;tick<1200;++tick)
            {
                const auto begin=std::chrono::steady_clock::now(); b.step();
                times.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count());
                if (moving(b.snapshot().reaches[0].state)) activeTimes.push_back(times.back());
            }
            std::sort(times.begin(),times.end());
            report << (reach ? "reach":"idle") << "_tick_mean_p95_max_us=" << std::accumulate(times.begin(),times.end(),0.0)/times.size() << ',' << times[times.size()*95/100] << ',' << times.back() << '\n';
            if (!activeTimes.empty())
            {
                std::sort(activeTimes.begin(),activeTimes.end());
                report << "active_ticks=" << activeTimes.size() << "\nactive_tick_mean_p95_max_us=" << std::accumulate(activeTimes.begin(),activeTimes.end(),0.0)/activeTimes.size() << ',' << activeTimes[activeTimes.size()*95/100] << ',' << activeTimes.back() << '\n';
            }
        }
        const auto memory=[]() -> std::size_t {
#ifdef _WIN32
            PROCESS_MEMORY_COUNTERS_EX info{};
            if (GetProcessMemoryInfo(GetCurrentProcess(),reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&info),sizeof(info))) return info.PrivateUsage;
#endif
            return 0;
        };
        BodyInstance load(1,asset,&profile,false); load.enqueue(SetDirectMode{true}); load.step();
        const auto bytesBefore=memory(); std::vector<double> stressed;
        auto t=generated(binding,0,22,EffectorId::LeftWrist); load.enqueue(StartReach{t}); load.step();
        const auto action=load.snapshot().reaches[0].action;
        for (int k=0;k<10000;++k)
        {
            load.enqueue(EyeCommand{EyeOperation::Gaze,{0,0,0}});
            if (k<1200 && k%30==0) { t.position.z+=k%60==0 ? .005f:-.005f; load.enqueue(RetargetReach{action,t}); }
            const auto begin=std::chrono::steady_clock::now(); load.step(); const auto display=load.display(.5);
            stressed.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count());
            JULIA_CHECK(display.toTick==load.snapshot().tick);
        }
        const auto bytesAfter=memory(),idsBeforeReset=load.commandHistorySize();
        load.enqueue(ResetBody{}); load.step(); JULIA_CHECK(load.commandHistorySize()==0);
        std::sort(stressed.begin(),stressed.end());
        report << "stress_ticks=10000\nstress_tick_plus_display_mean_p95_max_us=" << std::accumulate(stressed.begin(),stressed.end(),0.0)/stressed.size() << ',' << stressed[stressed.size()*95/100] << ',' << stressed.back()
            << "\nprivate_bytes_before_after=" << bytesBefore << ',' << bytesAfter << "\ncommand_ids_before_after_reset=" << idsBeforeReset << ',' << load.commandHistorySize()
            << "\nsnapshot_pose_arrays_bytes_each=" << (asset->nodes().size()*2+asset->bones().size())*sizeof(glm::mat4)
            << "\nreach_controller_bytes=" << sizeof(ArmReach) << '\n';
    }
}
int main(int argc,char** argv)
{
    try
    {
        const auto output=argc>1 ? std::filesystem::path{argv[1]}:std::filesystem::path{JULIA_TEST_STAGE5_DIR};
        std::filesystem::create_directories(output); std::filesystem::create_directories(JULIA_TEST_STAGE5_DIR);
        std::ofstream report(output/"arm_reach.txt"); report << std::setprecision(9);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile(); const BodyRigBinding binding(profile,*asset);
        solverCases(binding,report); actionCases(asset,profile,report); protocols(asset,profile,report); edgeCases(asset,profile,report); replay(asset,profile,report);
        report << "failures=" << JULIA_TEST_RESULT() << '\n';
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
    return JULIA_TEST_RESULT();
}
