#include "body_reach_test_support.hpp"
#include "../apps/common/SupportDemo.hpp"
#include "../apps/common/InteractionDemo.hpp"
#include <limits>
#include <sstream>
using namespace body_reach_test;
namespace
{
    void require(bool ok,const std::string& message) { if (!ok) throw std::runtime_error(message); }
    void accepted(BodyInstance& b,BodyOperation operation)
    { b.enqueue(std::move(operation)); b.step(); require(b.results().back().status==BodyCommandStatus::Applied,b.results().back().reason); }
    void rejected(BodyInstance& b,BodyOperation operation)
    { b.enqueue(std::move(operation)); b.step(); JULIA_CHECK(b.results().back().status==BodyCommandStatus::Rejected); }
    void settle(BodyInstance& b,bool allowFailure=false)
    {
        for (unsigned t=0;t<4000;++t) {
            const auto& o=b.snapshot().support;
            if (!allowFailure) require(o.result!="PlanRejected","action="+std::to_string(o.action)+" root="+std::to_string(o.requestedRoot.x)+","+std::to_string(o.requestedRoot.y)+","+std::to_string(o.requestedRoot.z)+" "+o.reason);
            if (o.phase!=SupportPhase::Planning && o.phase!=SupportPhase::Moving && o.phase!=SupportPhase::Aligning) return;
            b.step();
        } throw std::runtime_error("Support did not finish within bounded planning/execution horizon");
    }
    void initialize(BodyInstance& b)
    {
        accepted(b,SetDirectMode{true}); const auto q=b.snapshot().coordinates;
        accepted(b,EnableSupport{}); JULIA_CHECK(q==b.snapshot().coordinates);
        JULIA_CHECK(b.snapshot().support.phase==SupportPhase::Unaligned && b.snapshot().support.kind==SupportKind::None);
        accepted(b,AlignSupport{}); settle(b);
        require(b.snapshot().support.kind==SupportKind::KinematicSupport,"Initial alignment failed");
    }
    void profileAndIK(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const std::filesystem::path& output,std::ostream& report)
    {
        const BodyRigBinding binding(profile,*asset); const auto contacts=calibrateSoleContacts(binding);
        std::ofstream calibration(output/"sole-calibration.csv"); calibration << std::setprecision(12);
        std::ofstream rig(output/"leg-rig.txt"); rig << std::setprecision(12);
        for (unsigned s=0;s<2;++s) {
            const auto& c=contacts[s]; calibration << s << ',' << c.shoeVertices << ',' << c.lowVertices;
            for (const auto v:{c.sampledMin,c.sampledMax,glm::vec3{c.soleFromContact[3]}}) calibration << ',' << v.x << ',' << v.y << ',' << v.z;
            for (const auto p:c.points) calibration << ',' << p.x << ',' << p.y << ',' << p.z;
            calibration << '\n';
            for (const auto id:legJoints(s==0 ? LegSide::Left:LegSide::Right)) {
                const auto& j=profile.joints[binding.jointIndex(id)]; rig << j.name << ' ' << j.node.name << " -> " << j.segmentEnd.name << '\n';
                for (const auto& d:j.coordinates) rig << d.name << " axis=" << d.axis.x << ',' << d.axis.y << ',' << d.axis.z << " range=" << d.lower << ':' << d.upper << '\n';
                const auto& m=asset->nodes()[binding.jointNode(id)].localTransform;
                for (int row=0;row<4;++row) { for (int col=0;col<4;++col) rig << m[col][row] << ' '; rig << '\n'; }
            }
            DirectBodyControl source(binding),seed(binding); source.setEnabled(true); seed.setEnabled(true);
            const auto ids=legJoints(s==0 ? LegSide::Left:LegSide::Right); const auto e=s==0 ? EffectorId::LeftSole:EffectorId::RightSole;
            auto p0=pose(source); const auto ankle0=glm::vec3{binding.jointFrame(ids[2],p0.globals)[3]};
            source.setCoordinate(ids[1],0,.2f); const auto ankle1=glm::vec3{binding.jointFrame(ids[2],pose(source).globals)[3]};
            JULIA_CHECK(ankle1.z<ankle0.z); source.reset();
            float maxP=0,maxA=0;
            const auto world=matrix({{1.2f,.3f,-.4f},glm::angleAxis(.8f,glm::vec3{0,1,0})});
            for (unsigned sample=0;sample<40;++sample) {
                const float f=static_cast<float>(sample);
                source.setCoordinate(ids[0],0,.02f+.002f*f); source.setCoordinate(ids[0],1,.01f*std::sin(f)); source.setCoordinate(ids[0],2,.01f*std::cos(f));
                source.setCoordinate(ids[1],0,.02f+.002f*f); source.setCoordinate(ids[2],0,.01f*std::sin(f)); source.setCoordinate(ids[2],1,.01f*std::cos(f));
                const auto goal=world*binding.effectorFrame(e,pose(source).globals)*c.soleFromContact;
                const auto solution=solveLegPose(seed,s==0 ? LegSide::Left:LegSide::Right,c,world,goal);
                require(solution.success,"Reachable leg pose failed: "+solution.reason);
                maxP=std::max(maxP,solution.positionError); maxA=std::max(maxA,solution.angularError);
                for (unsigned k=0;k<6;++k) seed.setCoordinate(ids[k<3 ? 0:k==3 ? 1:2],k<3 ? k:k==3 ? 0:k-4,solution.coordinates[k]);
            }
            report << "IK_side=" << s << " goals=40 max_p=" << maxP << " max_angle=" << maxA << '\n';
            source.reset(); seed.reset(); source.setCoordinate(ids[1],0,-.075f);
            const auto extension=world*binding.effectorFrame(e,pose(source).globals)*c.soleFromContact;
            const auto solution=solveLegPose(seed,s==0 ? LegSide::Left:LegSide::Right,c,world,extension);
            JULIA_CHECK(solution.success && solution.iterations<=80);
            auto distant=extension; distant[3].y+=2;
            const auto failed=solveLegPose(seed,s==0 ? LegSide::Left:LegSide::Right,c,world,distant);
            JULIA_CHECK(!failed.success && failed.iterations<=80);
            for (unsigned k=0;k<6;++k) {
                const auto& d=profile.joints[binding.jointIndex(ids[k<3 ? 0:k==3 ? 1:2])].coordinates[k<3 ? k:k==3 ? 0:k-4];
                JULIA_CHECK(failed.coordinates[k]>=d.lower && failed.coordinates[k]<=d.upper);
            }
            auto invalid=world; invalid[0][0]=std::numeric_limits<float>::quiet_NaN(); bool threw=false;
            try { (void)solveLegPose(seed,s==0 ? LegSide::Left:LegSide::Right,c,invalid,extension); } catch (const std::invalid_argument&) { threw=true; }
            JULIA_CHECK(threw);
        }
        JULIA_CHECK(asset->nodes()[binding.jointNode(JointId::LeftHip)].localTransform!=asset->nodes()[binding.jointNode(JointId::RightHip)].localTransform);
    }
    struct Measurements
    {
        std::array<double,2> p{},a{},gap{},displayP{},displayA{},displayGap{},length{};
        double rootV=0,rootA=0,jointV=0,jointA=0;
        unsigned maxIterations=0,maxEvaluations=0,maxFk=0;
        std::size_t memory=0;
        std::vector<double> ticks,planning;
        glm::dvec3 rootVelocity{0}; std::vector<glm::dvec3> velocity;
        void record(BodyInstance& b,double microseconds)
        {
            const auto& s=b.snapshot(); const auto& prev=b.previousSnapshot(); const auto& o=s.support;
            ticks.push_back(microseconds); if (o.tickPlanningMicroseconds>0) planning.push_back(microseconds);
            maxIterations=std::max(maxIterations,o.tickIterations); maxEvaluations=std::max(maxEvaluations,o.tickEvaluations); maxFk=std::max(maxFk,o.tickFkChecks); memory=std::max(memory,o.storageBytes);
            const auto rv=(glm::dvec3{s.root.position}-glm::dvec3{prev.root.position})/runtime::BodyStepSeconds;
            rootV=std::max(rootV,glm::length(rv)); rootA=std::max(rootA,glm::length(rv-rootVelocity)/runtime::BodyStepSeconds); rootVelocity=rv;
            if (velocity.empty()) velocity.resize(s.coordinates.size());
            const auto& bnd=b.control()->binding();
            for (std::size_t j=0;j<s.coordinates.size();++j) {
                const auto v=(glm::dvec3{s.coordinates[j]}-glm::dvec3{prev.coordinates[j]})/runtime::BodyStepSeconds;
                for (unsigned k=0;k<bnd.profile().joints[j].coordinates.size();++k) {
                    jointV=std::max(jointV,std::abs(v[k])); jointA=std::max(jointA,std::abs(v[k]-velocity[j][k])/runtime::BodyStepSeconds);
                    const auto& limit=bnd.profile().joints[j].coordinates[k]; JULIA_CHECK(s.coordinates[j][k]>=limit.lower && s.coordinates[j][k]<=limit.upper);
                } velocity[j]=v;
            }
            for (unsigned side=0;side<2;++side) {
                const auto& foot=o.anchors[side]; const auto ids=legJoints(side==0 ? LegSide::Left:LegSide::Right);
                if (foot.bound) { p[side]=std::max(p[side],static_cast<double>(foot.positionError)); a[side]=std::max(a[side],static_cast<double>(foot.angularError)); gap[side]=std::max({gap[side],std::abs(static_cast<double>(foot.minGap)),std::abs(static_cast<double>(foot.maxGap))}); }
                for (unsigned link=0;link<2;++link) {
                    const double original=glm::length(glm::vec3{bnd.profile().joints[bnd.jointIndex(ids[link])].frameInBodyBind[3]}-glm::vec3{bnd.profile().joints[bnd.jointIndex(ids[link+1])].frameInBodyBind[3]});
                    if (o.enabled) length[side]=std::max(length[side],std::abs(glm::length(glm::dvec3{foot.chain[link+1]}-glm::dvec3{foot.chain[link]})-original));
                }
            }
            if (o.kind==SupportKind::KinematicSupport) for (double alpha:{0.,.1,.23,.5,.77,.9,1.}) {
                const auto before=hash(s); const auto d=b.display(alpha); JULIA_CHECK(hash(b.snapshot())==before);
                for (unsigned side=0;side<2;++side) if (d.support.anchors[side].bound) {
                    const auto& f=d.support.anchors[side]; displayP[side]=std::max(displayP[side],static_cast<double>(f.positionError)); displayA[side]=std::max(displayA[side],static_cast<double>(f.angularError));
                    displayGap[side]=std::max({displayGap[side],std::abs(static_cast<double>(f.minGap)),std::abs(static_cast<double>(f.maxGap))});
                }
            }
        }
        void report(std::ostream& out)
        {
            stats(out,"ticks_us",ticks); stats(out,"planning_ticks_us",planning);
            out << "q_v_a=" << jointV << ',' << jointA << " root_v_a=" << rootV << ',' << rootA << " budgets=" << maxIterations << ',' << maxEvaluations << ',' << maxFk << " storage=" << memory << '\n';
            JULIA_CHECK(rootV<=SupportRootSpeed && rootA<=SupportRootAcceleration && jointV<=SupportJointSpeed && jointA<=SupportJointAcceleration);
            JULIA_CHECK(maxIterations<=4 && maxEvaluations<=160 && maxFk<=4);
            for (unsigned side=0;side<2;++side) {
                out << "foot=" << side << " p_angle_gap=" << p[side] << ',' << a[side] << ',' << gap[side] << " display=" << displayP[side] << ',' << displayA[side] << ',' << displayGap[side] << " length=" << length[side] << '\n';
                JULIA_CHECK(p[side]<=SupportPositionTolerance && a[side]<=SupportAngularTolerance && gap[side]<=SupportSurfaceTolerance);
                JULIA_CHECK(displayP[side]<=SupportPositionTolerance && displayA[side]<=SupportAngularTolerance && displayGap[side]<=SupportSurfaceTolerance && length[side]<.00003);
            }
        }
    };
    void cycles(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,const std::filesystem::path& output,std::ostream& report)
    {
        for (unsigned world=0;world<2;++world) {
            BodyInstance b(1,asset,&profile,false);
            if (world) accepted(b,SetRoot{{{1.2f,.3f,-.4f},glm::angleAxis(.8f,glm::vec3{0,1,0})}});
            accepted(b,SetDirectMode{true});
            app::SupportDemo demo; demo.start(b); Measurements m; std::ofstream trace(output/(world ? "world.csv":"cycle.csv"));
            std::array<glm::mat4,2> anchors{}; bool captured=false;
            for (unsigned t=0;t<12000;++t) {
                demo.drive(b); const auto start=Clock::now(); b.step(); m.record(b,std::chrono::duration<double,std::micro>(Clock::now()-start).count());
                for (const auto& r:b.results()) require(r.status!=BodyCommandStatus::Rejected,r.reason);
                require(b.snapshot().support.result!="PlanRejected",b.snapshot().support.reason);
                app::supportTrace(trace,b.snapshot());
                const auto& o=b.snapshot().support;
                if (!captured && o.enabled) { for (unsigned side=0;side<2;++side) anchors[side]=o.anchors[side].desiredWorld; captured=true; }
                if (captured) for (unsigned side=0;side<2;++side) JULIA_CHECK(anchors[side]==o.anchors[side].desiredWorld);
            }
            require(demo.done(b.snapshot()),"Demo did not complete"); JULIA_CHECK(b.snapshot().root.position==demo.origin.position);
            const auto q=b.snapshot().coordinates; const auto globals=b.snapshot().pose.globals; const auto feet=b.snapshot().support.anchors;
            for (unsigned t=0;t<10000;++t) b.step();
            JULIA_CHECK(q==b.snapshot().coordinates && globals==b.snapshot().pose.globals);
            for (unsigned side=0;side<2;++side) JULIA_CHECK(feet[side].actualWorld==b.snapshot().support.anchors[side].actualWorld);
            report << "cycle_world=" << world << " hold=10000_ticks_no_drift\n"; m.report(report);
        }
    }
    void negatives(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        BodyInstance b(1,asset,&profile,false); rejected(b,EnableSupport{}); initialize(b);
        rejected(b,SetSupportSurface{{0,2,0,true}}); rejected(b,SetSupportSurface{{1,1,0,true}});
        const auto anchors=b.snapshot().support.anchors;
        for (const auto p:{glm::vec3{0,.2f,0},glm::vec3{0,-.2f,0},glm::vec3{2,0,0},glm::vec3{std::numeric_limits<float>::quiet_NaN(),0,0}}) rejected(b,MoveSupportRoot{p});
        const auto initialQ=b.snapshot().coordinates;
        accepted(b,MoveSupportRoot{{0,.024f,0}}); settle(b,true);
        JULIA_CHECK(b.snapshot().support.result=="PlanRejected" && b.snapshot().root.position==glm::vec3{0} && b.snapshot().coordinates==initialQ);
        rejected(b,SetCoordinate{JointId::LeftKnee,0,.1f}); rejected(b,StartReach{}); rejected(b,SetHandClosure{});
        const auto freePose=app::interactionDemoPose(b.control()->binding(),ArmSide::Left);
        accepted(b,CreateInteractionSphere{1,freePose}); const auto handle=b.snapshot().object->handle;
        rejected(b,GraspObject{handle,ArmSide::Left});
        accepted(b,MoveSupportRoot{{.006f,0,0}});
        rejected(b,MoveSupportRoot{{.01f,0,0}}); // Already planning: preserve original job.
        while (b.snapshot().support.phase==SupportPhase::Planning) b.step();
        require(b.snapshot().support.phase==SupportPhase::Moving,b.snapshot().support.reason);
        for (int k=0;k<50;++k) b.step();
        const auto beforeClipPause=b.snapshot().root.position;
        accepted(b,ClipCommand{ClipOperation::Pause}); JULIA_CHECK(b.snapshot().root.position!=beforeClipPause);
        runtime::FixedStepScheduler scheduler; scheduler.setPaused(true); const auto pausedTick=b.snapshot().tick, pausedHash=hash(b.snapshot());
        scheduler.advance(.1,[&] { b.step(); }); JULIA_CHECK(b.snapshot().tick==pausedTick && hash(b.snapshot())==pausedHash);
        scheduler.setPaused(false); scheduler.runTicks(1,[&] { b.step(); }); JULIA_CHECK(b.snapshot().tick==pausedTick+1);
        const auto movingRoot=b.snapshot().root.position;
        rejected(b,MoveSupportRoot{{0,1,0}}); accepted(b,CancelSupportMotion{});
        JULIA_CHECK(b.snapshot().root.position!=movingRoot && b.snapshot().support.cancellationPending);
        settle(b); JULIA_CHECK(b.snapshot().root.position==glm::vec3(.006f,0,0));
        JULIA_CHECK(b.snapshot().support.result=="Cancelled" && b.snapshot().object->worldPose.position==freePose.position);
        for (unsigned side=0;side<2;++side) JULIA_CHECK(anchors[side].desiredWorld==b.snapshot().support.anchors[side].desiredWorld);
        accepted(b,MoveSupportRoot{{0,0,0}}); accepted(b,CancelSupportMotion{}); JULIA_CHECK(b.snapshot().support.phase==SupportPhase::Holding);
        accepted(b,MoveSupportRoot{{0,0,0}}); while (b.snapshot().support.phase==SupportPhase::Planning) b.step();
        accepted(b,SetSupportSurface{{1,2,0,false}}); JULIA_CHECK(b.snapshot().support.kind==SupportKind::None);
        settle(b); JULIA_CHECK(b.snapshot().support.phase==SupportPhase::Lost);
        for (unsigned side=0;side<2;++side) JULIA_CHECK(!b.snapshot().support.anchors[side].bound && anchors[side].desiredWorld==b.snapshot().support.anchors[side].desiredWorld);
        rejected(b,EnableSupport{}); accepted(b,SetSupportSurface{{1,3,0,true}}); accepted(b,EnableSupport{}); accepted(b,AlignSupport{}); settle(b);
        accepted(b,MoveSupportRoot{{.005f,0,0}}); while (b.snapshot().support.phase==SupportPhase::Planning) b.step();
        accepted(b,DisableSupport{}); JULIA_CHECK(b.snapshot().support.disablePending); settle(b); JULIA_CHECK(!b.snapshot().support.enabled);
        // Free object remains world-fixed across both root discontinuities and reset.
        accepted(b,SetRoot{{{.3f,.1f,-.2f},glm::angleAxis(.2f,glm::vec3{0,1,0})}});
        JULIA_CHECK(b.snapshot().object->worldPose.position==freePose.position);
        const auto epoch=b.snapshot().epoch; accepted(b,ResetBody{}); JULIA_CHECK(b.snapshot().epoch==epoch+1 && !b.snapshot().support.enabled);
        JULIA_CHECK(b.submit({999,1,b.snapshot().tick+1,epoch,EnableSupport{}}).status==BodyCommandStatus::Rejected);
        JULIA_CHECK(b.snapshot().object->worldPose.position==freePose.position);
        accepted(b,SetDirectMode{true}); accepted(b,StartReach{fixture(b.control()->binding()).target}); rejected(b,EnableSupport{});
        accepted(b,ResetBody{}); accepted(b,SetDirectMode{true});
        accepted(b,GraspObject{handle,ArmSide::Left}); rejected(b,EnableSupport{});
        // Complete the grasp and reject support while the object is Attached.
        for (unsigned t=0;t<2500 && b.snapshot().interaction.phase!=InteractionPhase::Holding;++t) b.step();
        require(b.snapshot().object->state==ObjectState::Attached,"Attachment conflict fixture did not attach"); rejected(b,EnableSupport{});
        BodyInstance plain(2,asset,nullptr,false); rejected(plain,EnableSupport{}); plain.step(); JULIA_CHECK(plain.snapshot().pose.globals.size()==asset->nodes().size());
        for (unsigned phase=0;phase<3;++phase) {
            BodyInstance interrupted(1,asset,&profile,false); initialize(interrupted);
            if (phase<2) { accepted(interrupted,MoveSupportRoot{{.005f,0,0}}); if (phase==1) while (interrupted.snapshot().support.phase==SupportPhase::Planning) interrupted.step(); }
            accepted(interrupted,ResetBody{}); JULIA_CHECK(!interrupted.snapshot().support.enabled && interrupted.snapshot().epoch==1);
            JULIA_CHECK(interrupted.previousSnapshot().tick==interrupted.snapshot().tick);
        }
        BodyInstance teleported(1,asset,&profile,false); initialize(teleported);
        accepted(teleported,MoveSupportRoot{{.005f,0,0}}); accepted(teleported,SetRoot{{{.1f,0,0},{1,0,0,0}}});
        JULIA_CHECK(!teleported.snapshot().support.enabled && teleported.previousSnapshot().tick==teleported.snapshot().tick);
        // A floor above the current sole is diagnosed, not repaired by teleportation.
        accepted(teleported,SetSupportSurface{{1,2,.01f,true}}); accepted(teleported,EnableSupport{}); rejected(teleported,AlignSupport{});
        BodyInstance currentPose(1,asset,&profile,false); accepted(currentPose,SetDirectMode{true});
        accepted(currentPose,SetCoordinate{JointId::Pelvis,2,.005f}); accepted(currentPose,SetCoordinate{JointId::LeftKnee,0,.01f});
        const auto nonzero=currentPose.snapshot().coordinates; accepted(currentPose,EnableSupport{}); JULIA_CHECK(currentPose.snapshot().coordinates==nonzero);
        accepted(currentPose,AlignSupport{}); settle(currentPose);
        JULIA_CHECK(currentPose.snapshot().support.kind==SupportKind::KinematicSupport && currentPose.control()->coordinate(JointId::Pelvis,2)==.005f);
        accepted(currentPose,ResetCoordinates{}); JULIA_CHECK(!currentPose.snapshot().support.enabled);
        report << "negatives=policy_invalid_inputs_busy_cancel_disable_surface_loss_versions_root_reset_epoch_reach_interaction_attached_free_world_PASS\n";
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        const auto run=[&](int fps) {
            BodyInstance b(1,asset,&profile,false); accepted(b,SetDirectMode{true}); app::SupportDemo demo; demo.start(b); std::vector<std::string> records; records.reserve(12000);
            const auto tick=[&] { demo.drive(b); b.step(); std::ostringstream s; app::supportTrace(s,b.snapshot()); s << ',' << hash(b.snapshot());
                for (const auto& r:b.results()) s << ',' << r.id << ',' << static_cast<int>(r.status) << ',' << r.reason;
                records.push_back(s.str()); };
            if (!fps) for (unsigned t=0;t<12000;++t) tick();
            else { runtime::FixedStepScheduler scheduler;
                for (int f=0;f<100*fps;++f) { scheduler.advance(1.0/fps,tick); const auto before=hash(b.snapshot()); (void)b.display(scheduler.alpha()); JULIA_CHECK(hash(b.snapshot())==before); }
            }
            require(demo.done(b.snapshot()),"Replay did not finish"); return records;
        };
        const auto reference=run(0);
        for (int fps:{30,60,144}) { const auto history=run(fps); JULIA_CHECK(history==reference); report << "replay_" << fps << "=" << history.size() << "_identical_ticks\n"; }
    }
}
int main(int argc,char** argv)
{
    try {
        const auto output=argc>1 ? std::filesystem::path{argv[1]}:std::filesystem::path{"build/diagnostics/stage6g/cpu"};
        std::filesystem::create_directories(output); std::ofstream report(output/"support.txt"); report << std::setprecision(12);
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx"); const auto profile=raphtaliaBodyProfile();
        const auto before=memory();
        profileAndIK(asset,profile,output,report); cycles(asset,profile,output,report); negatives(asset,profile,report); replay(asset,profile,report);
        report << "private_usage_before_after=" << before << ',' << memory() << " sizeof_snapshot=" << sizeof(BodySnapshot) << " sizeof_support=" << sizeof(SupportObservation) << '\n';
        report << "failures=" << JULIA_TEST_RESULT() << '\n'; return JULIA_TEST_RESULT();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

