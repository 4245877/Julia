#include "julia/embodiment/BodyInstance.hpp"
#include "julia/model_import/ModelImporter.hpp"
#include "pose_test_support.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

using namespace julia;
using namespace julia::embodiment;
namespace
{
    template<class F> bool rejects(F&& fn) { try { fn(); } catch (const std::exception&) { return true; } return false; }
    std::size_t findClip(const model::ModelAsset& asset, std::string_view suffix)
    {
        for (std::size_t i=0;i<asset.animations().size();++i)
            if (asset.animations()[i].name.ends_with(suffix)) return i;
        throw std::runtime_error("Missing test clip");
    }
    float compare(const BodySnapshot& a,const BodySnapshot& b)
    {
        JULIA_CHECK(a.tick==b.tick && a.time==b.time && a.epoch==b.epoch);
        JULIA_CHECK(a.playback==b.playback && a.clipIndex==b.clipIndex && a.direct==b.direct && a.eyesEnabled==b.eyesEnabled);
        JULIA_CHECK(std::abs(a.clipTime-b.clipTime)<1e-12);
        JULIA_CHECK(a.coordinates==b.coordinates && a.root.position==b.root.position && a.root.rotation==b.root.rotation);
        JULIA_CHECK(a.eyes.gazeDegrees==b.eyes.gazeDegrees && a.eyes.blinkLeft==b.eyes.blinkLeft && a.eyes.blinkRight==b.eyes.blinkRight);
        const auto delta=std::max({pose_test::delta(a.pose.locals,b.pose.locals),pose_test::delta(a.pose.globals,b.pose.globals),
            pose_test::delta(a.pose.palette,b.pose.palette)});
        JULIA_CHECK(delta<=1e-6f);
        return delta;
    }
    void stream(BodyInstance& body)
    {
        std::vector<BodyCommand> commands;
        std::uint64_t id=1;
        const auto add=[&](std::uint64_t tick,BodyOperation op) { commands.push_back({id++,body.id(),tick,0,std::move(op)}); };
        add(1,ClipCommand{ClipOperation::Select,0}); add(1,ClipCommand{ClipOperation::Play});
        add(20,EyeCommand{EyeOperation::Blink,{1,1,0}});
        add(30,EyeCommand{EyeOperation::Gaze,{8,5,0}});
        add(40,ClipCommand{ClipOperation::Pause}); add(60,ClipCommand{ClipOperation::Seek,.75});
        add(80,SetDirectMode{true}); add(81,SetCoordinate{JointId::LeftElbow,0,.3f});
        add(95,SetCoordinate{JointId::LeftElbow,0,.2f,LimitPolicy::Clamp,true});
        add(120,SetDirectMode{false}); add(121,ClipCommand{ClipOperation::Play});
        add(160,EyeCommand{EyeOperation::Target,{1,4,9}}); add(220,EyeCommand{EyeOperation::ClearTarget});
        add(250,ClipCommand{ClipOperation::Select,static_cast<double>(findClip(body.asset(),"TEST_blink_natural"))});
        add(250,ClipCommand{ClipOperation::Play}); add(290,SetDirectMode{true});
        add(291,EyeCommand{EyeOperation::ManualBlink,{.3f,.7f,0}}); add(330,SetDirectMode{false});
        add(400,SetRoot{{{1,.2f,-2},glm::angleAxis(.3f,glm::vec3{0,1,0})}});
        add(420,ResetCoordinates{}); add(500,EyeCommand{EyeOperation::Natural});
        add(600,ClipCommand{ClipOperation::Stop}); add(700,EyeCommand{EyeOperation::ResetSeed});
        add(1100,ResetBody{}); add(1150,SetDirectMode{true}); // Cancelled by reset.
        // Deliberately reverse arrival order, including commands sharing a tick.
        for (auto it=commands.rbegin();it!=commands.rend();++it)
            JULIA_CHECK(body.submit(*it).status==BodyCommandStatus::Queued);
    }
    void replay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile,std::ostream& report)
    {
        constexpr std::size_t ticks=1200;
        BodyInstance exact(1,asset,&profile); stream(exact);
        runtime::FixedStepScheduler scheduler;
        std::vector<BodySnapshot> reference; reference.reserve(ticks);
        std::vector<std::vector<BodyCommandResult>> outcomes;
        scheduler.runTicks(ticks,[&] { exact.step(); reference.push_back(exact.snapshot()); outcomes.push_back(exact.results()); });
        JULIA_CHECK(exact.snapshot().tick==ticks && exact.snapshot().time==10);
        report << "headless_ticks=" << ticks << "\nheadless_time=" << exact.snapshot().time << '\n';
        for (int fps : {30,60,144,0})
        {
            BodyInstance body(1,asset,&profile); stream(body);
            runtime::FixedStepScheduler frames;
            float maximum=0; std::size_t frame=0; double elapsed=0;
            while (elapsed<10-1e-12)
            {
                constexpr double irregular[]{.003,.019,.041,.007,.013,.027};
                const double dt=std::min(10-elapsed,fps ? 1.0/fps:irregular[frame%6]);
                frames.advance(dt,[&] {
                    body.step(); maximum=std::max(maximum,compare(body.snapshot(),reference[static_cast<std::size_t>(body.snapshot().tick-1)]));
                    const auto& expected=outcomes[static_cast<std::size_t>(body.snapshot().tick-1)];
                    JULIA_CHECK(body.results().size()==expected.size());
                    for (std::size_t k=0;k<std::min(body.results().size(),expected.size());++k)
                        JULIA_CHECK(body.results()[k].id==expected[k].id && body.results()[k].tick==expected[k].tick &&
                            body.results()[k].status==expected[k].status && body.results()[k].reason==expected[k].reason &&
                            body.results()[k].epoch==expected[k].epoch);
                });
                elapsed+=dt; ++frame;
                const auto display=body.display(frames.alpha());
                const auto again=body.display(frames.alpha());
                JULIA_CHECK(display.pose.palette==again.pose.palette && display.time==again.time);
            }
            JULIA_CHECK(body.snapshot().tick==ticks && frames.diagnostics().discardedSeconds==0);
            report << "fps=" << fps << " frames=" << frame << " compared_ticks=" << body.snapshot().tick << " max_pose_delta=" << maximum << '\n';
        }
    }
    void commands(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile)
    {
        BodyInstance body(7,asset,&profile),other(8,asset,&profile);
        JULIA_CHECK(body.submit({1,8,1,0,ResetBody{}}).status==BodyCommandStatus::Rejected);
        JULIA_CHECK(body.submit({1,7,0,0,ResetBody{}}).status==BodyCommandStatus::Rejected);
        JULIA_CHECK(body.submit({1,7,1,9,ResetBody{}}).status==BodyCommandStatus::Rejected);
        JULIA_CHECK(body.submit({0,7,1,0,ResetBody{}}).status==BodyCommandStatus::Rejected);
        body.submit({4,7,1,0,SetCoordinate{JointId::LeftElbow,0,.2f}});
        body.submit({2,7,1,0,SetCoordinate{JointId::LeftElbow,0,.1f}});
        JULIA_CHECK(body.submit({2,7,1,0,ResetBody{}}).status==BodyCommandStatus::Rejected);
        body.submit({3,7,1,0,SetDirectMode{true}});
        JULIA_CHECK(body.control()->coordinate(JointId::LeftElbow,0)==0);
        body.step();
        JULIA_CHECK(body.results().size()==3 && body.results()[0].id==2 && body.results()[1].id==3 && body.results()[2].id==4);
        JULIA_CHECK(body.control()->coordinate(JointId::LeftElbow,0)==.2f);
        JULIA_CHECK(other.snapshot().tick==0 && other.control()->coordinate(JointId::LeftElbow,0)==0);
        JULIA_CHECK(body.submit({50,7,1,0,ResetBody{}}).status==BodyCommandStatus::Rejected);
        body.enqueue(SetCoordinate{JointId::LeftElbow,0,100,LimitPolicy::Clamp}); body.step();
        JULIA_CHECK(body.results().back().status==BodyCommandStatus::Clamped);
        const auto q=body.control()->coordinate(JointId::LeftElbow,0);
        body.enqueue(SetCoordinate{JointId::LeftElbow,0,100});
        body.enqueue(SetCoordinate{JointId::LeftElbow,0,std::numeric_limits<float>::quiet_NaN()});
        body.enqueue(SetCoordinate{static_cast<JointId>(999),0,0});
        body.enqueue(SetCoordinate{JointId::LeftElbow,99,0});
        body.enqueue(ClipCommand{ClipOperation::Select,-1});
        body.enqueue(ClipCommand{ClipOperation::Seek,std::numeric_limits<double>::infinity()});
        body.enqueue(EyeCommand{static_cast<EyeOperation>(999)});
        body.enqueue(SetRoot{{{0,0,0},{0,0,0,0}}});
        body.step();
        for (const auto& result:body.results()) JULIA_CHECK(result.status==BodyCommandStatus::Rejected);
        JULIA_CHECK(body.control()->coordinate(JointId::LeftElbow,0)==q);
        const auto next=body.snapshot().tick+1;
        const auto reset=body.enqueue(ResetBody{});
        body.enqueue(SetCoordinate{JointId::LeftElbow,0,.5f});
        body.submit({999,7,next+10,0,SetDirectMode{true}});
        body.step();
        JULIA_CHECK(body.results().size()==3 && body.results()[0].id==reset.id);
        JULIA_CHECK(body.results()[1].status==BodyCommandStatus::Cancelled && body.results()[2].status==BodyCommandStatus::Cancelled);
        JULIA_CHECK(body.pendingCommands()==0 && body.snapshot().epoch==1 && !body.control()->enabled());
        JULIA_CHECK(body.previousSnapshot().tick==body.snapshot().tick && body.player().timeSeconds()==0);
        JULIA_CHECK(body.submit({1,7,next+1,0,SetDirectMode{true}}).status==BodyCommandStatus::Rejected);
        JULIA_CHECK(body.enqueue(SetDirectMode{true}).status==BodyCommandStatus::Queued); body.step();
        JULIA_CHECK(body.control()->enabled());
        // Reset PRNG/overrides/target/settings and compare a full future against a fresh instance.
        body.enqueue(ResetBody{}); body.step();
        BodyInstance fresh(7,asset,&profile); fresh.step();
        for (int i=0;i<600;++i)
        {
            body.step(); fresh.step();
            JULIA_CHECK(body.snapshot().pose.palette==fresh.snapshot().pose.palette);
            JULIA_CHECK(body.eyes().behavior().stats().blinkCount==fresh.eyes().behavior().stats().blinkCount);
        }
    }
    void clocks(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile)
    {
        BodyInstance body(1,asset,&profile);
        body.enqueue(ClipCommand{ClipOperation::Select,static_cast<double>(findClip(*asset,"TEST_head_torso"))});
        body.enqueue(ClipCommand{ClipOperation::Play}); body.step();
        body.enqueue(ClipCommand{ClipOperation::Pause}); body.enqueue(EyeCommand{EyeOperation::Gaze,{8,0,0}}); body.step();
        const double time=body.player().timeSeconds(); const auto eyes=body.snapshot().eyes.gazeDegrees;
        for (int i=0;i<30;++i) body.step();
        JULIA_CHECK(body.player().timeSeconds()==time && body.snapshot().eyes.gazeDegrees!=eyes);
        const auto before=body.snapshot().tick;
        body.enqueue(ClipCommand{ClipOperation::Seek,10000}); body.step();
        JULIA_CHECK(body.player().timeSeconds()==body.player().clip()->durationSeconds() && body.snapshot().tick==before+1);
        JULIA_CHECK(body.previousSnapshot().tick==body.snapshot().tick);
        body.enqueue(ClipCommand{ClipOperation::Seek,body.player().clip()->durationSeconds()-runtime::BodyStepSeconds*.5});
        body.enqueue(ClipCommand{ClipOperation::Play}); body.step();
        JULIA_CHECK(body.previousSnapshot().tick==body.snapshot().tick); // Loop seam discards display history.
        body.enqueue(ClipCommand{ClipOperation::Pause});
        body.enqueue(ClipCommand{ClipOperation::Seek,-100}); body.step(); JULIA_CHECK(body.player().timeSeconds()==0);
        runtime::FixedStepScheduler scheduler; scheduler.setPaused(true);
        body.enqueue(SetDirectMode{true}); const auto paused=body.snapshot().tick;
        scheduler.advance(1,[&] { body.step(); });
        JULIA_CHECK(body.snapshot().tick==paused && body.pendingCommands()==1 && scheduler.diagnostics().pausedSeconds==1);
        JULIA_CHECK(rejects([&] { scheduler.runTicks(1,[&] { body.step(); }); }));
        scheduler.setPaused(false); scheduler.advance(runtime::BodyStepSeconds,[&] { body.step(); });
        JULIA_CHECK(body.snapshot().tick==paused+1 && body.control()->enabled());
        // Bounded overload retains no whole-tick backlog; long OS gaps/minimize freeze.
        const auto n=body.snapshot().tick;
        JULIA_CHECK(scheduler.advance(.2,[&] { body.step(); })==8);
        JULIA_CHECK(body.snapshot().tick==n+8 && scheduler.diagnostics().overloadFrames==1);
        JULIA_CHECK(std::abs(scheduler.diagnostics().discardedSeconds-16*runtime::BodyStepSeconds)<1e-12);
        JULIA_CHECK(scheduler.advance(0,[&] { body.step(); })==0);
        scheduler.advance(30,[&] { body.step(); }); scheduler.advance(.03,[&] { body.step(); },true);
        JULIA_CHECK(body.snapshot().tick==n+8 && scheduler.diagnostics().suspendedFrames==2 && scheduler.alpha()==0);
        scheduler.runTicks(100,[&] { body.step(); }); JULIA_CHECK(body.snapshot().tick==n+108);
        JULIA_CHECK(rejects([&] { scheduler.advance(-1,[] {}); }));
        JULIA_CHECK(rejects([&] { scheduler.advance(std::numeric_limits<double>::quiet_NaN(),[] {}); }));
        JULIA_CHECK(rejects([&] { (void)body.display(-.1); }));
        runtime::FixedStepScheduler partial;
        partial.advance(runtime::BodyStepSeconds*.5,[] {});
        partial.setPaused(false); JULIA_CHECK(partial.alpha()==.5); // Idempotent setter preserves time.
        partial.setPaused(true); partial.setPaused(true);
        JULIA_CHECK(partial.alpha()==0 && partial.diagnostics().pausedSeconds==runtime::BodyStepSeconds*.5);
    }
    void eyesAndDisplay(const std::shared_ptr<const model::ModelAsset>& asset,const BodyProfile& profile)
    {
        BodyInstance body(1,asset,&profile);
        body.enqueue(ClipCommand{ClipOperation::Select,static_cast<double>(findClip(*asset,"TEST_blink_natural"))});
        body.enqueue(ClipCommand{ClipOperation::Play}); body.enqueue(EyeCommand{EyeOperation::ManualBlink,{1,1,0}}); body.step();
        AnimationPlayer reference(*asset); reference.selectClip(body.player().clipIndex()); reference.play(); reference.update(runtime::BodyStepSeconds);
        JULIA_CHECK(body.snapshot().pose.palette==std::vector<glm::mat4>(reference.palette().begin(),reference.palette().end()));
        body.enqueue(SetDirectMode{true}); body.step();
        JULIA_CHECK(pose_test::delta(body.snapshot().pose.palette,reference.palette())>1e-4f);
        // Autonomous clocks keep running beneath an authored overlay, whose palette wins.
        BodyInstance hidden(2,asset,&profile);
        hidden.enqueue(ClipCommand{ClipOperation::Select,static_cast<double>(findClip(*asset,"TEST_blink_natural"))});
        hidden.enqueue(ClipCommand{ClipOperation::Play}); hidden.enqueue(EyeCommand{EyeOperation::Gaze,{8,0,0}}); hidden.step();
        for (int i=0;i<30;++i) hidden.step();
        JULIA_CHECK(hidden.snapshot().eyes.gazeDegrees.x>7);
        AnimationPlayer authored(*asset); authored.selectClip(hidden.player().clipIndex()); authored.seek(hidden.player().timeSeconds());
        JULIA_CHECK(pose_test::delta(hidden.snapshot().pose.palette,authored.palette())==0);
        body.enqueue(EyeCommand{EyeOperation::Enable,{0,0,0}}); body.step();
        JULIA_CHECK(body.snapshot().pose.palette==asset->skinningMatrices());
        body.enqueue(SetCoordinate{JointId::LeftElbow,0,.6f}); body.step();
        const auto display=body.display(.5);
        BodyRigBinding binding(profile,*asset); DirectBodyControl half(binding); half.setEnabled(true);
        half.setCoordinate(JointId::LeftElbow,0,.3f); animation::Pose expected(asset->skeleton()); half.apply(expected.locals);
        animation::computeGlobalTransforms(asset->skeleton(),expected.locals,expected.globals);
        animation::buildSkinningPalette(asset->skeleton(),expected.globals,expected.palette);
        JULIA_CHECK(pose_test::delta(display.pose.palette,expected.palette)<1e-6f);
        JULIA_CHECK(display.time==(body.previousSnapshot().time+body.snapshot().time)*.5);
        body.enqueue(EyeCommand{EyeOperation::Natural}); body.enqueue(EyeCommand{EyeOperation::Target,{1,4,8}}); body.step(); body.step();
        const auto saved=body.snapshot(); const auto stats=body.eyes().behavior().stats().blinkCount;
        for (int i=0;i<50;++i) (void)body.display(static_cast<double>(i)/50);
        JULIA_CHECK(compare(saved,body.snapshot())==0 && body.eyes().behavior().stats().blinkCount==stats);
        const RootPose root{{1,2,3},glm::angleAxis(.7f,glm::vec3{0,1,0})};
        body.enqueue(SetRoot{root}); body.step();
        JULIA_CHECK(body.previousSnapshot().tick==body.snapshot().tick && body.display(0).root.position==root.position);
        const auto metric=profile.bodyFromResource*body.resourceRoot(root);
        const auto wanted=glm::translate(glm::mat4{1},root.position)*glm::mat4_cast(root.rotation)*profile.bodyFromResource;
        JULIA_CHECK(pose_test::delta(std::span{&metric,1},std::span{&wanted,1})<1e-5f);
    }
    void affineDisplay()
    {
        auto asset=std::make_shared<model::ModelAsset>();
        auto root=glm::scale(glm::mat4{1},glm::vec3{-2,3,4}); root[1][0]=.7f;
        asset->addNode({"root",animation::NoNode,root});
        const animation::NodeTRS rest{{0,2,0},{1,0,0,0},{1,1,1}};
        asset->addNode({"joint",0,glm::translate(glm::mat4{1},rest.translation),glm::mat4{1},rest});
        asset->addNode({"tip",1,glm::translate(glm::mat4{1},glm::vec3{0,2,0})});
        asset->addBone({"tip",glm::inverse(asset->nodes()[2].globalBindTransform),2});
        animation::AnimationClip clip; clip.durationTicks=1; clip.ticksPerSecond=1;
        animation::AnimationChannel c; c.nodeIndex=1; c.translations={{0,{0,2,0}},{1,{1,2,0}}};
        c.rotations={{0,{1,0,0,0}},{1,glm::angleAxis(1.0f,glm::vec3{0,0,1})}};
        clip.channels.push_back(c); clip.channels.push_back({0}); asset->addAnimation(clip);
        BodyProfile profile; profile.name="affine"; profile.rigFingerprint=bodyRigFingerprint(*asset);
        const auto pivot=glm::vec3{asset->nodes()[1].globalBindTransform[3]};
        profile.joints.push_back({JointId::LeftElbow,SegmentId::LeftForearm,"hinge",{"joint","root"},
            glm::translate(glm::mat4{1},pivot),{{"q","positive Z",{0,0,1},-1,1}},{"tip","joint"}});
        BodyInstance body(1,asset,&profile,false);
        body.enqueue(ClipCommand{ClipOperation::Play}); body.step(); body.step();
        const auto display=body.display(.5); animation::Pose expected(asset->skeleton());
        animation::sampleAnimation(asset->skeleton(),asset->animations()[0],1.5*runtime::BodyStepSeconds,expected.locals);
        JULIA_CHECK(pose_test::delta(display.pose.locals,expected.locals)<1e-6f);
        JULIA_CHECK(display.pose.locals[0]==root); // Exact constant shear/reflection.
        body.enqueue(SetDirectMode{true}); body.step(); body.enqueue(SetCoordinate{JointId::LeftElbow,0,.6f}); body.step();
        const auto direct=body.display(.5);
        BodyRigBinding binding(profile,*asset); DirectBodyControl half(binding); half.setEnabled(true);
        half.setCoordinate(JointId::LeftElbow,0,.3f); half.apply(expected.locals);
        JULIA_CHECK(pose_test::delta(direct.pose.locals,expected.locals)<1e-6f && direct.pose.locals[0]==root);
    }
}
int main(int argc,char** argv)
{
    try
    {
        const auto output=argc>1 ? std::filesystem::path{argv[1]}:std::filesystem::path{"build/diagnostics/stage4/cpu"};
        std::filesystem::create_directories(output); std::ofstream report(output/"body_instance.txt");
        const auto asset=model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_TEST_ASSET_DIR}/"models/Raphtalia_prepared.fbx");
        const auto profile=raphtaliaBodyProfile();
        replay(asset,profile,report); commands(asset,profile); clocks(asset,profile); eyesAndDisplay(asset,profile); affineDisplay();
        report << "commands,epochs,instances,clocks,authored_eyes,direct,reset,suspend,overload,pure_display,affine=checked\n"
            << "failures=" << JULIA_TEST_RESULT() << '\n';
        std::cout << "BodyInstance: 1200 exact ticks; 30/60/144/irregular replay; commands, clocks, eyes and snapshots checked\n";
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return JULIA_TEST_RESULT();
}
