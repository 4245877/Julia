#include <glad/glad.h>
#include "ReadySceneVerification.hpp"
#include "../common/ReadyCubeTrace.hpp"
#include <fstream>

namespace julia::desktop
{
    using namespace embodiment;

    void saveSceneFrame(const std::filesystem::path& file,const SceneFrame& frame)
    {
        std::ofstream out(file,std::ios::binary);
        out << "P7\nWIDTH " << frame.width << "\nHEIGHT " << frame.height << "\nDEPTH 4\nMAXVAL 255\nTUPLTYPE RGB_ALPHA\nENDHDR\n";
        const auto stride=static_cast<std::size_t>(frame.width)*4;
        std::vector<unsigned char> row(stride);
        for (int y=frame.height-1;y>=0;--y) {
            std::copy_n(frame.rgba.data()+stride*y,stride,row.data());
            for (std::size_t x=0;x<stride;x+=4) if (row[x+3]>0 && row[x+3]<255)
                for (std::size_t channel=x;channel<x+3;++channel)
                    row[channel]=static_cast<unsigned char>(std::min(255,(row[channel]*255+row[x+3]/2)/row[x+3]));
            out.write(reinterpret_cast<const char*>(row.data()),static_cast<std::streamsize>(stride));
        }
        if (!out) throw std::runtime_error("Could not save scene frame");
    }

    namespace
    {
        bool samePixel(const SceneFrame& a,const SceneFrame& b,std::size_t offset)
        {
            for (std::size_t k=0;k<4;++k) if (std::abs(a.rgba[offset+k]-b.rgba[offset+k])>2) return false;
            return true;
        }

        void checkOcclusion(const CaptureSceneFrame& capture,const BodyDisplay& frame,bool avatar,std::ostream& report)
        {
            const auto scene=capture(frame,avatar,{},1,true);
            const auto character=capture(frame,avatar,{},0,true);
            const auto cube=capture(frame,avatar,{},1,false);
            if (scene.rgba.size()!=cube.rgba.size() || scene.rgba.size()!=character.rgba.size())
                throw std::runtime_error("Cube depth check requires matching framebuffers");
            std::size_t covered=0,visible=0;
            for (std::size_t k=0;k<cube.rgba.size();k+=4) {
                // Restrict to fully shaded cube interior samples and exclude
                // subpixel MSAA seams and the gray Full Window background.
                if (cube.rgba[k+3]!=255 || cube.rgba[k+1]-cube.rgba[k]<45) continue;
                if (samePixel(cube,character,k)) continue;
                if (samePixel(scene,character,k)) ++covered;
                if (samePixel(scene,cube,k)) ++visible;
            }
            if (!covered || !visible) throw std::runtime_error(avatar ? "Avatar cube occlusion absent":"Full Window cube occlusion absent");
            report << (avatar ? "avatar":"window") << "_cube_pixels_visible=" << visible << " occluded=" << covered << '\n';
        }
    }

    void verifyDesktopReadyCube(BodyInstance& body,const std::filesystem::path& output,
        std::string_view rate,const CaptureSceneFrame& capture)
    {
        std::filesystem::create_directories(output);
        std::ofstream trace(output/"ready-cube.csv"), report(output/"desktop.txt");
        if (!trace || !report) throw std::runtime_error("Cannot open cube verification files");
        report << std::setprecision(10) << "process=julia_desktop\nintervals=" << rate
            << "\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        app::readyCubeTraceHeader(trace);
        app::ReadyCubeDemo demo;
        body.setIdleView(app::readyCubeReplayView);
        runtime::FixedStepScheduler scheduler;
        constexpr double mixed[]{1./30,1./144,1./60,.011,.024};
        constexpr glm::vec3 front{.24f,.10f,.30f}, side{.35f,.12f,-.21f};
        std::size_t frames=0,attached=0,intermediate=0;
        bool appeared=false, movedFrame=false, faded=false;
        float maxFoot=0,maxSole=0,maxFootAngle=0,maxRelative=0;
        double maxRelativeAngle=0;
        const auto opacity=[&](const BodyDisplay& d) { return demo.opacity(d.time/runtime::BodyStepSeconds); };
        const auto shot=[&](std::string_view name,const BodyDisplay& d,bool avatar=false,
            std::optional<glm::vec3> angle={},std::optional<glm::vec3> target={}) {
            std::optional<SceneCloseUp> closeUp;
            if (angle) closeUp=SceneCloseUp{target ? *target:d.object->worldPose.position,*angle};
            saveSceneFrame(output/(std::string{name}+".pam"),capture(d,avatar,closeUp,opacity(d),true));
            report << name << " from_tick=" << d.fromTick << " to_tick=" << d.toTick
                << " display_tick=" << d.time/runtime::BodyStepSeconds << " opacity=" << opacity(d) << '\n';
        };

        while (!demo.done() && !demo.failed() && body.snapshot().tick<app::ReadyCubeDemo::TickLimit) {
            const double delta=rate=="30" ? 1./30:rate=="144" ? 1./144:mixed[frames%std::size(mixed)];
            scheduler.advance(delta,[&] {
                if (demo.done() || demo.failed()) return;
                demo.drive(body);
                if (demo.done() || demo.failed()) return;
                body.step();
                const auto& s=body.snapshot();
                app::readyCubeTrace(trace,s,demo);
                if (rate!="mixed" || demo.completed()) return;
                const auto& p=body.previousSnapshot();
                if (s.idle.phase==IdlePhase::Idle && p.idle.phase==IdlePhase::Resuming && !s.object) {
                    const bool returned=p.reaches[0].action!=0;
                    const auto d=body.display(1);
                    shot(returned ? "10-return-window":"01-idle-window",d);
                    if (returned) shot("10-return-avatar",d,true);
                    else {
                        const auto worldFromBody=rigidMatrix({d.root.position,d.root.rotation});
                        const auto& binding=body.control()->binding();
                        for (const auto hand:{EffectorId::LeftPalm,EffectorId::RightPalm}) {
                            const bool left=hand==EffectorId::LeftPalm;
                            const auto palm=worldFromBody*binding.effectorFrame(hand,d.pose.globals);
                            const auto target=glm::vec3{palm*glm::vec4{0,.020f,0,1}};
                            const auto angle=glm::vec3{palm*glm::vec4{left ? .10f:-.10f,.04f,.22f,0}};
                            shot(left ? "01-idle-left-hand":"01-idle-right-hand",d,false,angle,target);
                        }
                    }
                }
                if (s.interaction.phase==InteractionPhase::ReadyToGrasp && p.interaction.phase!=s.interaction.phase)
                    shot("03-open-closeup",body.display(1),false,front);
                if (s.interaction.result=="Grasped" && p.interaction.result!="Grasped") {
                    const auto d=body.display(1);
                    shot("04-grasp-window",d);
                    shot("04-grasp-avatar",d,true);
                    shot("04-grasp-closeup",d,false,front);
                    shot("04-grasp-side",d,false,side);
                    checkOcclusion(capture,d,false,report);
                    checkOcclusion(capture,d,true,report);
                }
                if (s.interaction.result=="Moved" && p.interaction.result!="Moved") {
                    const auto d=body.display(1);
                    shot("06-inspect-avatar",d,true);
                    shot("06-inspect-closeup",d,false,front);
                    shot("06-inspect-side",d,false,side);
                }
                if (s.interaction.phase==InteractionPhase::ReleaseOpening && p.hands[0].closure>.52f && s.hands[0].closure<=.52f) {
                    const auto d=body.display(1);
                    shot("07-release-closeup",d,false,front);
                    shot("07-release-side",d,false,side);
                    shot("07-release-avatar",d,true);
                }
                if (s.interaction.result=="Released" && p.interaction.result!="Released")
                    shot("09-separated-window",body.display(1));
            });
            ++frames;
            if (!body.snapshot().tick) continue;
            const auto d=body.display(scheduler.alpha());
            for (const auto& a:d.support.anchors) {
                if (!a.bound || a.positionError>SupportPositionTolerance || a.angularError>SupportAngularTolerance ||
                    a.minGap < -SupportSurfaceTolerance || a.maxGap>SupportSurfaceTolerance)
                    throw std::runtime_error("Interpolated cube scene violated support");
                maxFoot=std::max(maxFoot,a.positionError);
                maxFootAngle=std::max(maxFootAngle,a.angularError);
                maxSole=std::max({maxSole,-a.minGap,a.maxGap});
            }
            const float fade=opacity(d);
            if (rate=="mixed" && !demo.completed() && d.object) {
                if (!appeared && d.interaction.phase==InteractionPhase::Idle && fade>.40f && fade<.60f) {
                    appeared=true;
                    shot("02-appear-window",d);
                    shot("02-appear-avatar",d,true);
                    const auto isolated=capture(d,true,{},fade,false);
                    const auto fadedWindow=capture(d,false,{},fade,false);
                    const auto solidWindow=capture(d,false,{},1,false);
                    unsigned maxAlpha=0;
                    for (std::size_t k=3;k<isolated.rgba.size();k+=4)
                        maxAlpha=std::max(maxAlpha,static_cast<unsigned>(isolated.rgba[k]));
                    report << "avatar_fade_alpha=" << maxAlpha << "/255\n";
                    if (std::abs(static_cast<float>(maxAlpha)-fade*255)>2)
                        throw std::runtime_error("Avatar cube did not composite at scene opacity");
                    unsigned blended=0;
                    for (std::size_t k=0;k<solidWindow.rgba.size();k+=4)
                        if (solidWindow.rgba[k+1]-solidWindow.rgba[k]>45 && fadedWindow.rgba[k+1]>128 &&
                            fadedWindow.rgba[k+1]<solidWindow.rgba[k+1] && fadedWindow.rgba[k+3]==255) ++blended;
                    if (!blended) throw std::runtime_error("Full Window cube did not blend against its background");
                    report << "window_fade_blended_pixels=" << blended << '\n';
                }
                if (!faded && d.interaction.phase==InteractionPhase::Retreat && fade>.30f && fade<.55f) {
                    faded=true;
                    shot("08-fade-window",d);
                    shot("08-fade-avatar",d,true);
                    shot("08-fade-side",d,false,side);
                }
            }
            if (d.object && d.object->state==ObjectState::Attached) {
                const auto grip=rigidMatrix({d.root.position,d.root.rotation})*
                    body.control()->binding().effectorFrame(EffectorId::LeftGrip,d.pose.globals);
                const auto relative=rigidPose(glm::inverse(grip)*rigidMatrix(d.object->worldPose));
                const auto captured=rigidPose(d.object->effectorFromObject);
                maxRelative=std::max(maxRelative,glm::length(relative.position-captured.position));
                maxRelativeAngle=std::max(maxRelativeAngle,glm::length(orientationResidual(relative.rotation,captured.rotation)));
                ++attached;
                if (scheduler.alpha()>.001 && scheduler.alpha()<.999) ++intermediate;
                const bool sampleFrame=rate=="30" || (scheduler.alpha()>.15 && scheduler.alpha()<.95);
                if (!movedFrame && sampleFrame &&
                    d.interaction.phase==InteractionPhase::Transport && body.previousSnapshot().object && body.snapshot().object &&
                    glm::length(body.snapshot().object->worldPose.position-body.previousSnapshot().object->worldPose.position)>.0003f) {
                    shot(rate=="mixed" ? "05-transport-intertick":"transport-frame",d,false,front);
                    movedFrame=true;
                }
            }
        }
        if (!demo.done()) throw std::runtime_error(demo.failed() ? demo.reason():"Cube replay reached tick limit");
        if (scheduler.diagnostics().overloadFrames || !attached || !movedFrame || (rate!="30" && !intermediate) ||
            maxRelative>1e-6f || maxRelativeAngle>2e-6 || (rate=="mixed" && !(appeared && faded)))
            throw std::runtime_error("Incomplete or inconsistent cube replay");
        report << "cycles=" << demo.completed() << "\nticks=" << body.snapshot().tick << "\nframes=" << frames
            << "\nattached_frames=" << attached << "\nintermediate_attached_frames=" << intermediate
            << "\nmax_foot_error_m=" << maxFoot << "\nmax_sole_gap_m=" << maxSole << "\nmax_foot_angle_rad=" << maxFootAngle
            << "\nmax_grip_error_m=" << maxRelative << "\nmax_grip_error_rad=" << maxRelativeAngle << "\nGL_errors=0\nresult=PASS\n";
        if (!(trace << std::flush) || !(report << std::flush)) throw std::runtime_error("Cannot write cube verification files");
    }
}
