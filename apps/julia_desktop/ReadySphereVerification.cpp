#include <glad/glad.h>
#include "ReadySceneVerification.hpp"
#include "../common/ReadySphereDemo.hpp"
#include "../common/ReadySphereTrace.hpp"
#include <cmath>
#include <fstream>

namespace julia::desktop
{
    using namespace embodiment;

    namespace
    {
        const char* milestone(const BodySnapshot& s,const BodySnapshot& p)
        {
            if (s.idle.phase==IdlePhase::Ready && p.idle.phase!=IdlePhase::Ready) return "01-ready";
            if (s.interaction.phase==InteractionPhase::Align && p.interaction.frameError>.030f &&
                s.interaction.frameError<=.030f) return "02-approaching";
            if (s.interaction.phase==InteractionPhase::ReadyToGrasp && p.interaction.phase!=s.interaction.phase) return "02-open";
            if (s.interaction.phase==InteractionPhase::Closing && p.hands[0].closure<.5f && s.hands[0].closure>=.5f) return "03-closing";
            if (s.interaction.result=="Grasped" && p.interaction.result!="Grasped") return "04-grasped";
            if (s.interaction.result=="Moved" && p.interaction.result!="Moved") return "05-moved";
            if (s.interaction.phase==InteractionPhase::ReleaseOpening && p.object && p.object->state==ObjectState::Attached &&
                s.object && s.object->state==ObjectState::Free) return "06-detached";
            if (s.interaction.phase==InteractionPhase::ReleaseOpening && p.hands[0].closure>.5f && s.hands[0].closure<=.5f) return "06-opening";
            if (s.interaction.phase==InteractionPhase::Retreat && p.interaction.phase!=s.interaction.phase) return "07-retreat";
            if (s.interaction.result=="Released" && p.interaction.result!="Released") return "08-retreated";
            if (s.idle.phase==IdlePhase::Idle && p.idle.phase==IdlePhase::Resuming && s.tick>144) return "09-idle";
            return nullptr;
        }

        void checkSphereOcclusion(const CaptureSceneFrame& capture,const BodyDisplay& frame,
            const glm::vec3& angle,bool avatar,std::ostream& report)
        {
            const SceneCloseUp closeUp{frame.object->worldPose.position,angle};
            auto skin=frame;
            skin.object.reset();
            const auto hand=capture(skin,avatar,closeUp,1,true);
            const auto sphere=capture(frame,avatar,closeUp,1,false);
            const auto scene=capture(frame,avatar,closeUp,1,true);
            if (hand.rgba.size()!=sphere.rgba.size() || scene.rgba.size()!=sphere.rgba.size())
                throw std::runtime_error("Sphere depth check requires matching framebuffers");
            const auto same=[&](const SceneFrame& a,const SceneFrame& b,std::size_t offset) {
                for (std::size_t k=0;k<4;++k) if (std::abs(a.rgba[offset+k]-b.rgba[offset+k])>2) return false;
                return true;
            };
            std::size_t visible=0,covered=0;
            for (std::size_t i=0;i<sphere.rgba.size();i+=4) {
                const auto* pixel=sphere.rgba.data()+i;
                if (pixel[3]!=255 || pixel[0]<220 || pixel[0]-pixel[1]<70 || pixel[0]-pixel[2]<100) continue;
                if (same(sphere,hand,i)) continue;
                if (same(scene,sphere,i)) ++visible;
                if (same(scene,hand,i)) ++covered;
            }
            report << (avatar ? "avatar":"window") << "_sphere_pixels_visible=" << visible << " occluded=" << covered << '\n';
            if (!visible || !covered) throw std::runtime_error("Sphere wireframe did not respect hand occlusion");
        }
    }

    void verifyDesktopReadySphere(BodyInstance& body,const std::filesystem::path& output,
        std::string_view rate,const CaptureSceneFrame& capture)
    {
        std::filesystem::create_directories(output);
        std::ofstream trace(output/"ready-sphere.csv"), report(output/"desktop.txt");
        if (!trace || !report) throw std::runtime_error("Cannot open Ready sphere verification files");
        app::readySphereTraceHeader(trace);
        app::ReadySphereDemo demo;
        runtime::FixedStepScheduler scheduler;
        constexpr double varied[]{1./30,1./144,1./60,.011,.024};
        glm::vec3 front{0},side{0};
        bool framed=false;
        std::size_t frames=0,sampledAttached=0;
        float footPosition=0,footAngle=0,footGap=0,attachedOffset=0;
        unsigned captures=0;
        bool capturedTransport=false;
        report << std::setprecision(10) << "process=julia_desktop\nframe_intervals=" << rate
            << "\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        const auto shot=[&](std::string_view name,const BodyDisplay& d,bool avatar=false,
            std::optional<glm::vec3> angle={}) {
            std::optional<SceneCloseUp> closeUp;
            if (angle) closeUp=SceneCloseUp{d.object->worldPose.position,*angle};
            saveSceneFrame(output/(std::string{name}+".pam"),capture(d,avatar,closeUp,1,true));
            ++captures;
        };
        while (!demo.done() && !demo.failed() && body.snapshot().tick<app::ReadySphereDemo::TickLimit) {
            const double dt=rate=="30" ? 1./30:rate=="144" ? 1./144:varied[frames%std::size(varied)];
            scheduler.advance(dt,[&] {
                demo.drive(body);
                if (demo.done() || demo.failed()) return;
                body.step();
                const auto& s=body.snapshot();
                app::readySphereTrace(trace,s);
                if (!framed && s.object) {
                    // Use the free sphere's grip axes to choose the views, then keep
                    // those world directions for approach, transport and release.
                    const auto rotation=s.object->worldPose.rotation;
                    front=rotation*glm::vec3{.065f,.11f,.24f};
                    side=rotation*glm::vec3{-.225f,.050f,.150f};
                    framed=true;
                }
                if (rate=="mixed" && demo.completed()==0) {
                    if (const auto* name=milestone(s,body.previousSnapshot())) {
                        const auto d=body.display(1);
                        shot(name,d,false,s.object ? std::optional{front}:std::nullopt);
                        report << name << "_tick=" << s.tick << '\n';
                        const std::string_view stage=name;
                        if (stage=="02-approaching" || stage=="02-open" || stage=="03-closing" || stage=="04-grasped" ||
                            stage=="05-moved" || stage=="06-detached" || stage=="06-opening") {
                            shot(std::string{name}+"-side",d,false,side);
                            shot(std::string{name}+"-avatar",d,true,front);
                        }
                        if (stage=="04-grasped") {
                            checkSphereOcclusion(capture,d,side,false,report);
                            checkSphereOcclusion(capture,d,side,true,report);
                        }
                    }
                }
            });
            ++frames;
            if (!body.snapshot().tick) continue;
            const auto d=body.display(scheduler.alpha());
            for (const auto& a:d.support.anchors) {
                if (!a.bound || a.positionError>SupportPositionTolerance || a.angularError>SupportAngularTolerance ||
                    a.minGap < -SupportSurfaceTolerance || a.maxGap>SupportSurfaceTolerance)
                    throw std::runtime_error("Invalid support in interpolated desktop pose");
                footPosition=std::max(footPosition,a.positionError); footAngle=std::max(footAngle,a.angularError);
                footGap=std::max({footGap,-a.minGap,a.maxGap});
            }
            if (d.object && d.object->state==ObjectState::Attached) {
                const auto wrist=rigidMatrix({d.root.position,d.root.rotation})*
                    body.control()->binding().effectorFrame(d.object->gripEffector(ArmSide::Left),d.pose.globals);
                const auto center=glm::vec3{(wrist*d.object->effectorFromObject)[3]};
                const float residual=glm::length(d.object->worldPose.position-center);
                if (residual>1e-6f) throw std::runtime_error("Desktop sphere diverged from interpolated grip");
                attachedOffset=std::max(attachedOffset,residual);
                ++sampledAttached;
                const auto& before=body.previousSnapshot().object;
                const auto& actual=body.snapshot().object;
                if (!capturedTransport && d.interaction.phase==InteractionPhase::Transport && before && actual &&
                    glm::length(actual->worldPose.position-before->worldPose.position)>.0004f) {
                    shot("transport-frame",d,false,front);
                    report << "transport_frame=" << frames << " from_tick=" << d.fromTick << " to_tick=" << d.toTick
                        << " alpha=" << scheduler.alpha() << '\n';
                    capturedTransport=true;
                }
            }
        }
        const auto& s=body.snapshot();
        if (!demo.done()) throw std::runtime_error(demo.failed() ? demo.reason():"Ready sphere verification hit its tick limit");
        if (scheduler.diagnostics().overloadFrames || !sampledAttached || !capturedTransport || (rate=="mixed" && captures!=26))
            throw std::runtime_error("Incomplete Ready sphere desktop frame coverage");
        report << "cycles=" << demo.completed() << "\nticks=" << s.tick << "\nframes=" << frames
            << "\ncaptures=" << captures << "\nattached_frames=" << sampledAttached << "\nmax_attachment_error_m=" << attachedOffset
            << "\nmax_display_foot_error_m=" << footPosition << "\nmax_display_foot_angle_rad=" << footAngle
            << "\nmax_display_sole_gap_m=" << footGap << "\nGL_errors=0\nresult=PASS\n";
        if (!(trace << std::flush) || !(report << std::flush)) throw std::runtime_error("Cannot write Ready sphere verification files");
    }
}
