#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "BodyDiagnostics.hpp"
#include "../common/ReadySphereDemo.hpp"
#include "../common/ReadySphereTrace.hpp"
#include <cmath>
#include <fstream>

namespace julia::desktop
{
    using namespace embodiment;

    namespace
    {
        void captureSphere(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
            const render_opengl::GLModel& model,const BodyInstance& body,BodyDiagnostics& diagnostics,
            const glm::mat4& fit,const BodyDisplay& display,const std::filesystem::path& output,
            const glm::vec3& angle,bool close)
        {
            const auto fromWorld=fit*glm::inverse(body.control()->binding().profile().bodyFromResource);
            const auto target=close ? display.object->worldPose.position:glm::vec3{0,.82f,0};
            render_opengl::CameraConfig camera;
            camera.target=glm::vec3{fromWorld*glm::vec4{target,1}};
            camera.position=glm::vec3{fromWorld*glm::vec4{target+angle,1}};
            camera.nearPlane=.004f;
            renderer.setCamera(camera);
            diagnostics.overlay=!close;
            int width=0,height=0;
            glfwGetFramebufferSize(window,&width,&height);
            if (width<=0 || height<=0) throw std::runtime_error("Ready sphere capture requires a framebuffer");
            renderer.resize(width,height);
            const auto transform=fit*body.resourceRoot(display.root);
            renderer.beginFrame();
            renderer.renderModel(model,transform,display.pose.palette);
            diagnostics.draw(renderer,transform,display);
            const auto stride=static_cast<std::size_t>(width)*3;
            std::vector<unsigned char> pixels(stride*height);
            glPixelStorei(GL_PACK_ALIGNMENT,1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output,std::ios::binary);
            image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y)
                image.write(reinterpret_cast<const char*>(pixels.data()+stride*y),static_cast<std::streamsize>(stride));
            if (!image || glGetError()!=GL_NO_ERROR) throw std::runtime_error("Ready sphere GPU capture failed");
            glfwSwapBuffers(window);
        }

        const char* milestone(const BodySnapshot& s,const BodySnapshot& p)
        {
            if (s.idle.phase==IdlePhase::Ready && p.idle.phase!=IdlePhase::Ready) return "01-ready";
            if (s.interaction.phase==InteractionPhase::ReadyToGrasp && p.interaction.phase!=s.interaction.phase) return "02-open";
            if (s.interaction.phase==InteractionPhase::Closing && p.hands[0].closure<.5f && s.hands[0].closure>=.5f) return "03-closing";
            if (s.interaction.result=="Grasped" && p.interaction.result!="Grasped") return "04-grasped";
            if (s.interaction.result=="Moved" && p.interaction.result!="Moved") return "05-moved";
            if (s.interaction.phase==InteractionPhase::ReleaseOpening && p.hands[0].closure>.5f && s.hands[0].closure<=.5f) return "06-opening";
            if (s.interaction.phase==InteractionPhase::Retreat && p.interaction.phase!=s.interaction.phase) return "07-detached";
            if (s.interaction.result=="Released" && p.interaction.result!="Released") return "08-retreated";
            if (s.idle.phase==IdlePhase::Idle && p.idle.phase==IdlePhase::Resuming && s.tick>144) return "09-idle";
            return nullptr;
        }
    }

    void verifyDesktopReadySphere(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,std::string_view rate)
    {
        std::filesystem::create_directories(output);
        std::ofstream trace(output/"ready-sphere.csv"), report(output/"desktop.txt");
        if (!trace || !report) throw std::runtime_error("Cannot open Ready sphere verification files");
        app::readySphereTraceHeader(trace);
        app::ReadySphereDemo demo;
        runtime::FixedStepScheduler scheduler;
        constexpr double varied[]{1./30,1./144,1./60,.011,.024};
        std::size_t frames=0,sampledAttached=0;
        float footPosition=0,footAngle=0,footGap=0,attachedOffset=0;
        unsigned captures=0;
        bool capturedTransport=false;
        report << std::setprecision(10) << "process=julia_desktop\nframe_intervals=" << rate
            << "\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        while (!demo.done() && !demo.failed() && body.snapshot().tick<app::ReadySphereDemo::TickLimit) {
            const double dt=rate=="30" ? 1./30:rate=="144" ? 1./144:varied[frames%std::size(varied)];
            scheduler.advance(dt,[&] {
                demo.drive(body);
                if (demo.done() || demo.failed()) return;
                body.step();
                const auto& s=body.snapshot();
                app::readySphereTrace(trace,s);
                if (rate=="mixed" && demo.completed()==0) {
                    if (const auto* name=milestone(s,body.previousSnapshot())) {
                        const bool close=s.object.has_value();
                        const auto file=output/(std::string{name}+".ppm");
                        captureSphere(window,renderer,model,body,diagnostics,fit,body.display(1),file,
                            close ? glm::vec3{.29f,.14f,.33f}:glm::vec3{.1f,.2f,2.55f},close);
                        report << name << "_tick=" << s.tick << '\n';
                        ++captures;
                        if (std::string_view{name}=="04-grasped" || std::string_view{name}=="07-detached") {
                            captureSphere(window,renderer,model,body,diagnostics,fit,body.display(1),
                                output/(std::string{name}+"-side.ppm"),{.40f,.08f,-.22f},true);
                            ++captures;
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
                    body.control()->binding().effectorFrame(EffectorId::LeftGrip,d.pose.globals);
                const auto center=glm::vec3{(wrist*d.object->effectorFromObject)[3]};
                const float residual=glm::length(d.object->worldPose.position-center);
                if (residual>1e-6f) throw std::runtime_error("Desktop sphere diverged from interpolated grip");
                attachedOffset=std::max(attachedOffset,residual);
                ++sampledAttached;
                const auto& before=body.previousSnapshot().object;
                const auto& actual=body.snapshot().object;
                if (!capturedTransport && d.interaction.phase==InteractionPhase::Transport && before && actual &&
                    glm::length(actual->worldPose.position-before->worldPose.position)>.0004f) {
                    captureSphere(window,renderer,model,body,diagnostics,fit,d,output/"transport-frame.ppm",{.30f,.14f,.34f},true);
                    report << "transport_frame=" << frames << " from_tick=" << d.fromTick << " to_tick=" << d.toTick
                        << " alpha=" << scheduler.alpha() << '\n';
                    capturedTransport=true;
                    ++captures;
                }
            }
            if (frames%60==0) glfwPollEvents();
        }
        const auto& s=body.snapshot();
        if (!demo.done()) throw std::runtime_error(demo.failed() ? demo.reason():"Ready sphere verification hit its tick limit");
        if (scheduler.diagnostics().overloadFrames || !sampledAttached || !capturedTransport || (rate=="mixed" && captures!=12))
            throw std::runtime_error("Incomplete Ready sphere desktop frame coverage");
        report << "cycles=" << demo.completed() << "\nticks=" << s.tick << "\nframes=" << frames
            << "\ncaptures=" << captures << "\nattached_frames=" << sampledAttached << "\nmax_attachment_error_m=" << attachedOffset
            << "\nmax_display_foot_error_m=" << footPosition << "\nmax_display_foot_angle_rad=" << footAngle
            << "\nmax_display_sole_gap_m=" << footGap << "\nGL_errors=0\nresult=PASS\n";
        if (!(trace << std::flush) || !(report << std::flush)) throw std::runtime_error("Cannot write Ready sphere verification files");
    }
}
