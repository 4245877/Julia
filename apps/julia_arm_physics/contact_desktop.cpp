#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "ContactBench.hpp"
#include "ContactHud.hpp"
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include <glm/gtc/constants.hpp>

namespace
{
    using namespace julia::physics;
    using julia::render_opengl::DebugLineVertex;
    struct Input
    {
        PhysicsWorld* world=nullptr;
        julia::runtime::FixedStepScheduler* scheduler=nullptr;
        bool reset=false;
        unsigned camera=0;
        ContactSettings settings;
    };
    void key(GLFWwindow* window,int code,int,int action,int)
    {
        if(action!=GLFW_PRESS) return;
        auto& input=*static_cast<Input*>(glfwGetWindowUserPointer(window));
        if(code==GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window,GLFW_TRUE);
        else if(code==GLFW_KEY_SPACE) input.scheduler->setPaused(!input.scheduler->paused());
        else if(code==GLFW_KEY_R) input.reset=true;
        else if(code==GLFW_KEY_V) input.camera=(input.camera+1)%2;
        else if(code==GLFW_KEY_C) input.world->cancelContact();
        else if(code==GLFW_KEY_D) input.world->command(CommandKind::Disable);
        else if(code==GLFW_KEY_S) input.world->startContact(input.settings);
        else if(code==GLFW_KEY_UP||code==GLFW_KEY_DOWN)
            input.world->contactForceTarget(input.world->snapshot().contact.goalForce+(code==GLFW_KEY_UP?1:-1));
    }
    std::vector<unsigned char> pixels()
    {
        std::vector<unsigned char> p(960*720*3); glReadBuffer(GL_BACK); glPixelStorei(GL_PACK_ALIGNMENT,1);
        glReadPixels(0,0,960,720,GL_RGB,GL_UNSIGNED_BYTE,p.data()); return p;
    }
    void save(const std::filesystem::path& path,const std::vector<unsigned char>& p)
    {
        std::ofstream out{path,std::ios::binary}; out<<"P6\n960 720\n255\n";
        for(int y=719;y>=0;--y) out.write(reinterpret_cast<const char*>(p.data()+static_cast<std::size_t>(y)*960*3),960*3);
    }
    std::vector<DebugLineVertex> overlay(const Definition& def,const Snapshot& s)
    {
        std::vector<DebugLineVertex> v;
        const auto line=[&](glm::dvec3 a,glm::dvec3 b,glm::vec3 c) { v.push_back({glm::vec3{a},c}); v.push_back({glm::vec3{b},c}); };
        for(std::size_t i=0;i<def.links.size();++i)
        {
            const auto color=def.contactTool&&i==def.contactTool->link?glm::vec3{0,1,1}:glm::vec3{.2f,.5f,.5f};
            const auto& b=def.links[i]; const auto& f=s.worldFromJoint[i];
            const auto a=glm::dvec3{f*glm::dvec4{b.capsuleStart,1}},end=glm::dvec3{f*glm::dvec4{b.capsuleEnd,1}};
            line(a,end,color);
            for(int axis=0;axis<3;++axis) for(int k=0;k<32;++k)
            {
                const double t=glm::two_pi<double>()*k/32,next=glm::two_pi<double>()*(k+1)/32;
                glm::dvec3 u{0},w{0}; u[(axis+1)%3]=std::cos(t); u[(axis+2)%3]=std::sin(t);
                w[(axis+1)%3]=std::cos(next); w[(axis+2)%3]=std::sin(next);
                line(end+b.radius*u,end+b.radius*w,color);
            }
        }
        const auto base=glm::dvec3{s.worldFromJoint[0][3]};
        line(base+glm::dvec3{-.15,0,0},base+glm::dvec3{.15,0,0},{1,1,1}); line(base,base+glm::dvec3{0,.15,0},{1,1,1});
        if(def.wall)
        {
            std::array<glm::dvec3,8> p;
            for(unsigned i=0;i<8;++i) p[i]=def.wall->center+def.wall->halfSize*glm::dvec3{(i&1)?1.:-1.,(i&2)?1.:-1.,(i&4)?1.:-1.};
            for(unsigned i=0;i<8;++i) for(unsigned bit:{1u,2u,4u}) if(!(i&bit)) line(p[i],p[i|bit],{1,.4f,.2f});
        }
        const auto& c=s.contact; const auto p=c.pair.active?c.pair.pointWorld:c.toolPointWorld;
        line(p,p+c.normalWorld*.1,{1,1,0});
        if(c.pair.active) for(int k=0;k<3;++k) { glm::dvec3 d{0}; d[k]=.004; line(p-d,p+d,{1,1,0}); }
        line(p+glm::dvec3{.006,0,0},p+glm::dvec3{.006,0,0}+c.normalWorld*c.referenceForce*.015,{1,.2f,1});
        line(p,p+c.pair.forceOnToolWorld*.015,{1,0,0});
        line(p,p+c.pair.relativeTangentVelocityWorld*2.,{0,1,0});
        return v;
    }
    std::string deterministic(const Snapshot& s)
    { std::ostringstream o; contactCsvRow(o,s); driveCsvRow(o,s,8); return o.str(); }
    void callbacks(GLFWwindow* window,const ArmRig& rig,const contact_bench::Options& options)
    {
        auto callbackOptions=options; callbackOptions.scenario="cycle"; callbackOptions.feedback=true;
        PhysicsWorld w{contact_bench::description(rig,callbackOptions),contact_bench::initial(options.pose)}; contact_bench::reset(w,callbackOptions);
        julia::runtime::FixedStepScheduler scheduler; Input in{&w,&scheduler}; glfwSetWindowUserPointer(window,&in);
        in.settings=contact_bench::settings(callbackOptions);
        const auto q=w.snapshot().q;
        key(window,GLFW_KEY_SPACE,0,GLFW_PRESS,0); key(window,GLFW_KEY_R,0,GLFW_PRESS,0); key(window,GLFW_KEY_V,0,GLFW_PRESS,0);
        if(!scheduler.paused()||!in.reset||in.camera!=1||w.snapshot().q!=q) throw std::runtime_error("Contact UI pause/reset/view callbacks");
        key(window,GLFW_KEY_C,0,GLFW_PRESS,0);
        if(w.snapshot().contact.failure!=ContactFailure::Cancelled) throw std::runtime_error("Contact UI cancel callback");
        key(window,GLFW_KEY_D,0,GLFW_PRESS,0); w.step();
        for(unsigned k=0;k<6;++k) if(w.snapshot().appliedTorque[k]!=0) throw std::runtime_error("Contact UI disable callback");
        key(window,GLFW_KEY_S,0,GLFW_PRESS,0);
        if(w.snapshot().mode!=DriveMode::Contact) throw std::runtime_error("Contact UI start callback");
        for(unsigned k=0;k<1000&&w.snapshot().contact.phase!=ContactPhase::Hold;++k) w.step();
        if(w.snapshot().contact.phase!=ContactPhase::Hold) throw std::runtime_error("Contact UI force callback fixture did not reach Hold");
        const auto heldQ=w.snapshot().q;
        key(window,GLFW_KEY_UP,0,GLFW_PRESS,0);
        if(w.snapshot().contact.goalForce!=4||w.snapshot().q!=heldQ) throw std::runtime_error("Contact UI force increase callback");
        key(window,GLFW_KEY_DOWN,0,GLFW_PRESS,0);
        if(w.snapshot().contact.goalForce!=3||w.snapshot().q!=heldQ) throw std::runtime_error("Contact UI force decrease callback");
        key(window,GLFW_KEY_C,0,GLFW_PRESS,0);
        if(w.snapshot().contact.phase!=ContactPhase::Unload) throw std::runtime_error("Contact UI loaded cancel callback");
        glfwSetWindowUserPointer(window,nullptr);
    }
}
int main(int argc,char** argv)
{
    GLFWwindow* window=nullptr;
    try
    {
        const auto o=contact_bench::parse(argc,argv);
        if(!glfwInit()) throw std::runtime_error("GLFW init");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
        glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE); glfwWindowHint(GLFW_SRGB_CAPABLE,GLFW_TRUE);
        glfwWindowHint(GLFW_RESIZABLE,GLFW_FALSE); glfwWindowHint(GLFW_VISIBLE,o.verify?GLFW_FALSE:GLFW_TRUE);
        window=glfwCreateWindow(960,720,"Julia managed contact / FIXED ARM",nullptr,nullptr);
        if(!window) throw std::runtime_error("GLFW create window");
        glfwMakeContextCurrent(window); glfwSwapInterval(o.verify?0:1);
        if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) throw std::runtime_error("GLAD load");
        {
            const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_PHYSICS_ASSETS}/"models/Raphtalia_prepared.fbx");
            const ArmRig rig{*asset,o.side,ArmChain::Full};
            if(o.verify) callbacks(window,rig,o);
            auto model=julia::render_opengl::GLModelLoader::upload(asset);
            PhysicsWorld world{contact_bench::description(rig,o),contact_bench::initial(o.pose)}; verifyFramesBeforeDynamics(rig,world); contact_bench::reset(world,o);
            auto view=publish(rig,world.snapshot()); auto log=std::make_unique<contact_bench::Log>(o.output); contact_bench::metadata(o.output,rig,world,o);
            std::ofstream report{o.output/"gpu.txt"}; report<<"GPU="<<glGetString(GL_RENDERER)<<" GL="<<glGetString(GL_VERSION)<<'\n';
            julia::render_opengl::OpenGLRenderer renderer; renderer.initialize(960,720,JULIA_PHYSICS_ASSETS);
            julia::render_opengl::CameraConfig camera; camera.position={1.65f,1.45f,2.5f}; camera.target={.08f,.9f,.1f}; camera.orthographicHeight=1.85f;
            julia::runtime::FixedStepScheduler scheduler; Input input{&world,&scheduler};
            input.settings=contact_bench::settings(o);
            glfwSetWindowUserPointer(window,&input); glfwSetKeyCallback(window,key);
            const auto draw=[&]() {
                renderer.setCamera(camera); renderer.beginFrame(); renderer.renderModel(model,rig.binding().profile().bodyFromResource,view.actualPose.palette);
                renderer.renderDebugLines(overlay(world.definition(),view.physics)); renderer.renderDebugLines(contact_bench::hud(view.physics,camera,scheduler.paused()));
                if(glGetError()!=GL_NO_ERROR) throw std::runtime_error("Contact GL render error");
            };
            draw(); save(o.output/"frame_0.ppm",pixels());
            GLuint query=0; glGenQueries(1,&query); unsigned renders=0,captures=1,experiment=0; std::uint64_t nextCapture=240,lastCapture=0;
            double last=glfwGetTime(),gpuTotal=0;
            const auto limit=o.verify||o.explicitTicks?o.ticks:std::numeric_limits<std::uint64_t>::max();
            while(!glfwWindowShouldClose(window)&&world.snapshot().tick<limit)
            {
                glfwPollEvents();
                if(input.reset)
                {
                    contact_bench::reset(world,o); scheduler=julia::runtime::FixedStepScheduler{}; input.reset=false;
                    view=publish(rig,world.snapshot());
                    const auto path=o.output/("reset_"+std::to_string(++experiment)); log=std::make_unique<contact_bench::Log>(path); contact_bench::metadata(path,rig,world,o);
                }
                const auto now=glfwGetTime();
                scheduler.advance(o.verify?1./o.fps:now-last,[&] {
                    if(world.snapshot().tick>=limit) return;
                    contact_bench::step(world,o); view=publish(rig,world.snapshot()); log->row(view,world.definition());
                },glfwGetWindowAttrib(window,GLFW_ICONIFIED)!=0); last=now;
                if(input.camera==1) { camera.target=glm::vec3{view.physics.contact.toolPointWorld}; camera.position=camera.target+glm::vec3{.7f,.2f,.8f}; camera.orthographicHeight=.75f; }
                else { camera.position={1.65f,1.45f,2.5f}; camera.target={.08f,.9f,.1f}; camera.orthographicHeight=1.85f; }
                glBeginQuery(GL_TIME_ELAPSED,query); draw(); glEndQuery(GL_TIME_ELAPSED);
                GLuint64 ns=0; glGetQueryObjectui64v(query,GL_QUERY_RESULT,&ns); gpuTotal+=static_cast<double>(ns)/1000; ++renders;
                const auto tick=world.snapshot().tick;
                if(o.verify&&tick!=lastCapture&&(tick>=nextCapture||tick==limit))
                {
                    const auto before=deterministic(world.snapshot()); const auto image=pixels();
                    save(o.output/("frame_"+std::to_string(tick)+".ppm"),image); ++captures;
                    draw(); if(image!=pixels()||before!=deterministic(world.snapshot())) throw std::runtime_error("Contact repeated draw changed state/pixels");
                    const auto saved=camera; camera.target=glm::vec3{view.physics.contact.toolPointWorld}; camera.position=camera.target+glm::vec3{.6f,.2f,.8f}; camera.orthographicHeight=.55f;
                    draw(); save(o.output/("tool_"+std::to_string(tick)+".ppm"),pixels()); camera=saved;
                    lastCapture=tick; nextCapture+=240;
                }
                std::ostringstream title; title<<"Julia contact "<<(o.side==ArmSide::Left?"L ":"R ")<<contactPhaseName(view.physics.contact.phase)
                    <<" goal/ref/N "<<view.physics.contact.goalForce<<'/'<<view.physics.contact.referenceForce<<'/'<<view.physics.contact.pair.normalForce;
                glfwSetWindowTitle(window,title.str().c_str()); glfwSwapBuffers(window);
            }
            glDeleteQueries(1,&query);
            if(o.verify&&(world.snapshot().tick!=limit||world.snapshot().worldSteps!=limit*o.substeps||captures<5||renders!=(limit*o.fps+119)/120))
                throw std::runtime_error("Contact exact ticks/captures/frame count");
            report<<"fps="<<o.fps<<" ticks="<<world.snapshot().tick<<" renders="<<renders<<" captures="<<captures<<" GL_errors=0 callback_checks="<<(o.verify?"PASS":"not-run")
                <<" OS_input=not-tested GPU_mean_us="<<(renders?gpuTotal/renders:0)<<"\nPASS\n";
            std::cout<<"PASS contact desktop "<<contactPhaseName(world.snapshot().contact.phase)<<'\n';
        }
        glfwDestroyWindow(window); glfwTerminate(); return 0;
    }
    catch(const std::exception& e) { std::cerr<<"CONTACT DESKTOP FAILURE: "<<e.what()<<'\n'; if(window) glfwDestroyWindow(window); glfwTerminate(); return 1; }
}
