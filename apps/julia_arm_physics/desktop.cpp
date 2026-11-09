#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "BenchIO.hpp"
#include "BenchHud.hpp"
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include <algorithm>
#include <glm/gtc/constants.hpp>
#include <chrono>
#include <sstream>

namespace
{
    using namespace julia::physics;
    using julia::render_opengl::DebugLineVertex;
    constexpr int Width=960, Height=720;
    using Pixels=std::vector<unsigned char>;
    using Clock=std::chrono::steady_clock;
    struct Input
    {
        PhysicsWorld* world=nullptr;
        julia::runtime::FixedStepScheduler* scheduler=nullptr;
        BenchCase scenario=BenchCase::Hold;
        bool reset=false;
        unsigned selected=0, camera=0;
    };
    void key(GLFWwindow* window,int code,int,int action,int)
    {
        if(action!=GLFW_PRESS) return;
        auto& input=*static_cast<Input*>(glfwGetWindowUserPointer(window));
        if(code==GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window,GLFW_TRUE);
        else if(code==GLFW_KEY_SPACE) input.scheduler->setPaused(!input.scheduler->paused());
        else if(code==GLFW_KEY_R) input.reset=true;
        else if(code==GLFW_KEY_TAB) input.selected=(input.selected+1)%input.world->snapshot().dofs;
        else if(code==GLFW_KEY_V) input.camera=(input.camera+1)%3;
        else if(code>=GLFW_KEY_1&&code<=GLFW_KEY_7) { input.scenario=static_cast<BenchCase>(code-GLFW_KEY_1); input.reset=true; }
        else if(code==GLFW_KEY_D) input.world->command(CommandKind::Disable);
        else if(code==GLFW_KEY_I) input.world->applyWrench(2,{0,0,-30});
        else if(code==GLFW_KEY_LEFT||code==GLFW_KEY_RIGHT||code==GLFW_KEY_UP||code==GLFW_KEY_DOWN)
        {
            auto target=input.world->snapshot().target;
            const auto selected=input.world->snapshot().dofs==2?1u:input.selected;
            if(code==GLFW_KEY_LEFT) target[0]-=.05;
            if(code==GLFW_KEY_RIGHT) target[0]+=.05;
            if(code==GLFW_KEY_DOWN) target[selected]-=.05;
            if(code==GLFW_KEY_UP) target[selected]+=.05;
            std::string reason;
            if(!input.world->command(CommandKind::Target,target,&reason)) std::cerr<<reason<<'\n';
        }
    }
    void verifyCallbacks(GLFWwindow* window,const ArmRig& rig,BenchCase scenario)
    {
        PhysicsWorld world{benchDefinition(rig,scenario)}; resetBench(world,scenario);
        julia::runtime::FixedStepScheduler scheduler; Input input{&world,&scheduler,scenario,false};
        glfwSetWindowUserPointer(window,&input);
        const auto initial=world.snapshot();
        key(window,GLFW_KEY_TAB,0,GLFW_PRESS,0);
        key(window,GLFW_KEY_UP,0,GLFW_PRESS,0);
        if(input.selected!=1||world.snapshot().q!=initial.q||std::abs(world.snapshot().target[1]-initial.target[1]-.05)>1e-12)
            throw std::runtime_error("Coordinate selection/target callback failed");
        if(initial.dofs==MaxCoordinates)
        {
            key(window,GLFW_KEY_DOWN,0,GLFW_PRESS,0);
            for(unsigned n=0;n<MaxCoordinates;++n)
            {
                key(window,GLFW_KEY_TAB,0,GLFW_PRESS,0); const auto before=world.snapshot().target;
                key(window,GLFW_KEY_UP,0,GLFW_PRESS,0);
                auto expected=before; expected[input.selected]+=.05;
                if(world.snapshot().target!=expected||world.snapshot().q!=initial.q) throw std::runtime_error("Six-axis target callback failed");
                key(window,GLFW_KEY_DOWN,0,GLFW_PRESS,0);
            }
        }
        key(window,GLFW_KEY_SPACE,0,GLFW_PRESS,0); key(window,GLFW_KEY_V,0,GLFW_PRESS,0);
        key(window,GLFW_KEY_R,0,GLFW_PRESS,0);
        if(!scheduler.paused()||input.camera!=1||!input.reset) throw std::runtime_error("Pause/camera/reset callback failed");
        key(window,GLFW_KEY_D,0,GLFW_PRESS,0); key(window,GLFW_KEY_I,0,GLFW_PRESS,0); world.step();
        if(world.snapshot().mode!=DriveMode::Disabled||world.snapshot().externalForce.z!=-30) throw std::runtime_error("Drive/wrench callback failed");
        glfwSetWindowUserPointer(window,nullptr);
    }
    Pixels readPixels()
    {
        Pixels pixels(static_cast<std::size_t>(Width)*Height*3);
        glReadBuffer(GL_BACK); glPixelStorei(GL_PACK_ALIGNMENT,1);
        glReadPixels(0,0,Width,Height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data()); return pixels;
    }
    void savePixels(const std::filesystem::path& path,const Pixels& pixels)
    {
        std::ofstream out{path,std::ios::binary}; out<<"P6\n"<<Width<<' '<<Height<<"\n255\n";
        for(int y=Height-1;y>=0;--y) out.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*Width*3),Width*3);
        if(!out) throw std::runtime_error("GPU capture write failed");
    }
    std::size_t changed(const Pixels& a,const Pixels& b)
    {
        std::size_t count=0; for(std::size_t i=0;i<a.size();++i) if(a[i]!=b[i]) ++count; return count;
    }
    void line(std::vector<DebugLineVertex>& v,glm::dvec3 a,glm::dvec3 b,glm::vec3 color)
    { v.push_back({glm::vec3{a},color}); v.push_back({glm::vec3{b},color}); }
    void cross(std::vector<DebugLineVertex>& v,glm::dvec3 p,double radius,glm::vec3 color)
    {
        for(int k=0;k<3;++k) { glm::dvec3 d{0}; d[k]=radius; line(v,p-d,p+d,color); }
    }
    std::vector<DebugLineVertex> overlay(const ArmRig& rig,const Definition& def,const RenderSnapshot& view)
    {
        std::vector<DebugLineVertex> v;
        const auto target=rig.visualFrames(view.targetPose);
        for(std::size_t i=0;i<2;++i) line(v,glm::dvec3{target[i][3]},glm::dvec3{target[i+1][3]},{1,.2f,1});
        for(int k=0;k<3;++k) line(v,glm::dvec3{target[2][3]},glm::dvec3{target[2][3]}+glm::dvec3{target[2][k]}*.07,{1,.2f,1});
        const auto& s=view.physics;
        for(int k=0;k<3;++k) line(v,glm::dvec3{s.worldFromJoint[2][3]},glm::dvec3{s.worldFromJoint[2][3]}+glm::dvec3{s.worldFromJoint[2][k]}*.07,{.2f,1,.2f});
        for(std::size_t i=0;i<def.links.size();++i)
        {
            const auto& b=def.links[i]; const auto& frame=s.worldFromJoint[i];
            const auto a=glm::dvec3{frame*glm::dvec4{b.capsuleStart,1}}, end=glm::dvec3{frame*glm::dvec4{b.capsuleEnd,1}};
            line(v,a,end,{0,1,1}); cross(v,s.comWorld[i],.012+std::sqrt(b.mass)*.006,{1,.8f,0});
            const auto y=glm::length(end-a)>1e-8?glm::normalize(end-a):glm::dvec3{0,1,0};
            const auto x=glm::normalize(glm::cross(y,std::abs(y.z)<.9?glm::dvec3{0,0,1}:glm::dvec3{1,0,0}));
            const auto z=glm::cross(x,y);
            for(int k=0;k<24;++k)
            {
                const double t=glm::two_pi<double>()*k/24, next=glm::two_pi<double>()*(k+1)/24;
                const auto d=b.radius*(x*std::cos(t)+z*std::sin(t)), dn=b.radius*(x*std::cos(next)+z*std::sin(next));
                line(v,a+d,a+dn,{.1f,.7f,.7f}); line(v,end+d,end+dn,{.1f,.7f,.7f});
                if(k%6==0) line(v,a+d,end+d,{.1f,.7f,.7f});
            }
        }
        const auto base=glm::dvec3{s.worldFromJoint[0][3]};
        line(v,base+glm::dvec3{-.16,0,0},base+glm::dvec3{.16,0,0},{1,1,1});
        line(v,base,base+glm::dvec3{0,.20,0},{1,1,1}); // Explicit fixed fixture.
        cross(v,view.targetWrist,.025,{1,.2f,1}); cross(v,view.actualWrist,.018,{.2f,1,.2f});
        line(v,view.actualWrist,view.targetWrist,{1,.2f,.2f});
        if(def.wall)
        {
            std::array<glm::dvec3,8> corners;
            for(unsigned i=0;i<8;++i) corners[i]=def.wall->center+def.wall->halfSize*glm::dvec3{(i&1)?1.:-1.,(i&2)?1.:-1.,(i&4)?1.:-1.};
            for(unsigned i=0;i<8;++i) for(unsigned bit:{1u,2u,4u}) if(!(i&bit)) line(v,corners[i],corners[i|bit],{1,.3f,.2f});
        }
        for(const auto& c:s.contacts)
        {
            cross(v,c.point,.012,{1,1,0}); line(v,c.point,c.point+c.normal*.08,{1,1,0});
            if(c.force) line(v,c.point,c.point+c.normal*std::min(.25,c.force->x*.002),{1,0,0});
        }
        return v;
    }
}
int main(int argc,char** argv)
{
    GLFWwindow* window=nullptr;
    try
    {
        const auto options=bench::parse(argc,argv);
        if(!glfwInit()) throw std::runtime_error("GLFW initialization failed");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
        glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE); glfwWindowHint(GLFW_SRGB_CAPABLE,GLFW_TRUE);
        glfwWindowHint(GLFW_RESIZABLE,GLFW_FALSE); glfwWindowHint(GLFW_VISIBLE,options.verify?GLFW_FALSE:GLFW_TRUE);
        window=glfwCreateWindow(Width,Height,"Julia physical arm — FIXED BASE",nullptr,nullptr);
        if(!window) throw std::runtime_error("GLFW window creation failed");
        glfwMakeContextCurrent(window); glfwSwapInterval(options.verify?0:1);
        if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) throw std::runtime_error("GLAD load failed");
        {
            const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_PHYSICS_ASSETS}/"models/Raphtalia_prepared.fbx");
            const ArmRig rig{*asset,options.side,options.chain};
            if(options.verify) verifyCallbacks(window,rig,options.scenario);
            auto model=julia::render_opengl::GLModelLoader::upload(asset);
            auto world=std::make_unique<PhysicsWorld>(benchDefinition(rig,options.scenario));
            verifyFramesBeforeDynamics(rig,*world); bench::reset(*world,options.scenario,options);
            auto view=publish(rig,world->snapshot());
            auto log=std::make_unique<bench::Log>(options.output); bench::metadata(options.output,rig,*world);
            std::ofstream report{options.output/"gpu.txt"};
            report<<"GPU="<<glGetString(GL_RENDERER)<<" OpenGL="<<glGetString(GL_VERSION)<<"\n";
            julia::render_opengl::OpenGLRenderer renderer; renderer.initialize(Width,Height,JULIA_PHYSICS_ASSETS);
            julia::render_opengl::CameraConfig camera;
            camera.position={1.65f,1.45f,2.5f}; camera.target={.08f,.90f,.10f}; camera.orthographicHeight=1.85f;
            auto displayCamera=camera;
            const auto setCamera=[&](const julia::render_opengl::CameraConfig& c) { displayCamera=c; renderer.setCamera(c); };
            setCamera(camera);
            julia::runtime::FixedStepScheduler scheduler;
            Input input{world.get(),&scheduler,options.scenario,false};
            const auto bodyFromResource=rig.binding().profile().bodyFromResource;
            const auto draw=[&](bool debug) {
                renderer.beginFrame(); renderer.renderModel(model,bodyFromResource,view.actualPose.palette);
                if(debug) {
                    renderer.renderDebugLines(overlay(rig,world->definition(),view));
                    renderer.renderDebugLines(bench::hud(view,displayCamera,input.selected,scheduler.paused()));
                }
                if(glGetError()!=GL_NO_ERROR) throw std::runtime_error("OpenGL render error");
            };
            draw(false); const auto first=readPixels();
            draw(true); savePixels(options.output/"frame_0.ppm",readPixels());
            glfwSetWindowUserPointer(window,&input); glfwSetKeyCallback(window,key);
            double last=glfwGetTime(), renderSum=0,gpuSum=0,paletteSum=0;
            unsigned renders=0,captures=1; std::uint64_t nextCapture=120,lastCapture=0;
            unsigned experiment=0; auto activeCase=options.scenario;
            GLuint query=0; glGenQueries(1,&query);
            const auto limit=options.verify||options.explicitTicks?options.ticks:std::numeric_limits<std::uint64_t>::max();
            std::cout<<"Fixed base. 1..7: passive/hold/heavy/inertia/weak/impulse/contact; R reset; Space pause; Tab coordinate; Up/Down target; V camera; D passive; I pulse; Esc exit.\n";
            while(!glfwWindowShouldClose(window)&&world->snapshot().tick<limit)
            {
                glfwPollEvents();
                if(input.reset)
                {
                    if(activeCase!=input.scenario)
                    {
                        world=std::make_unique<PhysicsWorld>(benchDefinition(rig,input.scenario));
                        verifyFramesBeforeDynamics(rig,*world); activeCase=input.scenario;
                    }
                    bench::reset(*world,input.scenario,options);
                    input.world=world.get(); input.reset=false;
                    scheduler=julia::runtime::FixedStepScheduler{};
                    view=publish(rig,world->snapshot()); // Collapse display history at experiment entry.
                    const auto runDirectory=options.output/("reset_"+std::to_string(++experiment)+"_"+benchCaseName(activeCase));
                    log=std::make_unique<bench::Log>(runDirectory); bench::metadata(runDirectory,rig,*world);
                }
                const auto now=glfwGetTime();
                const auto tick=[&] {
                    if(world->snapshot().tick>=limit) return;
                    stepBench(*world,input.scenario); view=publish(rig,world->snapshot()); log->row(view,world->definition());
                };
                scheduler.advance(options.verify?1./options.fps:now-last,tick,glfwGetWindowAttrib(window,GLFW_ICONIFIED)!=0); last=now;
                // Draw every scheduler frame, including repeated states at 144 Hz.
                // Only read back/export the selected control frames.
                const auto currentTick=world->snapshot().tick;
                const bool capture=options.verify&&currentTick!=lastCapture&&
                    (currentTick>=nextCapture||currentTick==246||currentTick==limit);
                auto activeCamera=camera;
                const float sign=options.side==ArmSide::Left?1.f:-1.f;
                if(input.camera==1) { activeCamera.target=glm::vec3{view.actualWrist}; activeCamera.position=activeCamera.target+glm::vec3{sign*.6f,.25f,.65f}; activeCamera.orthographicHeight=.4f; }
                if(input.camera==2) { activeCamera.position={-sign*1.8f,1.6f,1.5f}; }
                setCamera(activeCamera);
                const auto stateBefore=world->snapshot();
                const auto begin=Clock::now(); glBeginQuery(GL_TIME_ELAPSED,query); draw(true); glEndQuery(GL_TIME_ELAPSED);
                renderSum+=std::chrono::duration<double,std::micro>(Clock::now()-begin).count();
                paletteSum+=renderer.lastPaletteUploadCpuMilliseconds()*1000;
                GLuint64 ns=0; glGetQueryObjectui64v(query,GL_QUERY_RESULT,&ns); gpuSum+=static_cast<double>(ns)/1000; ++renders;
                if(capture)
                {
                    lastCapture=currentTick;
                    const auto image=readPixels();
                    savePixels(options.output/("frame_"+std::to_string(view.physics.tick)+".ppm"),image); ++captures;
                    draw(true); if(image!=readPixels()) throw std::runtime_error("Repeated GPU draw changed snapshot pixels");
                    if(world->snapshot().q!=stateBefore.q||world->snapshot().tick!=stateBefore.tick) throw std::runtime_error("Render advanced physics");
                    auto closeCamera=camera; closeCamera.target=glm::vec3{view.actualWrist}; closeCamera.position=closeCamera.target+glm::vec3{sign*.8f,.2f,.8f};
                    closeCamera.orthographicHeight=.85f; setCamera(closeCamera); draw(true);
                    savePixels(options.output/("arm_"+std::to_string(view.physics.tick)+".ppm"),readPixels());
                    closeCamera.orthographicHeight=.3f; setCamera(closeCamera); draw(true);
                    savePixels(options.output/("wrist_"+std::to_string(view.physics.tick)+".ppm"),readPixels());
                    closeCamera.position=closeCamera.target+glm::vec3{-sign*.8f,.4f,-.6f}; closeCamera.orthographicHeight=.85f;
                    setCamera(closeCamera); draw(true);
                    savePixels(options.output/("reverse_"+std::to_string(view.physics.tick)+".ppm"),readPixels());
                    setCamera(activeCamera);
                    if(world->snapshot().tick>=nextCapture) nextCapture+=120;
                }
                std::ostringstream title;
                title<<std::fixed<<std::setprecision(2)<<"FIXED "<<(options.side==ArmSide::Left?"L":"R")<<" "<<benchCaseName(input.scenario)<<" sel="<<input.selected;
                for(unsigned k=0;k<view.physics.dofs;++k)
                    title<<" |"<<k<<" q*/q "<<view.physics.target[k]<<'/'<<view.physics.q[k]<<" v "<<view.physics.v[k]
                        <<" Nm "<<view.physics.calculatedTorque[k]<<'/'<<view.physics.appliedTorque[k]
                        <<" S"<<view.physics.saturatedThisTick[k]<<" lim "<<view.physics.limitViolation[k];
                title<<" contacts "<<view.physics.contacts.size()<<" N "<<view.physics.peakNormalForceThisTick<<" solver "<<view.physics.solverIterations;
                glfwSetWindowTitle(window,title.str().c_str()); glfwSwapBuffers(window);
            }
            glDeleteQueries(1,&query);
            draw(false); const auto meshDelta=changed(first,readPixels());
            if(options.verify&&(meshDelta<=1000||captures<5||world->snapshot().tick!=limit||world->snapshot().worldSteps!=limit*Substeps))
                throw std::runtime_error("GPU pose/capture/exact tick acceptance failed");
            if(options.verify&&renders!=(limit*options.fps+119)/120)
                throw std::runtime_error("Expected one draw per scheduler frame");
            if(glGetError()!=GL_NO_ERROR) throw std::runtime_error("OpenGL final error");
            report<<"case="<<benchCaseName(input.scenario)<<" fps="<<options.fps<<" ticks="<<world->snapshot().tick<<" steps="<<world->snapshot().worldSteps
                <<" renders="<<renders<<" captures="<<captures<<" mesh_changed_bytes="<<meshDelta<<" GL_errors=0\n"
                <<"render_submit_mean_us="<<(renders?renderSum/renders:0)<<" GPU_elapsed_mean_us="<<(renders?gpuSum/renders:0)
                <<" callback_checks="<<(options.verify?"PASS":"not-run")<<" OS_input=not-tested "
                <<" palette_upload_mean_us="<<(renders?paletteSum/renders:0)<<" (query synchronized)\nPASS\n";
            std::cout<<"PASS desktop physics; ticks="<<world->snapshot().tick<<" captures="<<captures<<'\n';
        }
        glfwDestroyWindow(window); glfwTerminate(); return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr<<"PHYSICS DESKTOP FAILURE: "<<e.what()<<'\n';
        if(window) glfwDestroyWindow(window); glfwTerminate(); return 1;
    }
}
