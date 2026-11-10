#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "BodyBenchIO.hpp"
#include "../julia_arm_physics/BenchHud.hpp"
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "julia/runtime/FixedStepScheduler.hpp"
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>
#include <sstream>

namespace
{
    using namespace julia::physics;
    using julia::render_opengl::DebugLineVertex;
    constexpr int Width=1100,Height=800;
    using Pixels=std::vector<unsigned char>;
    struct Input
    {
        PhysicsWorld* world=nullptr;
        julia::runtime::FixedStepScheduler* scheduler=nullptr;
        BodyCase scenario=BodyCase::Drop;
        bool reset=false;
        unsigned selected=0,camera=0;
    };
    void key(GLFWwindow* window,int code,int,int action,int)
    {
        if(action!=GLFW_PRESS) return;
        auto& i=*static_cast<Input*>(glfwGetWindowUserPointer(window));
        if(code==GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window,GLFW_TRUE);
        else if(code==GLFW_KEY_R) i.reset=true;
        else if(code==GLFW_KEY_SPACE) i.scheduler->setPaused(!i.scheduler->paused());
        else if(code==GLFW_KEY_V) i.camera=(i.camera+1)%3;
        else if(code==GLFW_KEY_TAB) i.selected=(i.selected+1)%i.world->snapshot().dofs;
        else if(code==GLFW_KEY_D) i.world->command(CommandKind::Disable);
        else if(code==GLFW_KEY_S) i.world->command(CommandKind::SmoothTarget,i.world->snapshot().target);
        else if(code==GLFW_KEY_I) i.world->applyWrench(3,i.world->snapshot().mode==DriveMode::Stand?i.world->snapshot().stand.rootGoal.orientation*glm::dvec3{3,0,0}:glm::dvec3{200,0,0});
        else if(code>=GLFW_KEY_1&&code<=GLFW_KEY_9) { i.scenario=static_cast<BodyCase>(code-GLFW_KEY_1); i.reset=true; }
        else if(code==GLFW_KEY_J||code==GLFW_KEY_L||code==GLFW_KEY_K||code==GLFW_KEY_O||code==GLFW_KEY_C)
        {
            std::string reason;
            const auto ok=code==GLFW_KEY_C?i.world->cancelStandCom(&reason):i.world->standComTarget(
                code==GLFW_KEY_J?glm::dvec2{-.003,0}:code==GLFW_KEY_L?glm::dvec2{.003,0}:code==GLFW_KEY_K?glm::dvec2{0,.003}:glm::dvec2{0},&reason);
            if(!ok) std::cerr<<reason<<'\n';
        }
        else if(code==GLFW_KEY_UP||code==GLFW_KEY_DOWN)
        {
            auto target=i.world->snapshot().target; target[i.selected]+=code==GLFW_KEY_UP?.02:-.02;
            std::string reason; if(!i.world->command(i.world->snapshot().mode==DriveMode::Servo?CommandKind::SmoothTarget:CommandKind::Target,target,&reason)) std::cerr<<reason<<'\n';
        }
    }
    void verifyCallbacks(GLFWwindow* window,const BodyRig& rig,const bodybench::Options& options)
    {
        PhysicsWorld w{bodyBenchDefinition(rig,BodyCase::Drop)}; bodybench::reset(rig,w,BodyCase::Drop,options);
        julia::runtime::FixedStepScheduler scheduler; Input i{&w,&scheduler}; glfwSetWindowUserPointer(window,&i);
        const auto initial=w.snapshot();
        for(unsigned k=0;k<41;++k)
        {
            key(window,GLFW_KEY_UP,0,GLFW_PRESS,0);
            if(w.snapshot().target[i.selected]!=.02||w.snapshot().q!=initial.q||w.snapshot().mode!=initial.mode||w.snapshot().root.position!=initial.root.position) throw std::runtime_error("Target callback wrote physics");
            key(window,GLFW_KEY_DOWN,0,GLFW_PRESS,0); key(window,GLFW_KEY_TAB,0,GLFW_PRESS,0);
        }
        key(window,GLFW_KEY_D,0,GLFW_PRESS,0); key(window,GLFW_KEY_I,0,GLFW_PRESS,0); w.step();
        if(w.snapshot().externalForce.x!=200||w.snapshot().mode!=DriveMode::Disabled) throw std::runtime_error("Drive/wrench callback failed");
        key(window,GLFW_KEY_SPACE,0,GLFW_PRESS,0); key(window,GLFW_KEY_R,0,GLFW_PRESS,0); key(window,GLFW_KEY_V,0,GLFW_PRESS,0);
        if(!scheduler.paused()||!i.reset||i.camera!=1) throw std::runtime_error("Pause/reset/camera callback failed");
        glfwSetWindowUserPointer(window,nullptr);
    }
    void line(std::vector<DebugLineVertex>& v,glm::dvec3 a,glm::dvec3 b,glm::vec3 color)
    { v.push_back({glm::vec3{a},color}); v.push_back({glm::vec3{b},color}); }
    void cross(std::vector<DebugLineVertex>& v,glm::dvec3 p,double radius,glm::vec3 color)
    { for(int k=0;k<3;++k) { glm::dvec3 d{0}; d[k]=radius; line(v,p-d,p+d,color); } }
    void sphere(std::vector<DebugLineVertex>& v,glm::dvec3 center,double radius,glm::vec3 color)
    {
        for(int axis=0;axis<3;++axis) for(int k=0;k<32;++k)
        {
            glm::dvec3 a{0},b{0}; const double t=glm::two_pi<double>()*k/32,next=glm::two_pi<double>()*(k+1)/32;
            a[(axis+1)%3]=radius*std::cos(t); a[(axis+2)%3]=radius*std::sin(t);
            b[(axis+1)%3]=radius*std::cos(next); b[(axis+2)%3]=radius*std::sin(next);
            line(v,center+a,center+b,color);
        }
    }
    std::vector<DebugLineVertex> overlay(const Definition& def,const Snapshot& s,bool proxies=true)
    {
        std::vector<DebugLineVertex> v;
        if(def.floorHeight) for(int k=-20;k<=20;++k)
        {
            const double t=k*.15,y=*def.floorHeight;
            line(v,{-3,y,t},{3,y,t},{.4f,.5f,.5f}); line(v,{t,y,-3},{t,y,3},{.4f,.5f,.5f});
        }
        if(proxies) for(std::size_t i=0;i<def.links.size();++i)
        {
            const auto& b=def.links[i]; const auto& frame=s.worldFromJoint[i];
            const glm::vec3 color{.05f,.85f,.95f};
            if(b.boxHalfSize)
            {
                std::array<glm::dvec3,8> corners;
                for(unsigned n=0;n<8;++n) corners[n]=glm::dvec3{frame*b.jointFromBox*glm::dvec4{*b.boxHalfSize*glm::dvec3{(n&1)?1.:-1.,(n&2)?1.:-1.,(n&4)?1.:-1.},1}};
                for(unsigned n=0;n<8;++n) for(unsigned bit:{1u,2u,4u}) if(!(n&bit)) line(v,corners[n],corners[n|bit],color);
            }
            else
            {
                const auto a=glm::dvec3{frame*glm::dvec4{b.capsuleStart,1}},end=glm::dvec3{frame*glm::dvec4{b.capsuleEnd,1}};
                sphere(v,a,b.radius,color); sphere(v,end,b.radius,color);
                if(glm::length(end-a)>1e-8)
                {
                    const auto y=glm::normalize(end-a),ref=std::abs(y.z)<.9?glm::dvec3{0,0,1}:glm::dvec3{1,0,0};
                    const auto x=glm::normalize(glm::cross(y,ref)),z=glm::cross(x,y);
                    for(unsigned k=0;k<8;++k)
                    { const double t=glm::two_pi<double>()*k/8; const auto d=b.radius*(x*std::cos(t)+z*std::sin(t)); line(v,a+d,end+d,color); }
                }
            }
            cross(v,s.comWorld[i],.012,{1,.6f,0});
            if(b.parent>=0) line(v,glm::dvec3{s.worldFromJoint[static_cast<std::size_t>(b.parent)][3]},glm::dvec3{frame[3]},{.3f,.8f,.2f});
        }
        sphere(v,s.totalComWorld,.035,{1,1,0}); cross(v,s.totalComWorld,.07,{1,1,0});
        cross(v,s.root.position,.05,{1,0,1});
        if(s.mode==DriveMode::Stand)
        {
            const glm::vec3 goalColor{1,.55f,.05f};
            cross(v,s.stand.rootGoal.position,.07,goalColor); sphere(v,s.stand.comGoal,s.stand.variant==StandVariant::Bounded7h?.002:.025,goalColor);
            if(s.stand.variant==StandVariant::Bounded7h||s.stand.commandSerial>0)
            {
                const auto& t=s.stand; const auto rotation=glm::mat3_cast(t.rootGoal.orientation);
                auto base=t.rootGoal.position+rotation*glm::dvec3{.40,0,0}; base.y=s.support.planeOrigin.y+.02;
                // Fixed metric inset. Its origin follows only the frozen placement goal.
                const auto origin=t.initialCom;
                const auto point=[&](glm::dvec3 p) { auto d=p-origin; d.y=0; return base+30.*d; };
                line(v,base-.16*rotation[0],base+.16*rotation[0],{.05f,.05f,.05f});
                line(v,base-.16*rotation[2],base+.16*rotation[2],{.05f,.05f,.05f});
                for(int tick=-5;tick<=5;++tick)
                {
                    const auto xTick=base+.03*static_cast<double>(tick)*rotation[0],zTick=base+.03*static_cast<double>(tick)*rotation[2];
                    line(v,xTick-.004*rotation[2],xTick+.004*rotation[2],{.05f,.05f,.05f});
                    line(v,zTick-.004*rotation[0],zTick+.004*rotation[0],{.05f,.05f,.05f});
                }
                cross(v,point(t.commandCom),.009,{1,0,1}); cross(v,point(t.comGoal),.006,{1,.5f,0});
                cross(v,point(s.totalComWorld),.003,{1,1,0});
                sphere(v,t.commandCom,.003,{1,0,1}); sphere(v,t.comGoal,.002,{1,.5f,0});
            }
            line(v,s.totalComWorld,s.totalComWorld+s.support.dynamics.comVelocity,glm::vec3{1,1,0});
            for(const auto& w:s.stand.predictedReaction)
            {
                // Offset glyphs distinguish predicted forces from measured forces.
                const auto origin=w.origin+glm::dvec3{0,.015,0};
                line(v,origin,origin+w.force*.0005,goalColor);
                const auto cop=pressureCenter(w,{0,0,0}); if(cop.point) cross(v,*cop.point,.022,goalColor);
            }
            const auto rotation=glm::mat3_cast(s.stand.rootGoal.orientation);
            for(int axis=0;axis<3;++axis) line(v,s.stand.rootGoal.position,s.stand.rootGoal.position+.12*rotation[axis],goalColor);
        }
        const auto& observation=s.support;
        if(observation.enabled&&observation.floorAvailable)
        {
            cross(v,observation.comProjection,.045,{1,1,0}); line(v,s.totalComWorld,observation.comProjection,{1,1,0});
            const auto region=[&](const PlanarRegion& r,glm::vec3 color) {
                for(const auto p:r.vertices) cross(v,p,.008,color);
                if(r.vertices.size()>=2) for(std::size_t k=0;k<r.vertices.size();++k) line(v,r.vertices[k],r.vertices[(k+1)%r.vertices.size()],color);
            };
            region(observation.loadedRegion,{.2f,1,.2f});
            for(std::size_t side=0;side<2;++side)
            {
                const auto& f=observation.feet[side]; const glm::vec3 color=side==0?glm::vec3{.2f,1,.3f}:glm::vec3{.3f,.5f,1};
                region(f.soleProjection,{.65f,.65f,.65f}); region(f.loadedRegion,color);
                if(f.cop.point) { cross(v,*f.cop.point,.035,color); sphere(v,*f.cop.point,.018,color); }
                const double magnitude=glm::length(f.wrench.force);
                line(v,f.wrench.origin,f.wrench.origin+f.wrench.force*std::min(.0005,.4/std::max(1.,magnitude)),color);
            }
            if(observation.combinedCop.point) cross(v,*observation.combinedCop.point,.05,{1,1,1});
        }
        for(const auto& c:s.contacts)
        {
            const bool self=c.link1>=0&&c.link2>=0;
            const bool foot=(c.link1==observation.feet[0].link||c.link2==observation.feet[0].link||c.link1==observation.feet[1].link||c.link2==observation.feet[1].link)&&!self;
            const glm::vec3 color=self?glm::vec3{1,.2f,1}:foot?glm::vec3{1,1,0}:glm::vec3{1,.35f,.05f};
            cross(v,c.point,.015,color); line(v,c.point,c.point+c.normal*.10,color);
            if(c.force) line(v,c.point,c.point+c.forceOn2World*.0001,color);
        }
        return v;
    }
    Pixels readPixels()
    {
        Pixels p(static_cast<std::size_t>(Width)*Height*3); glReadBuffer(GL_BACK); glPixelStorei(GL_PACK_ALIGNMENT,1);
        glReadPixels(0,0,Width,Height,GL_RGB,GL_UNSIGNED_BYTE,p.data()); return p;
    }
    void save(const std::filesystem::path& file,const Pixels& p)
    {
        std::ofstream out{file,std::ios::binary}; out<<"P6\n"<<Width<<' '<<Height<<"\n255\n";
        for(int y=Height-1;y>=0;--y) out.write(reinterpret_cast<const char*>(p.data()+static_cast<std::size_t>(y)*Width*3),Width*3);
        if(!out) throw std::runtime_error("Capture write failed");
    }
    void stats(std::ostream& out,const char* name,std::vector<double> v)
    {
        if(v.empty()) return;
        const auto mean=std::accumulate(v.begin(),v.end(),0.)/static_cast<double>(v.size()); std::sort(v.begin(),v.end());
        out<<name<<" mean="<<mean<<" p95="<<v[static_cast<std::size_t>(std::ceil(.95*static_cast<double>(v.size())))-1]<<" max="<<v.back()<<'\n';
    }
}
int main(int argc,char** argv)
{
    GLFWwindow* window=nullptr;
    try
    {
        const auto o=bodybench::parse(argc,argv);
        if(!glfwInit()) throw std::runtime_error("GLFW init failed");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
        glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE); glfwWindowHint(GLFW_SRGB_CAPABLE,GLFW_TRUE);
        glfwWindowHint(GLFW_RESIZABLE,GLFW_FALSE); glfwWindowHint(GLFW_VISIBLE,o.verify?GLFW_FALSE:GLFW_TRUE);
        window=glfwCreateWindow(Width,Height,"Julia free physical body",nullptr,nullptr);
        if(!window) throw std::runtime_error("GLFW window failed");
        glfwMakeContextCurrent(window); glfwSwapInterval(o.verify?0:1);
        if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) throw std::runtime_error("GLAD load failed");
        {
            const auto asset=julia::model_import::ModelImporter::loadFromFile(std::filesystem::path{JULIA_PHYSICS_ASSETS}/"models/Raphtalia_prepared.fbx");
            const BodyRig rig{*asset}; if(o.verify) verifyCallbacks(window,rig,o);
            auto model=julia::render_opengl::GLModelLoader::upload(asset);
            auto world=std::make_unique<PhysicsWorld>(bodybench::definition(rig,o.scenario,o)); verifyFramesBeforeDynamics(rig,*world);
            bodybench::reset(rig,*world,o.scenario,o); auto view=publish(rig,world->snapshot());
            auto log=std::make_unique<bodybench::Log>(o.output,o.substeps); bodybench::metadata(o.output,rig,*world); log->row(view);
            std::ofstream report{o.output/"gpu.txt"}; report<<"GPU="<<glGetString(GL_RENDERER)<<" OpenGL="<<glGetString(GL_VERSION)<<'\n';
            julia::render_opengl::OpenGLRenderer renderer; renderer.initialize(Width,Height,JULIA_PHYSICS_ASSETS);
            julia::runtime::FixedStepScheduler scheduler; Input input{world.get(),&scheduler,o.scenario};
            glfwSetWindowUserPointer(window,&input); glfwSetKeyCallback(window,key);
            julia::render_opengl::CameraConfig displayCamera;
            unsigned displayCameraIndex=0;
            const auto camera=[&](unsigned index)
            {
                julia::render_opengl::CameraConfig c;
                c.target=glm::vec3{view.physics.totalComWorld}; c.target.y=.75f;
                c.position=c.target+(index==0?glm::vec3{2,1.1f,3}:index==1?glm::vec3{-2,1,-3}:glm::vec3{0,3,.01f});
                c.orthographicHeight=2.8f;
                if(index==2)
                {
                    auto lo=view.physics.support.comProjection,hi=lo;
                    for(const auto& f:view.physics.support.feet) for(const auto p:f.soleProjection.vertices) { lo=glm::min(lo,p); hi=glm::max(hi,p); }
                    c.target=glm::vec3{(lo+hi)*.5}; c.position=c.target+glm::vec3{0,3,0}; c.up={0,0,-1};
                    c.orthographicHeight=static_cast<float>(std::max({view.physics.stand.variant==StandVariant::Bounded7h?1.15:.8,(hi.z-lo.z)*1.3,(hi.x-lo.x)*1.3*Height/Width}));
                }
                displayCameraIndex=index; displayCamera=c; renderer.setCamera(c);
            };
            const auto draw=[&](bool debug)
            {
                renderer.beginFrame(); if(displayCameraIndex!=2) renderer.renderModel(model,glm::mat4{view.worldFromResource},view.actualPose.palette);
                if(debug)
                {
                    auto lines=overlay(world->definition(),view.physics,displayCameraIndex!=2);
                    for(std::size_t k=1;k<view.referenceFrames.size();++k)
                    {
                        const auto parent=world->definition().links[k].parent;
                        if(parent>=0) line(lines,glm::dvec3{view.referenceFrames[static_cast<std::size_t>(parent)][3]},glm::dvec3{view.referenceFrames[k][3]},{1,.65f,0});
                    }
                    renderer.renderDebugLines(lines);
                    renderer.renderDebugLines(bench::driveHud(view.physics,displayCamera,input.selected,scheduler.paused(),Width,Height,false));
                }
                if(glGetError()!=GL_NO_ERROR) throw std::runtime_error("GL render error");
            };
            camera(0); draw(false); const auto first=readPixels(); draw(true); save(o.output/"frame_0.ppm",readPixels());
            std::vector<double> submission,gpu,palette; GLuint query=0; glGenQueries(1,&query);
            auto active=o.scenario; unsigned experiment=0,renders=0,captures=1; std::uint64_t nextCapture=30;
            const auto limit=o.verify||o.explicitTicks?o.ticks:std::numeric_limits<std::uint64_t>::max(); double last=glfwGetTime();
            std::cout<<"Free root. Explicit stand/stand-pulse: limited two-foot control; other cases no balance. R reset, Space pause, V camera, D passive, Esc exit.\n";
            while(!glfwWindowShouldClose(window)&&world->snapshot().tick<limit)
            {
                glfwPollEvents();
                if(input.reset)
                {
                    if(active!=input.scenario) { world=std::make_unique<PhysicsWorld>(bodybench::definition(rig,input.scenario,o)); active=input.scenario; }
                    bodybench::reset(rig,*world,active,o); scheduler=julia::runtime::FixedStepScheduler{}; input.world=world.get(); input.reset=false;
                    view=publish(rig,world->snapshot()); const auto dir=o.output/("reset_"+std::to_string(++experiment)+"_"+bodyCaseName(active));
                    log=std::make_unique<bodybench::Log>(dir,o.substeps); bodybench::metadata(dir,rig,*world); log->row(view);
                }
                const double now=glfwGetTime(); scheduler.advance(o.verify?1./o.fps:now-last,[&]
                { if(world->snapshot().tick<limit) { bodybench::step(*world,active,o); view=publish(rig,world->snapshot()); log->row(view); } },glfwGetWindowAttrib(window,GLFW_ICONIFIED)!=0); last=now;
                camera(input.camera); const auto state=world->snapshot(); const auto start=std::chrono::steady_clock::now();
                glBeginQuery(GL_TIME_ELAPSED,query); draw(true); glEndQuery(GL_TIME_ELAPSED);
                submission.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
                palette.push_back(renderer.lastPaletteUploadCpuMilliseconds()*1000); GLuint64 ns=0; glGetQueryObjectui64v(query,GL_QUERY_RESULT,&ns); gpu.push_back(static_cast<double>(ns)/1000); ++renders;
                if(o.verify&&(state.tick>=nextCapture||state.tick==limit))
                {
                    const auto pixels=readPixels(); save(o.output/("frame_"+std::to_string(state.tick)+".ppm"),pixels); ++captures;
                    draw(true); if(pixels!=readPixels()) throw std::runtime_error("Repeated render changed pixels");
                    if(world->snapshot().tick!=state.tick||world->snapshot().root.position!=state.root.position||world->snapshot().q!=state.q||
                        world->snapshot().drive.qRef!=state.drive.qRef||world->snapshot().drive.deliveredWork!=state.drive.deliveredWork||
                        world->snapshot().drive.absorbedWork!=state.drive.absorbedWork) throw std::runtime_error("Render advanced physics/control/work");
                    for(unsigned angle=1;angle<3;++angle) { camera(angle); draw(true); save(o.output/("angle"+std::to_string(angle)+"_"+std::to_string(state.tick)+".ppm"),readPixels()); }
                    camera(input.camera); nextCapture=state.tick<180?nextCapture+30:nextCapture+180;
                }
                double violation=0; for(unsigned k=0;k<state.dofs;++k) violation=std::max(violation,state.peakLimitViolationThisTick[k]);
                std::ostringstream title; title<<std::fixed<<std::setprecision(2)<<"FREE "<<bodyCaseName(active)<<" t="<<state.time<<" y="<<state.root.position.y<<"m v="<<glm::length(state.root.linearVelocityWorld)
                    <<"m/s q"<<input.selected<<'='<<state.q[input.selected]<<"rad tau="<<state.appliedTorque[input.selected]<<"Nm lim="<<std::setprecision(3)<<violation
                    <<"rad con="<<state.contacts.size()<<" N="<<std::setprecision(0)<<state.peakNormalForceThisTick<<" pen="<<std::setprecision(1)<<1000*state.maxPenetrationThisTick<<"mm it="<<state.solverIterations;
                glfwSetWindowTitle(window,title.str().c_str()); glfwSwapBuffers(window);
            }
            glDeleteQueries(1,&query); camera(0); draw(false); const auto final=readPixels(); std::size_t delta=0;
            for(std::size_t k=0;k<first.size();++k) if(first[k]!=final[k]) ++delta;
            if(o.verify&&(delta<1000||captures<5||world->snapshot().worldSteps!=limit*world->definition().substeps||renders!=(limit*o.fps+119)/120)) throw std::runtime_error("GPU/tick acceptance failed");
            report<<"case="<<bodyCaseName(active)<<" scheduler_hz="<<(o.verify?std::to_string(o.fps):"wall-clock")<<" ticks="<<world->snapshot().tick<<" steps="<<world->snapshot().worldSteps<<" renders="<<renders<<" captures="<<captures<<" mesh_changed_bytes="<<delta<<" GL_errors=0\n";
            stats(report,"render_submit_us",submission); stats(report,"GPU_elapsed_us",gpu); stats(report,"palette_upload_us",palette);
            report<<"query_synchronized=true callback_checks="<<(o.verify?"PASS":"not-run")<<" OS_input=not-tested\nPASS\n";
            std::cout<<"PASS desktop body ticks="<<world->snapshot().tick<<'\n';
        }
        glfwDestroyWindow(window); glfwTerminate(); return 0;
    }
    catch(const std::exception& e)
    { std::cerr<<"BODY DESKTOP FAILURE: "<<e.what()<<'\n'; if(window) glfwDestroyWindow(window); glfwTerminate(); return 1; }
}
