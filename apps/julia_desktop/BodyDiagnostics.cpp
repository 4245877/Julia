#include "BodyDiagnostics.hpp"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace julia::desktop
{
    using namespace embodiment;
    namespace
    {
        const DirectBodyControl& requireControl(const BodyInstance& body)
        { if (!body.control()) throw std::invalid_argument("Body diagnostics require calibration"); return *body.control(); }
    }
    BodyDiagnostics::BodyDiagnostics(BodyInstance& body):control_(requireControl(body)),body_(body)
    { select(JointId::LeftElbow); }
    void BodyDiagnostics::select(JointId id,std::size_t coordinate)
    {
        (void)control_.coordinate(id,coordinate);
        joint_=control_.binding().jointIndex(id); coordinate_=coordinate;
    }
    std::string BodyDiagnostics::title() const
    {
        const auto& j=control_.binding().profile().joints[joint_]; const auto& c=j.coordinates[coordinate_];
        std::ostringstream s;
        const auto& reach=body_.snapshot().reaches[reachArm_==ArmSide::Left ? 0:1];
        s << "IK " << (reachArm_==ArmSide::Left ? "L":"R");
        if (reach.action) s << " #" << reach.action << ' ' << (reach.cancellationPending && (reach.state==ReachState::Moving || reach.state==ReachState::Settling) ? "stopping":reachStateName(reach.state))
            << " err=" << std::fixed << std::setprecision(1) << reach.error*1000 << "mm " << reachReasonName(reach.reason);
        s << " target " << std::fixed << std::setprecision(2) << reachPosition_.x << ',' << reachPosition_.y << ',' << reachPosition_.z;
        s << " | BODY " << (control_.enabled() ? "ON ":"OFF ") << j.name << '/' << c.name
            << ' ' << std::setprecision(0) << glm::degrees(control_.coordinate(j.id,coordinate_))
            << "deg [" << glm::degrees(c.lower) << ',' << glm::degrees(c.upper) << "] " << result_;
        return s.str();
    }
    bool BodyDiagnostics::key(int key,int mods)
    {
        const auto& joints=control_.binding().profile().joints;
        if (pendingMode_ && body_.snapshot().tick>=pendingMode_->first) pendingMode_.reset();
        const bool enabled=pendingMode_ ? pendingMode_->second:control_.enabled();
        if (key==GLFW_KEY_F2)
        {
            body_.enqueue(SetDirectMode{!enabled}); pendingMode_=std::pair{body_.snapshot().tick+1,!enabled};
            result_="queued";
        }
        else if (!enabled) return false;
        else if (key==GLFW_KEY_F7)
        {
            if (mods&GLFW_MOD_SHIFT)
            {
                reachArm_=reachArm_==ArmSide::Left ? ArmSide::Right:ArmSide::Left;
                reachPosition_.x=-reachPosition_.x;
                result_="arm selected";
            }
            else
            {
                ReachTarget target; target.arm=reachArm_; target.position=reachPosition_;
                target.effector=reachArm_==ArmSide::Left ? EffectorId::LeftWrist:EffectorId::RightWrist;
                const auto& current=body_.snapshot().reaches[reachArm_==ArmSide::Left ? 0:1];
                if (current.state==ReachState::Moving || current.state==ReachState::Settling)
                    body_.enqueue(RetargetReach{current.action,target});
                else body_.enqueue(StartReach{target});
                result_="reach queued";
            }
        }
        else if (key==GLFW_KEY_DELETE)
        {
            const auto& current=body_.snapshot().reaches[reachArm_==ArmSide::Left ? 0:1];
            body_.enqueue(CancelReach{reachArm_,current.action}); result_="smooth stop queued";
        }
        // Six keys edit an arbitrary body-space position; F7 commits/retargets it.
        else if (key==GLFW_KEY_Z || key==GLFW_KEY_C || key==GLFW_KEY_V || key==GLFW_KEY_N || key==GLFW_KEY_O || key==GLFW_KEY_P)
        {
            const int component=key==GLFW_KEY_Z || key==GLFW_KEY_C ? 0:key==GLFW_KEY_V || key==GLFW_KEY_N ? 1:2;
            const float direction=key==GLFW_KEY_Z || key==GLFW_KEY_V || key==GLFW_KEY_O ? -1.0f:1.0f;
            reachPosition_[component]+=direction*((mods&GLFW_MOD_SHIFT) ? .005f:.025f); result_="target edited; F7 to commit";
        }
        else if (key==GLFW_KEY_F3)
        { joint_=(joint_+((mods&GLFW_MOD_SHIFT) ? joints.size()-1:1))%joints.size(); coordinate_=0; result_="selected"; }
        else if (key==GLFW_KEY_F4) { coordinate_=(coordinate_+1)%joints[joint_].coordinates.size(); result_="selected"; }
        else if (key==GLFW_KEY_F5) overlay=!overlay;
        else if (key==GLFW_KEY_BACKSPACE)
        { body_.enqueue(ResetCoordinates{}); result_="queued"; }
        else if (key==GLFW_KEY_EQUAL || key==GLFW_KEY_MINUS || key==GLFW_KEY_KP_ADD || key==GLFW_KEY_KP_SUBTRACT)
        {
            const bool positive=key==GLFW_KEY_EQUAL || key==GLFW_KEY_KP_ADD;
            const auto id=joints[joint_].id;
            body_.enqueue(SetCoordinate{id,coordinate_,glm::radians(positive ? 5.0f:-5.0f),LimitPolicy::Clamp,true});
            result_="queued";
        }
        else return false;
        std::cout << title() << '\n';
        return true;
    }
    void BodyDiagnostics::draw(render_opengl::OpenGLRenderer& renderer,const glm::mat4& viewerFromResource,
        const BodyDisplay& display) const
    {
        if (!display.direct || !overlay) return;
        const auto& globals=display.pose.globals;
        const auto& b=control_.binding(); const auto& p=b.profile();
        const auto coordinate=[&](JointId id,std::size_t k) { return display.coordinates[b.jointIndex(id)][static_cast<int>(k)]; };
        const auto displayFromBody=viewerFromResource*glm::inverse(p.bodyFromResource);
        std::vector<render_opengl::DebugLineVertex> lines;
        const auto line=[&](glm::vec3 a,glm::vec3 end,glm::vec3 color)
        { lines.push_back({glm::vec3{displayFromBody*glm::vec4{a,1}},color}); lines.push_back({glm::vec3{displayFromBody*glm::vec4{end,1}},color}); };
        const auto axes=[&](const glm::mat4& f,float length)
        {
            const auto origin=glm::vec3{f[3]};
            for (int k=0;k<3;++k) { glm::vec3 color{.05f}; color[k]=1; line(origin,origin+glm::vec3{f[k]}*length,color); }
        };
        for (const auto& j:p.joints)
        {
            std::uint32_t end=animation::NoNode;
            for (std::uint32_t n=0;n<b.asset().nodes().size();++n) if (b.asset().nodes()[n].name==j.segmentEnd.name) { end=n; break; }
            if (end!=animation::NoNode) line(glm::vec3{b.jointFrame(j.id,globals)[3]},glm::vec3{p.bodyFromResource*globals[end][3]},{.65f,.5f,.08f});
        }
        for (const auto& e:p.effectors) axes(b.effectorFrame(e.id,globals),.035f);
        const auto cross=[&](glm::vec3 point,glm::vec3 color,float radius)
        { for (int k=0;k<3;++k) { glm::vec3 d{0}; d[k]=radius; line(point-d,point+d,color); } };
        cross(reachPosition_,{1,.2f,1},.018f);
        for (std::size_t side=0;side<2;++side)
        {
            const auto& r=body_.snapshot().reaches[side]; if (!r.action) continue;
            const auto shoulder=side==0 ? JointId::LeftShoulder:JointId::RightShoulder,
                elbow=side==0 ? JointId::LeftElbow:JointId::RightElbow;
            const auto hand=glm::vec3{b.effectorFrame(r.target.effector,globals)[3]}, e=glm::vec3{b.jointFrame(elbow,globals)[3]},
                sh=glm::vec3{b.jointFrame(shoulder,globals)[3]};
            cross(r.goalBody,{1,0,1},.02f); cross(r.referenceBody,{0,1,1},.012f); cross(hand,{.1f,1,.2f},.012f);
            line(hand,r.goalBody,{1,.2f,.2f}); line(sh,e,{0,1,1}); line(e,hand,{0,1,1});
            line(e,e+r.bendNormalBody*.10f,{.8f,.3f,1});
            line(sh,sh+r.bendNormalBody*.04f,{.8f,.3f,1});
        }
        const auto& j=p.joints[joint_]; const auto& c=j.coordinates[coordinate_];
        const auto current=b.jointFrame(j.id,globals); axes(current,.09f);
        glm::mat4 total{1},previous{1};
        for (std::size_t k=0;k<j.coordinates.size();++k)
        {
            if (k<coordinate_) previous=glm::rotate(previous,coordinate(j.id,k),j.coordinates[k].axis);
            total=glm::rotate(total,coordinate(j.id,k),j.coordinates[k].axis);
        }
        const auto active=current*glm::inverse(total)*previous;
        const auto origin=glm::vec3{active[3]},axis=glm::vec3{active*glm::vec4{c.axis,0}};
        line(origin-axis*.1f,origin+axis*.1f,{1,1,0});
        auto tangent=glm::cross(c.axis,glm::vec3{0,1,0});
        if (glm::length(tangent)<.01f) tangent=glm::cross(c.axis,glm::vec3{1,0,0});
        tangent=glm::normalize(tangent)*.075f;
        const auto arc=[&](float angle) { return glm::vec3{active*glm::rotate(glm::mat4{1},angle,c.axis)*glm::vec4{tangent,1}}; };
        for (int k=0;k<32;++k)
            line(arc(c.lower+(c.upper-c.lower)*static_cast<float>(k)/32),arc(c.lower+(c.upper-c.lower)*static_cast<float>(k+1)/32),{1,.35f,.05f});
        line(origin,arc(c.lower),{1,.1f,.1f}); line(origin,arc(c.upper),{.1f,1,.1f});
        line(origin,arc(coordinate(j.id,coordinate_)),{1,1,0});
        renderer.renderDebugLines(lines);
    }

    void verifyDesktopReach(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,BodyDiagnostics& diagnostics,
        const glm::mat4& fit,const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_ik.txt"),csv(output/"ticks.csv");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        csv << "side,tick,action,state,error_m,actual_x,actual_y,actual_z\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step();
        require(body.snapshot().direct,"IK F2 callback failed");
        unsigned captures=0;
        const auto capture=[&](int side,int frame)
        {
            render_opengl::CameraConfig camera;
            camera.target=glm::vec3{fit*glm::vec4{model.asset().bounds().center(),1}};
            camera.position=camera.target+glm::vec3{side==0 ? .8f:-.8f,.1f,3.2f};
            renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick; const auto display=body.display(.5);
            renderer.beginFrame(); renderer.renderModel(model,fit,display.pose.palette); diagnostics.draw(renderer,fit,display);
            const auto again=body.display(.5);
            require(again.pose.palette==display.pose.palette && body.snapshot().tick==tick,"IK draw mutated simulation");
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
            glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(std::to_string(side)+"_"+std::to_string(frame)+".ppm"),std::ios::binary);
            image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image),"Cannot save IK screenshot");
            require(glGetError()==GL_NO_ERROR,"IK GL error"); ++captures;
            glfwSwapBuffers(window); glfwPollEvents();
        };
        for (int side=0;side<2;++side)
        {
            if (side) input(window,GLFW_KEY_F7,0,GLFW_PRESS,GLFW_MOD_SHIFT);
            DirectBodyControl source(body.control()->binding()); source.setEnabled(true);
            const auto shoulder=side==0 ? JointId::LeftShoulder:JointId::RightShoulder,
                elbow=side==0 ? JointId::LeftElbow:JointId::RightElbow;
            source.setCoordinate(shoulder,0,.45f); source.setCoordinate(shoulder,1,.25f); source.setCoordinate(elbow,0,.8f);
            animation::Pose targetPose(model.asset().skeleton()); source.apply(targetPose.locals);
            animation::computeGlobalTransforms(model.asset().skeleton(),targetPose.locals,targetPose.globals);
            const auto effector=side==0 ? EffectorId::LeftWrist:EffectorId::RightWrist;
            const auto goal=glm::vec3{body.control()->binding().effectorFrame(effector,targetPose.globals)[3]};
            diagnostics.setReachPosition(goal); input(window,GLFW_KEY_F7,0,GLFW_PRESS,0);
            const auto before=body.snapshot().coordinates;
            require(body.snapshot().coordinates==before,"F7 changed q outside tick");
            for (int tick=0;tick<800;++tick)
            {
                body.step(); const auto& r=body.snapshot().reaches[static_cast<std::size_t>(side)];
                csv << side << ',' << body.snapshot().tick << ',' << r.action << ',' << static_cast<int>(r.state) << ',' << r.error << ',' << r.actualBody.x << ',' << r.actualBody.y << ',' << r.actualBody.z << '\n';
                if (tick%45==0) capture(side,tick);
                if (r.state==ReachState::Succeeded) { capture(side,999); break; }
                require(tick<799,"Desktop IK did not complete");
            }
            require(body.snapshot().reaches[static_cast<std::size_t>(side)].error<=.002f,"Desktop IK accuracy failed");
            // Real keyboard editing, retarget during motion, then smooth cancellation.
            input(window,GLFW_KEY_P,0,GLFW_PRESS,0); input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
            for (int k=0;k<20;++k) body.step();
            input(window,GLFW_KEY_O,0,GLFW_PRESS,0); input(window,GLFW_KEY_F7,0,GLFW_PRESS,0); body.step();
            require(body.results().back().status==BodyCommandStatus::Applied,"Desktop retarget rejected");
            input(window,GLFW_KEY_DELETE,0,GLFW_PRESS,0); body.step();
            for (int k=0;k<800 && body.snapshot().reaches[static_cast<std::size_t>(side)].state!=ReachState::Cancelled;++k) body.step();
            require(body.snapshot().reaches[static_cast<std::size_t>(side)].state==ReachState::Cancelled,"Desktop cancel did not settle");
            capture(side,1000);
        }
        report << "captures=" << captures << "\ncallbacks=F2,F7,ShiftF7,O,P,Delete\nwrists=left,right\n"
            << "render=interpolated_read_only\nGL_errors=0\nresult=PASS\n";
    }

    void verifyDesktopBody(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,const std::filesystem::path& output,
        void (*input)(GLFWwindow*,int,int,int,int))
    {
        std::filesystem::create_directories(output);
        std::ofstream report(output/"desktop_body.txt"),samples(output/"poses.csv");
        if (!report || !samples) throw std::runtime_error("Cannot write body diagnostic evidence");
        report << "process=julia_desktop\nGL_RENDERER=" << glGetString(GL_RENDERER) << '\n';
        samples << "pose,view,colored_pixels,changed_pixels\n";
        const auto require=[](bool ok,const char* message) { if (!ok) throw std::runtime_error(message); };
        const auto& control=*body.control();
        body.enqueue(EyeCommand{EyeOperation::Enable,{0,0,0}}); body.step();
        std::size_t captures=0;
        std::array<std::vector<unsigned char>,3> zeroPixels;
        const auto capture=[&](const std::string& name,int view)
        {
            render_opengl::CameraConfig camera;
            const auto center=glm::vec3{fit*glm::vec4{model.asset().bounds().center(),1}};
            camera.target=center;
            camera.position=center+(view==0 ? glm::vec3{0,.1f,4}:glm::vec3{view==1 ? 4.0f:-4.0f,.1f,0});
            renderer.setCamera(camera);
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height); renderer.resize(width,height);
            const auto tick=body.snapshot().tick;
            const auto display=body.display(.5);
            renderer.beginFrame(); renderer.renderModel(model,fit,display.pose.palette);
            diagnostics.draw(renderer,fit,display);
            for (int repeat=0;repeat<3;++repeat)
            {
                const auto same=body.display(.5);
                renderer.beginFrame(); renderer.renderModel(model,fit,same.pose.palette);
                diagnostics.draw(renderer,fit,same);
                require(body.snapshot().tick==tick,"Render advanced simulation tick");
                require(same.pose.palette==display.pose.palette,"Repeated display changed palette");
            }
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
            glPixelStorei(GL_PACK_ALIGNMENT,1); glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream image(output/(name+(view==0 ? "_front.ppm":view==1 ? "_side.ppm":"_right_side.ppm")),std::ios::binary);
            image << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y=height-1;y>=0;--y) image.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
            require(static_cast<bool>(image),"Body capture failed");
            require(glGetError()==GL_NO_ERROR,"OpenGL body diagnostic error");
            glfwSwapBuffers(window); glfwPollEvents();
            std::size_t colored=0; for (std::size_t i=0;i<pixels.size();i+=3) if (std::max({pixels[i],pixels[i+1],pixels[i+2]})-std::min({pixels[i],pixels[i+1],pixels[i+2]})>25) ++colored;
            require(colored>1000,"Body capture contains no model/overlay");
            auto& base=zeroPixels[static_cast<std::size_t>(view)];
            if (name=="zero") base=pixels;
            std::size_t changed=0;
            if (!base.empty())
                for (std::size_t i=0;i<pixels.size();i+=3)
                    if (std::abs(static_cast<int>(pixels[i])-base[i])+std::abs(static_cast<int>(pixels[i+1])-base[i+1])+
                        std::abs(static_cast<int>(pixels[i+2])-base[i+2])>15) ++changed;
            if (view==0 && name!="zero" && !base.empty()) require(changed>10,"Body command did not change rendered pixels");
            ++captures;
            samples << name << ',' << view << ',' << colored << ',' << changed << '\n';
        };
        // Exercise the actual application callbacks for toggle, increments, reset and limits.
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0);
        require(!control.enabled(),"F2 mutated body before tick"); body.step();
        require(control.enabled(),"F2 body activation failed");
        diagnostics.select(JointId::LeftElbow);
        for (int k=0;k<4;++k) input(window,GLFW_KEY_EQUAL,0,GLFW_PRESS,0);
        require(control.coordinate(JointId::LeftElbow,0)==0,"Keyboard mutated q before tick"); body.step();
        require(std::abs(control.coordinate(JointId::LeftElbow,0)-glm::radians(20.0f))<1e-6f,"Elbow +20 command failed");
        for (int v=0;v<3;++v) capture("left_elbow20_overlay",v);
        input(window,GLFW_KEY_BACKSPACE,0,GLFW_PRESS,0);
        body.step();
        require(control.coordinate(JointId::LeftElbow,0)==0,"Body reset callback failed");
        for (int k=0;k<30;++k) input(window,GLFW_KEY_EQUAL,0,GLFW_PRESS,0);
        body.step();
        require(std::abs(control.coordinate(JointId::LeftElbow,0)-glm::radians(100.0f))<1e-6f,"Body clamp failed");
        body.enqueue(ResetCoordinates{}); body.step(); diagnostics.overlay=false;
        for (int v=0;v<3;++v) capture("zero",v);
        const auto pose=[&](const char* name,JointId id,std::size_t coordinate,float degrees)
        {
            body.enqueue(ResetCoordinates{}); diagnostics.select(id,coordinate);
            body.enqueue(SetCoordinate{id,coordinate,glm::radians(degrees)}); body.step();
            require(body.results().back().status==BodyCommandStatus::Applied,"Body pose rejected");
            for (int v=0;v<3;++v) capture(name,v);
        };
        pose("left_elbow20",JointId::LeftElbow,0,20); pose("right_elbow20",JointId::RightElbow,0,20);
        pose("left_elbow60",JointId::LeftElbow,0,60); pose("right_elbow60",JointId::RightElbow,0,60);
        pose("left_elbow100",JointId::LeftElbow,0,100); pose("right_elbow100",JointId::RightElbow,0,100);
        pose("left_knee20",JointId::LeftKnee,0,20); pose("right_knee20",JointId::RightKnee,0,20);
        pose("left_knee60",JointId::LeftKnee,0,60); pose("right_knee60",JointId::RightKnee,0,60);
        pose("left_shoulder45",JointId::LeftShoulder,1,45); pose("right_shoulder45",JointId::RightShoulder,1,45);
        pose("left_roll30",JointId::LeftShoulder,2,30); pose("right_roll30",JointId::RightShoulder,2,30);
        pose("left_wrist25",JointId::LeftWrist,0,25); pose("right_wrist25",JointId::RightWrist,0,25);
        pose("left_ankle20",JointId::LeftAnkle,0,20); pose("right_ankle20",JointId::RightAnkle,0,20);
        body.enqueue(ResetCoordinates{}); body.step(); diagnostics.overlay=true; diagnostics.select(JointId::LeftElbow);
        for (int v=0;v<3;++v) capture("zero_overlay",v);
        input(window,GLFW_KEY_F3,0,GLFW_PRESS,0); input(window,GLFW_KEY_F4,0,GLFW_PRESS,0);
        input(window,GLFW_KEY_F2,0,GLFW_PRESS,0); body.step(); require(!control.enabled(),"Body disable callback failed");
        report << "captures=" << captures << "\ncallbacks=queued_toggle,relative_increment,clamp,reset,select_joint,select_coordinate\n"
            << "snapshots=interpolated\nrepeat_draw=immutable\nGL_errors=0\nresult=PASS\n";
    }
}
