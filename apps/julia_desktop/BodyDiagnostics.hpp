#pragma once
#include "julia/embodiment/BodyProfile.hpp"
#include "julia/embodiment/Animation.hpp"
#include "julia/embodiment/BodyInstance.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include <filesystem>
#include <optional>
#include "../common/InteractionDemo.hpp"
#include "../common/SupportDemo.hpp"
#include "../common/StanceReachDemo.hpp"

struct GLFWwindow;
namespace julia::desktop
{
    class BodyDiagnostics
    {
    public:
        explicit BodyDiagnostics(embodiment::BodyInstance& body);
        bool key(int key,int mods);
        std::string title() const;
        void select(embodiment::JointId id,std::size_t coordinate=0);
        void draw(render_opengl::OpenGLRenderer& renderer,const glm::mat4& viewerFromResource,
            const embodiment::BodyDisplay& display) const;
        void updateInteractionDemo() { if (demo_.stage) demo_.drive(body_); if (supportDemo_.stage) supportDemo_.drive(body_); if (stanceDemo_.stage) stanceDemo_.drive(body_); }
        bool overlay = true;
        void setReachPosition(glm::vec3 position) { reachPosition_=position; }
    private:
        app::InteractionDemo demo_;
        app::SupportDemo supportDemo_;
        app::StanceReachDemo stanceDemo_;
        std::array<embodiment::SoleContact,2> soleContacts_;
        const embodiment::DirectBodyControl& control_;
        embodiment::BodyInstance& body_;
        std::optional<std::pair<std::uint64_t,bool>> pendingMode_;
        std::size_t joint_=0,coordinate_=0;
        std::string result_="ready";
        embodiment::ArmSide reachArm_=embodiment::ArmSide::Left;
        embodiment::MovementStrategy reachStrategy_=embodiment::MovementStrategy::DirectOnly;
        glm::vec3 reachPosition_{.22f,1.10f,.32f};
        std::uint64_t sceneRevision_=0;
        unsigned reachConfigurations_=1;
        embodiment::RouteSelection reachSelection_=embodiment::RouteSelection::FirstValid;
        bool elbowPositive_=true;
        embodiment::ReachTask reachTask_=embodiment::ReachTask::PositionOnly;
        glm::quat reachOrientation_{1,0,0,0};
        unsigned spatialSample_=2, reachEffector_=0;
    };
    void verifyDesktopBody(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopReach(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopCollision(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopDetour(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopMultiIK(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopRanked(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopSupport(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopStanceReach(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopInteraction(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
    void verifyDesktopSpatial(GLFWwindow* window,render_opengl::OpenGLRenderer& renderer,
        const render_opengl::GLModel& model,embodiment::BodyInstance& body,
        BodyDiagnostics& diagnostics,const glm::mat4& fit,
        const std::filesystem::path& output,void (*input)(GLFWwindow*,int,int,int,int));
}
