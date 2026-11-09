#pragma once
#include "julia/embodiment/BodyProfile.hpp"
#include "julia/embodiment/Animation.hpp"
#include "julia/embodiment/BodyInstance.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include <filesystem>
#include <optional>

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
        bool overlay = true;
        void setReachPosition(glm::vec3 position) { reachPosition_=position; }
    private:
        const embodiment::DirectBodyControl& control_;
        embodiment::BodyInstance& body_;
        std::optional<std::pair<std::uint64_t,bool>> pendingMode_;
        std::size_t joint_=0,coordinate_=0;
        std::string result_="ready";
        embodiment::ArmSide reachArm_=embodiment::ArmSide::Left;
        embodiment::MovementStrategy reachStrategy_=embodiment::MovementStrategy::DirectOnly;
        glm::vec3 reachPosition_{.22f,1.10f,.32f};
        std::uint64_t sceneRevision_=0;
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
}
