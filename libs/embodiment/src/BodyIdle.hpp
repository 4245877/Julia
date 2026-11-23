#pragma once

#include "julia/embodiment/BodyInstance.hpp"

namespace julia::embodiment
{
    class BodyIdle
    {
    public:
        BodyIdle(const DirectBodyControl& control, const RootPose& root);
        void requestStance(const SetIdleStance& command, const DirectBodyControl& control, const RootPose& root);
        bool advance(LegSupport& support, const std::optional<IdleView>& view);
        const IdleObservation& observation() const { return observation_; }
        DirectBodyControl& proposed() { return proposed_; }
        const RootPose& root() const { return root_; }

    private:
        static constexpr std::size_t JointCount = 14;
        struct Pose
        {
            glm::vec3 position{0};
            std::array<glm::vec3, JointCount> joints{};
        };
        DirectBodyControl proposed_;
        RootPose root_;
        Pose rest_, ready_, current_, velocity_{}, transitionStart_, transitionVelocity_{};
        std::array<std::size_t, JointCount> indices_{};
        IdleObservation observation_{IdlePhase::Resuming};
        glm::vec3 eyeCenter_{0};
        glm::vec2 look_{0}, lookVelocity_{0};
        unsigned transitionTick_ = 0;

        Pose sample(double seconds) const;
        glm::vec2 viewAngles(const IdleView& view,glm::vec3 rootPosition) const;
        void followView(const std::optional<IdleView>& view);
        void setJoints(const Pose& pose);
    };
}
