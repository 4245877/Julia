#pragma once

#include "julia/render/EyeBehavior.hpp"
#include "julia/render_opengl/Animation.hpp"
#include <array>

namespace julia::render_opengl
{
    // Raphtalia's deliberately limited eye/lid override, before hierarchy/LBS.
    // Owns no GL resources. The imported model must outlive this object.
    class EyePoseLayer
    {
    public:
        explicit EyePoseLayer(const GLModel& model);
        bool available() const { return available_; }
        void setEnabled(bool value) { enabled_ = value && available_; }
        bool enabled() const { return enabled_; }
        render::EyeBehavior& behavior() { return behavior_; }
        const render::EyeBehavior& behavior() const { return behavior_; }
        // Point in imported model scene coordinates, before fit/model transform.
        // Both eyes use one distant direction (no artificial convergence).
        void setTargetPoint(const glm::vec3& point);
        void clearTargetPoint();
        bool hasTargetPoint() const { return targeted_; }
        static bool authoredEyes(const AnimationClip* clip);
        void update(double seconds);
        void apply(std::span<glm::mat4> locals, const AnimationClip* baseClip);
        std::size_t lidCount() const { return lids_.size(); }

    private:
        struct Lid
        {
            std::uint32_t node = 0;
            bool left = false;
            // Source bake sampled at its original times, reparameterized by
            // actual closure, retaining the nonlinear depth arc.
            std::vector<glm::vec3> offsets;
        };
        const GLModel& model_;
        render::EyeBehavior behavior_{};
        bool available_ = false, enabled_ = false, targeted_ = false;
        std::array<std::uint32_t, 2> eyes_{};
        std::uint32_t head_ = 0;
        std::vector<Lid> lids_;
        std::vector<float> closures_;
        std::vector<const AnimationClip*> gazeClips_;
        std::vector<glm::mat4> globals_;
        glm::vec3 target_{0}, eyeCenter_{0};
        glm::vec2 targetGaze_{0};
        double elapsed_ = 0;
    };
}
