#include "BodyIdle.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>

namespace julia::embodiment
{
    namespace
    {
        constexpr unsigned transitionTicks = 144;
        constexpr auto dt = static_cast<float>(runtime::BodyStepSeconds);

        struct JointMotion
        {
            JointId id;
            glm::vec3 rest, breath, sway, drift; // Calibrated flexion, bend/abduction, yaw/roll in degrees.
            float lookShare = 0;
        };
        constexpr JointMotion joints[]{
            {JointId::Pelvis,        {0,0,0},      {-.38f,0,0},      {.08f,.8f,.7f},   {.22f,.18f,.24f}},
            {JointId::Lumbar,        {.7f,0,0},    {-.65f,0,0},      {0,-.38f,-.2f},  {.1f,.12f,0}},
            {JointId::Thorax,        {0,0,0},      {-1.25f,0,0},     {0,-.24f,-.4f},  {0,.2f,.22f}},
            {JointId::Chest,         {-.5f,0,0},   {-1.9f,0,0},      {.15f,-.18f,0},  {0,.18f,-.15f}},
            {JointId::Neck,          {-.5f,0,0},   {.95f,0,0},       {0,.24f,-.17f},  {.42f,0,0},       .38f},
            {JointId::Head,          {0,0,0},      {1.1f,0,0},       {0,-.83f,.15f},  {.48f,.7f,.84f},  .62f},
            {JointId::LeftClavicle,  {0,0,0},      {.95f,0,0},       {.27f,0,0},      {.15f,0,0}},
            {JointId::RightClavicle, {0,0,0},      {.86f,0,0},       {-.22f,0,0},     {-.15f,0,0}},
            {JointId::LeftShoulder,  {1,1,0},      {-.7f,.54f,0},    {1.34f,.52f,.7f},{.42f,.22f,.47f}},
            {JointId::RightShoulder, {9,-2,-2},    {-.59f,.5f,0},    {-1.12f,-.48f,0},{-.58f,.24f,-.39f}},
            {JointId::LeftElbow,     {4,0,0},      {.85f,0,0},       {.65f,0,0},      {.55f,0,0}},
            {JointId::RightElbow,    {6,0,0},      {.74f,0,0},       {-.72f,0,0},     {.39f,0,0}},
            {JointId::LeftWrist,     {-2,1,0},     {.48f,-.16f,0},   {.75f,.42f,0},   {.38f,.34f,0}},
            {JointId::RightWrist,    {-2,0,0},     {.54f,.14f,0},    {-.66f,.35f,0},  {-.24f,.43f,0}},
        };

        float wave(double seconds, double period, double offset = 0)
        {
            return static_cast<float>(std::sin(glm::two_pi<double>() * seconds / period + offset));
        }
    }

    const char* idlePhaseName(IdlePhase phase)
    {
        switch (phase)
        {
        case IdlePhase::Disabled: return "Disabled";
        case IdlePhase::Resuming: return "Resuming";
        case IdlePhase::Idle: return "Idle";
        case IdlePhase::Settling: return "Settling";
        case IdlePhase::Ready: return "Ready";
        case IdlePhase::Fault: return "Fault";
        }
        return "Invalid";
    }

    BodyIdle::BodyIdle(const DirectBodyControl& control, const RootPose& root) : proposed_(control), root_(root)
    {
        static_assert(std::size(joints) == JointCount);
        const auto& binding = control.binding();
        for (std::size_t i = 0; i < JointCount; ++i)
        {
            indices_[i] = binding.jointIndex(joints[i].id);
            rest_.joints[i] = glm::radians(joints[i].rest);
        }
        // Leave both legs room to flex as the pelvis moves around the planted stance.
        root_.position.y -= .014f;
        rest_.position = root_.position;
        eyeCenter_ = glm::vec3{binding.profile().joints[binding.jointIndex(JointId::Head)].frameInBodyBind[3]};
        eyeCenter_ += glm::vec3{0,.072f,.036f};
        proposed_.reset();
        proposed_.setEnabled(true);
        setJoints(rest_);
        transitionStart_ = current_ = rest_;
    }

    void BodyIdle::requestStance(bool ready, const DirectBodyControl& control, const RootPose& root)
    {
        const auto phase = observation_.phase;
        if ((ready && (phase == IdlePhase::Settling || phase == IdlePhase::Ready)) ||
            (!ready && (phase == IdlePhase::Resuming || phase == IdlePhase::Idle)))
            return;
        transitionStart_.position = root.position;
        for (std::size_t i = 0; i < JointCount; ++i)
            transitionStart_.joints[i] = control.coordinates()[indices_[i]];
        transitionVelocity_ = velocity_;
        current_ = transitionStart_;
        transitionTick_ = 0;
        observation_.phase = ready ? IdlePhase::Settling : IdlePhase::Resuming;
    }

    void BodyIdle::followView(const std::optional<IdleView>& view)
    {
        glm::vec2 target{0};
        if (view)
        {
            const auto local = view->directional ? view->value : view->value - root_.position;
            const auto direction = glm::conjugate(root_.rotation) * local - (view->directional ? glm::vec3{0} : eyeCenter_);
            const float length = glm::length(direction);
            if (length > 1e-5f)
            {
                const auto degrees = glm::degrees(glm::vec2{std::atan2(direction.x, direction.z),
                    -std::atan2(direction.y, std::hypot(direction.x, direction.z))});
                const float front = glm::smoothstep(-.12f, .63f, direction.z / length);
                target = glm::clamp(degrees, glm::vec2{-30,-14}, glm::vec2{30,12}) * front;
            }
        }
        // Critically damped head tracking keeps velocity continuous when the viewport changes.
        constexpr float rate = 8;
        const float decay = std::exp(-rate * dt);
        const auto error = look_ - target;
        const auto slope = lookVelocity_ + rate * error;
        look_ = target + (error + slope * dt) * decay;
        lookVelocity_ = (lookVelocity_ - rate * slope * dt) * decay;
    }

    BodyIdle::Pose BodyIdle::sample(double seconds) const
    {
        // Slow modulation avoids a fixed breathing loop. The arms lag the torso slightly.
        const float breath = wave(seconds, 4.8, .3 * wave(seconds, 23.1));
        const float armBreath = wave(seconds - .23, 4.8, .3 * wave(seconds, 23.1));
        const float sway = .74f * wave(seconds, 12.7, -.6) + .26f * wave(seconds, 21.4, .8);
        const float drift = wave(seconds, 17.9, .4 + .4 * wave(seconds, 31.3));
        Pose pose = rest_;
        pose.position += root_.rotation * glm::vec3{.011f * sway, .0036f * breath, .004f * drift};
        for (std::size_t i = 0; i < JointCount; ++i)
        {
            const auto& joint = joints[i];
            const float inhale = joint.id >= JointId::LeftClavicle ? armBreath : breath;
            pose.joints[i] += glm::radians(joint.breath * inhale + joint.sway * sway + joint.drift * drift +
                glm::vec3{look_.y,0,look_.x} * joint.lookShare);
        }
        return pose;
    }

    void BodyIdle::setJoints(const Pose& pose)
    {
        const auto& definitions = proposed_.binding().profile().joints;
        for (std::size_t i = 0; i < JointCount; ++i)
            for (std::size_t k = 0; k < definitions[indices_[i]].coordinates.size(); ++k)
                proposed_.setCoordinate(joints[i].id, k, pose.joints[i][static_cast<int>(k)], LimitPolicy::Clamp);
    }

    bool BodyIdle::advance(LegSupport& support, const std::optional<IdleView>& view)
    {
        const auto phase = observation_.phase;
        if (phase == IdlePhase::Ready || phase == IdlePhase::Fault) return false;
        const bool settling = phase == IdlePhase::Settling;
        const double seconds = observation_.phaseSeconds + (settling ? 0 : runtime::BodyStepSeconds);
        followView(view);
        auto next = settling ? rest_ : sample(seconds);
        const bool transitioning = phase != IdlePhase::Idle;
        if (transitioning)
        {
            ++transitionTick_;
            const float u = static_cast<float>(transitionTick_) / transitionTicks;
            const float weight = u*u*u * (10 + u * (-15 + 6*u));
            const float rest = 1 - u;
            // Blend into the new trajectory and carry the outgoing velocity at u=0.
            const float momentum = transitionTick_ * dt * rest*rest*rest;
            next.position = glm::mix(transitionStart_.position, next.position, weight) + transitionVelocity_.position * momentum;
            for (std::size_t i = 0; i < JointCount; ++i)
                next.joints[i] = glm::mix(transitionStart_.joints[i], next.joints[i], weight) + transitionVelocity_.joints[i] * momentum;
        }
        const auto worldFromBody = glm::translate(glm::mat4{1}, next.position) * glm::mat4_cast(root_.rotation);
        const bool finished = transitioning && transitionTick_ == transitionTicks;
        const bool ready = finished && settling;
        setJoints(next);
        if (!support.track(proposed_, worldFromBody, ready))
        {
            observation_.phase = IdlePhase::Fault;
            return false;
        }
        for (std::size_t i = 0; i < JointCount; ++i)
            next.joints[i] = proposed_.coordinates()[indices_[i]];
        velocity_.position = (next.position - current_.position) / dt;
        for (std::size_t i = 0; i < JointCount; ++i)
            velocity_.joints[i] = (next.joints[i] - current_.joints[i]) / dt;
        current_ = next;
        root_.position = next.position;
        observation_.phaseSeconds = seconds;
        if (finished) observation_.phase = ready ? IdlePhase::Ready : IdlePhase::Idle;
        if (ready) velocity_ = {};
        return true;
    }
}
