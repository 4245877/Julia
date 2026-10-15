#include "julia/render/OrbitCamera.hpp"

#include <algorithm>
#include <cmath>

#include <glm/gtc/constants.hpp>

namespace julia::render
{
    namespace
    {
        constexpr float MinimumAllowedDistance = 0.001f;

        bool isFinite(const glm::vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) &&
                   std::isfinite(value.z);
        }

        // Exact solution of x'' + 2*w*x' + w*w*(x-goal) = 0.
        // Unlike a per-frame lerp, this preserves acceleration and velocity
        // across frames and is independent of the number of time steps.
        template<typename T>
        void damp(T& value, T& velocity, const T& goal, float rate, float dt)
        {
            const T offset = value - goal;
            const T step = velocity + offset * rate;
            const float decay = std::exp(-rate * dt);
            value = goal + (offset + step * dt) * decay;
            velocity = (velocity - step * (rate * dt)) * decay;
        }
    }

    OrbitCamera::OrbitCamera(const glm::vec3& target, const glm::vec3& position,
                             OrbitCameraSettings settings)
        : settings_(settings)
    {
        const OrbitCameraSettings defaults{};
        if (!std::isfinite(settings_.rotationRadiansPerPixel) ||
            settings_.rotationRadiansPerPixel < 0.0f)
            settings_.rotationRadiansPerPixel = defaults.rotationRadiansPerPixel;
        if (!std::isfinite(settings_.zoomExponentPerStep) ||
            settings_.zoomExponentPerStep < 0.0f)
            settings_.zoomExponentPerStep = defaults.zoomExponentPerStep;
        if (!std::isfinite(settings_.minDistance) ||
            settings_.minDistance < MinimumAllowedDistance)
            settings_.minDistance = defaults.minDistance;
        if (!std::isfinite(settings_.maxDistance))
            settings_.maxDistance = defaults.maxDistance;
        settings_.maxDistance = std::max(settings_.maxDistance, settings_.minDistance);
        if (!std::isfinite(settings_.responseTime) || settings_.responseTime < 0.005f)
            settings_.responseTime = defaults.responseTime;

        target_ = isFinite(target) ? target : glm::vec3{0.0f};
        desiredTarget_ = target_;
        const glm::vec3 offset = isFinite(position)
            ? position - target_ : glm::vec3{0.0f, 0.0f, 1.0f};
        const float length = glm::length(offset);
        glm::vec3 direction{0.0f, 0.0f, 1.0f};
        distance_ = settings_.minDistance;
        if (std::isfinite(length) && length > MinimumAllowedDistance)
        {
            direction = offset / length;
            distance_ = std::clamp(length, settings_.minDistance, settings_.maxDistance);
        }
        glm::vec3 up{0.0f, 1.0f, 0.0f};
        if (std::abs(glm::dot(up, direction)) > 0.999f)
            up = glm::vec3{0.0f, 0.0f, 1.0f};
        const glm::vec3 right = glm::normalize(glm::cross(up, direction));
        up = glm::normalize(glm::cross(direction, right));
        orientation_ = glm::quat_cast(glm::mat3{right, up, direction});
        desiredOrientation_ = orientation_;
        logDistance_ = desiredLogDistance_ = std::log(distance_);
        updatePosition();
    }

    void OrbitCamera::orbit(float horizontalPixels, float verticalPixels)
    {
        if (!std::isfinite(horizontalPixels) || !std::isfinite(verticalPixels))
            return;
        const float yaw = std::remainder(
            -horizontalPixels * settings_.rotationRadiansPerPixel, glm::two_pi<float>());
        const float pitch = std::remainder(
            verticalPixels * settings_.rotationRadiansPerPixel, glm::two_pi<float>());
        if (!std::isfinite(yaw) || !std::isfinite(pitch))
            return;
        // Local-axis rotations retain the existing free orbit through poles.
        desiredOrientation_ = glm::normalize(desiredOrientation_ *
            glm::angleAxis(yaw, glm::vec3{0.0f, 1.0f, 0.0f}) *
            glm::angleAxis(pitch, glm::vec3{1.0f, 0.0f, 0.0f}));
    }

    void OrbitCamera::zoom(float wheelSteps)
    {
        if (!std::isfinite(wheelSteps))
            return;
        // Accumulate against the goal, so fast wheel events are never lost.
        desiredLogDistance_ = std::clamp(
            desiredLogDistance_ - wheelSteps * settings_.zoomExponentPerStep,
            std::log(settings_.minDistance), std::log(settings_.maxDistance));
    }

    void OrbitCamera::pan(float horizontalPixels, float verticalPixels,
                          float viewportHeight, float fovDegrees)
    {
        if (!std::isfinite(horizontalPixels) || !std::isfinite(verticalPixels) ||
            !std::isfinite(viewportHeight) || viewportHeight <= 0.0f ||
            !std::isfinite(fovDegrees) || fovDegrees <= 0.0f || fovDegrees >= 179.0f)
            return;
        const float unitsPerPixel = 2.0f * distance_ *
            std::tan(glm::radians(fovDegrees) * 0.5f) / viewportHeight;
        moveLocal(glm::vec3{-horizontalPixels, verticalPixels, 0.0f} * unitsPerPixel);
    }

    void OrbitCamera::moveLocal(const glm::vec3& displacement)
    {
        if (isFinite(displacement))
            setTarget(desiredTarget_ + glm::normalize(orientation_) * displacement);
    }

    void OrbitCamera::setTarget(const glm::vec3& target)
    {
        if (isFinite(target))
            desiredTarget_ = target;
    }

    void OrbitCamera::update(double deltaSeconds)
    {
        if (!std::isfinite(deltaSeconds) || deltaSeconds <= 0.0)
            return;
        // After a long suspension the spring is already settled. Bounding the
        // time at one minute only avoids overflow for invalid external clocks.
        const float dt = static_cast<float>(std::min(deltaSeconds, 60.0));
        const float rate = 2.0f / settings_.responseTime;
        glm::quat goal = desiredOrientation_;
        if (glm::dot(orientation_, goal) < 0.0f)
            goal = -goal;
        // Keep the spring's raw quaternion and velocity unnormalised: the
        // analytic solution then composes exactly across different frame rates.
        damp(orientation_, orientationVelocity_, goal, rate, dt);
        damp(target_, targetVelocity_, desiredTarget_, rate, dt);
        damp(logDistance_, zoomVelocity_, desiredLogDistance_, rate, dt);
        const float bounded = std::clamp(logDistance_,
            std::log(settings_.minDistance), std::log(settings_.maxDistance));
        if (bounded != logDistance_)
        {
            logDistance_ = bounded;
            zoomVelocity_ = 0.0f;
        }
        distance_ = std::clamp(std::exp(logDistance_),
            settings_.minDistance, settings_.maxDistance);
        updatePosition();
    }

    const glm::vec3& OrbitCamera::position() const noexcept { return position_; }
    const glm::vec3& OrbitCamera::target() const noexcept { return target_; }
    glm::vec3 OrbitCamera::up() const noexcept { return up_; }
    float OrbitCamera::distance() const noexcept { return distance_; }

    void OrbitCamera::updatePosition()
    {
        const glm::quat rotation = glm::normalize(orientation_);
        up_ = rotation * glm::vec3{0.0f, 1.0f, 0.0f};
        position_ = target_ + rotation * glm::vec3{0.0f, 0.0f, distance_};
    }
}
