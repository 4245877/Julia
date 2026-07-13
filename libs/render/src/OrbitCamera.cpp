#include "julia/render/OrbitCamera.hpp"

#include <algorithm>
#include <cmath>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

namespace julia::render
{
    namespace
    {
        constexpr float MinimumAllowedDistance = 0.001f;

        bool isFinite(const glm::vec3& value)
        {
            return
                std::isfinite(value.x) &&
                std::isfinite(value.y) &&
                std::isfinite(value.z);
        }
    }

    OrbitCamera::OrbitCamera(
        const glm::vec3& target,
        const glm::vec3& position,
        OrbitCameraSettings settings
    )
        : settings_(settings)
    {
        const OrbitCameraSettings defaults{};

        if (!std::isfinite(settings_.rotationRadiansPerPixel) ||
            settings_.rotationRadiansPerPixel < 0.0f)
        {
            settings_.rotationRadiansPerPixel =
                defaults.rotationRadiansPerPixel;
        }

        if (!std::isfinite(settings_.zoomExponentPerStep) ||
            settings_.zoomExponentPerStep < 0.0f)
        {
            settings_.zoomExponentPerStep = defaults.zoomExponentPerStep;
        }

        if (!std::isfinite(settings_.minDistance) ||
            settings_.minDistance < MinimumAllowedDistance)
        {
            settings_.minDistance = defaults.minDistance;
        }

        if (!std::isfinite(settings_.maxDistance))
        {
            settings_.maxDistance = defaults.maxDistance;
        }

        settings_.maxDistance = std::max(
            settings_.maxDistance,
            settings_.minDistance
        );

        target_ = isFinite(target) ? target : glm::vec3{0.0f};
        const glm::vec3 safePosition = isFinite(position)
            ? position
            : target_ + glm::vec3{0.0f, 0.0f, 1.0f};

        const glm::vec3 offset = safePosition - target_;
        const float initialDistance = glm::length(offset);

        if (std::isfinite(initialDistance) &&
            initialDistance > MinimumAllowedDistance)
        {
            directionFromTarget_ = offset / initialDistance;
            distance_ = std::clamp(
                initialDistance,
                settings_.minDistance,
                settings_.maxDistance
            );
        }
        else
        {
            distance_ = settings_.minDistance;
        }

        orthonormalizeOrientation();
        updatePosition();
    }

    void OrbitCamera::orbit(float horizontalPixels, float verticalPixels)
    {
        if (!std::isfinite(horizontalPixels) || !std::isfinite(verticalPixels))
        {
            return;
        }

        const float horizontalAngle = std::remainder(
            -horizontalPixels * settings_.rotationRadiansPerPixel,
            glm::two_pi<float>()
        );

        if (std::isfinite(horizontalAngle) && horizontalAngle != 0.0f)
        {
            const glm::quat horizontalRotation = glm::angleAxis(
                horizontalAngle,
                up_
            );

            directionFromTarget_ =
                horizontalRotation * directionFromTarget_;
            up_ = horizontalRotation * up_;
        }

        orthonormalizeOrientation();

        const glm::vec3 viewDirection = -directionFromTarget_;
        const glm::vec3 right = glm::normalize(
            glm::cross(viewDirection, up_)
        );

        const float verticalAngle = std::remainder(
            verticalPixels * settings_.rotationRadiansPerPixel,
            glm::two_pi<float>()
        );

        if (std::isfinite(verticalAngle) && verticalAngle != 0.0f)
        {
            const glm::quat verticalRotation = glm::angleAxis(
                verticalAngle,
                right
            );

            directionFromTarget_ = verticalRotation * directionFromTarget_;
            up_ = verticalRotation * up_;
        }

        orthonormalizeOrientation();
        updatePosition();
    }

    void OrbitCamera::zoom(float wheelSteps)
    {
        if (!std::isfinite(wheelSteps) || wheelSteps == 0.0f)
        {
            return;
        }

        const float zoomFactor = std::exp(
            -wheelSteps * settings_.zoomExponentPerStep
        );

        distance_ = std::clamp(
            distance_ * zoomFactor,
            settings_.minDistance,
            settings_.maxDistance
        );

        updatePosition();
    }

    void OrbitCamera::setTarget(const glm::vec3& target)
    {
        if (!isFinite(target))
        {
            return;
        }

        target_ = target;
        updatePosition();
    }

    const glm::vec3& OrbitCamera::position() const noexcept
    {
        return position_;
    }

    const glm::vec3& OrbitCamera::target() const noexcept
    {
        return target_;
    }

    glm::vec3 OrbitCamera::up() const noexcept
    {
        return up_;
    }

    float OrbitCamera::distance() const noexcept
    {
        return distance_;
    }

    void OrbitCamera::orthonormalizeOrientation()
    {
        directionFromTarget_ = glm::normalize(directionFromTarget_);

        const glm::vec3 viewDirection = -directionFromTarget_;

        glm::vec3 upCandidate = up_;
        if (!isFinite(upCandidate) ||
            glm::length(upCandidate) <= MinimumAllowedDistance ||
            std::abs(glm::dot(glm::normalize(upCandidate), viewDirection)) > 0.999f)
        {
            upCandidate = glm::vec3{0.0f, 1.0f, 0.0f};

            if (std::abs(glm::dot(upCandidate, viewDirection)) > 0.999f)
            {
                upCandidate = glm::vec3{0.0f, 0.0f, 1.0f};
            }
        }

        const glm::vec3 right = glm::normalize(
            glm::cross(viewDirection, upCandidate)
        );

        up_ = glm::normalize(glm::cross(right, viewDirection));
    }

    void OrbitCamera::updatePosition()
    {
        position_ = target_ + directionFromTarget_ * distance_;
    }
}
