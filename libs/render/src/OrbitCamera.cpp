#include "julia/render/OrbitCamera.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <glm/gtc/constants.hpp>

namespace julia::render
{
    namespace
    {
        bool finite(const glm::vec3& v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

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

    OrbitCamera::OrbitCamera(CameraSubject subject, OrbitCameraSettings settings)
        : settings_(settings), subject_(subject)
    {
        if (!finite(subject.min) || !finite(subject.max) ||
            glm::any(glm::lessThan(subject.max, subject.min)))
            throw std::invalid_argument("Camera requires finite ordered subject bounds");
        const auto size = subject.max - subject.min;
        height_ = std::max({size.x, size.y, size.z});
        if (height_ < 0.0001f || !std::isfinite(height_))
            throw std::invalid_argument("Camera requires nondegenerate subject bounds");
        if (size.y > 0.0001f) height_ = size.y;
        if (subject_.face && (!finite(*subject_.face) ||
            glm::any(glm::lessThan(*subject_.face, subject.min)) ||
            glm::any(glm::greaterThan(*subject_.face, subject.max))))
            subject_.face.reset();
        const auto positive = [](float v) { return std::isfinite(v) && v > 0.0f; };
        if (!positive(settings.rotationRadiansPerPixel) || !positive(settings.zoomExponentPerStep) ||
            !std::isfinite(settings.minPitch) || !std::isfinite(settings.maxPitch) ||
            settings.minPitch <= -glm::half_pi<float>() || settings.maxPitch >= glm::half_pi<float>() ||
            settings.minPitch > 0.0f || settings.maxPitch < 0.0f ||
            !positive(settings.minDistanceInHeights) || !positive(settings.maxDistanceInHeights) ||
            settings.maxDistanceInHeights <= settings.minDistanceInHeights ||
            !positive(settings.collisionMarginInHeights) || !positive(settings.framingPadding) ||
            !positive(settings.faceHeight) || settings.faceHeight > 1.0f ||
            !positive(settings.faceFrameHeight) || !positive(settings.maxPanInHalfScreens) ||
            settings.maxPanInHalfScreens >= 1.0f || !positive(settings.responseTime) ||
            !positive(settings.keyboardPanHalfScreensPerSecond) || !positive(settings.keyboardZoomStepsPerSecond))
            throw std::invalid_argument("Invalid orbit camera settings");
        frameBody(true);
    }

    void OrbitCamera::setViewport(float width, float height, float fovDegrees)
    {
        if (!std::isfinite(width) || !std::isfinite(height) || width <= 0 || height <= 0 ||
            !std::isfinite(fovDegrees) || fovDegrees <= 1 || fovDegrees >= 179) return;
        aspect_ = width / height;
        tanHalfFov_ = std::tan(glm::radians(fovDegrees) * 0.5f);
        // Resizing preserves user composition and zoom. B explicitly refits.
        updatePosition();
    }

    float OrbitCamera::bodyDistance() const
    {
        const auto half = (subject_.max - subject_.min) * 0.5f;
        return half.z + settings_.framingPadding *
            std::max(half.y / tanHalfFov_, half.x / (tanHalfFov_ * aspect_));
    }

    float OrbitCamera::boundedLogDistance(float value) const
    {
        return std::clamp(value, std::log(height_ * settings_.minDistanceInHeights),
            std::log(std::max(height_ * settings_.maxDistanceInHeights, bodyDistance())));
    }

    void OrbitCamera::frameBody(bool immediate)
    {
        desiredAnchor_ = (subject_.min + subject_.max) * 0.5f;
        desiredYaw_ = yaw_ + std::remainder(-yaw_, glm::two_pi<float>());
        desiredPitch_ = 0.0f;
        desiredPan_ = glm::vec2{0.0f};
        desiredLogDistance_ = boundedLogDistance(std::log(bodyDistance()));
        yawVelocity_ = pitchVelocity_ = zoomVelocity_ = 0.0f;
        anchorVelocity_ = glm::vec3{0.0f};
        panVelocity_ = glm::vec2{0.0f};
        if (immediate)
        {
            yaw_ = desiredYaw_;
            pitch_ = desiredPitch_;
            anchor_ = desiredAnchor_;
            pan_ = desiredPan_;
            logDistance_ = desiredLogDistance_;
            updatePosition();
        }
    }

    void OrbitCamera::focusFace()
    {
        desiredAnchor_ = subject_.face.value_or(glm::vec3{
            (subject_.min.x + subject_.max.x) * 0.5f,
            subject_.min.y + height_ * settings_.faceHeight,
            (subject_.min.z + subject_.max.z) * 0.5f});
        desiredPan_ = glm::vec2{0.0f};
        const float halfFrame = height_ * settings_.faceFrameHeight * 0.5f;
        const float fit = halfFrame * settings_.framingPadding /
            (tanHalfFov_ * std::min(aspect_, 1.0f));
        desiredLogDistance_ = boundedLogDistance(std::log(fit));
        anchorVelocity_ = glm::vec3{0.0f};
        panVelocity_ = glm::vec2{0.0f};
        zoomVelocity_ = 0.0f;
    }

    void OrbitCamera::orbit(float horizontalPixels, float verticalPixels)
    {
        if (!std::isfinite(horizontalPixels) || !std::isfinite(verticalPixels)) return;
        const float yawDelta = -horizontalPixels * settings_.rotationRadiansPerPixel;
        const float pitchDelta = -verticalPixels * settings_.rotationRadiansPerPixel;
        if (!std::isfinite(yawDelta) || !std::isfinite(pitchDelta)) return;
        desiredYaw_ += std::remainder(yawDelta, glm::two_pi<float>());
        desiredPitch_ = std::clamp(desiredPitch_ + pitchDelta, settings_.minPitch, settings_.maxPitch);
    }

    void OrbitCamera::zoom(float wheelSteps)
    {
        if (std::isfinite(wheelSteps) && wheelSteps != 0.0f)
        {
            const float floor = std::max(height_ * settings_.minDistanceInHeights,
                minimumDistance(desiredAnchor_, desiredYaw_, desiredPitch_, desiredPan_));
            // Accumulate at the effective limit: reversing the wheel responds
            // immediately, even after many inward ticks against the character.
            const float start = std::max(desiredLogDistance_, std::log(floor));
            desiredLogDistance_ = std::max(std::log(floor),
                boundedLogDistance(start - wheelSteps * settings_.zoomExponentPerStep));
        }
    }

    void OrbitCamera::pan(float horizontalPixels, float verticalPixels, float viewportHeight)
    {
        if (!std::isfinite(viewportHeight) || viewportHeight <= 0) return;
        panNormalized(glm::vec2{-horizontalPixels, verticalPixels} * (2.0f / viewportHeight));
    }

    void OrbitCamera::panNormalized(const glm::vec2& halfScreens)
    {
        if (!std::isfinite(halfScreens.x) || !std::isfinite(halfScreens.y)) return;
        const glm::vec2 limit{settings_.maxPanInHalfScreens * aspect_, settings_.maxPanInHalfScreens};
        desiredPan_ = glm::clamp(desiredPan_ + halfScreens, -limit, limit);
    }

    void OrbitCamera::update(double deltaSeconds)
    {
        if (!std::isfinite(deltaSeconds) || deltaSeconds <= 0) return;
        const float dt = static_cast<float>(std::min(deltaSeconds, 60.0));
        const float rate = 2.0f / settings_.responseTime;
        damp(yaw_, yawVelocity_, desiredYaw_, rate, dt);
        damp(pitch_, pitchVelocity_, desiredPitch_, rate, dt);
        damp(anchor_, anchorVelocity_, desiredAnchor_, rate, dt);
        damp(pan_, panVelocity_, desiredPan_, rate, dt);
        damp(logDistance_, zoomVelocity_, desiredLogDistance_, rate, dt);
        if (pitch_ < settings_.minPitch || pitch_ > settings_.maxPitch)
        {
            pitch_ = std::clamp(pitch_, settings_.minPitch, settings_.maxPitch);
            pitchVelocity_ = 0;
        }
        const float turns = yaw_ - std::remainder(yaw_, glm::two_pi<float>());
        yaw_ -= turns;
        desiredYaw_ -= turns;
        updatePosition();
    }

    void OrbitCamera::updatePosition()
    {
        const glm::vec3 direction{std::sin(yaw_) * std::cos(pitch_), std::sin(pitch_),
            std::cos(yaw_) * std::cos(pitch_)};
        const glm::vec3 right{std::cos(yaw_), 0, -std::sin(yaw_)};
        up_ = glm::cross(direction, right);
        const glm::vec2 limit{settings_.maxPanInHalfScreens * aspect_, settings_.maxPanInHalfScreens};
        pan_ = glm::clamp(pan_, -limit, limit);
        desiredPan_ = glm::clamp(desiredPan_, -limit, limit);
        const auto frameOffset = tanHalfFov_ * (right * pan_.x + up_ * pan_.y);
        distance_ = std::max(std::exp(boundedLogDistance(logDistance_)),
            minimumDistance(anchor_, yaw_, pitch_, pan_));
        target_ = anchor_ + frameOffset * distance_;
        position_ = target_ + direction * distance_;
    }

    float OrbitCamera::minimumDistance(const glm::vec3& anchor, float yaw, float pitch, glm::vec2 pan) const
    {
        const glm::vec3 direction{std::sin(yaw) * std::cos(pitch), std::sin(pitch),
            std::cos(yaw) * std::cos(pitch)};
        const glm::vec3 right{std::cos(yaw), 0, -std::sin(yaw)};
        const auto up = glm::cross(direction, right);
        const auto ray = direction + tanHalfFov_ * (right * pan.x + up * pan.y);
        // Exit distance from inflated static bind bounds. Static geometry avoids
        // camera jitter from breathing, blinks and idle animation.
        const float margin = height_ * settings_.collisionMarginInHeights;
        const auto low = subject_.min - glm::vec3{margin};
        const auto high = subject_.max + glm::vec3{margin};
        float entry = 0.0f, exit = std::numeric_limits<float>::max();
        for (int axis = 0; axis < 3; ++axis)
        {
            if (std::abs(ray[axis]) < 1e-7f)
            {
                if (anchor[axis] < low[axis] || anchor[axis] > high[axis]) exit = -1;
                continue;
            }
            const float a = (low[axis] - anchor[axis]) / ray[axis];
            const float b = (high[axis] - anchor[axis]) / ray[axis];
            entry = std::max(entry, std::min(a, b));
            exit = std::min(exit, std::max(a, b));
        }
        return exit >= entry ? exit : 0.0f;
    }
}
