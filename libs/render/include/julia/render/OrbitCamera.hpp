#pragma once

#include <glm/glm.hpp>
#include <optional>

namespace julia::render
{
    // Scene-space, Y-up bind bounds. No dependency on a particular rig or renderer.
    struct CameraSubject
    {
        glm::vec3 min{-0.5f, 0.0f, -0.25f};
        glm::vec3 max{0.5f, 2.0f, 0.25f};
        std::optional<glm::vec3> face;
    };

    struct OrbitCameraSettings
    {
        float rotationRadiansPerPixel = glm::radians(0.22f);
        float zoomExponentPerStep = 0.12f;
        float minPitch = glm::radians(-35.0f);
        float maxPitch = glm::radians(70.0f);
        float minDistanceInHeights = 0.08f;
        float maxDistanceInHeights = 4.0f;
        float collisionMarginInHeights = 0.03f;
        float framingPadding = 1.12f;
        float faceHeight = 0.87f;
        float faceFrameHeight = 0.29f;
        float maxPanInHalfScreens = 0.65f;
        float keyboardPanHalfScreensPerSecond = 0.8f;
        float keyboardZoomStepsPerSecond = 7.0f;
        // Exact critically damped spring: 95% response in about 0.15 s.
        float responseTime = 0.065f;
    };

    // Input changes goals; update(seconds) advances the visible pose. Pixel
    // displacements are never multiplied by dt. Matrices belong to the renderer.
    class OrbitCamera
    {
    public:
        explicit OrbitCamera(CameraSubject subject, OrbitCameraSettings settings = {});
        void setViewport(float width, float height, float fovDegrees);
        void orbit(float horizontalPixels, float verticalPixels);
        void zoom(float wheelSteps);
        void pan(float horizontalPixels, float verticalPixels, float viewportHeight);
        void panNormalized(const glm::vec2& halfScreens);
        void frameBody(bool immediate = false);
        void focusFace();
        void update(double deltaSeconds);

        const glm::vec3& position() const noexcept { return position_; }
        const glm::vec3& target() const noexcept { return target_; }
        glm::vec3 up() const noexcept { return up_; }
        float distance() const noexcept { return distance_; }
        const OrbitCameraSettings& settings() const noexcept { return settings_; }

    private:
        OrbitCameraSettings settings_;
        CameraSubject subject_;
        float height_ = 2.0f, aspect_ = 1.0f, tanHalfFov_ = 0.41421356f;
        float yaw_ = 0.0f, desiredYaw_ = 0.0f, yawVelocity_ = 0.0f;
        float pitch_ = 0.0f, desiredPitch_ = 0.0f, pitchVelocity_ = 0.0f;
        float logDistance_ = 0.0f, desiredLogDistance_ = 0.0f, zoomVelocity_ = 0.0f;
        glm::vec3 anchor_{0.0f}, desiredAnchor_{0.0f}, anchorVelocity_{0.0f};
        // Screen-space composition, independent of the character's orbit pivot.
        glm::vec2 pan_{0.0f}, desiredPan_{0.0f}, panVelocity_{0.0f};
        glm::vec3 position_{0.0f}, target_{0.0f}, up_{0.0f, 1.0f, 0.0f};
        float distance_ = 1.0f;

        float bodyDistance() const;
        float boundedLogDistance(float value) const;
        float minimumDistance(const glm::vec3& anchor, float yaw, float pitch, glm::vec2 pan) const;
        void updatePosition();
    };
}
