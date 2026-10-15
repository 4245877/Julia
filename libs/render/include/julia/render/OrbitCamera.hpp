#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace julia::render
{
    struct OrbitCameraSettings
    {
        float rotationRadiansPerPixel = glm::radians(0.25f);
        float zoomExponentPerStep = 0.12f;
        float minDistance = 0.35f;
        float maxDistance = 30.0f;
        // Critically damped response: about 95% of an input in 0.15 seconds.
        float responseTime = 0.065f;
    };

    // Input changes the desired pose; update(seconds) advances the visible pose.
    // Pixel deltas are displacements and must NOT be multiplied by delta time.
    class OrbitCamera
    {
    public:
        OrbitCamera(
            const glm::vec3& target,
            const glm::vec3& position,
            OrbitCameraSettings settings = {}
        );

        void orbit(float horizontalPixels, float verticalPixels);
        void zoom(float wheelSteps);
        void pan(float horizontalPixels, float verticalPixels,
                 float viewportHeight, float fovDegrees);
        void moveLocal(const glm::vec3& displacement);
        void update(double deltaSeconds);

        void setTarget(const glm::vec3& target);

        const glm::vec3& position() const noexcept;
        const glm::vec3& target() const noexcept;
        glm::vec3 up() const noexcept;
        float distance() const noexcept;

    private:
        OrbitCameraSettings settings_{};
        glm::vec3 target_{0.0f};
        glm::vec3 position_{0.0f, 0.0f, 1.0f};
        glm::vec3 up_{0.0f, 1.0f, 0.0f};
        float distance_ = 1.0f;

        glm::vec3 desiredTarget_{0.0f};
        glm::vec3 targetVelocity_{0.0f};
        glm::quat orientation_{1.0f, 0.0f, 0.0f, 0.0f};
        glm::quat desiredOrientation_{1.0f, 0.0f, 0.0f, 0.0f};
        glm::quat orientationVelocity_{0.0f, 0.0f, 0.0f, 0.0f};
        float logDistance_ = 0.0f;
        float desiredLogDistance_ = 0.0f;
        float zoomVelocity_ = 0.0f;

        void updatePosition();
    };
}
