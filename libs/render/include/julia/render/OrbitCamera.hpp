#pragma once

#include <glm/glm.hpp>

namespace julia::render
{
    struct OrbitCameraSettings
    {
        float rotationRadiansPerPixel = glm::radians(0.25f);
        float zoomExponentPerStep = 0.12f;
        float minDistance = 0.35f;
        float maxDistance = 30.0f;
    };

    // Free orbit camera that always looks at target(). The cursor layer
    // supplies pixel deltas, while this class owns the camera math and limits.
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

        void setTarget(const glm::vec3& target);

        const glm::vec3& position() const noexcept;
        const glm::vec3& target() const noexcept;
        glm::vec3 up() const noexcept;
        float distance() const noexcept;

    private:
        OrbitCameraSettings settings_{};
        glm::vec3 target_{0.0f};
        glm::vec3 position_{0.0f, 0.0f, 1.0f};
        glm::vec3 directionFromTarget_{0.0f, 0.0f, 1.0f};
        glm::vec3 up_{0.0f, 1.0f, 0.0f};
        float distance_ = 1.0f;

        void orthonormalizeOrientation();
        void updatePosition();
    };
}
