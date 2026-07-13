#include "julia/render/OrbitCamera.hpp"

#include "julia_test.hpp"

#include <cmath>

#include <glm/gtc/constants.hpp>

namespace
{
    constexpr float Epsilon = 0.0001f;

    bool nearlyEqual(float left, float right)
    {
        return std::abs(left - right) <= Epsilon;
    }

    bool nearlyEqual(const glm::vec3& left, const glm::vec3& right)
    {
        return
            nearlyEqual(left.x, right.x) &&
            nearlyEqual(left.y, right.y) &&
            nearlyEqual(left.z, right.z);
    }
}

int main()
{
    using julia::render::OrbitCamera;
    using julia::render::OrbitCameraSettings;

    OrbitCameraSettings settings{};
    settings.rotationRadiansPerPixel = glm::half_pi<float>();
    settings.zoomExponentPerStep = std::log(2.0f);
    settings.minDistance = 1.0f;
    settings.maxDistance = 8.0f;

    const glm::vec3 target{1.0f, 2.0f, 3.0f};
    OrbitCamera camera{target, target + glm::vec3{0.0f, 0.0f, 4.0f}, settings};

    JULIA_CHECK(nearlyEqual(camera.target(), target));
    JULIA_CHECK(nearlyEqual(camera.position(), target + glm::vec3{0.0f, 0.0f, 4.0f}));
    JULIA_CHECK(nearlyEqual(camera.distance(), 4.0f));

    // Horizontal and vertical input move the camera on its orbit while the
    // target and radius remain unchanged.
    camera.orbit(-0.5f, -0.25f);
    JULIA_CHECK(nearlyEqual(camera.target(), target));
    JULIA_CHECK(nearlyEqual(glm::length(camera.position() - target), 4.0f));
    JULIA_CHECK(camera.position().x > target.x);
    JULIA_CHECK(camera.position().y > target.y);
    JULIA_CHECK(camera.position().z > target.z);
    JULIA_CHECK(!nearlyEqual(camera.up(), glm::vec3{0.0f, 1.0f, 0.0f}));

    camera.zoom(1.0f);
    JULIA_CHECK(nearlyEqual(camera.distance(), 2.0f));
    JULIA_CHECK(nearlyEqual(glm::length(camera.position() - target), 2.0f));

    camera.zoom(100.0f);
    JULIA_CHECK(nearlyEqual(camera.distance(), settings.minDistance));

    camera.zoom(-100.0f);
    JULIA_CHECK(nearlyEqual(camera.distance(), settings.maxDistance));

    const glm::vec3 newTarget{-2.0f, 0.5f, 7.0f};
    camera.setTarget(newTarget);
    JULIA_CHECK(nearlyEqual(camera.target(), newTarget));
    JULIA_CHECK(nearlyEqual(
        glm::length(camera.position() - newTarget),
        settings.maxDistance
    ));

    // A free orbit can cross a pole without making lookAt's up vector
    // collinear with the view direction.
    OrbitCamera poleCamera{
        target,
        target + glm::vec3{0.0f, 0.0f, 4.0f},
        settings
    };
    poleCamera.orbit(0.0f, -1.0f);
    const glm::vec3 viewDirection = glm::normalize(
        poleCamera.target() - poleCamera.position()
    );
    JULIA_CHECK(std::abs(glm::dot(viewDirection, poleCamera.up())) <= Epsilon);

    const glm::vec3 positionBeforeHorizontalOrbit = poleCamera.position();
    poleCamera.orbit(-0.5f, 0.0f);
    JULIA_CHECK(!nearlyEqual(
        poleCamera.position(),
        positionBeforeHorizontalOrbit
    ));
    JULIA_CHECK(nearlyEqual(
        glm::length(poleCamera.position() - poleCamera.target()),
        4.0f
    ));

    return JULIA_TEST_RESULT();
}
