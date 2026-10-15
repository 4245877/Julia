#include "julia/render/OrbitCamera.hpp"

#include "julia_test.hpp"

#include <cmath>
#include <limits>

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
    camera.update(1.0);
    JULIA_CHECK(nearlyEqual(camera.target(), target));
    JULIA_CHECK(nearlyEqual(glm::length(camera.position() - target), 4.0f));
    JULIA_CHECK(camera.position().x > target.x);
    JULIA_CHECK(camera.position().y > target.y);
    JULIA_CHECK(camera.position().z > target.z);
    JULIA_CHECK(!nearlyEqual(camera.up(), glm::vec3{0.0f, 1.0f, 0.0f}));

    camera.zoom(1.0f);
    camera.update(1.0);
    JULIA_CHECK(nearlyEqual(camera.distance(), 2.0f));
    JULIA_CHECK(nearlyEqual(glm::length(camera.position() - target), 2.0f));

    camera.zoom(100.0f);
    camera.update(1.0);
    JULIA_CHECK(nearlyEqual(camera.distance(), settings.minDistance));

    camera.zoom(-100.0f);
    camera.update(1.0);
    JULIA_CHECK(nearlyEqual(camera.distance(), settings.maxDistance));

    const glm::vec3 newTarget{-2.0f, 0.5f, 7.0f};
    camera.setTarget(newTarget);
    camera.update(1.0);
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
    poleCamera.update(1.0);
    const glm::vec3 viewDirection = glm::normalize(
        poleCamera.target() - poleCamera.position()
    );
    JULIA_CHECK(std::abs(glm::dot(viewDirection, poleCamera.up())) <= Epsilon);

    const glm::vec3 positionBeforeHorizontalOrbit = poleCamera.position();
    poleCamera.orbit(-0.5f, 0.0f);
    poleCamera.update(1.0);
    JULIA_CHECK(!nearlyEqual(
        poleCamera.position(),
        positionBeforeHorizontalOrbit
    ));
    JULIA_CHECK(nearlyEqual(
        glm::length(poleCamera.position() - poleCamera.target()),
        4.0f
    ));

    // Input changes goals only; the visible pose starts gently, then slows
    // down to the exact destination without perpetual inertia.
    OrbitCamera smooth{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}};
    smooth.orbit(-80.0f, 30.0f);
    smooth.zoom(2.0f);
    smooth.setTarget({0.5f, -0.2f, 0.3f});
    JULIA_CHECK(nearlyEqual(smooth.position(), {0.0f, 0.0f, 4.0f}));
    smooth.update(0.001);
    const float firstStep = smooth.target().x;
    smooth.update(0.001);
    JULIA_CHECK(firstStep > 0.0f && firstStep < 0.001f);
    JULIA_CHECK(smooth.target().x - firstStep > firstStep);
    smooth.update(0.148);
    JULIA_CHECK(smooth.target().x > 0.47f && smooth.target().x < 0.5f);
    smooth.update(1.0);
    JULIA_CHECK(nearlyEqual(smooth.target(), {0.5f, -0.2f, 0.3f}));
    JULIA_CHECK(nearlyEqual(smooth.distance(), 4.0f * std::exp(-0.24f)));
    const auto settled = smooth.position();
    smooth.update(1.0);
    JULIA_CHECK(nearlyEqual(smooth.position(), settled));

    // Compare the transient pose, not only the final pose, at several FPS.
    const auto simulate = [](int fps) {
        OrbitCamera value{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}};
        value.orbit(-120.0f, 55.0f);
        value.zoom(3.0f);
        value.setTarget({0.5f, -0.2f, 0.3f});
        for (int i = 0; i < fps / 10; ++i)
            value.update(1.0 / static_cast<double>(fps));
        return value;
    };
    const auto reference = simulate(30);
    for (const int fps : {60, 120, 240})
    {
        const auto value = simulate(fps);
        JULIA_CHECK(nearlyEqual(reference.position(), value.position()));
        JULIA_CHECK(nearlyEqual(reference.target(), value.target()));
        JULIA_CHECK(nearlyEqual(reference.up(), value.up()));
    }
    OrbitCamera irregular{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}};
    irregular.orbit(-120.0f, 55.0f);
    irregular.zoom(3.0f);
    irregular.setTarget({0.5f, -0.2f, 0.3f});
    for (const double dt : {0.007, 0.025, 0.004, 0.041, 0.023})
        irregular.update(dt);
    JULIA_CHECK(nearlyEqual(reference.position(), irregular.position()));

    // Pan follows screen axes, and local movement supports all three axes.
    OrbitCamera panCamera{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 4.0f}};
    panCamera.pan(100.0f, -50.0f, 1000.0f, 90.0f);
    panCamera.update(1.0);
    JULIA_CHECK(nearlyEqual(panCamera.target(), {-0.8f, -0.4f, 0.0f}));
    panCamera.moveLocal({1.0f, 2.0f, -3.0f});
    panCamera.update(1.0);
    JULIA_CHECK(nearlyEqual(panCamera.target(), {0.2f, 1.6f, -3.0f}));
    JULIA_CHECK(nearlyEqual(panCamera.position() - panCamera.target(), {0.0f, 0.0f, 4.0f}));

    // Rapid direction changes and extreme wheel input stay inside bounds.
    for (int i = 0; i < 500; ++i)
    {
        camera.zoom((i % 2 == 0) ? 1000.0f : -1000.0f);
        camera.orbit(3.0f, -2.0f);
        camera.update(1.0 / 144.0);
        JULIA_CHECK(camera.distance() >= settings.minDistance - Epsilon);
        JULIA_CHECK(camera.distance() <= settings.maxDistance + Epsilon);
        const auto view = glm::normalize(camera.target() - camera.position());
        JULIA_CHECK(std::abs(glm::dot(view, camera.up())) < Epsilon);
    }
    const auto beforeInvalid = camera.position();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    camera.orbit(nan, 0.0f);
    camera.zoom(nan);
    camera.moveLocal({nan, 0.0f, 0.0f});
    camera.update(nan);
    camera.update(-1.0);
    camera.update(0.0);
    JULIA_CHECK(nearlyEqual(camera.position(), beforeInvalid));

    return JULIA_TEST_RESULT();
}
