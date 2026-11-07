#include "julia/render/OrbitCamera.hpp"
#include "../apps/julia_desktop/CameraInput.hpp"
#include "julia_test.hpp"
#include <cmath>
#include <limits>
#include <glm/gtc/matrix_transform.hpp>

namespace
{
    using julia::render::OrbitCamera;
    using julia::render::CameraSubject;
    constexpr float Epsilon = 0.0001f;
    bool near(float a, float b) { return std::abs(a - b) < Epsilon; }
    bool near(glm::vec3 a, glm::vec3 b) { return glm::length(a - b) < Epsilon; }
    glm::vec3 direction(const OrbitCamera& c) { return glm::normalize(c.position() - c.target()); }
    CameraSubject subject{{-0.7f, 0, -0.3f}, {0.7f, 2, 0.3f}, glm::vec3{0, 1.73f, 0.05f}};

    glm::vec3 project(const OrbitCamera& c, const glm::vec3& point, float aspect = 1.0f)
    {
        const auto clip = glm::perspective(glm::radians(45.0f), aspect, .02f, 200.0f) *
            glm::lookAt(c.position(), c.target(), c.up()) * glm::vec4{point, 1};
        return glm::vec3{clip} / clip.w;
    }
}

int main()
{
    OrbitCamera camera{subject};
    const auto initial = camera.position();
    JULIA_CHECK(near(camera.target(), {0,1,0}));
    JULIA_CHECK(near(direction(camera), {0,0,1}));

    // Full-body fit accounts for both FOV axes, depth, portrait and wide windows.
    for (const float aspect : {0.3f, 1.0f, 2.4f})
    {
        camera.setViewport(1000 * aspect, 1000, 45);
        camera.frameBody(true);
        for (int i = 0; i < 8; ++i)
        {
            const glm::vec3 corner{(i & 1) ? subject.max.x : subject.min.x,
                (i & 2) ? subject.max.y : subject.min.y, (i & 4) ? subject.max.z : subject.min.z};
            const auto p = project(camera, corner, aspect);
            JULIA_CHECK(std::abs(p.x) < 1 && std::abs(p.y) < 1 && p.z < 1);
        }
    }
    camera.setViewport(1000, 1000, 45);
    camera.frameBody(true);
    camera.orbit(-100, -20000);
    camera.update(1);
    JULIA_CHECK(near(std::asin(direction(camera).y), camera.settings().maxPitch));
    JULIA_CHECK(camera.up().y > 0);
    camera.orbit(-200, 40000);
    camera.update(1);
    JULIA_CHECK(near(std::asin(direction(camera).y), camera.settings().minPitch));
    JULIA_CHECK(camera.up().y > 0);
    JULIA_CHECK(near(camera.up().x * direction(camera).z - camera.up().z * direction(camera).x, 0));

    // Face focusing preserves the chosen angle, has no immediate discontinuity,
    // and remains stable without any per-frame animation/bone tracking.
    const auto beforeFace = camera.position();
    const auto angle = direction(camera);
    camera.focusFace();
    JULIA_CHECK(near(camera.position(), beforeFace));
    camera.update(1);
    JULIA_CHECK(near(camera.target(), *subject.face));
    JULIA_CHECK(near(direction(camera), angle));
    const auto facePose = camera.position();
    for (int i = 0; i < 500; ++i) camera.update(1.0 / 144.0);
    JULIA_CHECK(near(camera.position(), facePose));
    camera.frameBody();
    JULIA_CHECK(near(camera.position(), facePose));
    camera.update(1);
    JULIA_CHECK(near(camera.position(), initial));

    camera.zoom(10000);
    camera.update(1);
    const float closest = camera.distance();
    camera.zoom(-1);
    camera.update(1);
    JULIA_CHECK(camera.distance() > closest * 1.1f);
    camera.frameBody(true);

    CameraSubject fallback = subject;
    fallback.face.reset();
    OrbitCamera unknownRig{fallback};
    unknownRig.focusFace();
    unknownRig.update(1);
    JULIA_CHECK(near(unknownRig.target().y, 2 * unknownRig.settings().faceHeight));
    // Scaling and translation of a different model preserve framing proportions.
    CameraSubject scaled{subject.min * 7.0f + glm::vec3{3,4,5},
        subject.max * 7.0f + glm::vec3{3,4,5}, *subject.face * 7.0f + glm::vec3{3,4,5}};
    OrbitCamera big{scaled};
    JULIA_CHECK(near(big.position(), initial * 7.0f + glm::vec3{3,4,5}));

    // Pan is in screen pixels at near/far distances. Orbiting keeps the
    // character pivot in the same screen location rather than drifting away.
    for (float zoom : {0.0f, 8.0f})
    {
        OrbitCamera pan{subject};
        pan.zoom(zoom);
        pan.update(1);
        const auto pivot = pan.target();
        const auto p0 = project(pan, pivot);
        pan.pan(100, 50, 1000);
        pan.update(1);
        const auto p1 = project(pan, pivot);
        JULIA_CHECK(near(p1.x - p0.x, 0.2f));
        JULIA_CHECK(near(p1.y - p0.y, -0.1f));
        pan.orbit(-300, -80);
        pan.update(1);
        JULIA_CHECK(near(project(pan, pivot), p1));
        pan.pan(100000, -100000, 1000);
        pan.update(1);
        const auto limited = project(pan, pivot);
        JULIA_CHECK(std::abs(limited.x) <= .651f && std::abs(limited.y) <= .651f);
    }

    // No eye position enters the inflated bind box, even during rapid zoom,
    // pan, angle and face/body transitions. This also checks transient limits.
    for (int i = 0; i < 1500; ++i)
    {
        if (i % 101 == 0) camera.focusFace();
        if (i % 137 == 0) camera.frameBody();
        camera.orbit(13, (i % 19 < 10) ? -12.0f : 14.0f);
        camera.zoom(i % 43 < 40 ? 30.0f : -100.0f);
        camera.pan(2, -1, 1000);
        camera.update(1.0 / 144.0);
        const auto p = camera.position();
        const float margin = 2 * camera.settings().collisionMarginInHeights - Epsilon;
        JULIA_CHECK(glm::any(glm::lessThanEqual(p, subject.min - glm::vec3{margin})) ||
            glm::any(glm::greaterThanEqual(p, subject.max + glm::vec3{margin})));
        JULIA_CHECK(camera.distance() >= 2 * camera.settings().minDistanceInHeights - Epsilon);
        JULIA_CHECK(camera.distance() <= 2 * camera.settings().maxDistanceInHeights + Epsilon);
        JULIA_CHECK(camera.up().y > 0);
        JULIA_CHECK(std::abs(glm::dot(direction(camera), camera.up())) < Epsilon);
    }

    // Analytic smoothing agrees during the transient, including irregular dt.
    const auto simulate = [](int fps) {
        OrbitCamera c{subject};
        c.orbit(-120, 55);
        c.zoom(3);
        c.pan(40, -20, 1000);
        for (int i = 0; i < fps / 10; ++i) c.update(1.0 / fps);
        return c;
    };
    const auto reference = simulate(30);
    for (const int fps : {60,120,240})
    {
        const auto c = simulate(fps);
        JULIA_CHECK(near(c.position(), reference.position()));
        JULIA_CHECK(near(c.up(), reference.up()));
    }
    OrbitCamera irregular{subject};
    irregular.orbit(-120, 55);
    irregular.zoom(3);
    irregular.pan(40, -20, 1000);
    for (double dt : {.007,.025,.004,.041,.023}) irregular.update(dt);
    JULIA_CHECK(near(irregular.position(), reference.position()));
    const auto beforeInvalid = irregular.position();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    irregular.orbit(nan,0);
    irregular.zoom(nan);
    irregular.pan(nan,0,1000);
    irregular.setViewport(0,0,nan);
    irregular.update(nan);
    irregular.update(-1);
    JULIA_CHECK(near(irregular.position(), beforeInvalid));

    // Input ownership, click threshold, button chords, capture and cancellation.
    using julia::desktop::CameraInput;
    using julia::desktop::CameraInputContext;
    using Button = CameraInput::Button;
    CameraInput input;
    OrbitCamera controlled{subject};
    input.press(Button::Left, false, 20, 20);
    input.move(400,300,1000,controlled);
    controlled.update(1);
    JULIA_CHECK(near(controlled.position(), initial));
    input.press(Button::Right, false, 20,20);
    input.move(22,21,1000,controlled);
    input.release(Button::Right);
    controlled.update(1);
    JULIA_CHECK(near(controlled.position(), initial));
    input.press(Button::Right, false, 20,20);
    input.press(Button::Middle, false, 20,20);
    input.release(Button::Middle);
    JULIA_CHECK(input.active());
    input.move(100,30,1000,controlled);
    input.release(Button::Right);
    controlled.update(1);
    JULIA_CHECK(!near(controlled.position(), initial));
    JULIA_CHECK(near(controlled.target(), {0,1,0}));
    for (int block = 0; block < 3; ++block)
    {
        const auto pose = controlled.position();
        input.press(Button::Right,false,20,20);
        CameraInputContext blocked;
        blocked.focused = block != 0;
        blocked.inside = block != 1;
        blocked.mouseCapturedByUi = block == 2;
        input.setContext(blocked);
        input.move(300,300,1000,controlled);
        input.scroll(100,controlled);
        input.press(Button::Middle,false,20,20);
        JULIA_CHECK(!input.active());
        input.setContext({});
        input.move(500,500,1000,controlled);
        controlled.update(1);
        JULIA_CHECK(near(controlled.position(), pose));
    }
    CameraInputContext textInput;
    textInput.keyboardCapturedByUi = true;
    input.setContext(textInput);
    JULIA_CHECK(!input.acceptsKeyboard());
    JULIA_CHECK(input.acceptsMouse());
    input.setContext({});
    for (bool middle : {false,true})
    {
        controlled.frameBody(true);
        input.press(middle ? Button::Middle : Button::Right, !middle,20,20);
        input.move(100,40,1000,controlled);
        input.release(middle ? Button::Middle : Button::Right);
        controlled.update(1);
        JULIA_CHECK(!near(controlled.target(), {0,1,0}));
        JULIA_CHECK(near(direction(controlled), {0,0,1}));
    }
    return JULIA_TEST_RESULT();
}
