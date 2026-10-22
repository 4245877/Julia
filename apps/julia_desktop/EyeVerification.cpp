#include "EyeVerification.hpp"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_inverse.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
    using namespace julia::render_opengl;
    using Pixels = std::vector<unsigned char>;
    using Clock = std::chrono::steady_clock;

    void require(bool condition, const std::string& reason)
    {
        if (!condition) throw std::runtime_error("Desktop eye verification: " + reason);
    }

    float matrixDelta(const glm::mat4& a, const glm::mat4& b)
    {
        float maximum = 0;
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
            {
                require(std::isfinite(a[column][row]) && std::isfinite(b[column][row]), "nonfinite transform");
                maximum = std::max(maximum, std::abs(a[column][row] - b[column][row]));
            }
        return maximum;
    }

    float delta(std::span<const glm::mat4> a, std::span<const glm::mat4> b)
    {
        require(a.size() == b.size(), "transform array size changed");
        float maximum = 0;
        for (std::size_t i = 0; i < a.size(); ++i) maximum = std::max(maximum, matrixDelta(a[i], b[i]));
        return maximum;
    }

    std::size_t differences(const Pixels& a, const Pixels& b)
    {
        require(a.size() == b.size(), "framebuffer size changed");
        std::size_t result = 0;
        for (std::size_t i = 0; i < a.size(); ++i) result += a[i] != b[i] ? 1 : 0;
        return result;
    }

    void save(const std::filesystem::path& path, const Pixels& pixels, int width, int height)
    {
        std::ofstream file(path, std::ios::binary);
        file << "P6\n" << width << ' ' << height << "\n255\n";
        for (int y = height - 1; y >= 0; --y)
            file.write(reinterpret_cast<const char*>(pixels.data() + static_cast<std::size_t>(y) * width * 3), width * 3);
        require(static_cast<bool>(file), "cannot write " + path.string());
    }

    void saveHalfSize(const std::filesystem::path& path, const Pixels& pixels, int width, int height)
    {
        const int reducedWidth = width / 2, reducedHeight = height / 2;
        require(reducedWidth > 0 && reducedHeight > 0, "framebuffer too small for motion evidence");
        Pixels reduced(static_cast<std::size_t>(reducedWidth) * reducedHeight * 3);
        for (int y = 0; y < reducedHeight; ++y)
            for (int x = 0; x < reducedWidth; ++x)
                for (int channel = 0; channel < 3; ++channel)
                    reduced[(static_cast<std::size_t>(y) * reducedWidth + x) * 3 + channel] =
                        pixels[(static_cast<std::size_t>(y * 2) * width + x * 2) * 3 + channel];
        save(path, reduced, reducedWidth, reducedHeight);
    }

    std::size_t findClip(const GLModel& model, const std::string& suffix)
    {
        const auto& clips = model.animations();
        for (std::size_t i = 0; i < clips.size(); ++i)
            if (clips[i].name.ends_with(suffix)) return i;
        throw std::runtime_error("Desktop eye verification: missing " + suffix);
    }

    bool eyeNode(const std::string& name)
    {
        return name.starts_with("eye.") || name.starts_with("lid.");
    }

    glm::quat rotation(const glm::mat4& matrix)
    {
        glm::mat3 basis{matrix};
        for (int axis = 0; axis < 3; ++axis) basis[axis] = glm::normalize(basis[axis]);
        return glm::normalize(glm::quat_cast(basis));
    }

    glm::vec3 gazeDirection(glm::vec2 degrees)
    {
        const auto angles = glm::radians(degrees);
        return {std::sin(angles.x) * std::cos(angles.y), std::sin(angles.y),
            std::cos(angles.x) * std::cos(angles.y)};
    }

    float angularDistance(glm::vec3 a, glm::vec3 b)
    {
        // atan2 remains accurate for tiny corrections, unlike acos(dot(float)).
        return glm::degrees(std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b)));
    }

    const char* gazeState(julia::render::GazeState state)
    {
        using julia::render::GazeState;
        switch (state)
        {
        case GazeState::Fixating: return "Fixating";
        case GazeState::Saccading: return "Saccading";
        case GazeState::Tracking: return "Tracking";
        }
        return "Unknown";
    }
}

void verifyDesktopEyes(GLFWwindow* window, OpenGLRenderer& renderer,
    const GLModel& model, AnimationPlayer& player, EyePoseLayer& eyes,
    const glm::mat4& fit, const std::filesystem::path& output, DesktopKeyCallback input)
{
    std::filesystem::create_directories(output);
    std::ofstream report(output / "desktop_eyes.txt"), samples(output / "eye_samples.csv");
    require(report && samples, "cannot open report files");
    require(model.boneCount() == 429 && model.animations().size() == 15, "prepared model must retain 429 bones / 15 clips");
    require(eyes.available(), "eye/lid mapping unavailable");
    player.setEyeLayer(&eyes);
    report << std::setprecision(10) << "process=julia_desktop\nGL_VENDOR=" << glGetString(GL_VENDOR)
        << "\nGL_RENDERER=" << glGetString(GL_RENDERER) << "\nGL_VERSION=" << glGetString(GL_VERSION)
        << "\nbones=" << model.boneCount() << "\nclips=" << model.animations().size()
        << "\nwindow_visible=" << glfwGetWindowAttrib(window, GLFW_VISIBLE) << '\n';
    samples << "sample,view,gaze_yaw_degrees,gaze_pitch_degrees,blink_left,blink_right,changed_rgb_bytes_vs_neutral,image\n";

    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    require(width > 0 && height > 0, "empty framebuffer");
    renderer.resize(width, height);
    const glm::vec3 center{fit * glm::vec4{model.bounds().center(), 1}};
    glm::vec3 face = center + glm::vec3{0, .7f, 0};
    for (const auto& node : model.nodes())
        if (node.name.starts_with("head.x"))
        {
            face = glm::vec3{fit * node.globalBindTransform * glm::vec4{0, 0, 0, 1}} + glm::vec3{0, .12f, 0};
            break;
        }
    std::size_t imageCount = 0;
    const auto render = [&](int view) {
        CameraConfig camera;
        camera.target = view == 3 ? center : face;
        camera.position = camera.target + (view == 3 ? glm::vec3{0, .1f, 3.6f} :
            view == 1 ? glm::vec3{.34f, .015f, .80f} :
            view == 2 ? glm::vec3{-.34f, .015f, .80f} : glm::vec3{0, .015f, .82f});
        if (view != 3) { camera.fovDegrees = 18; camera.nearPlane = .03f; }
        renderer.setCamera(camera);
        renderer.beginFrame();
        renderer.renderModel(model, fit, player.palette());
        Pixels pixels(static_cast<std::size_t>(width) * height * 3);
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
        require(glGetError() == GL_NO_ERROR, "OpenGL error on framebuffer readback");
        glfwSwapBuffers(window);
        glfwPollEvents();
        return pixels;
    };
    eyes.setEnabled(true);
    eyes.clearTargetPoint();
    auto& behavior = eyes.behavior();
    behavior.reset(0x52415048u);
    behavior.setAutoBlink(false);
    behavior.setAutoGaze(false);
    behavior.setGazeDegrees({0, 0});
    behavior.setBlink(0, 0);
    player.stop();
    const auto bind = model.skinningMatrices();
    const std::vector<Pixels> neutral{render(0), render(1), render(2), render(3)};
    const auto capture = [&](const std::string& name, int view) {
        const auto pixels = render(view);
        const auto& state = behavior.output();
        const std::string file = name + "_view_" + std::to_string(view) + ".ppm";
        save(output / file, pixels, width, height);
        samples << name << ',' << view << ',' << state.gazeDegrees.x << ',' << state.gazeDegrees.y
            << ',' << state.blinkLeft << ',' << state.blinkRight << ',' << differences(pixels, neutral[view]) << ',' << file << '\n';
        ++imageCount;
        return pixels;
    };
    const auto settle = [&] {
        // A stopped player keeps its bind base while the independent eye clock runs.
        for (int i = 0; i < 240; ++i) player.update(1.0 / 240.0);
        player.stop();
        delta(player.palette(), bind);
    };
    capture("neutral", 0);
    struct GazeSample { const char* name; glm::vec2 degrees; };
    for (const auto& sample : {GazeSample{"gaze_left", {8, 0}}, GazeSample{"gaze_right", {-8, 0}},
        GazeSample{"gaze_up", {0, 5}}, GazeSample{"gaze_down", {0, -5}},
        GazeSample{"gaze_diagonal", {8, 5}}})
    {
        behavior.setGazeDegrees(sample.degrees);
        settle();
        require(differences(capture(sample.name, 0), neutral[0]) > 0, "controlled gaze did not change framebuffer");
        capture(sample.name, 1);
        capture(sample.name, 2);
    }
    // A manual fixation does not drift, even when advanced for many seconds.
    const std::vector<glm::mat4> fixed(player.palette().begin(), player.palette().end());
    for (int i = 0; i < 1200; ++i) player.update(1.0 / 120.0);
    require(delta(player.palette(), fixed) < 1e-5f, "manual fixation drift");
    behavior.setGazeDegrees({0, 0});
    settle();
    require(delta(player.palette(), bind) < 1e-4f, "return to neutral");
    capture("gaze_return", 0);

    struct BlinkSample { const char* name; float left; float right; };
    for (const auto& sample : {BlinkSample{"blink_half", .5f, .5f}, BlinkSample{"blink_full", 1, 1},
        BlinkSample{"blink_left", 1, 0}, BlinkSample{"blink_right", 0, 1},
        BlinkSample{"blink_asymmetric", .25f, .75f}})
    {
        behavior.setBlink(sample.left, sample.right);
        settle();
        require(differences(capture(sample.name, 0), neutral[0]) > 0, "manual eyelids did not change framebuffer");
        capture(sample.name, 1);
        capture(sample.name, 2);
    }
    behavior.setGazeDegrees({8, 5});
    behavior.setBlink(.5f, .5f);
    settle();
    for (int view = 0; view < 3; ++view) capture("gaze_diagonal_blink_half", view);
    behavior.setGazeDegrees({0, 0});
    settle();
    // Compare every eyelid local matrix with the independently baked FBX poses.
    AnimationPlayer reference(model);
    reference.selectClip(findClip(model, "TEST_blink_independent"));
    float closureError = 0;
    std::size_t lidCount = 0;
    for (const auto& node : model.nodes()) lidCount += node.name.starts_with("lid.") ? 1 : 0;
    require(lidCount == 188, "expected all 188 prepared eyelid bones");
    for (int side = 0; side < 2; ++side)
    {
        behavior.setBlink(side == 0 ? 1.0f : 0.0f, side == 1 ? 1.0f : 0.0f);
        settle();
        reference.seek((side == 0 ? 11.0 : 33.0) / 30.0);
        for (std::size_t i = 0; i < model.nodes().size(); ++i)
            if (model.nodes()[i].name.starts_with("lid."))
                closureError = std::max(closureError, matrixDelta(player.localTransforms()[i], reference.localTransforms()[i]));
    }
    require(closureError < .0003f, "independent closure deviates from baked eyelid transforms");
    reference.selectClip(findClip(model, "TEST_blink_natural"));
    reference.seek(8.0 / 30.0);
    const std::vector<glm::mat4> openSource(reference.localTransforms().begin(), reference.localTransforms().end());
    reference.seek(11.0 / 30.0);
    const std::vector<glm::mat4> closedSource(reference.localTransforms().begin(), reference.localTransforms().end());
    std::size_t referenceNode = model.nodes().size();
    for (std::size_t i = 0; i < model.nodes().size(); ++i)
        if (model.nodes()[i].name == "lid.upper.L.16") referenceNode = i;
    require(referenceNode < model.nodes().size(), "missing closure reference rib");
    const float closureRange = closedSource[referenceNode][3].y - openSource[referenceNode][3].y;
    require(std::abs(closureRange) > .0001f, "source closure reference has no vertical movement");
    float midposeError = 0, nonlinearArc = 0;
    for (int sample = 0; sample <= 12; ++sample)
    {
        reference.seek((8.0 + sample * .25) / 30.0);
        const float closure = std::clamp((reference.localTransforms()[referenceNode][3].y - openSource[referenceNode][3].y) / closureRange, 0.0f, 1.0f);
        behavior.setBlink(closure, closure);
        player.stop();
        for (std::size_t i = 0; i < model.nodes().size(); ++i)
            if (model.nodes()[i].name.starts_with("lid."))
            {
                midposeError = std::max(midposeError, matrixDelta(player.localTransforms()[i], reference.localTransforms()[i]));
                const auto straight = glm::mix(glm::vec3{openSource[i][3]}, glm::vec3{closedSource[i][3]}, closure);
                nonlinearArc = std::max(nonlinearArc, glm::length(glm::vec3{reference.localTransforms()[i][3]} - straight));
            }
    }
    require(midposeError < .0003f && nonlinearArc > .0001f, "source nonlinear eyelid arc was not preserved");
    report << "lid_nodes=" << lidCount << "\nclosed_lid_max_local_delta_vs_fbx=" << closureError
        << "\nmidpose_lid_max_local_delta_vs_fbx=" << midposeError << "\nsource_nonlinear_arc_vs_straight_translation=" << nonlinearArc
        << "\nmanual_fixation_10s=PASS\ncontrolled_gaze_and_independent_blink=PASS\n";

    // Overlay masks preserve all body, head, hair and helper local transforms.
    float nonEyeError = 0;
    for (const std::string suffix : {"TEST_head_torso", "TEST_elbows_90", "TEST_deep_body_cloth"})
    {
        const auto clipIndex = findClip(model, suffix);
        player.selectClip(clipIndex);
        reference.selectClip(clipIndex);
        behavior.setGazeDegrees({4, -3});
        behavior.setBlink(.65f, .25f);
        for (int step = 0; step < 4; ++step) eyes.update(.25);
        const double sampleTime = model.animations()[clipIndex].durationSeconds() * .5;
        player.seek(sampleTime);
        reference.seek(sampleTime);
        for (std::size_t i = 0; i < model.nodes().size(); ++i)
            if (!eyeNode(model.nodes()[i].name))
                nonEyeError = std::max(nonEyeError, matrixDelta(player.localTransforms()[i], reference.localTransforms()[i]));
        require(delta(player.palette(), reference.palette()) > .0001f, "eye overlay absent over " + suffix);
        capture(suffix + "_overlay", 3);
        capture(suffix + "_overlay", 0);
    }
    require(nonEyeError == 0, "eye layer modified unrelated local bones");
    report << "non_eye_local_delta_over_body_clips=" << nonEyeError << '\n';

    // These older diagnostics own eye rotation, although they contain no blink.
    // Keep that authored gaze exactly and still layer the independent eyelids.
    float authoredGazeError = 0;
    for (const std::string suffix : {"TEST_eyes_ears", "TEST_practical_ranges"})
    {
        const auto clipIndex = findClip(model, suffix);
        player.selectClip(clipIndex);
        reference.selectClip(clipIndex);
        behavior.setGazeDegrees({-6, 4});
        behavior.setBlink(.65f, .25f);
        for (int step = 0; step < 4; ++step) eyes.update(.25);
        for (int sample = 0; sample <= 8; ++sample)
        {
            const double seconds = player.clip()->durationSeconds() * sample / 8.0;
            player.seek(seconds);
            reference.seek(seconds);
            for (std::size_t i = 0; i < model.nodes().size(); ++i)
                if (model.nodes()[i].name.starts_with("eye."))
                    authoredGazeError = std::max(authoredGazeError, matrixDelta(player.localTransforms()[i], reference.localTransforms()[i]));
            require(delta(player.palette(), reference.palette()) > .0001f, "blink absent over authored gaze " + suffix);
            if (sample == 4) capture(suffix + "_blink_overlay", 0);
        }
    }
    require(authoredGazeError == 0, "procedural gaze replaced authored nonblink eye animation");
    report << "authored_nonblink_eye_local_delta=" << authoredGazeError << '\n';

    // A fixed scene-space target is reprojected into the animated head each pose.
    const auto headClip = findClip(model, "TEST_head_torso");
    player.selectClip(headClip);
    behavior.setBlink(0, 0);
    const glm::vec3 target{glm::inverse(fit) * glm::vec4{face + glm::vec3{.025f, .01f, 1.5f}, 1}};
    eyes.setTargetPoint(target);
    std::vector<glm::mat4> targetStart;
    float targetEyeChange = 0;
    for (int step = 0; step < 3; ++step)
    {
        const double seconds = player.clip()->durationSeconds() * step * .25;
        for (int settleTarget = 0; settleTarget < 8; ++settleTarget)
        {
            eyes.update(.1);
            player.seek(seconds);
        }
        if (step == 0) targetStart.assign(player.localTransforms().begin(), player.localTransforms().end());
        for (std::size_t i = 0; i < model.nodes().size(); ++i)
            if (model.nodes()[i].name.starts_with("eye."))
                targetEyeChange = std::max(targetEyeChange, matrixDelta(player.localTransforms()[i], targetStart[i]));
        capture("target_head_" + std::to_string(step), 0);
        capture("target_head_" + std::to_string(step), 3);
    }
    require(targetEyeChange > .0001f, "scene target did not compensate animated head rotation");
    const std::vector<glm::mat4> targetEnd(player.localTransforms().begin(), player.localTransforms().end());
    eyes.clearTargetPoint();
    player.update(0);
    for (std::size_t i = 0; i < model.nodes().size(); ++i)
        if (model.nodes()[i].name.starts_with("eye."))
            require(matrixDelta(player.localTransforms()[i], targetEnd[i]) < 1e-5f, "target release snapped eyes");
    report << "fixed_target_eye_local_change=" << targetEyeChange
        << "\ntarget_release_continuity=PASS\nfixed_target_note=eye-only compensation is clamped to safe gaze range\n";

    // Authored TEST_blink_* poses must bypass both procedural layers completely.
    float bypassError = 0;
    behavior.setGazeDegrees({6, 4});
    behavior.setBlink(.75f, .25f);
    for (int step = 0; step < 4; ++step) eyes.update(.25);
    for (const std::string suffix : {"TEST_blink_natural", "TEST_blink_independent", "TEST_blink_gaze", "TEST_blink_head"})
    {
        const auto clipIndex = findClip(model, suffix);
        player.selectClip(clipIndex);
        reference.selectClip(clipIndex);
        for (int sample = 0; sample <= 16; ++sample)
        {
            const auto seconds = player.clip()->durationSeconds() * sample / 16.0;
            player.seek(seconds);
            reference.seek(seconds);
            bypassError = std::max(bypassError, delta(player.palette(), reference.palette()));
        }
    }
    require(bypassError == 0, "procedural eyes doubled baked TEST_blink_* poses");
    report << "baked_blink_overlay_bypass_max_palette_delta=" << bypassError << '\n';
    // G also exits a selected baked diagnostic so its ownership cannot hide
    // the reenabled automatic behavior.
    input(window, GLFW_KEY_G, 0, GLFW_PRESS, 0);
    require(player.state() == PlaybackState::Stopped && eyes.enabled() && behavior.autoBlink() && behavior.autoGaze(),
        "desktop G did not leave the selected baked blink diagnostic");
    report << "desktop_G_from_baked_blink=PASS\n";

    // Exercise the actual desktop callbacks and pause behavior with active eyes.
    player.selectClip(headClip);
    player.play();
    player.update(.1);
    input(window, GLFW_KEY_SPACE, 0, GLFW_PRESS, 0);
    const std::vector<glm::mat4> paused(player.palette().begin(), player.palette().end());
    const double pauseTime = player.timeSeconds();
    player.update(2.0);
    require(player.state() == PlaybackState::Paused && pauseTime == player.timeSeconds() &&
        delta(player.palette(), paused) == 0, "desktop pause did not freeze complete eye/body pose");
    input(window, GLFW_KEY_SPACE, 0, GLFW_PRESS, 0);
    player.update(.01);
    require(player.state() == PlaybackState::Playing, "desktop resume callback");
    input(window, GLFW_KEY_X, 0, GLFW_PRESS, 0);
    require(!eyes.enabled() && player.state() == PlaybackState::Stopped && delta(player.palette(), bind) == 0,
        "desktop X did not restore exact bind and disable eyes");
    input(window, GLFW_KEY_G, 0, GLFW_PRESS, 0);
    require(eyes.enabled() && behavior.autoBlink() && behavior.autoGaze(), "desktop G did not restore natural eyes");
    const auto motionDirectory = output / "motion";
    std::filesystem::create_directories(motionDirectory);
    int motionFrames = 0;
    for (int frame = 0; frame < 720; ++frame)
    {
        player.update(1.0 / 60.0);
        if (frame % 60 == 0) capture("automatic_" + std::to_string(frame / 60), 0);
        if (frame % 2 == 0)
        {
            saveHalfSize(motionDirectory / ("frame_" + std::to_string(motionFrames) + ".ppm"), render(0), width, height);
            ++motionFrames;
        }
        delta(player.palette(), bind);
    }
    report << "automatic_simulated_seconds=12\nmotion_frames=" << motionFrames << "\nmotion_fps=30\n"
        << "motion_resolution=" << width / 2 << 'x' << height / 2 << '\n';
    behavior.setAutoBlink(false);
    behavior.setAutoGaze(false);
    behavior.setGazeDegrees({0, 0});
    behavior.setBlink(0, 0);
    behavior.releaseBlink();
    behavior.triggerBlink();
    float maxTriggeredClosure = 0;
    for (int frame = 0; frame < 120; ++frame)
    {
        player.update(1.0 / 240.0);
        maxTriggeredClosure = std::max(maxTriggeredClosure, behavior.output().blinkLeft);
        if (frame == 12 || frame == 24 || frame == 48 || frame == 96)
            capture("trigger_blink_" + std::to_string(frame), 0);
    }
    require(maxTriggeredClosure > .95f && behavior.output().blinkLeft == 0 && behavior.output().blinkRight == 0,
        "programmatic blink did not fully close and reopen");
    report << "desktop_pause_stop_resume=PASS\nprogrammatic_blink_peak=" << maxTriggeredClosure << '\n';

    // Measure the transforms that the renderer actually receives. Endpoint-only
    // screenshots cannot detect a saccade compressed into one presentation frame.
    using julia::render::GazeState;
    std::array<std::size_t, 2> eyeIndices{};
    std::size_t headIndex = model.nodes().size();
    for (std::size_t index = 0; index < model.nodes().size(); ++index)
    {
        const auto& name = model.nodes()[index].name;
        if (name == "eye.L") eyeIndices[0] = index;
        if (name == "eye.R") eyeIndices[1] = index;
        if (name == "head.x_018") headIndex = index;
    }
    require(headIndex < model.nodes().size(), "missing temporal gaze head reference");
    const auto headBind = rotation(model.nodes()[headIndex].globalBindTransform);
    const auto actualDirections = [&] {
        std::array<glm::vec3, 2> result{};
        for (std::size_t side = 0; side < result.size(); ++side)
        {
            const auto index = eyeIndices[side];
            const auto relative = rotation(player.localTransforms()[index]) *
                glm::inverse(model.nodes()[index].bindTRS.rotation);
            result[side] = glm::normalize((headBind * relative * glm::inverse(headBind)) * glm::vec3{0, 0, 1});
        }
        return result;
    };
    const auto degreesOf = [](glm::vec3 direction) {
        return glm::degrees(glm::vec2{std::atan2(direction.x, direction.z),
            std::atan2(direction.y, std::hypot(direction.x, direction.z))});
    };
    std::ofstream motionSamples(output / "gaze_motion_samples.csv");
    motionSamples << std::setprecision(10)
        << "sequence,frame,time_seconds,delta_seconds,current_yaw,current_pitch,target_yaw,target_pitch,"
        << "remaining_degrees,actual_speed_degrees_per_second,state,elapsed_seconds,duration_seconds,"
        << "left_yaw,left_pitch,right_yaw,right_pitch,actual_step_degrees,blink_left,blink_right\n";
    auto previousActual = actualDirections();
    float maxBoneOutputError = 0, maxBinocularError = 0, maxPresentationSpeed = 0;
    const auto recordMotion = [&](const std::string& sequence, int frame, double time, double dt) {
        const auto actual = actualDirections();
        const auto left = degreesOf(actual[0]), right = degreesOf(actual[1]);
        const auto& pose = behavior.output();
        const auto& motion = behavior.motion();
        const float step = angularDistance(previousActual[0], actual[0]);
        const float speed = dt > 0 ? step / static_cast<float>(dt) : 0;
        const float expectedError = angularDistance(actual[0], gazeDirection(pose.gazeDegrees));
        maxBoneOutputError = std::max(maxBoneOutputError, expectedError);
        maxBinocularError = std::max(maxBinocularError, angularDistance(actual[0], actual[1]));
        maxPresentationSpeed = std::max(maxPresentationSpeed, speed);
        require(expectedError < .002f, "actual bone direction differs from interpolated gaze: " + sequence);
        require(angularDistance(actual[0], actual[1]) < .002f, "eyes diverged: " + sequence);
        motionSamples << sequence << ',' << frame << ',' << time << ',' << dt << ','
            << pose.gazeDegrees.x << ',' << pose.gazeDegrees.y << ','
            << motion.targetDegrees.x << ',' << motion.targetDegrees.y << ','
            << angularDistance(actual[0], gazeDirection(motion.targetDegrees)) << ',' << speed << ','
            << gazeState(motion.state) << ',' << motion.elapsedSeconds << ',' << motion.durationSeconds << ','
            << left.x << ',' << left.y << ',' << right.x << ',' << right.y << ',' << step << ','
            << pose.blinkLeft << ',' << pose.blinkRight << '\n';
        previousActual = actual;
        return step;
    };
    const auto prepareMotion = [&](glm::vec2 start) {
        eyes.clearTargetPoint();
        eyes.setEnabled(true);
        behavior.reset(0x52415048u);
        behavior.setAutoBlink(false);
        behavior.setAutoGaze(false);
        behavior.setBlink(0, 0);
        behavior.setGazeDegrees(start);
        player.stop();
        for (int tick = 0; tick < 240; ++tick) player.update(1.0 / 240.0);
        previousActual = actualDirections();
    };
    struct Transition { const char* name; glm::vec2 start; glm::vec2 finish; };
    const std::array transitions{
        Transition{"forward_left", {0, 0}, {8, 0}}, Transition{"left_right", {8, 0}, {-8, 0}},
        Transition{"forward_up", {0, 0}, {0, 5}}, Transition{"up_down", {0, 5}, {0, -5}},
        Transition{"forward_diagonal", {0, 0}, {6, 4}}, Transition{"diagonal_forward", {6, 4}, {0, 0}},
        Transition{"nearby", {1, 1}, {1.35f, .8f}}, Transition{"distant", {-7, -4}, {7, 4}}};
    float peakFraction30 = 0, peakFraction60 = 0, peakFraction120 = 0, peakFractionJitter = 0;
    int transitionCases = 0;
    for (int rate : {30, 60, 120, 0})
        for (const auto& transition : transitions)
        {
            prepareMotion(transition.start);
            const std::string name = std::string{transition.name} + (rate ? "_" + std::to_string(rate) : "_jitter");
            behavior.setGazeDegrees(transition.finish);
            const float amplitude = angularDistance(gazeDirection(transition.start), gazeDirection(transition.finish));
            const auto duration = behavior.motion().durationSeconds;
            recordMotion(name, 0, 0, 0);
            float priorDistance = amplitude;
            float peakFraction = 0;
            int intermediateFrames = 0;
            double time = 0;
            for (int frame = 1; time < .5; ++frame)
            {
                constexpr std::array jitter{1.0 / 50, 1.0 / 90, 1.0 / 45, 1.0 / 120, 1.0 / 60};
                const double dt = rate ? 1.0 / rate : jitter[static_cast<std::size_t>(frame - 1) % jitter.size()];
                time += dt;
                player.update(dt);
                const float step = recordMotion(name, frame, time, dt);
                const float remaining = angularDistance(actualDirections()[0], gazeDirection(transition.finish));
                require(remaining <= priorDistance + .002f, "saccade overshot or reversed: " + name);
                require(step <= behavior.settings().saccadeMaxSpeedDegrees * static_cast<float>(dt + 1.0 / 240) + .002f,
                    "saccade exceeded angular speed bound: " + name);
                if (remaining > .002f && remaining < amplitude - .002f) ++intermediateFrames;
                peakFraction = std::max(peakFraction, step / amplitude);
                priorDistance = remaining;
            }
            require(intermediateFrames >= 2, "saccade has too few actual intermediate bone poses: " + name);
            require(priorDistance < .002f && behavior.motion().state == GazeState::Fixating,
                "saccade did not finish exactly: " + name);
            require(duration >= behavior.settings().saccadeMinSeconds && duration <= behavior.settings().saccadeMaxSeconds,
                "saccade duration outside configured bounds");
            require(peakFraction < .65f, "one ordinary frame contains most of a gaze transition: " + name);
            auto& ratePeak = rate == 30 ? peakFraction30 : rate == 60 ? peakFraction60 : rate == 120 ? peakFraction120 : peakFractionJitter;
            ratePeak = std::max(ratePeak, peakFraction);
            ++transitionCases;
        }

    prepareMotion({-7, -3});
    behavior.setGazeDegrees({7, 3});
    player.update(.05);
    previousActual = actualDirections();
    const auto beforeRetarget = previousActual;
    behavior.setGazeDegrees({0, 5});
    player.update(0);
    require(angularDistance(beforeRetarget[0], actualDirections()[0]) < .0001f, "retarget snapped actual eye bones");
    require(glm::length(behavior.motion().startDegrees - behavior.output().gazeDegrees) < .0001f,
        "retarget did not start from the currently rendered gaze");
    recordMotion("retarget", 0, 0, 0);
    for (int frame = 1; frame <= 30; ++frame)
    {
        player.update(1.0 / 60);
        recordMotion("retarget", frame, frame / 60.0, 1.0 / 60);
    }
    require(glm::length(behavior.output().gazeDegrees - glm::vec2{0, 5}) < .001f, "retarget missed final direction");

    prepareMotion({0, 0});
    glm::vec2 lastTrackingTarget{0};
    float maxTrackingStep = 0;
    for (int frame = 1; frame <= 300; ++frame)
    {
        const double time = frame / 60.0;
        if (frame <= 180)
            lastTrackingTarget = {static_cast<float>(4 * std::sin(time * 1.7)), static_cast<float>(2 * std::sin(time * 1.1))};
        behavior.trackGazeDegrees(lastTrackingTarget);
        player.update(1.0 / 60);
        maxTrackingStep = std::max(maxTrackingStep, recordMotion("tracking_stop", frame, time, 1.0 / 60));
    }
    require(maxTrackingStep <= behavior.settings().pursuitMaxSpeedDegrees / 60.0f + .002f,
        "smooth pursuit exceeded its speed limit");
    require(glm::length(behavior.output().gazeDegrees - lastTrackingTarget) < .01f,
        "pursuit did not settle after the target stopped");

    prepareMotion({0, 0});
    behavior.setGazeDegrees({8, 4});
    player.update(.025);
    const auto beforeGap = actualDirections();
    player.update(2.0); // The documented suspend policy freezes procedural time.
    player.update(0);
    require(angularDistance(beforeGap[0], actualDirections()[0]) < .0001f, "suspend or zero-dt changed actual eyes");
    player.update(.25); // A real short stall advances to the correct finite endpoint.
    require(glm::length(behavior.output().gazeDegrees - glm::vec2{8, 4}) < .001f, "frame gap left gaze stuck in transit");

    prepareMotion({-4, -2});
    player.selectClip(headClip);
    player.play();
    behavior.setGazeDegrees({5, 3});
    behavior.triggerBlink(true, false);
    for (int frame = 1; frame <= 120; ++frame)
    {
        if (frame == 45) behavior.triggerBlink(false, true);
        player.update(1.0 / 60);
        recordMotion("head_independent_blink", frame, frame / 60.0, 1.0 / 60);
    }

    // Ten minutes include the complete player/pose layer, not just behavior output.
    prepareMotion({0, 0});
    behavior.releaseGaze();
    behavior.releaseBlink();
    behavior.setAutoGaze(true);
    behavior.setAutoBlink(true);
    for (int frame = 1; frame <= 600 * 60; ++frame)
    {
        player.update(1.0 / 60);
        const float step = recordMotion("automatic_10min", frame, frame / 60.0, 1.0 / 60);
        require(step < behavior.settings().saccadeMaxSpeedDegrees / 60.0f + .002f,
            "automatic gaze exceeded physical speed bound");
    }
    report << "temporal_transition_cases=" << transitionCases
        << "\nmax_frame_fraction_30hz=" << peakFraction30 << "\nmax_frame_fraction_60hz=" << peakFraction60
        << "\nmax_frame_fraction_120hz=" << peakFraction120 << "\nmax_frame_fraction_jitter=" << peakFractionJitter
        << "\nmax_actual_bone_vs_behavior_degrees=" << maxBoneOutputError
        << "\nmax_binocular_direction_error_degrees=" << maxBinocularError
        << "\nmax_actual_gaze_speed_degrees_per_second=" << maxPresentationSpeed
        << "\nactual_bone_retarget_continuity=PASS\npursuit_stop=PASS\nframe_gap_policy=PASS"
        << "\nactual_pose_automatic_soak_seconds=600\nactual_pose_temporal_checks=PASS\n";

    // --review-eyes also presents these sequences at normal wall-clock speed.
    // Hidden CTest verification captures the identical poses without waiting.
    const bool pacedReview = glfwGetWindowAttrib(window, GLFW_VISIBLE) == GLFW_TRUE;
    const auto focusedDirectory = output / "gaze_motion";
    std::filesystem::create_directories(focusedDirectory);
    for (const std::string name : {"short", "wide", "pursuit", "automatic_blink"})
    {
        const int frameCount = name == "automatic_blink" ? 720 : name == "pursuit" ? 240 : 150;
        const auto directory = focusedDirectory / name;
        std::filesystem::create_directories(directory);
        prepareMotion(name == "wide" ? glm::vec2{-7, -3} : glm::vec2{0});
        if (name == "automatic_blink")
        {
            behavior.releaseGaze();
            behavior.releaseBlink();
            behavior.setAutoGaze(true);
            behavior.setAutoBlink(true);
        }
        if (pacedReview)
            glfwSetWindowTitle(window, ("JULIA gaze review | " + name + " | 60 Hz").c_str());
        const auto playbackStart = Clock::now();
        int saccadeBlinkFrames = 0;
        for (int frame = 0; frame < frameCount; ++frame)
        {
            const double time = frame / 60.0;
            if (name == "short" && (frame == 36 || frame == 90))
                behavior.setGazeDegrees(frame == 36 ? glm::vec2{2, 1} : glm::vec2{0});
            else if (name == "wide" && (frame == 36 || frame == 90))
                behavior.setGazeDegrees(frame == 36 ? glm::vec2{7, 3} : glm::vec2{-7, -3});
            else if (name == "pursuit")
                behavior.trackGazeDegrees(time < 3.0 ? glm::vec2{static_cast<float>(4 * std::sin(time * 2)),
                    static_cast<float>(2 * std::sin(time * 1.2))} : lastTrackingTarget);
            if (name == "pursuit" && time < 3.0)
                lastTrackingTarget = behavior.motion().targetDegrees;
            // Keep the first wide arc unobstructed; the return demonstrates
            // real eye movement continuing underneath a complete blink.
            if (name == "wide" && frame == 90) behavior.triggerBlink();
            player.update(1.0 / 60);
            if (behavior.motion().state == GazeState::Saccading &&
                std::max(behavior.output().blinkLeft, behavior.output().blinkRight) > .01f)
                ++saccadeBlinkFrames;
            recordMotion("visual_" + name, frame, (frame + 1) / 60.0, 1.0 / 60);
            if (pacedReview)
                std::this_thread::sleep_until(playbackStart + std::chrono::duration<double>{(frame + 1) / 60.0});
            saveHalfSize(directory / ("frame_" + std::to_string(frame) + ".ppm"), render(0), width, height);
        }
        const double wallSeconds = std::chrono::duration<double>(Clock::now() - playbackStart).count();
        report << "visual_" << name << "_frames=" << frameCount
            << "\nvisual_" << name << "_wall_seconds=" << wallSeconds
            << "\nvisual_" << name << "_measured_fps=" << frameCount / wallSeconds
            << "\nvisual_" << name << "_saccade_blink_overlap_frames=" << saccadeBlinkFrames << '\n';
        if (name == "wide") require(saccadeBlinkFrames >= 3, "wide review did not show movement during blinking");
    }
    report << "visual_gaze_fps=60\nvisual_paced_review=" << pacedReview << '\n';
    motionSamples.flush();
    require(static_cast<bool>(motionSamples), "cannot flush gaze temporal samples");

    for (int cycle = 0; cycle < 1000; ++cycle)
    {
        input(window, GLFW_KEY_PAGE_DOWN, 0, GLFW_PRESS, 0);
        player.update(.013);
        delta(player.palette(), bind);
        if (cycle % 100 == 0)
        {
            renderer.beginFrame();
            renderer.renderModel(model, fit, player.palette());
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }
    report << "switches_with_enabled_eye_layer=1000\n";

    // CPU evaluation only: this measures overlay cost without claiming GPU FPS.
    player.selectClip(headClip);
    player.setLooping(true);
    player.play();
    behavior.releaseBlink();
    behavior.releaseGaze();
    behavior.setAutoBlink(true);
    behavior.setAutoGaze(true);
    constexpr int benchmarkFrames = 5000;
    double updateMilliseconds[2]{};
    for (int overlay = 0; overlay < 2; ++overlay)
    {
        eyes.setEnabled(overlay != 0);
        for (int warmup = 0; warmup < 200; ++warmup) player.update(1.0 / 120.0);
        const auto start = Clock::now();
        for (int frame = 0; frame < benchmarkFrames; ++frame) player.update(1.0 / 120.0);
        updateMilliseconds[overlay] = std::chrono::duration<double, std::milli>(Clock::now() - start).count() / benchmarkFrames;
    }
    report << "update_benchmark_frames=" << benchmarkFrames
        << "\nbase_update_cpu_ms=" << updateMilliseconds[0]
        << "\neye_update_cpu_ms=" << updateMilliseconds[1]
        << "\neye_update_cpu_delta_ms=" << updateMilliseconds[1] - updateMilliseconds[0]
        << "\nperformance_note=CPU pose evaluation only; no GPU performance inference\n";
    require(glGetError() == GL_NO_ERROR, "final OpenGL error");
    input(window, GLFW_KEY_X, 0, GLFW_PRESS, 0);
    report << "saved_images=" << imageCount << "\ngl_errors=0\nresult=PASS\n";
    report.flush();
    samples.flush();
    require(report && samples, "cannot flush evidence files");
    std::cout << "Desktop eye verification PASS: " << imageCount << " framebuffer images\n";
}
