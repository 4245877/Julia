#include "AnimationVerification.hpp"
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

namespace
{
    using namespace julia::render_opengl;
    using Clock = std::chrono::steady_clock;
    using Pixels = std::vector<unsigned char>;

    void require(bool condition, const std::string& message)
    {
        if (!condition) throw std::runtime_error("Desktop animation verification: " + message);
    }

    float delta(std::span<const glm::mat4> a, std::span<const glm::mat4> b)
    {
        require(a.size() == b.size(), "palette size changed");
        float value = 0;
        for (std::size_t i = 0; i < a.size(); ++i)
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                {
                    require(std::isfinite(a[i][c][r]) && std::isfinite(b[i][c][r]), "NaN/Inf matrix");
                    value = std::max(value, std::abs(a[i][c][r] - b[i][c][r]));
                }
        return value;
    }

    std::size_t privateBytes()
    {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS_EX memory{};
        if (K32GetProcessMemoryInfo(GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&memory), sizeof(memory)))
            return memory.PrivateUsage;
#endif
        return 0;
    }

    void save(const std::filesystem::path& path, const Pixels& pixels, int width, int height)
    {
        std::ofstream stream(path, std::ios::binary);
        stream << "P6\n" << width << ' ' << height << "\n255\n";
        for (int y = height - 1; y >= 0; --y)
            stream.write(reinterpret_cast<const char*>(pixels.data() + static_cast<std::size_t>(y) * width * 3), width * 3);
        require(static_cast<bool>(stream), "cannot write " + path.string());
    }

    std::vector<double> sampleSeconds(const AnimationClip& clip)
    {
        std::vector<double> frames;
        if (clip.name.ends_with("TEST_blink_natural")) frames = {1,9,10,10.5,11,12,14,17,20,31};
        else if (clip.name.ends_with("TEST_blink_independent")) frames = {1,8,10,12,15,18.5,22,30,32,34,37,40.5,44,61};
        else if (clip.name.ends_with("TEST_blink_gaze")) frames = {1,10,11,12,15,28,29,30,33,46,47,48,51,64,65,66,69,82,83,84,87,100,101,102,105,121};
        else if (clip.name.ends_with("TEST_blink_head")) frames = {1,18,19,20,22,25,28,48,49,50,52,55,58,78,79,80,82,85,88,121};
        if (!frames.empty())
        {
            for (auto& time : frames) time = (time - 1) / 30.0;
            return frames;
        }
        return {0, clip.durationSeconds() * .25, clip.durationSeconds() * .5,
            clip.durationSeconds() * .75, clip.durationSeconds()};
    }
}

void verifyDesktopAnimations(GLFWwindow* window, OpenGLRenderer& renderer,
    const GLModel& model, AnimationPlayer& player, const glm::mat4& fit,
    const std::filesystem::path& output, DesktopKeyCallback input)
{
    std::filesystem::create_directories(output);
    std::ofstream report(output / "desktop_animation.txt"), samples(output / "samples.csv");
    require(report && samples, "cannot open evidence files");
    require(model.animations().size() == 15 && model.boneCount() == 429, "prepared 429/15 asset expected");
    report << std::setprecision(10) << "process=julia_desktop\nGL_VENDOR=" << glGetString(GL_VENDOR)
        << "\nGL_RENDERER=" << glGetString(GL_RENDERER) << "\nGL_VERSION=" << glGetString(GL_VERSION)
        << "\nmeshes=" << model.meshCount() << "\nnodes=" << model.nodes().size()
        << "\nbones=" << model.boneCount() << "\nclips=" << model.animations().size() << '\n';
    GLint maxTexels = 0;
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maxTexels);
    report << "max_palette_matrices=" << maxTexels / 4 << "\npalette_bytes=" << player.palette().size_bytes() << '\n';
    report << "window_visible=" << glfwGetWindowAttrib(window, GLFW_VISIBLE) << '\n';
    std::size_t textured = 0, masked = 0, blended = 0;
    for (const auto& mesh : model.meshes())
    {
        textured += mesh.material().diffuseTexture ? 1 : 0;
        masked += mesh.material().alphaMode == AlphaMode::Mask ? 1 : 0;
        blended += mesh.material().alphaMode == AlphaMode::Blend ? 1 : 0;
    }
    report << "textured_meshes=" << textured << "\nmasked_meshes=" << masked << "\nblended_meshes=" << blended
        << "\nmaterials_status=" << (textured ? "assigned_images" : "SOURCE_IMAGES_MISSING; imported diffuse only") << '\n';

    const auto bind = model.skinningMatrices();
    const auto center = glm::vec3{fit * glm::vec4{model.bounds().center(), 1}};
    auto face = center + glm::vec3{0,.7f,0};
    for (const auto& node : model.nodes())
        if (node.name.starts_with("head.x"))
        {
            face = glm::vec3{fit * node.globalBindTransform * glm::vec4{0,0,0,1}} + glm::vec3{0,.12f,0};
            break;
        }
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    renderer.resize(width, height);
    const auto render = [&](int view) {
        CameraConfig camera;
        camera.target = view == 2 ? face : center;
        camera.position = camera.target + glm::vec3{0, view == 2 ? .015f : .1f, view == 2 ? .82f : view == 1 ? -3.6f : 3.6f};
        if (view == 2) { camera.fovDegrees = 18; camera.nearPlane = .03f; }
        renderer.setCamera(camera);
        renderer.beginFrame();
        renderer.renderModel(model, fit, player.palette());
        Pixels pixels(static_cast<std::size_t>(width) * height * 3);
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
        require(glGetError() == GL_NO_ERROR, "OpenGL error during rendered sample");
        glfwSwapBuffers(window);
        glfwPollEvents();
        return pixels;
    };
    input(window, GLFW_KEY_X, 0, GLFW_PRESS, 0);
    require(player.state() == PlaybackState::Stopped && delta(player.palette(), bind) == 0, "stop/bind callback");
    const Pixels neutral[]{render(0), render(1), render(2)};
    for (int view = 0; view < 3; ++view) save(output / ("bind_" + std::to_string(view) + ".ppm"), neutral[view], width, height);
    samples << "clip_index,clip_name,time_seconds,view,changed_rgb_bytes_vs_bind,max_palette_delta,image\n";
    unsigned changedClips = 0, images = 3;
    float maximumFrameRateDelta = 0;
    for (std::size_t i = 0; i < model.animations().size(); ++i)
    {
        player.selectClip(i);
        const auto& clip = *player.clip();
        player.setLooping(false);
        player.play();
        player.update(.17 * clip.durationSeconds());
        input(window, GLFW_KEY_SPACE, 0, GLFW_PRESS, 0);
        const std::vector<glm::mat4> paused(player.palette().begin(), player.palette().end());
        const double pausedTime = player.timeSeconds();
        player.update(3.0);
        require(player.state() == PlaybackState::Paused && player.timeSeconds() == pausedTime &&
            delta(player.palette(), paused) == 0, "pause callback " + clip.name);
        input(window, GLFW_KEY_SPACE, 0, GLFW_PRESS, 0);
        player.update(clip.durationSeconds() * 2);
        require(player.state() == PlaybackState::Finished && player.timeSeconds() == clip.durationSeconds(), "nonloop endpoint");
        input(window, GLFW_KEY_L, 0, GLFW_PRESS, 0);
        player.play();
        player.update(clip.durationSeconds() * 2.25);
        require(std::abs(player.timeSeconds() - clip.durationSeconds() * .25) < 1e-8, "repeat timing");

        std::vector<glm::mat4> rateReference;
        const double target = clip.durationSeconds() * .37;
        for (int fps : {24,60,144})
        {
            player.stop(); player.play();
            const auto whole = static_cast<int>(std::floor(target * fps));
            for (int frame = 0; frame < whole; ++frame) player.update(1.0 / fps);
            player.update(std::max(0.0, target - static_cast<double>(whole) / fps));
            if (rateReference.empty()) rateReference.assign(player.palette().begin(), player.palette().end());
            maximumFrameRateDelta = std::max(maximumFrameRateDelta, delta(player.palette(), rateReference));
        }
        bool changed = false;
        int sampleIndex = 0;
        for (double time : sampleSeconds(clip))
        {
            player.seek(time);
            player.pause();
            const bool blink = clip.name.find("TEST_blink_") != std::string::npos;
            for (int view = 0; view < 3; ++view)
            {
                if (!blink && view == 2 && time != clip.durationSeconds() * .5) continue;
                const auto pixels = render(view);
                std::size_t differences = 0;
                for (std::size_t p = 0; p < pixels.size(); ++p) differences += pixels[p] != neutral[view][p] ? 1 : 0;
                changed |= differences > 0;
                const std::string file = "clip_" + std::to_string(i) + "_sample_" +
                    std::to_string(sampleIndex) + "_view_" + std::to_string(view) + ".ppm";
                save(output / file, pixels, width, height);
                samples << i << ',' << clip.name << ',' << std::setprecision(12) << time << ',' << view << ','
                    << differences << ',' << delta(player.palette(), bind) << ',' << file << '\n';
                ++images;
            }
            ++sampleIndex;
        }
        require(changed, "clip did not change desktop pixels: " + clip.name);
        ++changedClips;
        input(window, GLFW_KEY_R, 0, GLFW_PRESS, 0);
        require(delta(player.palette(), bind) == 0 && render(2) == neutral[2], "exact bind reset " + clip.name);
        report << "clip=" << clip.name << " duration_s=" << clip.durationSeconds() << " tps=" << clip.ticksPerSecond
            << " channels=" << clip.channels.size() << " pause_loop_finish_bind=PASS changed_pixels=YES\n";
    }
    require(maximumFrameRateDelta < .001f, "frame-rate dependent sampling");
    report << "frame_rates=24,60,144\nmax_palette_delta_between_rates=" << maximumFrameRateDelta << '\n';
    render(0);
    glFinish();
    const auto memoryBefore = privateBytes();
    for (int cycle = 0; cycle < 3000; ++cycle)
    {
        input(window, GLFW_KEY_PAGE_DOWN, 0, GLFW_PRESS, 0);
        player.update(.013);
        delta(player.palette(), bind);
        renderer.beginFrame();
        renderer.renderModel(model, fit, player.palette());
        if (cycle % 60 == 0) { glfwSwapBuffers(window); glfwPollEvents(); }
        if ((cycle + 1) % 1000 == 0)
        {
            glFinish();
            report << "private_bytes_after_" << cycle + 1 << "_switches=" << privateBytes() << '\n';
        }
    }
    glFinish();
    report << "switches_with_draws=3000\nprivate_bytes_before_switches=" << memoryBefore
        << "\nprivate_bytes_after_switches=" << privateBytes() << '\n';
    // CPU update and actual GPU draws measured separately; no readback in this loop.
    GLuint timer = 0;
    glGenQueries(1, &timer);
    for (int animated = 0; animated < 2; ++animated)
    {
        // Same body camera and natural-blink clip for both measurements.
        for (std::size_t i = 0; i < model.animations().size(); ++i)
            if (model.animations()[i].name.ends_with("TEST_blink_natural")) player.selectClip(i);
        if (animated) player.play();
        else player.stop();
        double updateMs = 0, uploadMs = 0;
        constexpr int frames = 240;
        for (int warmup = 0; warmup < 60; ++warmup)
        {
            player.update(1.0 / 120.0);
            renderer.beginFrame();
            renderer.renderModel(model, fit, player.palette());
        }
        glFinish();
        const auto start = Clock::now();
        glBeginQuery(GL_TIME_ELAPSED, timer);
        for (int frame = 0; frame < frames; ++frame)
        {
            const auto beforeUpdate = Clock::now();
            player.update(1.0 / 120.0);
            updateMs += std::chrono::duration<double, std::milli>(Clock::now() - beforeUpdate).count();
            renderer.beginFrame();
            renderer.renderModel(model, fit, player.palette());
            uploadMs += renderer.lastPaletteUploadCpuMilliseconds();
        }
        glEndQuery(GL_TIME_ELAPSED);
        glFinish();
        const auto wallMs = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        GLuint64 gpuNanoseconds = 0;
        glGetQueryObjectui64v(timer, GL_QUERY_RESULT, &gpuNanoseconds);
        report << (animated ? "animated" : "bind") << "_benchmark resolution=" << width << 'x' << height
            << " frames=" << frames << " update_cpu_ms=" << updateMs / frames << " palette_upload_cpu_ms="
            << uploadMs / frames << " gpu_frame_ms=" << static_cast<double>(gpuNanoseconds) / 1e6 / frames
            << " wall_frame_ms=" << wallMs / frames << " throughput_fps=" << frames * 1000.0 / wallMs << '\n';
    }
    glDeleteQueries(1, &timer);
    require(glGetError() == GL_NO_ERROR, "final OpenGL error");
    report << "clips_with_pixel_changes=" << changedClips << "\nsaved_images=" << images
        << "\nbind_reset=EXACT\ngl_errors=0\nresult=PASS\n";
    require(report && samples, "cannot flush evidence");
    std::cout << "Desktop animation verification PASS: " << changedClips << " clips, " << images << " images\n";
}
