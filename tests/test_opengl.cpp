#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render/OrbitCamera.hpp"
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "julia_test.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <glm/gtc/matrix_transform.hpp>

namespace
{
    constexpr int Width = 640;
    constexpr int Height = 480;
    using Pixels = std::vector<unsigned char>;

    Pixels readFrame()
    {
        Pixels pixels(static_cast<std::size_t>(Width * Height * 3));
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, Width, Height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
        return pixels;
    }

    void saveFrame(const Pixels& pixels, const std::filesystem::path& path)
    {
        std::ofstream file(path, std::ios::binary);
        file << "P6\n" << Width << " " << Height << "\n255\n";
        for (int y = Height - 1; y >= 0; --y)
            file.write(reinterpret_cast<const char*>(pixels.data() + y * Width * 3), Width * 3);
    }

    std::size_t changedBytes(const Pixels& a, const Pixels& b)
    {
        std::size_t changed = 0;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (a[i] != b[i]) ++changed;
        return changed;
    }
}

int main(int argc, char** argv)
{
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 8);
    GLFWwindow* window = glfwCreateWindow(Width, Height, "JULIA render test", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    try
    {
        julia::render_opengl::OpenGLRenderer renderer;
        renderer.initialize(Width, Height);
        renderer.beginFrame();
        const Pixels background = readFrame();
        // Every background pixel has exactly the same neutral grey value.
        JULIA_CHECK(std::abs(static_cast<int>(background[0]) - 128) <= 1);
        for (const auto channel : background)
            if (channel != background[0]) { JULIA_CHECK(false); break; }

        auto model = julia::render_opengl::GLModelLoader::loadFromFile(
            std::filesystem::path{JULIA_TEST_ASSET_DIR} / "models" / "Raphtalia1.fbx");
        JULIA_CHECK(model.meshCount() > 0);
        const auto fit = model.fitToHeightTransform(2.2f);
        const glm::vec3 center{fit * glm::vec4{model.bounds().center(), 1.0f}};
        julia::render::OrbitCamera camera{center, center + glm::vec3{0.0f, 0.1f, 3.6f}};
        const auto render = [&] {
            julia::render_opengl::CameraConfig config;
            config.position = camera.position();
            config.target = camera.target();
            config.up = camera.up();
            renderer.setCamera(config);
            renderer.beginFrame();
            renderer.renderModel(model, fit);
            return readFrame();
        };
        Pixels previous;
        const std::filesystem::path output = argc > 1 ? argv[1] : ".";
        std::filesystem::create_directories(output);
        for (int stage = 0; stage < 4; ++stage)
        {
            if (stage == 1) camera.orbit(-120.0f, 40.0f);
            if (stage == 2) camera.zoom(2.0f);
            if (stage == 3) camera.pan(45.0f, -25.0f, static_cast<float>(Height), 45.0f);
            // Render actual intermediate frames while motion settles.
            for (int frame = 0; frame < 30; ++frame)
            {
                camera.update(1.0 / 120.0);
                (void)render();
            }
            renderer.beginFrame();
            JULIA_CHECK(changedBytes(background, readFrame()) == 0);
            auto pixels = render();
            JULIA_CHECK(changedBytes(background, pixels) > 10000);
            std::size_t coloredPixels = 0;
            for (std::size_t i = 0; i < pixels.size(); i += 3)
                if (pixels[i] != pixels[i + 1] || pixels[i + 1] != pixels[i + 2]) ++coloredPixels;
            JULIA_CHECK(coloredPixels > 1000);
            if (!previous.empty()) JULIA_CHECK(changedBytes(previous, pixels) > 10000);
            saveFrame(pixels, output / ("stage-" + std::to_string(stage) + ".ppm"));
            previous = std::move(pixels);
        }
        renderer.resize(320, 240);
        renderer.resize(Width, Height);
        (void)render();

        // Exercise the other shipped FBX too, including its imported skeleton.
        auto rigged = julia::render_opengl::GLModelLoader::loadFromFile(
            std::filesystem::path{JULIA_TEST_ASSET_DIR} / "models" / "Raphtalia.fbx");
        const auto rigFit = rigged.fitToHeightTransform(2.2f);
        const glm::vec3 rigCenter{rigFit * glm::vec4{rigged.bounds().center(), 1.0f}};
        julia::render_opengl::CameraConfig rigCamera;
        rigCamera.target = rigCenter;
        rigCamera.position = rigCenter + glm::vec3{0.0f, 0.1f, 3.6f};
        renderer.setCamera(rigCamera);
        const auto bindPalette = rigged.skinningMatrices();
        renderer.beginFrame();
        renderer.renderModel(rigged, rigFit, bindPalette);
        const auto rigPixels = readFrame();
        JULIA_CHECK(changedBytes(background, rigPixels) > 10000);
        saveFrame(rigPixels, output / "rig-bind.ppm");
        if (!bindPalette.empty())
        {
            // A controlled pose delta tests actual uploaded vertex weights.
            auto movedPalette = bindPalette;
            for (auto& matrix : movedPalette)
                matrix = glm::translate(glm::mat4{1.0f},
                    glm::vec3{rigged.bounds().size().y * 0.08f, 0.0f, 0.0f}) * matrix;
            renderer.beginFrame();
            renderer.renderModel(rigged, rigFit, movedPalette);
            JULIA_CHECK(changedBytes(rigPixels, readFrame()) > 1000);
        }

        // GPU timing excludes synchronous pixel readback and vsync. No fixed
        // performance threshold: GPU/driver/load vary across machines.
        GLuint timer = 0;
        glGenQueries(1, &timer);
        glFinish();
        const double started = glfwGetTime();
        glBeginQuery(GL_TIME_ELAPSED, timer);
        constexpr int stressFrames = 360;
        for (int frame = 0; frame < stressFrames; ++frame)
        {
            renderer.beginFrame();
            const auto turn = glm::rotate(glm::mat4{1.0f},
                static_cast<float>(frame) * 0.006f, glm::vec3{0.0f, 1.0f, 0.0f});
            renderer.renderModel(rigged, turn * rigFit, bindPalette);
        }
        glEndQuery(GL_TIME_ELAPSED);
        GLuint64 nanoseconds = 0;
        glGetQueryObjectui64v(timer, GL_QUERY_RESULT, &nanoseconds);
        const double elapsed = glfwGetTime() - started;
        glDeleteQueries(1, &timer);
        JULIA_CHECK(glGetError() == GL_NO_ERROR);
        std::cout << "Stress: " << stressFrames << " frames at " << Width << "x" << Height
                  << ", GPU average " << static_cast<double>(nanoseconds) / 1e6 / stressFrames
                  << " ms, wall average " << elapsed * 1000.0 / stressFrames << " ms\n"
                  << "Rigged FBX: " << rigged.meshCount() << " meshes, " << rigged.boneCount()
                  << " palette entries, " << rigged.nodes().size() << " nodes\n"
                  << "Renderer: " << glGetString(GL_RENDERER) << "\n";
        std::cout << "OpenGL: " << glGetString(GL_VERSION) << "\n"
                  << "Meshes: " << model.meshCount() << ", bones: " << model.boneCount()
                  << ", background RGB: " << static_cast<int>(background[0]) << "\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << "\n";
        JULIA_CHECK(false);
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return JULIA_TEST_RESULT();
}
