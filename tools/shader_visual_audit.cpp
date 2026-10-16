#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using namespace julia::render_opengl;
    constexpr int Width = 1000;
    constexpr int Height = 1000;
    constexpr int Samples = 8;
    using Pixels = std::vector<unsigned char>;

    struct Options
    {
        std::filesystem::path shaderAssets = JULIA_AUDIT_ASSET_DIR;
        std::filesystem::path modelAssets = JULIA_AUDIT_ASSET_DIR;
        std::filesystem::path output = "build/anime-audit/current";
        int warmup = 30;
        int frames = 90;
        int motionFrames = 48;
    };

    int positiveInteger(const std::string& value)
    {
        std::size_t consumed = 0;
        const int number = std::stoi(value, &consumed);
        if (consumed != value.size() || number < 1 || number > 10000)
            throw std::invalid_argument("Frame counts must be integers in [1, 10000]");
        return number;
    }

    Options arguments(int argc, char** argv)
    {
        Options options;
        for (int i = 1; i < argc; ++i)
        {
            const std::string option = argv[i];
            if (option == "--help")
            {
                std::cout << "shader_visual_audit [--shader-assets ROOT] [--model-assets ROOT] "
                    "[--output-dir DIR] [--warmup N] [--frames N] [--motion-frames N]\n"
                    "ROOT contains shaders/ or models/. Fixed 1000x1000, 8x MSAA, sRGB.\n";
                std::exit(0);
            }
            if (++i == argc) throw std::invalid_argument("Missing value for " + option);
            if (option == "--shader-assets") options.shaderAssets = argv[i];
            else if (option == "--model-assets") options.modelAssets = argv[i];
            else if (option == "--output-dir") options.output = argv[i];
            else if (option == "--warmup") options.warmup = positiveInteger(argv[i]);
            else if (option == "--frames") options.frames = positiveInteger(argv[i]);
            else if (option == "--motion-frames") options.motionFrames = positiveInteger(argv[i]);
            else throw std::invalid_argument("Unknown argument: " + option);
        }
        return options;
    }

    void checkGl(const std::string& stage)
    {
        const GLenum error = glGetError();
        if (error != GL_NO_ERROR)
            throw std::runtime_error(stage + ": OpenGL error " + std::to_string(error));
    }

    Pixels readFrame()
    {
        Pixels pixels(static_cast<std::size_t>(Width * Height * 3));
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, Width, Height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
        checkGl("Readback");
        return pixels;
    }

    void saveFrame(const Pixels& pixels, const std::filesystem::path& path)
    {
        std::ofstream file(path, std::ios::binary);
        if (!file) throw std::runtime_error("Cannot write " + path.string());
        file << "P6\n" << Width << ' ' << Height << "\n255\n";
        for (int y = Height - 1; y >= 0; --y)
            file.write(reinterpret_cast<const char*>(pixels.data() + y * Width * 3), Width * 3);
        if (!file) throw std::runtime_error("Incomplete image: " + path.string());
    }

    std::size_t changedBytes(const Pixels& a, const Pixels& b)
    {
        if (a.size() != b.size()) throw std::logic_error("Mismatched frame sizes");
        std::size_t count = 0;
        for (std::size_t i = 0; i < a.size(); ++i) if (a[i] != b[i]) ++count;
        return count;
    }

    std::uint64_t pixelHash(const Pixels& pixels)
    {
        std::uint64_t hash = 14695981039346656037ull;
        for (const auto pixel : pixels) { hash ^= pixel; hash *= 1099511628211ull; }
        return hash;
    }

    std::size_t coloredPixels(const Pixels& pixels)
    {
        std::size_t count = 0;
        for (std::size_t i = 0; i < pixels.size(); i += 3)
        {
            const auto maximum = std::max({pixels[i], pixels[i + 1], pixels[i + 2]});
            const auto minimum = std::min({pixels[i], pixels[i + 1], pixels[i + 2]});
            if (maximum - minimum > 2) ++count;
        }
        return count;
    }

    CameraConfig orbitCamera(float angle, bool face = false)
    {
        CameraConfig camera;
        camera.target = {0.0f, face ? 1.83f : 1.1f, 0.0f};
        const float distance = face ? 1.05f : 3.6f;
        camera.position = camera.target + glm::vec3{
            std::sin(angle) * distance, face ? 0.015f : 0.10f,
            std::cos(angle) * distance};
        return camera;
    }

    template<typename Render>
    double benchmark(Render&& render, int warmup, int frames, double& wallMilliseconds)
    {
        for (int frame = 0; frame < warmup; ++frame) render(frame, warmup);
        glFinish();
        checkGl("Warmup");
        GLuint query = 0;
        glGenQueries(1, &query);
        const double started = glfwGetTime();
        glBeginQuery(GL_TIME_ELAPSED, query);
        for (int frame = 0; frame < frames; ++frame) render(frame, frames);
        glEndQuery(GL_TIME_ELAPSED);
        GLuint64 nanoseconds = 0;
        glGetQueryObjectui64v(query, GL_QUERY_RESULT, &nanoseconds);
        wallMilliseconds = (glfwGetTime() - started) * 1000.0 / frames;
        glDeleteQueries(1, &query);
        checkGl("GPU benchmark");
        return static_cast<double>(nanoseconds) / 1e6 / frames;
    }

    void auditModel(const Options& options, const std::string& name, bool rigged,
                    std::ostream& report, std::ostream& timing)
    {
        OpenGLRenderer renderer;
        renderer.initialize(Width, Height, options.shaderAssets);
        auto model = GLModelLoader::loadFromFile(options.modelAssets / "models" / (name + ".fbx"),
            {.rebuildSmoothNormals = rigged});
        const auto fit = model.fitToHeightTransform(2.2f);
        const auto palette = rigged ? model.skinningMatrices() : std::vector<glm::mat4>{};
        if (model.empty() || (rigged && palette.empty()))
            throw std::runtime_error("Expected nonempty geometry/skeleton: " + name);
        std::size_t triangles = 0;
        for (const auto& mesh : model.meshes()) triangles += mesh.indexCount() / 3;
        report << "\nmodel=" << name << " meshes=" << model.meshCount()
               << " triangles=" << triangles << " bones=" << model.boneCount()
               << " palette=" << palette.size() << " rebuildSmoothNormals=" << rigged << '\n';

        const auto render = [&](const CameraConfig& camera, const glm::mat4& transform) {
            renderer.setCamera(camera);
            renderer.beginFrame();
            renderer.renderModel(model, transform, palette);
            checkGl(name + " draw");
        };
        const auto front = orbitCamera(0.0f);
        const std::array<std::pair<std::string, CameraConfig>, 4> views{{
            {"full", front}, {"face", orbitCamera(0.0f, true)},
            {"profile", orbitCamera(glm::half_pi<float>())},
            {"back", orbitCamera(glm::pi<float>())}}};
        for (const auto& [view, camera] : views)
        {
            render(camera, fit);
            const auto first = readFrame();
            const auto colored = coloredPixels(first);
            if (colored < 1000)
                throw std::runtime_error(name + " " + view + ": expected visible colored model");
            render(camera, fit);
            if (changedBytes(first, readFrame()) != 0)
                throw std::runtime_error(name + " " + view + ": repeated frame differs");
            saveFrame(first, options.output / (name + "-" + view + ".ppm"));
            report << "view=" << view << " colored_pixels=" << colored
                   << " repeat_changed_bytes=0 fnv1a64=" << pixelHash(first) << '\n';
        }

        render(front, fit);
        const auto reference = readFrame();
        for (const std::string mode : {"camera", "model"})
        {
            std::size_t transitions = 0;
            Pixels previous = reference;
            for (int frame = 0; frame < options.motionFrames; ++frame)
            {
                const float angle = glm::two_pi<float>() * static_cast<float>(frame) /
                    static_cast<float>(options.motionFrames);
                const auto camera = mode == "camera" ? orbitCamera(angle) : front;
                const auto transform = mode == "model" ?
                    glm::rotate(glm::mat4{1.0f}, angle, {0.0f, 1.0f, 0.0f}) * fit : fit;
                render(camera, transform);
                auto pixels = readFrame();
                transitions += changedBytes(previous, pixels) != 0 ? 1u : 0u;
                if (frame == 0 || frame == options.motionFrames / 4 ||
                    frame == options.motionFrames / 2 || frame == options.motionFrames * 3 / 4)
                    saveFrame(pixels, options.output / (name + "-" + mode + "-" + std::to_string(frame) + ".ppm"));
                previous = std::move(pixels);
            }
            render(front, fit);
            const auto resetChanges = changedBytes(reference, readFrame());
            if (resetChanges != 0 || (options.motionFrames > 1 && transitions == 0))
                throw std::runtime_error(name + " " + mode + ": motion/reset verification failed");
            report << "motion=" << mode << " frames=" << options.motionFrames
                   << " changing_transitions=" << transitions << " restored_changed_bytes=" << resetChanges << '\n';
            double wall = 0.0;
            const double gpu = benchmark([&](int frame, int frames) {
                const float angle = glm::two_pi<float>() * static_cast<float>(frame) / static_cast<float>(frames);
                render(mode == "camera" ? orbitCamera(angle) : front,
                    mode == "model" ? glm::rotate(glm::mat4{1.0f}, angle, {0.0f, 1.0f, 0.0f}) * fit : fit);
            }, options.warmup, options.frames, wall);
            timing << name << ',' << mode << ',' << Width << ',' << Height << ',' << Samples << ','
                   << options.warmup << ',' << options.frames << ',' << gpu << ',' << wall << '\n';
            report << "benchmark=" << mode << " gpu_ms=" << gpu << " wall_ms=" << wall << '\n';
            std::cout << name << ' ' << mode << ": GPU " << gpu << " ms/frame, wall " << wall << " ms/frame\n";
        }
        report << "gl_errors=0 deterministic_reset=PASS\n";
        std::cout << name << ": saved four views; motion, exact reset, GL checks PASS\n";
    }
}

int main(int argc, char** argv)
{
    GLFWwindow* window = nullptr;
    try
    {
        const auto options = arguments(argc, argv);
        glfwSetErrorCallback([](int error, const char* message) {
            std::cerr << "GLFW " << error << ": " << (message ? message : "") << '\n';
        });
        if (!glfwInit()) throw std::runtime_error("Cannot initialize GLFW");
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
        glfwWindowHint(GLFW_SAMPLES, Samples);
        window = glfwCreateWindow(Width, Height, "JULIA anime shader audit", nullptr, nullptr);
        if (!window) throw std::runtime_error("Cannot create OpenGL audit context");
        glfwMakeContextCurrent(window);
        glfwSwapInterval(0);
        if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
            throw std::runtime_error("Cannot initialize glad");
        GLint actualSamples = 0;
        glGetIntegerv(GL_SAMPLES, &actualSamples);
        int framebufferWidth = 0, framebufferHeight = 0;
        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
        if (actualSamples != Samples || framebufferWidth != Width || framebufferHeight != Height)
            throw std::runtime_error("Requested 1000x1000 8x MSAA framebuffer is unavailable; actual " +
                std::to_string(framebufferWidth) + "x" + std::to_string(framebufferHeight) + " samples=" + std::to_string(actualSamples));
        std::filesystem::create_directories(options.output);
        std::ofstream report(options.output / "report.txt");
        std::ofstream timing(options.output / "timing.csv");
        if (!report || !timing) throw std::runtime_error("Cannot create report files");
        report << std::fixed << std::setprecision(6)
               << "shader_assets=" << std::filesystem::absolute(options.shaderAssets).string()
               << "\nmodel_assets=" << std::filesystem::absolute(options.modelAssets).string()
               << "\nrenderer=" << glGetString(GL_RENDERER) << "\nversion=" << glGetString(GL_VERSION)
               << "\nresolution=" << Width << 'x' << Height << " samples=" << actualSamples
               << "\nwarmup=" << options.warmup << " measured_frames=" << options.frames
               << "\nfit_height=2.2 front=+Z profile=+X back=-Z face_target_y=1.83\n";
        timing << std::fixed << std::setprecision(6)
               << "model,motion,width,height,samples,warmup,measured_frames,gpu_ms,wall_ms\n";
        auditModel(options, "Raphtalia1", false, report, timing);
        auditModel(options, "Raphtalia", true, report, timing);
        checkGl("Final check");
        report << "\nresult=PASS\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Shader visual audit failed: " << error.what() << '\n';
        if (window) glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
}
