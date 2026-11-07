#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render/OrbitCamera.hpp"
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"
#include "AnimationVerification.hpp"
#include "EyeVerification.hpp"
#include "CameraInput.hpp"
#include "julia/render_opengl/EyePoseLayer.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <fstream>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#ifndef JULIA_ASSET_DIR
#define JULIA_ASSET_DIR "assets"
#endif

namespace
{
    std::filesystem::path assetDirectory(const char* executable)
    {
        std::filesystem::path executablePath{executable};
#ifdef _WIN32
        wchar_t buffer[32768]{};
        const DWORD length = GetModuleFileNameW(nullptr, buffer, 32768);
        if (length > 0 && length < 32768)
            executablePath = std::filesystem::path{buffer};
#endif
        const auto bundledAssets = std::filesystem::absolute(executablePath)
            .parent_path() / "assets";
        return std::filesystem::is_directory(bundledAssets)
            ? bundledAssets : std::filesystem::path{JULIA_ASSET_DIR};
    }

    julia::render::CameraSubject cameraSubject(
        const julia::render_opengl::GLModel& model, const glm::mat4& fit)
    {
        julia::render::CameraSubject subject;
        subject.min = glm::vec3{fit * glm::vec4{model.bounds().min, 1}};
        subject.max = glm::vec3{fit * glm::vec4{model.bounds().max, 1}};
        const float height = subject.max.y - subject.min.y;
        std::optional<glm::vec3> head;
        glm::vec3 eyeSum{0};
        int eyeCount = 0;
        for (const auto& node : model.nodes())
        {
            auto name = node.name;
            std::transform(name.begin(), name.end(), name.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            // Common rig names, with namespace prefixes removed. Unknown rigs
            // use the proportional bounds fallback in OrbitCamera.
            if (const auto colon = name.rfind(':'); colon != std::string::npos)
                name.erase(0, colon + 1);
            const auto point = glm::vec3{fit * node.globalBindTransform * glm::vec4{0,0,0,1}};
            if (point.y < subject.min.y + height * 0.6f ||
                glm::any(glm::lessThan(point, subject.min)) ||
                glm::any(glm::greaterThan(point, subject.max))) continue;
            if (!head && (name == "head" || name.starts_with("head.") || name.starts_with("head_")))
                head = point + glm::vec3{0, height * 0.055f, 0};
            if (name == "eye.l" || name == "eye.r" || name == "lefteye" || name == "righteye" ||
                name == "left_eye" || name == "right_eye")
            {
                eyeSum += point;
                ++eyeCount;
            }
        }
        subject.face = eyeCount == 2 ? std::optional<glm::vec3>{eyeSum * 0.5f} : head;
        return subject;
    }

    struct WindowState
    {
        julia::render_opengl::OpenGLRenderer* renderer = nullptr;
        julia::render::OrbitCamera* orbitCamera = nullptr;
        julia::render_opengl::AnimationPlayer* player = nullptr;
        julia::render_opengl::EyePoseLayer* eyes = nullptr;
        const julia::render_opengl::GLModel* model = nullptr;
        julia::render_opengl::CameraConfig cameraConfig{};
        julia::desktop::CameraInput cameraInput;
        julia::desktop::CameraInputContext inputContext;
        double cursorX = 0, cursorY = 0;
        // A future UI routes consumed events here before camera callbacks.
        // The current desktop has only a native title bar and no overlay UI.
        void syncInputContext() { cameraInput.setContext(inputContext); }

        void applyCamera() const
        {
            if (!renderer || !orbitCamera)
            {
                return;
            }

            auto updatedConfig = cameraConfig;
            updatedConfig.position = orbitCamera->position();
            updatedConfig.target = orbitCamera->target();
            updatedConfig.up = orbitCamera->up();
            renderer->setCamera(updatedConfig);
        }
    };

    WindowState* getWindowState(GLFWwindow* window)
    {
        return static_cast<WindowState*>(glfwGetWindowUserPointer(window));
    }

    void glfwErrorCallback(int error, const char* description)
    {
        std::cerr << "GLFW error " << error << ": "
                  << (description ? description : "(no description)") << "\n";
    }

    void APIENTRY glDebugCallback(
        GLenum /*source*/,
        GLenum type,
        GLuint /*id*/,
        GLenum severity,
        GLsizei /*length*/,
        const GLchar* message,
        const void* /*userParam*/
    )
    {
        // Notification-level chatter (buffer hints, etc.) is not actionable.
        if (severity == GL_DEBUG_SEVERITY_NOTIFICATION)
        {
            return;
        }

        std::cerr << "[GL]" << (type == GL_DEBUG_TYPE_ERROR ? " ERROR:" : "")
                  << " " << (message ? message : "") << "\n";
    }

    void framebufferSizeCallback(GLFWwindow* window, int width, int height)
    {
        WindowState* state = getWindowState(window);

        if (state && state->renderer)
        {
            state->renderer->resize(width, height);
            if (state->orbitCamera)
                state->orbitCamera->setViewport(static_cast<float>(width),
                    static_cast<float>(height), state->cameraConfig.fovDegrees);
            state->cameraInput.cancel();
        }
    }

    void mouseButtonCallback(
        GLFWwindow* window,
        int button,
        int action,
        int mods
    )
    {
        WindowState* state = getWindowState(window);

        if (!state || (button != GLFW_MOUSE_BUTTON_LEFT &&
                       button != GLFW_MOUSE_BUTTON_RIGHT &&
                       button != GLFW_MOUSE_BUTTON_MIDDLE))
        {
            return;
        }

        state->syncInputContext();
        using Button = julia::desktop::CameraInput::Button;
        const auto cameraButton = button == GLFW_MOUSE_BUTTON_LEFT ? Button::Left :
            button == GLFW_MOUSE_BUTTON_RIGHT ? Button::Right : Button::Middle;
        if (action == GLFW_PRESS)
        {
            state->cameraInput.press(cameraButton, (mods & GLFW_MOD_SHIFT) != 0,
                state->cursorX, state->cursorY);
        }
        else if (action == GLFW_RELEASE) state->cameraInput.release(cameraButton);
    }

    void cursorPositionCallback(GLFWwindow* window, double x, double y)
    {
        WindowState* state = getWindowState(window);

        if (!state || !state->orbitCamera) return;
        state->cursorX = x;
        state->cursorY = y;
        state->syncInputContext();
        int width = 0, height = 0;
        glfwGetWindowSize(window, &width, &height);
        // Also reject out-of-window positions before GLFW's enter/leave event.
        if (x < 0 || y < 0 || x >= width || y >= height) state->cameraInput.cancel();
        // Cursor deltas and height use logical pixels, including on HiDPI.
        state->cameraInput.move(x, y, static_cast<float>(height), *state->orbitCamera);
    }

    void scrollCallback(
        GLFWwindow* window,
        double /*xOffset*/,
        double yOffset
    )
    {
        WindowState* state = getWindowState(window);

        if (!state || !state->orbitCamera)
        {
            return;
        }

        state->syncInputContext();
        state->cameraInput.scroll(yOffset, *state->orbitCamera);
    }

    void focusCallback(GLFWwindow* window, int focused)
    {
        if (auto* state = getWindowState(window))
        {
            state->inputContext.focused = focused != 0;
            state->syncInputContext();
        }
    }

    void cursorEnterCallback(GLFWwindow* window, int entered)
    {
        if (auto* state = getWindowState(window))
        {
            state->inputContext.inside = entered != 0;
            if (entered) glfwGetCursorPos(window, &state->cursorX, &state->cursorY);
            state->syncInputContext();
        }
    }

    void keyCallback(GLFWwindow* window, int key, int, int action, int)
    {
        if (action != GLFW_PRESS) return;
        auto* state = getWindowState(window);
        if (!state || !state->player) return;
        state->syncInputContext();
        if (!state->cameraInput.acceptsKeyboard()) return;
        auto& player = *state->player;
        if (key == GLFW_KEY_SPACE)
        {
            if (player.state() == julia::render_opengl::PlaybackState::Playing) player.pause();
            else player.play();
        }
        else if (key == GLFW_KEY_X || key == GLFW_KEY_R)
        {
            if (state->eyes) state->eyes->setEnabled(false);
            player.stop();
        }
        else if (key == GLFW_KEY_L) player.setLooping(!player.looping());
        else if (key == GLFW_KEY_COMMA || key == GLFW_KEY_PERIOD)
        {
            player.pause();
            player.seek(player.timeSeconds() + (key == GLFW_KEY_PERIOD ? 1.0 : -1.0) / 30.0);
        }
        else if (key == GLFW_KEY_PAGE_DOWN || key == GLFW_KEY_PAGE_UP)
        {
            const auto count = state->model->animations().size();
            if (count)
            {
                player.selectClip((player.clipIndex() + (key == GLFW_KEY_PAGE_DOWN ? 1 : count - 1)) % count);
                player.play();
            }
        }
        else if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4)
        {
            constexpr std::string_view names[]{"TEST_blink_natural", "TEST_blink_independent",
                "TEST_blink_gaze", "TEST_blink_head"};
            const auto& clips = state->model->animations();
            for (std::size_t i = 0; i < clips.size(); ++i)
                if (clips[i].name.ends_with(names[key - GLFW_KEY_1]))
                {
                    player.selectClip(i);
                    player.play();
                    break;
                }
        }
        else if (key == GLFW_KEY_F1)
        {
            for (std::size_t i = 0; i < state->model->animations().size(); ++i)
                std::cout << i << ": " << state->model->animations()[i].name << " ("
                    << state->model->animations()[i].durationSeconds() << " s)\n";
        }
        else if (state->eyes && key == GLFW_KEY_G)
        {
            auto& eyes = *state->eyes;
            eyes.setEnabled(true);
            eyes.clearTargetPoint();
            eyes.behavior().releaseBlink();
            eyes.behavior().releaseGaze();
            eyes.behavior().setAutoBlink(true);
            eyes.behavior().setAutoGaze(true);
            if (julia::render_opengl::EyePoseLayer::authoredEyes(player.clip())) player.stop();
        }
        else if (state->eyes && (key == GLFW_KEY_H || key == GLFW_KEY_J || key == GLFW_KEY_K ||
            key == GLFW_KEY_U || key == GLFW_KEY_I || key == GLFW_KEY_HOME || key == GLFW_KEY_T ||
            (key >= GLFW_KEY_KP_1 && key <= GLFW_KEY_KP_9) || key == GLFW_KEY_F8 ||
            key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_RIGHT_BRACKET))
        {
            auto& eyes = *state->eyes;
            auto& behavior = eyes.behavior();
            if (key == GLFW_KEY_H) behavior.setAutoBlink(!behavior.autoBlink());
            else if (key == GLFW_KEY_J) behavior.setAutoGaze(!behavior.autoGaze());
            else if (key == GLFW_KEY_K) behavior.triggerBlink();
            else if (key == GLFW_KEY_U || key == GLFW_KEY_I)
                behavior.triggerBlink(key == GLFW_KEY_U, key == GLFW_KEY_I);
            else if (key == GLFW_KEY_HOME) { eyes.clearTargetPoint(); behavior.neutral(); }
            else if (key == GLFW_KEY_T)
            {
                if (eyes.hasTargetPoint()) eyes.clearTargetPoint();
                else eyes.setTargetPoint(glm::vec3{glm::inverse(state->model->fitToHeightTransform(2.2f)) *
                    glm::vec4{state->orbitCamera->position(),1}});
            }
            else if (key == GLFW_KEY_F8) behavior.reset(0x52415048u);
            else if (key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_RIGHT_BRACKET)
            {
                auto settings = behavior.settings();
                settings.blinkMeanSeconds = std::clamp(settings.blinkMeanSeconds +
                    (key == GLFW_KEY_RIGHT_BRACKET ? .5 : -.5), 1.0, 15.0);
                behavior.setSettings(settings);
            }
            else
            {
                const int direction = key - GLFW_KEY_KP_1;
                eyes.clearTargetPoint();
                behavior.setGazeDegrees({static_cast<float>(direction % 3 - 1) * 8,
                    static_cast<float>(direction / 3 - 1) * 5});
            }
            player.update(0);
        }
        else if (key == GLFW_KEY_F || key == GLFW_KEY_B)
        {
            state->cameraInput.cancel();
            if (key == GLFW_KEY_F) state->orbitCamera->focusFace();
            else state->orbitCamera->frameBody();
        }
    }

    void processInput(GLFWwindow* window, double deltaSeconds)
    {
        auto* state = getWindowState(window);
        if (!state || !state->orbitCamera) return;
        state->syncInputContext();
        if (!state->cameraInput.acceptsKeyboard()) return;
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        const auto pressed = [window](int key) {
            return glfwGetKey(window, key) == GLFW_PRESS;
        };
        glm::vec2 direction{
            static_cast<float>(pressed(GLFW_KEY_D) || pressed(GLFW_KEY_RIGHT)) -
                static_cast<float>(pressed(GLFW_KEY_A) || pressed(GLFW_KEY_LEFT)),
            static_cast<float>(pressed(GLFW_KEY_E) || pressed(GLFW_KEY_UP)) -
                static_cast<float>(pressed(GLFW_KEY_Q) || pressed(GLFW_KEY_DOWN))
        };
        if (glm::length(direction) > 0.0f)
        {
            const float speed = state->orbitCamera->settings().keyboardPanHalfScreensPerSecond;
            state->orbitCamera->panNormalized(glm::normalize(direction) * speed *
                static_cast<float>(deltaSeconds));
        }
        const float zoom = static_cast<float>(pressed(GLFW_KEY_W)) - static_cast<float>(pressed(GLFW_KEY_S));
        state->orbitCamera->zoom(zoom * state->orbitCamera->settings().keyboardZoomStepsPerSecond *
            static_cast<float>(deltaSeconds));
    }

    void verifyCameraCallbacks(GLFWwindow* window)
    {
        auto& state = *getWindowState(window);
        auto& camera = *state.orbitCamera;
        const auto require = [](bool ok, const char* message) {
            if (!ok) throw std::runtime_error(message);
        };
        const auto same = [](glm::vec3 a, glm::vec3 b) { return glm::length(a - b) < 0.0001f; };
        const auto begin = [&](int button, int mods = 0) {
            // GLFW ignores cursor warps in an unfocused/hidden window. Feed the
            // same position event the native button callback normally follows.
            cursorPositionCallback(window, 100,100);
            mouseButtonCallback(window, button, GLFW_PRESS, mods);
        };
        const auto capture = [&](const char* name) {
            int width = 0, height = 0;
            glfwGetFramebufferSize(window, &width, &height);
            const std::filesystem::path output{"verification/desktop_camera"};
            std::filesystem::create_directories(output);
            state.applyCamera();
            state.renderer->beginFrame();
            state.renderer->renderModel(*state.model, state.model->fitToHeightTransform(2.2f),
                state.player->palette());
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
            std::ofstream file(output / (std::string{name} + ".ppm"), std::ios::binary);
            file << "P6\n" << width << ' ' << height << "\n255\n";
            for (int y = height - 1; y >= 0; --y)
                file.write(reinterpret_cast<const char*>(pixels.data() +
                    static_cast<std::size_t>(y) * width * 3), width * 3);
            require(static_cast<bool>(file), "Cannot save camera verification frame");
        };
        state.inputContext = {};
        state.syncInputContext();
        camera.frameBody(true);
        const auto initial = camera.position();
        const auto pivot = camera.target();
        capture("body");
        begin(GLFW_MOUSE_BUTTON_LEFT);
        cursorPositionCallback(window, 220,160);
        mouseButtonCallback(window, GLFW_MOUSE_BUTTON_LEFT, GLFW_RELEASE, 0);
        camera.update(1);
        require(same(camera.position(), initial), "Left interaction moved camera");
        begin(GLFW_MOUSE_BUTTON_RIGHT);
        cursorPositionCallback(window, 220,160);
        mouseButtonCallback(window, GLFW_MOUSE_BUTTON_RIGHT, GLFW_RELEASE, 0);
        camera.update(1);
        require(!same(camera.position(), initial) && same(camera.target(), pivot), "Orbit callback failed");
        const auto beforePan = camera.target();
        begin(GLFW_MOUSE_BUTTON_MIDDLE);
        cursorPositionCallback(window, 140,120);
        mouseButtonCallback(window, GLFW_MOUSE_BUTTON_MIDDLE, GLFW_RELEASE, 0);
        camera.update(1);
        require(!same(camera.target(), beforePan), "Pan callback failed");
        const float distance = camera.distance();
        scrollCallback(window, 0,2);
        camera.update(1);
        require(camera.distance() < distance, "Zoom callback failed");
        keyCallback(window, GLFW_KEY_F, 0, GLFW_PRESS, 0);
        camera.update(1);
        require(camera.target().y > pivot.y, "Face focus callback failed");
        capture("face_angle");
        keyCallback(window, GLFW_KEY_B, 0, GLFW_PRESS, 0);
        camera.update(1);
        require(same(camera.position(), initial), "Camera reset callback failed");
        // Cancellation must survive return to the window without a new press.
        for (int reason = 0; reason < 3; ++reason)
        {
            begin(GLFW_MOUSE_BUTTON_RIGHT);
            if (reason == 0) focusCallback(window, GLFW_FALSE);
            if (reason == 1) cursorEnterCallback(window, GLFW_FALSE);
            if (reason == 2) { state.inputContext.mouseCapturedByUi = true; state.syncInputContext(); }
            scrollCallback(window, 0,10);
            cursorPositionCallback(window, 250,180);
            state.inputContext = {};
            state.syncInputContext();
            cursorPositionCallback(window, 300,180);
            camera.update(1);
            require(!state.cameraInput.active() && same(camera.position(), initial), "Cancelled drag resumed");
        }
        state.inputContext.keyboardCapturedByUi = true;
        keyCallback(window, GLFW_KEY_F, 0, GLFW_PRESS, 0);
        camera.update(1);
        require(same(camera.position(), initial), "UI keyboard capture ignored");
        state.inputContext = {};
        state.syncInputContext();
        camera.focusFace();
        camera.update(1);
        capture("face_front");
        camera.zoom(1000);
        camera.update(1);
        capture("closest_front");
        camera.orbit(-glm::radians(90.0f) / camera.settings().rotationRadiansPerPixel, 0);
        camera.update(1);
        capture("closest_profile");
        int width = 0, height = 0;
        glfwGetWindowSize(window, &width, &height);
        glfwSetWindowSize(window, 240,900);
        glfwPollEvents();
        int portraitWidth = 0, portraitHeight = 0;
        glfwGetFramebufferSize(window, &portraitWidth, &portraitHeight);
        framebufferSizeCallback(window, portraitWidth,portraitHeight);
        camera.frameBody(true);
        require(camera.distance() > distance, "Portrait window did not affect body fitting");
        capture("portrait_body");
        framebufferSizeCallback(window, 0,0);
        glfwSetWindowSize(window, width,height);
        glfwPollEvents();
        glfwGetFramebufferSize(window, &width, &height);
        framebufferSizeCallback(window, width,height);
        camera.frameBody(true);
        state.applyCamera();
        std::cout << "Camera callbacks verified: ownership, orbit/pan/zoom, F/B, UI capture, focus/leave, resize\n";
    }
}

int main(int argc, char** argv)
{
    const bool verification = argc > 1 && std::string_view{argv[1]} == "--verify";
    const bool animationVerification = argc > 1 &&
        (std::string_view{argv[1]} == "--verify-animations" || std::string_view{argv[1]} == "--review-animations");
    if (animationVerification && argc != 3)
    {
        std::cerr << "Usage: julia_desktop --verify-animations|--review-animations output_directory\n";
        return 2;
    }
    const bool eyeVerification = argc > 1 &&
        (std::string_view{argv[1]} == "--verify-eyes" || std::string_view{argv[1]} == "--review-eyes");
    const bool noEyes = argc > 1 && std::string_view{argv[1]} == "--no-eyes";
    if (eyeVerification && argc != 3)
    {
        std::cerr << "Usage: julia_desktop --verify-eyes|--review-eyes output_directory\n";
        return 2;
    }
    // Модель можно передать аргументом командной строки; иначе берётся
    // ассет по умолчанию из каталога ресурсов.
    const std::filesystem::path modelPath =
        (argc > 1 && !verification && !animationVerification && !eyeVerification && !noEyes)
            ? std::filesystem::path{argv[1]}
            : assetDirectory(argv[0]) / "models" / "Raphtalia_prepared.fbx";

    glfwSetErrorCallback(glfwErrorCallback);

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Required on macOS to get a core-profile context; harmless elsewhere.
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);

    // Ask for a debug context so GL_KHR_debug can report errors via callback.
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);

    // Сглаживание краёв модели и обводки.
    glfwWindowHint(GLFW_SAMPLES, animationVerification || eyeVerification ? 4 : 8);

    const bool hidden = verification || (animationVerification && std::string_view{argv[1]} == "--verify-animations") ||
        (eyeVerification && std::string_view{argv[1]} == "--verify-eyes");
    glfwWindowHint(GLFW_VISIBLE, hidden ? GLFW_FALSE : GLFW_TRUE);
    GLFWwindow* window = glfwCreateWindow(verification ? 640 : animationVerification || eyeVerification ? 768 : 1280,
        verification ? 480 : animationVerification || eyeVerification ? 768 : 900, "Julia", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(verification || animationVerification || eyeVerification ? 0 : 1);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::cerr << "Failed to initialize glad\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << "\n";

    // Available when the driver exposes GL_KHR_debug (glad leaves the pointer
    // null otherwise). Synchronous output keeps messages next to the call.
    if (glDebugMessageCallback)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glDebugCallback, nullptr);
    }

    GLint samples = 0;
    glGetIntegerv(GL_SAMPLES, &samples);
    std::cout << "MSAA samples: " << samples << "\n";

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);

    int result = 0;

    try
    {
        julia::render_opengl::OpenGLRenderer renderer;
        renderer.initialize(framebufferWidth, framebufferHeight, assetDirectory(argv[0]));

        std::cout << "Loading model: " << modelPath.string() << "\n";

        // Both bundled rigged exports contain per-face normals, including the
        // prepared default. Repair these known assets; other models may carry
        // deliberately authored toon normals that must remain unchanged.
        const auto isBundledAsset = [&](const char* name) {
            std::error_code assetComparisonError;
            return std::filesystem::equivalent(modelPath,
                assetDirectory(argv[0]) / "models" / name, assetComparisonError) ||
                std::filesystem::equivalent(modelPath,
                    std::filesystem::path{JULIA_ASSET_DIR} / "models" / name,
                    assetComparisonError);
        };
        const bool repairFacetedNormals = isBundledAsset("Raphtalia.fbx") ||
            isBundledAsset("Raphtalia_prepared.fbx");
        auto model = julia::render_opengl::GLModelLoader::loadFromFile(modelPath,
            {.rebuildSmoothNormals = repairFacetedNormals});

        std::cout << "Model loaded\n";
        std::cout << "Meshes: " << model.meshCount() << "\n";
        std::cout << "Bones: " << model.boneCount() << "\n";
        std::cout << "Animation clips: " << model.animations().size() << "\n";
        std::size_t texturedMeshes = 0;
        for (const auto& mesh : model.meshes())
            if (mesh.material().diffuseTexture) ++texturedMeshes;
        std::cout << "Textured meshes: " << texturedMeshes << '/' << model.meshCount() << "\n";
        if (texturedMeshes == 0)
            std::cerr << "Material diagnostic: this asset has no assigned images; using imported diffuse values.\n";
        julia::render_opengl::AnimationPlayer player{model};
        julia::render_opengl::EyePoseLayer eyes{model};
        eyes.setEnabled(!animationVerification && !noEyes);
        player.setEyeLayer(&eyes);
        std::cout << "Procedural eyes: " << (eyes.available() ? "ready (188 baked eyelid bones)" : "unavailable for this rig") << '\n';

        const glm::mat4 fitModel = model.fitToHeightTransform(2.2f);

        julia::render_opengl::CameraConfig cameraConfig{};
        const auto subject = cameraSubject(model, fitModel);
        const float subjectHeight = subject.max.y - subject.min.y;
        cameraConfig.nearPlane = subjectHeight * 0.01f;
        cameraConfig.farPlane = subjectHeight * 100.0f;
        julia::render::OrbitCamera orbitCamera{subject};
        orbitCamera.setViewport(static_cast<float>(framebufferWidth),
            static_cast<float>(framebufferHeight), cameraConfig.fovDegrees);
        orbitCamera.frameBody(true);

        WindowState windowState{};
        windowState.renderer = &renderer;
        windowState.orbitCamera = &orbitCamera;
        windowState.cameraConfig = cameraConfig;
        windowState.player = &player;
        windowState.eyes = &eyes;
        windowState.model = &model;
        glfwGetCursorPos(window, &windowState.cursorX, &windowState.cursorY);
        windowState.inputContext.focused = hidden || glfwGetWindowAttrib(window, GLFW_FOCUSED) != 0;
        windowState.inputContext.inside = hidden || glfwGetWindowAttrib(window, GLFW_HOVERED) != 0;
        windowState.syncInputContext();

        glfwSetWindowUserPointer(window, &windowState);
        glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetCursorPosCallback(window, cursorPositionCallback);
        glfwSetScrollCallback(window, scrollCallback);
        glfwSetWindowFocusCallback(window, focusCallback);
        glfwSetCursorEnterCallback(window, cursorEnterCallback);
        glfwSetKeyCallback(window, keyCallback);

        windowState.applyCamera();

        std::cout
            << "Camera: right drag orbit; middle or Shift+right drag pan; wheel/W/S zoom.\n"
            << "A/D/Q/E or arrows pan; F face (keep angle); B reset front/body; Esc exit.\n"
            << "Left mouse is reserved for character interaction.\n";
        std::cout << "Animation: PgUp/PgDn select, Space play/pause, X/R stop + bind, L loop,\n"
            << "1..4 blink clips, comma/period step 1/30 s, F face / B body, F1 list clips.\n";
        std::cout << "Eyes: G natural/resume, H auto blink, J auto gaze, K blink, U/I wink,\n"
            << "numpad 1..9 gaze, Home forward, T hold camera point, [/] blink interval, F8 fixed seed.\n";
        keyCallback(window, GLFW_KEY_F1, 0, GLFW_PRESS, 0);
        if (verification) verifyCameraCallbacks(window);
        if (animationVerification)
        {
            verifyDesktopAnimations(window, renderer, model, player, fitModel,
                std::filesystem::path{argv[2]}, keyCallback);
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (eyeVerification)
        {
            verifyDesktopEyes(window, renderer, model, player, eyes, fitModel,
                std::filesystem::path{argv[2]}, keyCallback);
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        double previousTime = glfwGetTime();
        double titleTime = previousTime;
        int titleFrames = 0;
        int verificationFrame = 0;
        while (!glfwWindowShouldClose(window))
        {
            // Poll before updating to render the latest input in this frame.
            glfwPollEvents();
            const double now = glfwGetTime();
            const double deltaSeconds = verification ? 1.0 / 120.0 : now - previousTime;
            previousTime = now;
            if (verification)
            {
                // Exercise the actual desktop input callbacks without depending
                // on OS focus or synthetic global mouse/keyboard injection.
                if (verificationFrame == 10 || verificationFrame == 70)
                {
                    const int button = verificationFrame == 10
                        ? GLFW_MOUSE_BUTTON_RIGHT : GLFW_MOUSE_BUTTON_MIDDLE;
                    cursorPositionCallback(window, 100,100);
                    mouseButtonCallback(window, button, GLFW_PRESS, 0);
                    cursorPositionCallback(window, 180.0,130.0);
                    mouseButtonCallback(window, button, GLFW_RELEASE, 0);
                }
                if (verificationFrame == 40) scrollCallback(window, 0.0, 2.0);
                if (++verificationFrame >= 120)
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            if (glfwGetWindowAttrib(window, GLFW_ICONIFIED))
            {
                windowState.cameraInput.cancel();
                glfwWaitEventsTimeout(0.05);
                previousTime = glfwGetTime();
                continue;
            }
            // A long OS pause must not turn held keys into a large jump.
            processInput(window, deltaSeconds <= 0.25 ? deltaSeconds : 0.0);
            orbitCamera.update(deltaSeconds);
            windowState.applyCamera();
            player.update(deltaSeconds);

            renderer.beginFrame();

            glm::mat4 modelMatrix{1.0f};

            modelMatrix = modelMatrix * fitModel;

            renderer.renderModel(model, modelMatrix, player.palette());
            ++titleFrames;
            if (now - titleTime >= .25)
            {
                constexpr const char* states[]{"stopped/bind", "playing", "paused", "finished"};
                std::ostringstream title;
                title << "JULIA | " << (player.clip() ? player.clip()->name : "no clips")
                    << " | " << states[static_cast<int>(player.state())] << ' '
                    << std::fixed << std::setprecision(2) << player.timeSeconds() << '/'
                    << (player.clip() ? player.clip()->durationSeconds() : 0.0) << " s | loop "
                    << (player.looping() ? "on" : "off") << " | " << std::setprecision(0)
                    << titleFrames / (now - titleTime) << " FPS | eyes " << (eyes.enabled() ? "on" : "off")
                    << " blink " << eyes.behavior().output().blinkLeft << '/' << eyes.behavior().output().blinkRight
                    << " | RMB orbit MMB pan Wheel zoom F face B reset | PgUp/Dn Space X";
                glfwSetWindowTitle(window, title.str().c_str());
                titleFrames = 0;
                titleTime = now;
            }

            if (verification && glGetError() != GL_NO_ERROR)
                throw std::runtime_error("OpenGL error during desktop verification");

            glfwSwapBuffers(window);
        }
        if (verification)
            std::cout << "Desktop verification completed: orbit, zoom, pan, 120 frames\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Fatal error: " << exception.what() << "\n";
        result = 1;
    }

    glfwSetWindowUserPointer(window, nullptr);

    glfwDestroyWindow(window);
    glfwTerminate();

    return result;
}
