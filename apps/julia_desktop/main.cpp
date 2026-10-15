#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render/OrbitCamera.hpp"
#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string_view>

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

    struct WindowState
    {
        julia::render_opengl::OpenGLRenderer* renderer = nullptr;
        julia::render::OrbitCamera* orbitCamera = nullptr;
        julia::render_opengl::CameraConfig cameraConfig{};
        int dragButton = -1;
        bool panning = false;
        double lastCursorX = 0.0;
        double lastCursorY = 0.0;

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
        const WindowState* state = getWindowState(window);

        if (state && state->renderer)
        {
            state->renderer->resize(width, height);
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

        if (action == GLFW_PRESS)
        {
            state->dragButton = button;
            state->panning = button != GLFW_MOUSE_BUTTON_LEFT ||
                             (mods & GLFW_MOD_SHIFT) != 0;
            glfwGetCursorPos(
                window,
                &state->lastCursorX,
                &state->lastCursorY
            );
        }
        else if (action == GLFW_RELEASE && state->dragButton == button)
        {
            state->dragButton = -1;
        }
    }

    void cursorPositionCallback(GLFWwindow* window, double x, double y)
    {
        WindowState* state = getWindowState(window);

        if (!state || state->dragButton < 0 || !state->orbitCamera)
        {
            return;
        }

        const double deltaX = x - state->lastCursorX;
        const double deltaY = y - state->lastCursorY;

        state->lastCursorX = x;
        state->lastCursorY = y;

        if (state->panning)
        {
            int width = 0;
            int height = 0;
            // Cursor deltas use logical window pixels, including on HiDPI.
            glfwGetWindowSize(window, &width, &height);
            state->orbitCamera->pan(static_cast<float>(deltaX),
                static_cast<float>(deltaY), static_cast<float>(height),
                state->cameraConfig.fovDegrees);
        }
        else
        {
            state->orbitCamera->orbit(
                static_cast<float>(deltaX), static_cast<float>(deltaY));
        }
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

        state->orbitCamera->zoom(static_cast<float>(yOffset));
    }

    void focusCallback(GLFWwindow* window, int focused)
    {
        if (!focused)
        {
            if (auto* state = getWindowState(window))
                state->dragButton = -1;
        }
    }

    void processInput(GLFWwindow* window, double deltaSeconds)
    {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        auto* state = getWindowState(window);
        if (!state || !state->orbitCamera || !glfwGetWindowAttrib(window, GLFW_FOCUSED))
            return;
        const auto pressed = [window](int key) {
            return glfwGetKey(window, key) == GLFW_PRESS;
        };
        glm::vec3 direction{
            static_cast<float>(pressed(GLFW_KEY_D) || pressed(GLFW_KEY_RIGHT)) -
                static_cast<float>(pressed(GLFW_KEY_A) || pressed(GLFW_KEY_LEFT)),
            static_cast<float>(pressed(GLFW_KEY_E) || pressed(GLFW_KEY_UP)) -
                static_cast<float>(pressed(GLFW_KEY_Q) || pressed(GLFW_KEY_DOWN)),
            static_cast<float>(pressed(GLFW_KEY_S)) - static_cast<float>(pressed(GLFW_KEY_W))
        };
        if (glm::length(direction) > 0.0f)
        {
            const float speed = state->orbitCamera->distance() * 0.8f;
            state->orbitCamera->moveLocal(glm::normalize(direction) * speed *
                static_cast<float>(deltaSeconds));
        }
    }
}

int main(int argc, char** argv)
{
    const bool verification = argc > 1 && std::string_view{argv[1]} == "--verify";
    // Модель можно передать аргументом командной строки; иначе берётся
    // ассет по умолчанию из каталога ресурсов.
    const std::filesystem::path modelPath =
        (argc > 1 && !verification)
            ? std::filesystem::path{argv[1]}
            : assetDirectory(argv[0]) / "models" / "Raphtalia1.fbx";

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
    glfwWindowHint(GLFW_SAMPLES, 8);

    glfwWindowHint(GLFW_VISIBLE, verification ? GLFW_FALSE : GLFW_TRUE);
    GLFWwindow* window = glfwCreateWindow(verification ? 640 : 1920,
        verification ? 480 : 1080, "Julia", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(verification ? 0 : 1);

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

        auto model = julia::render_opengl::GLModelLoader::loadFromFile(modelPath);

        std::cout << "Model loaded\n";
        std::cout << "Meshes: " << model.meshCount() << "\n";
        std::cout << "Bones: " << model.boneCount() << "\n";

        const glm::mat4 fitModel = model.fitToHeightTransform(2.2f);

        const glm::vec3 modelCenter{
            fitModel * glm::vec4{model.bounds().center(), 1.0f}
        };

        julia::render_opengl::CameraConfig cameraConfig{};
        const glm::vec3 initialOffset =
            cameraConfig.position - cameraConfig.target;

        cameraConfig.target = modelCenter;
        cameraConfig.position = modelCenter + initialOffset;

        julia::render::OrbitCamera orbitCamera{
            modelCenter,
            cameraConfig.position
        };

        WindowState windowState{};
        windowState.renderer = &renderer;
        windowState.orbitCamera = &orbitCamera;
        windowState.cameraConfig = cameraConfig;

        glfwSetWindowUserPointer(window, &windowState);
        glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetCursorPosCallback(window, cursorPositionCallback);
        glfwSetScrollCallback(window, scrollCallback);
        glfwSetWindowFocusCallback(window, focusCallback);

        windowState.applyCamera();

        std::cout
            << "Camera controls: left drag to orbit, wheel to zoom, right/middle\n"
            << "or Shift+left drag to pan; WASD/QE or arrows to move; Esc to exit\n";

        double previousTime = glfwGetTime();
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
                        ? GLFW_MOUSE_BUTTON_LEFT : GLFW_MOUSE_BUTTON_RIGHT;
                    mouseButtonCallback(window, button, GLFW_PRESS, 0);
                    cursorPositionCallback(window, windowState.lastCursorX + 80.0,
                                           windowState.lastCursorY + 30.0);
                    mouseButtonCallback(window, button, GLFW_RELEASE, 0);
                }
                if (verificationFrame == 40) scrollCallback(window, 0.0, 2.0);
                if (++verificationFrame >= 120)
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            if (glfwGetWindowAttrib(window, GLFW_ICONIFIED))
            {
                windowState.dragButton = -1;
                glfwWaitEventsTimeout(0.05);
                previousTime = glfwGetTime();
                continue;
            }
            // A long OS pause must not turn held keys into a large jump.
            processInput(window, deltaSeconds <= 0.25 ? deltaSeconds : 0.0);
            orbitCamera.update(deltaSeconds);
            windowState.applyCamera();

            renderer.beginFrame();

            glm::mat4 modelMatrix{1.0f};

            modelMatrix = modelMatrix * fitModel;

            renderer.renderModel(model, modelMatrix);

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
