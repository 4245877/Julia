#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "julia/render_opengl/GLModelLoader.hpp"
#include "julia/render_opengl/OpenGLRenderer.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <iostream>
#include <stdexcept>

#ifndef JULIA_ASSET_DIR
#define JULIA_ASSET_DIR "assets"
#endif

namespace
{
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
        auto* renderer = static_cast<julia::render_opengl::OpenGLRenderer*>(
            glfwGetWindowUserPointer(window)
        );

        if (renderer)
        {
            renderer->resize(width, height);
        }
    }

    void processInput(GLFWwindow* window)
    {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }
}

int main(int argc, char** argv)
{
    // Модель можно передать аргументом командной строки; иначе берётся
    // ассет по умолчанию из каталога ресурсов.
    const std::filesystem::path modelPath =
        (argc > 1)
            ? std::filesystem::path{argv[1]}
            : std::filesystem::path{JULIA_ASSET_DIR} / "models" / "Raphtalia1.fbx";

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

    GLFWwindow* window = glfwCreateWindow(1920, 1080, "Julia", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

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
        renderer.initialize(framebufferWidth, framebufferHeight);

        glfwSetWindowUserPointer(window, &renderer);
        glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

        std::cout << "Loading model: " << modelPath.string() << "\n";

        auto model = julia::render_opengl::GLModelLoader::loadFromFile(modelPath);

        std::cout << "Model loaded\n";
        std::cout << "Meshes: " << model.meshCount() << "\n";
        std::cout << "Bones: " << model.boneCount() << "\n";

        const glm::mat4 fitModel = model.fitToHeightTransform(2.2f);

        while (!glfwWindowShouldClose(window))
        {
            processInput(window);

            renderer.beginFrame();

            glm::mat4 modelMatrix{1.0f};

            modelMatrix = modelMatrix * fitModel;

            renderer.renderModel(model, modelMatrix);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
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
