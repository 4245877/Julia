#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>

class OpenGLWindow
{
public:
    OpenGLWindow(int width, int height, const char* title);
    ~OpenGLWindow();

    void run(); // ������ ��������� �����

private:
    int windowWidth;
    int windowHeight;
    const char* windowTitle;
    GLFWwindow* window;

    bool initialize(); // ������������� GLFW � GLAD
    void processInput(); // ��������� �����
    void render(); // ��������� �����
};

OpenGLWindow::OpenGLWindow(int width, int height, const char* title)
    : windowWidth(width), windowHeight(height), windowTitle(title), window(nullptr)
{
    if (!initialize())
    {
        std::cerr << "Failed to initialize OpenGLWindow." << std::endl;
        exit(EXIT_FAILURE);
    }
}

OpenGLWindow::~OpenGLWindow()
{
    glfwTerminate();
}

bool OpenGLWindow::initialize()
{
    // ������������� GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return false;
    }

    // ������������� ��������� GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // ��� macOS
#endif

    // �������� ����
    window = glfwCreateWindow(windowWidth, windowHeight, windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return false;
    }

    // ������������� �������� ����
    glfwMakeContextCurrent(window);

    // ������������� GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD." << std::endl;
        return false;
    }

    // ������ ������� ������� ���������
    glViewport(0, 0, windowWidth, windowHeight);

    return true;
}

void OpenGLWindow::processInput()
{
    // �������� ���� ��� ������� ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
}

void OpenGLWindow::render()
{
    // ������� ����� (������ ���� ����)
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

void OpenGLWindow::run()
{
    // �������� ���� ����
    while (!glfwWindowShouldClose(window))
    {
        // ��������� �����
        processInput();

        // ��������� �����
        render();

        // ������ ������
        glfwSwapBuffers(window);

        // ������������ �������
        glfwPollEvents();
    }
}
