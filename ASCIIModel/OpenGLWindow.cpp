// OpenGLWindow.cpp
#include "OpenGLWindow.h" // ���������� ���������� ������ ������
#include "Engine.h" // ���������� Engine ��� �������� ������������� GLFW
#include <stdexcept> // ��� std::runtime_error

// �����������
OpenGLWindow::OpenGLWindow(int width, int height, const char* title)
    : windowWidth(width), windowHeight(height), windowTitle(title ? title : "OpenGL Window"), window(nullptr)
{
    if (!initialize())
    {
        throw std::runtime_error("�� ������� ���������������� OpenGLWindow.");
    }
    setWindowPosition();
}

// ����������
OpenGLWindow::~OpenGLWindow()
{
    if (window) { // ���������, ���� �� ���� ������� ����� ������� terminate
        glfwDestroyWindow(window); // ���������� ����
    }
    glfwTerminate(); // ��������� ������ GLFW, ���������� ��� �������
}

 
bool OpenGLWindow::initialize()
{
    // ���������, ��� �� GLFW ��������������� (�������������� ������)
    if (!Engine::IsGLFWInitialized()) { // ������������, ��� � ��� ���� Engine::IsGLFWInitialized()
        std::cerr << "\x1b[31m����������� ������:\x1b[0m ������� ���������������� ���� OpenGL ��� ��������������� ������������� GLFW �������!" << std::endl;
        return false;
    }

    // ������������� ��������� GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // �������� ����
    window = glfwCreateWindow(static_cast<int>(windowWidth), static_cast<int>(windowHeight), windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "\x1b[31m������:\x1b[0m �� ������� ������� ���� GLFW." << std::endl;
        // glfwTerminate() ����� �� ��������. Engine �����, ��� ������.
        return false;
    }

    // ������������� �������� ����
    glfwMakeContextCurrent(window);

    // ������������� GLAD (��� ��������� ������ �����, ����� �������� ���������)
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "\x1b[31m������:\x1b[0m �� ������� ���������������� GLAD." << std::endl;
        // glfwTerminate() ����� �� ��������.
        // glfwDestroyWindow(window); // ���� ����� ����������, ���� GLAD �� ����������.
        return false;
    }

    glViewport(0, 0, static_cast<int>(windowWidth), static_cast<int>(windowHeight));

    return true;
}

// ��������� ������� ���� � ������ ������ ����
void OpenGLWindow::setWindowPosition()
{
    if (!window) return; // ���� ���� �� �������, ������ �� ������

    // �������� ������ ������
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    if (mode)
    {
        // ������������ ���������� ��� ������� ������� ����
        int posX = mode->width - static_cast<int>(windowWidth);
        int posY = mode->height - static_cast<int>(windowHeight) - 40; // ��������� ������ ������ ����� (����� 40 ��������)

        // ������������� ������� ����
        glfwSetWindowPos(window, posX, posY);
    }
    else
    {
        std::cerr << "\x1b[33mWARNING:\x1b[33m �� ������� �������� ���������� ��� ��������� ������� ����." << std::endl;
    }
}

// ��������� ����� (�������� ���� �� ESC)
void OpenGLWindow::processInput()
{
    if (!window) return;

    // �������� ���� ��� ������� ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
    // ����� ����� ����� �������� ������ ��������� �����, ����������� ��� ���� (��������, ������������ �������������� ������)
}

// ������� ������
void OpenGLWindow::renderClear()
{
    // ������� ����� (������ ���� ����)
    glClearColor(0.4f, 0.4f, 0.9f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
}

// �������� ���� ����
void OpenGLWindow::run()
{
    if (!window) {
        std::cerr << "������: ���� �� ����������������. ������ ����������." << std::endl;
        return;
    }

    while (!glfwWindowShouldClose(window))
    {
        processInput();

        // �������� ������� ����������, ���� ��� �����������
        if (renderCallback) {
            renderCallback(); // << �������� ���� �����
        }
        // ������������������ ������ renderClear() ������ �� ����� �����,
        // ��� ��� ������� ������ ������ ����������� � Engine::render()
        // // renderClear(); 

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void OpenGLWindow::setRenderCallback(std::function<void()> callback) {
    this->renderCallback = callback;
}