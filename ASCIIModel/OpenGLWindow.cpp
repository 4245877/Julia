// OpenGLWindow.cpp
#include "OpenGLWindow.h" // ���������� ���������� ������ ������

// �����������
OpenGLWindow::OpenGLWindow(float width, float height, const char* title)
    : windowWidth(width), windowHeight(height), windowTitle(title ? title : "OpenGL Window"), window(nullptr)
{
    if (!initialize())
    {
        std::cerr << "������: �� ������� ���������������� OpenGLWindow." << std::endl;
        // ����� ����� ������� ���������� ��� ������� exit, � ����������� �� ����� ��������� ��������� ������
        glfwTerminate(); // ������� GLFW �������, ���� ������������� ���� �� �������
        exit(EXIT_FAILURE); // ������ ���������� ���������
    }
    setWindowPosition(); // ������������� ������� ���� ����� �������������
}

// ����������
OpenGLWindow::~OpenGLWindow()
{
    if (window) { // ���������, ���� �� ���� ������� ����� ������� terminate
        glfwDestroyWindow(window); // ���������� ����
    }
    glfwTerminate(); // ��������� ������ GLFW, ���������� ��� �������
}

// ������������� GLFW, �������� ���� � ������������� GLAD
bool OpenGLWindow::initialize()
{
    // ������������� GLFW
    if (!glfwInit())
    {
        std::cerr << "\x1b[31m������:\x1b[0m �� ������� ���������������� GLFW." << std::endl;
        return false;
    }

    // ������������� ��������� GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE); // ������� ����� � ������ ����������

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // ��� macOS
#endif

    // �������� ����
    window = glfwCreateWindow(static_cast<int>(windowWidth), static_cast<int>(windowHeight), windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "\x1b[31m������:\x1b[0m �� ������� ������� ���� GLFW." << std::endl;
        glfwTerminate(); // ������� ������� GLFW, ��� ��� ���� �� �������
        return false;
    }

    // ������������� �������� ����
    glfwMakeContextCurrent(window);

    // ������������� GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "\x1b[31m������:\x1b[0m �� ������� ���������������� GLAD." << std::endl;
        // glfwDestroyWindow(window) �� ����� �����, �.�. ���� GLAD �� ����������������, �������� ��� ������
        glfwTerminate(); // ������� ������� GLFW
        return false;
    }

    // ������ ������� ������� ���������
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
        std::cerr << "��������������: �� ������� �������� ���������� ��� ��������� ������� ����." << std::endl;
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
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // ����� ������� ������� ������ �������, ��� ����������� ��������
}

// �������� ���� ����
void OpenGLWindow::run()
{
    if (!window) {
        std::cerr << "������: ���� �� ����������������. ������ ����������." << std::endl;
        return;
    }
    // �������� ���� ����
    // ������ ���������� ����� (������������ � �.�.) ������ ���� � Engine::render()
    // OpenGLWindow::run() ������ �������� ������ �� ���� ������� ���� � ������� ��������
    while (!glfwWindowShouldClose(window))
    {
        // ��������� �����
        processInput();

        // ������� ������ (���������� �� OpenGLWindow, ���� �����)
        // renderClear(); // ��������������, ���� ������, ����� ���� ���� ������� ����� ������ ����.
                         // ������ ��� ������ Engine ����� ���������� �����.

        // �����: �������� ������ ���������� (����� Engine::render()) ������ ���� �����,
        // �� ��� ����� Engine ������ ���� ��������, ��� ���� ���� ������ ����������� �����.
        // ������ � ������� ��� ������, �����������, ��� Engine ����� �������� ������ ����.
        // ���� Engine::runWindow() �������� OpenGLWindow::run(), �� ������ ����� �����
        // ����� ����� �������� ������ ���������� � ���������� Engine.

        // ������ ����, ��� ��� ����� �� ����, ���� �� Engine ����������� � run ��� ��� ��������:
        // engine.update(deltaTime);
        // engine.render();

        // ������ ������
        glfwSwapBuffers(window);

        // ������������ ������� (����, ��������� ������� ���� � �.�.)
        glfwPollEvents();
    }
}

