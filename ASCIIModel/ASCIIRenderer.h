#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>

class OpenGLWindow
{
public:
    OpenGLWindow(int width, int height, const char* title);
    ~OpenGLWindow();

    void run(); // Запуск основного цикла

private:
    int windowWidth;
    int windowHeight;
    const char* windowTitle;
    GLFWwindow* window;

    bool initialize(); // Инициализация GLFW и GLAD
    void processInput(); // Обработка ввода
    void render(); // Рендеринг кадра
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
    // Инициализация GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return false;
    }

    // Устанавливаем параметры GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Для macOS
#endif

    // Создание окна
    window = glfwCreateWindow(windowWidth, windowHeight, windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return false;
    }

    // Устанавливаем контекст окна
    glfwMakeContextCurrent(window);

    // Инициализация GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD." << std::endl;
        return false;
    }

    // Задаем размеры области просмотра
    glViewport(0, 0, windowWidth, windowHeight);

    return true;
}

void OpenGLWindow::processInput()
{
    // Закрытие окна при нажатии ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
}

void OpenGLWindow::render()
{
    // Очищаем экран (меняем цвет фона)
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

void OpenGLWindow::run()
{
    // Основной цикл окна
    while (!glfwWindowShouldClose(window))
    {
        // Обработка ввода
        processInput();

        // Рендеринг кадра
        render();

        // Меняем буферы
        glfwSwapBuffers(window);

        // Обрабатываем события
        glfwPollEvents();
    }
}
