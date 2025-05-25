// OpenGLWindow.cpp
#include "OpenGLWindow.h" // Подключаем объявление нашего класса
#include "Engine.h" // Подключаем Engine для проверки инициализации GLFW
#include <stdexcept> // для std::runtime_error

// Конструктор
OpenGLWindow::OpenGLWindow(int width, int height, const char* title)
    : windowWidth(width), windowHeight(height), windowTitle(title ? title : "OpenGL Window"), window(nullptr)
{
    if (!initialize())
    {
        throw std::runtime_error("Не удалось инициализировать OpenGLWindow.");
    }
    setWindowPosition();
}

// Деструктор
OpenGLWindow::~OpenGLWindow()
{
    if (window) { // Проверяем, было ли окно создано перед вызовом terminate
        glfwDestroyWindow(window); // Уничтожаем окно
    }
    glfwTerminate(); // Завершаем работу GLFW, освобождая все ресурсы
}

 
bool OpenGLWindow::initialize()
{
    // Проверяем, был ли GLFW инициализирован (дополнительная защита)
    if (!Engine::IsGLFWInitialized()) { // Предполагаем, что у вас есть Engine::IsGLFWInitialized()
        std::cerr << "\x1b[31mКРИТИЧЕСКАЯ ОШИБКА:\x1b[0m Попытка инициализировать окно OpenGL без предварительной инициализации GLFW движком!" << std::endl;
        return false;
    }

    // Устанавливаем параметры GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Создание окна
    window = glfwCreateWindow(static_cast<int>(windowWidth), static_cast<int>(windowHeight), windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "\x1b[31mОШИБКА:\x1b[0m Не удалось создать окно GLFW." << std::endl;
        // glfwTerminate() здесь не вызываем. Engine решит, что делать.
        return false;
    }

    // Устанавливаем контекст окна
    glfwMakeContextCurrent(window);

    // Инициализация GLAD (это правильно делать здесь, после создания контекста)
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "\x1b[31mОШИБКА:\x1b[0m Не удалось инициализировать GLAD." << std::endl;
        // glfwTerminate() здесь не вызываем.
        // glfwDestroyWindow(window); // Окно можно уничтожить, если GLAD не загрузился.
        return false;
    }

    glViewport(0, 0, static_cast<int>(windowWidth), static_cast<int>(windowHeight));

    return true;
}

// Установка позиции окна в правый нижний угол
void OpenGLWindow::setWindowPosition()
{
    if (!window) return; // Если окно не создано, ничего не делаем

    // Получаем размер экрана
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    if (mode)
    {
        // Рассчитываем координаты для правого нижнего угла
        int posX = mode->width - static_cast<int>(windowWidth);
        int posY = mode->height - static_cast<int>(windowHeight) - 40; // Учитываем высоту панели задач (около 40 пикселей)

        // Устанавливаем позицию окна
        glfwSetWindowPos(window, posX, posY);
    }
    else
    {
        std::cerr << "\x1b[33mWARNING:\x1b[33m Не удалось получить видеорежим для установки позиции окна." << std::endl;
    }
}

// Обработка ввода (закрытие окна по ESC)
void OpenGLWindow::processInput()
{
    if (!window) return;

    // Закрытие окна при нажатии ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
    // Здесь можно будет добавить другую обработку ввода, специфичную для окна (например, переключение полноэкранного режима)
}

// Очистка экрана
void OpenGLWindow::renderClear()
{
    // Очищаем экран (меняем цвет фона)
    glClearColor(0.4f, 0.4f, 0.9f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
}

// Основной цикл окна
void OpenGLWindow::run()
{
    if (!window) {
        std::cerr << "ОШИБКА: Окно не инициализировано. Запуск невозможен." << std::endl;
        return;
    }

    while (!glfwWindowShouldClose(window))
    {
        processInput();

        // Вызываем функцию рендеринга, если она установлена
        if (renderCallback) {
            renderCallback(); // << ДОБАВИТЬ ЭТОТ ВЫЗОВ
        }
        // Закомментированная строка renderClear() больше не нужна здесь,
        // так как очистка экрана должна происходить в Engine::render()
        // // renderClear(); 

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void OpenGLWindow::setRenderCallback(std::function<void()> callback) {
    this->renderCallback = callback;
}