// OpenGLWindow.cpp
#include "OpenGLWindow.h" // Подключаем объявление нашего класса

// Конструктор
OpenGLWindow::OpenGLWindow(float width, float height, const char* title)
    : windowWidth(width), windowHeight(height), windowTitle(title ? title : "OpenGL Window"), window(nullptr)
{
    if (!initialize())
    {
        std::cerr << "ОШИБКА: Не удалось инициализировать OpenGLWindow." << std::endl;
        // Здесь можно бросить исключение или вызвать exit, в зависимости от твоей стратегии обработки ошибок
        glfwTerminate(); // Очищаем GLFW ресурсы, если инициализация окна не удалась
        exit(EXIT_FAILURE); // Пример завершения программы
    }
    setWindowPosition(); // Устанавливаем позицию окна после инициализации
}

// Деструктор
OpenGLWindow::~OpenGLWindow()
{
    if (window) { // Проверяем, было ли окно создано перед вызовом terminate
        glfwDestroyWindow(window); // Уничтожаем окно
    }
    glfwTerminate(); // Завершаем работу GLFW, освобождая все ресурсы
}

// Инициализация GLFW, создание окна и инициализация GLAD
bool OpenGLWindow::initialize()
{
    // Инициализация GLFW
    if (!glfwInit())
    {
        std::cerr << "\x1b[31mОШИБКА:\x1b[0m не удалось инициализировать GLFW." << std::endl;
        return false;
    }

    // Устанавливаем параметры GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE); // Убираем рамку и кнопки управления

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Для macOS
#endif

    // Создание окна
    window = glfwCreateWindow(static_cast<int>(windowWidth), static_cast<int>(windowHeight), windowTitle, nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "\x1b[31mОШИБКА:\x1b[0m Не удалось создать окно GLFW." << std::endl;
        glfwTerminate(); // Очищаем ресурсы GLFW, так как окно не создано
        return false;
    }

    // Устанавливаем контекст окна
    glfwMakeContextCurrent(window);

    // Инициализация GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "\x1b[31mОШИБКА:\x1b[0m Не удалось инициализировать GLAD." << std::endl;
        // glfwDestroyWindow(window) не нужен здесь, т.к. если GLAD не инициализируется, контекст уже создан
        glfwTerminate(); // Очищаем ресурсы GLFW
        return false;
    }

    // Задаем размеры области просмотра
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
        std::cerr << "ПРЕДУПРЕЖДЕНИЕ: Не удалось получить видеорежим для установки позиции окна." << std::endl;
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
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Также добавил очистку буфера глубины, это стандартная практика
}

// Основной цикл окна
void OpenGLWindow::run()
{
    if (!window) {
        std::cerr << "ОШИБКА: Окно не инициализировано. Запуск невозможен." << std::endl;
        return;
    }
    // Основной цикл окна
    // Логика рендеринга сцены (треугольника и т.д.) должна быть в Engine::render()
    // OpenGLWindow::run() теперь отвечает только за цикл событий окна и базовые операции
    while (!glfwWindowShouldClose(window))
    {
        // Обработка ввода
        processInput();

        // Очистка экрана (вызывается из OpenGLWindow, если нужно)
        // renderClear(); // Раскомментируй, если хочешь, чтобы окно само очищало экран каждый кадр.
                         // Обычно это делает Engine перед отрисовкой сцены.

        // ВАЖНО: Основная логика рендеринга (вызов Engine::render()) должна быть здесь,
        // но для этого Engine должен быть доступен, или этот цикл должен управляться извне.
        // Сейчас я оставлю его пустым, предполагая, что Engine будет вызывать методы окна.
        // Если Engine::runWindow() вызывает OpenGLWindow::run(), то внутри этого цикла
        // нужно будет вызывать методы обновления и рендеринга Engine.

        // Пример того, как это могло бы быть, если бы Engine передавался в run или был доступен:
        // engine.update(deltaTime);
        // engine.render();

        // Меняем буферы
        glfwSwapBuffers(window);

        // Обрабатываем события (ввод, изменение размера окна и т.д.)
        glfwPollEvents();
    }
}

